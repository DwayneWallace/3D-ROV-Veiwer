"""Cross-platform, receive-only NMEA UDP viewer. Python 3.10+, standard library."""
import argparse
import datetime
import json
import math
import pathlib
import socket
import threading
import time
import sys
# Keep this folder limited to its application files, including on normal launches.
sys.dont_write_bytecode = True

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

ROOT = pathlib.Path(__file__).resolve().parent
LOCK = threading.Lock()
STATE = {"demo": False, "sources": {}}
PUBLIC_FIELDS = ("port", "packets", "parse_errors", "last_packet", "sender", "error",
                 "position", "depth", "attitude", "heading", "last_valid_position", "last_valid_depth")


def snapshot():
    with LOCK:
        now = time.time()
        sources = {}
        for name, source in STATE["sources"].items():
            sources[name] = {}
            for key, value in source.items():
                if key not in PUBLIC_FIELDS:
                    continue
                if isinstance(value, dict):
                    value = value.copy()
                sources[name][key] = value
        return {"demo": STATE["demo"], "sources": sources,
                "cage_velocity": cage_velocity(now), "now": now}


def viewer_addresses(bind, port):
    addresses_to_print = []
    if bind == "0.0.0.0":
        addresses = set()
        try:
            for result in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
                addresses.add(result[4][0])
        except OSError:
            pass
        # Choose the default-route interface without sending a UDP packet.
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
                probe.connect(("192.0.2.1", 9))
                addresses.add(probe.getsockname()[0])
        except OSError:
            pass
        for address in sorted(addresses):
            if address != "0.0.0.0" and not address.startswith("127."):
                addresses_to_print.append(address + ":" + str(port))
    elif bind.startswith("127.") or bind == "localhost":
        addresses_to_print.append(bind + ":" + str(port))
        
    else:
        addresses_to_print.append(bind + ":" + str(port))
    return "\n".join(addresses_to_print)


def coordinate(value, hemisphere, latitude):
    number = float(value)
    degrees = int(number // 100)
    minutes = number - degrees * 100
    limit = 90 if latitude else 180
    if hemisphere not in ("NS" if latitude else "EW") or not 0 <= minutes < 60:
        raise ValueError("Invalid coordinate")
    result = degrees + minutes / 60
    if result > limit:
        raise ValueError("Coordinate out of range")
    return -result if hemisphere in "SW" else result


def vertical_rate(source, meters, utc, now):
    """Work out vertical speed from depth readings. Down is positive here."""
    source_seconds = int(utc[:2]) * 3600 + int(utc[2:4]) * 60 + float(utc[4:])
    history = source.setdefault("vertical_samples", [])
    if history:
        elapsed_seconds = (source_seconds - history[-1][0] + 43200) % 86400 - 43200
        if elapsed_seconds == 0:
            return
        if elapsed_seconds < 0:
            return
        if elapsed_seconds > 5:
            history.clear()
    history.append([source_seconds, meters])
    recent_readings = []
    for reading in history[-32:]:
        age = (source_seconds - reading[0]) % 86400
        if age <= 3:
            recent_readings.append(reading)
    history[:] = recent_readings
    elapsed_seconds = (source_seconds-history[0][0]) % 86400
    speed = None
    if elapsed_seconds >= 1:
        speed = (meters - history[0][1]) / elapsed_seconds
    source["vertical_velocity"] = {
        "valid": elapsed_seconds >= 1,
        "meters_per_second": speed,
        "source_time": source_seconds, "received": now}


def cage_velocity(now):
    cage = STATE["sources"].get("Cage", {}).get("vertical_velocity", {})
    boat = STATE["sources"].get("Boat", {}).get("vertical_velocity", {})
    valid = True
    for reading in (cage, boat):
        if not reading.get("valid") or now - reading.get("received", 0) >= 5:
            valid = False
    if valid:
        delta = abs((cage["source_time"]-boat["source_time"]+43200) % 86400-43200)
        valid = delta <= 3
    speed = None
    if valid:
        speed = cage["meters_per_second"] - boat["meters_per_second"]
    return {"valid": valid, "meters_per_second": speed,
            "received": min(cage.get("received", 0), boat.get("received", 0))}


def parse(sentence, sonardyne_depth=False):
    """Depth profile is opt-in and calibrated to this vessel's captured output."""
    start = sentence.find("$")
    if start < 0:
        return None
    sentence = sentence[start:].strip()
    checksum = "absent"
    if "*" in sentence:
        body, suffix = sentence[1:].split("*", 1)
        computed = 0
        for char in body:
            computed ^= ord(char)
        if len(suffix) != 2 or computed != int(suffix, 16):
            raise ValueError("Checksum mismatch")
        checksum = "verified"
    else:
        body = sentence[1:]
    fields = body.split(",")
    if fields[0] in ("PASHR", "PRDID"):
        if fields[0] == "PASHR":
            valid = len(fields) > 11 and fields[11] == "0" and fields[3] == "T"
            values = (fields[2], fields[4], fields[5])
        else:
            valid = True
            values = (fields[3], fields[2], fields[1])
        angles = []
        for text in values:
            angle = None
            if text:
                number = float(text)
                if math.isfinite(number):
                    angle = number
            angles.append(angle)
            if angle is None:
                valid = False
        valid = valid and abs(angles[1]) <= 180 and abs(angles[2]) <= 90
        result = {"attitude_only": True, "attitude": {
            "heading": angles[0], "roll": angles[1], "pitch": angles[2],
            "valid": valid, "sentence": fields[0], "checksum": checksum},
            "heave": None}
        if fields[0] == "PASHR":
            heave = None
            if fields[6]:
                heave = float(fields[6])
            result["heave"] = {"meters": heave, "utc": fields[1],
                               "valid": valid and bool(fields[6])}
        return result
    if fields[0] == "PSONUSBL" and sonardyne_depth:
        value = float(fields[6])
        return {"depth_only": True, "depth": {
            "meters": value if math.isfinite(value) and value >= 0 else None,
            "valid": math.isfinite(value) and value >= 0 and fields[3] == "H",
            "sentence": "PSONUSBL", "utc": fields[1], "checksum": checksum,
            "reference": "Vessel profile; compared with Sonardyne display"}}
    kind = fields[0][-3:]
    if kind == "HDT":
        value = float(fields[1])
        return {"attitude_only": True, "heading": {
            "degrees": value if math.isfinite(value) else None,
            "valid": math.isfinite(value) and 0 <= value <= 360 and fields[2] == "T",
            "sentence": "HDT", "checksum": checksum}}
    if kind == "GGA":
        quality = int(fields[6])
        result = {"sentence": kind, "valid": quality > 0, "quality": quality,
                  "utc": fields[1], "checksum": checksum}
        if quality > 0:
            result.update(latitude=coordinate(fields[2], fields[3], True),
                          longitude=coordinate(fields[4], fields[5], False))
        if sonardyne_depth:
            value = -float(fields[9]) if fields[9] else None
            result["depth"] = {
                "meters": value if value is not None and math.isfinite(value) else None,
                "valid": quality > 0 and fields[10] == "M" and value is not None
                         and math.isfinite(value) and value >= 0,
                "sentence": "GGA (inverted altitude)", "utc": fields[1], "checksum": checksum,
                "reference": "Vessel profile; compared with Sonardyne display"}
        return result
    if kind == "RMC":
        result = {"sentence": kind, "valid": fields[2] == "A", "utc": fields[1],
                  "checksum": checksum}
        if result["valid"]:
            result.update(latitude=coordinate(fields[3], fields[4], True),
                          longitude=coordinate(fields[5], fields[6], False))
        return result
    return None


def update_source(source, sentence, now, sonardyne_depth=False):
    position = parse(sentence, sonardyne_depth)
    if position:
        heave = position.pop("heave", None)
        if heave:
            if heave["valid"] and math.isfinite(heave["meters"]):
                vertical_rate(source, heave["meters"], heave["utc"], now)
            else:
                source["vertical_samples"] = []
                source["vertical_velocity"] = {"valid": False}
        attitude = position.pop("attitude", None)
        if attitude:
            source["attitude"] = dict(attitude, received=now)
            source["heading"] = {"degrees": attitude["heading"], "valid": attitude["valid"], "received": now}
        heading = position.pop("heading", None)
        if heading:
            source["heading"] = dict(heading, received=now)
        depth = position.pop("depth", None)
        if depth:
            old = source.get("depth", {})
            changed = old.get("utc") != depth["utc"] or old.get("sentence") != depth["sentence"]
            source["depth"] = dict(depth, received=now,
                                   updated=now if changed else old.get("updated", now))
            if depth["valid"]:
                source["last_valid_depth"] = source["depth"].copy()
                vertical_rate(source, depth["meters"], depth["utc"], now)
            else:
                source["vertical_samples"] = []
                source["vertical_velocity"] = {"valid": False}
        if not position.get("depth_only") and not position.get("attitude_only"):
            old = source.get("position", {})
            changed = old.get("utc") != position.get("utc")
            position["updated"] = now if changed else old.get("updated", now)
            source["position"] = dict(position, received=now)
            if position.get("valid"):
                source["last_valid_position"] = source["position"].copy()


def receive(name, port, bind, capture, sonardyne_depth=False):
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        udp_socket.bind((bind, port))
    except OSError as exc:
        with LOCK:
            STATE["sources"][name]["error"] = str(exc)
        udp_socket.close()
        return
    while True:
        payload, sender = udp_socket.recvfrom(65535)
        now = time.time()
        message_text = payload.decode("ascii", errors="replace")
        with LOCK:
            source = STATE["sources"][name]
            source.update(last_packet=now, sender=sender[0], packets=source["packets"] + 1)
            for sentence in message_text.replace("\x00", "\n").replace("\r", "\n").splitlines():
                if not sentence.strip():
                    continue
                try:
                    update_source(source, sentence, now, sonardyne_depth)
                except (ValueError, IndexError) as exc:
                    source["parse_errors"] += 1
                    source["last_parse_error"] = str(exc)
        if capture:
            record = {"time": datetime.datetime.now(datetime.timezone.utc).isoformat(),
                      "source": name, "port": port, "sender": sender[0], "raw": message_text}
            with capture.open("a", encoding="utf-8") as output:
                output.write(json.dumps(record) + "\n")


def demo():
    while True:
        now = time.time()
        with LOCK:
            for index, source in enumerate(STATE["sources"].values()):
                source.update(last_packet=now, sender="SIMULATED", packets=source["packets"] + 1,
                              position={"latitude": 0.0 + index * .0002 + math.sin(now / 20) * .0001,
                                        "longitude": 0.0 + index * .0003,
                                        "valid": True, "sentence": "DEMO", "checksum": "simulated",
                                        "received": now})
        time.sleep(.5)


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def do_GET(self):
        if self.path == "/api/state":
            payload = json.dumps(snapshot(), separators=(",", ":"), allow_nan=False).encode()
            content_type = "application/json"
        elif self.path in ("/", "/index.html"):
            payload = (ROOT / "index.html").read_bytes()
            content_type = "text/html; charset=utf-8"
        elif self.path == "/scene.js":
            payload = (ROOT / "scene.js").read_bytes()
            content_type = "text/javascript; charset=utf-8"
        else:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def log_message(self, *_):
        pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--demo", action="store_true")
    parser.add_argument("--raw", action="store_true", help="Listen to individual beacon feeds instead of Fusion outputs")
    parser.add_argument("--sonardyne-depth", action="store_true",
                        help="Enable vessel-tested depth profile: negative GGA altitude / PSONUSBL field 6")
    parser.add_argument("--bind", default="0.0.0.0", help="Local IPv4 interface for UDP reception")
    parser.add_argument("--http-port", default=8765, type=int)
    parser.add_argument("--http-bind", default="127.0.0.1",
                        help="Web interface address; use 0.0.0.0 for access from the vessel network")
    parser.add_argument("--capture", type=pathlib.Path, help="Append received datagrams to a JSONL file")
    args = parser.parse_args()
    if args.raw:
        feeds = {"ROV 2109": 10012, "ROV 2311": 10013,
                 "ROV 2411": 10014, "Cage 2607": 10015}
    else:
        feeds = {"ROV": 10112, "Cage": 10113}
    feeds["Boat"] = 10100
    feeds["NMEA"] = 10110
    STATE["demo"] = args.demo
    for name, port in feeds.items():
        STATE["sources"][name] = {"port": port, "packets": 0, "parse_errors": 0}
    server = ThreadingHTTPServer((args.http_bind, args.http_port), Handler)
    if args.demo:
        threading.Thread(target=demo, daemon=True).start()
    else:
        for name, port in feeds.items():
            use_depth = args.sonardyne_depth and name not in ("Boat", "NMEA")
            receiver_thread = threading.Thread(
                target=receive,
                args=(name, port, args.bind, args.capture, use_depth),
                daemon=True,
            )
            receiver_thread.start()

    print(viewer_addresses(args.http_bind, server.server_port), flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        server.server_close()


if __name__ == "__main__":
    main()




