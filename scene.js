// World axes: east, north, up. SBG GPS antenna is the boat origin.
const canvas = document.getElementById('view');
const ctx = canvas.getContext('2d');
let state = {
    sources: {
    }, now: 0
};
let disconnected = true;
let azimuth = 0.1;
let elevation = 0.45;
let distance = 85;
let dragging = null;
let dirty = true;
window.addEventListener('resize', function () {
    dirty = true;
});
const rad = Math.PI / 180;
function dot(first, second) {
    let total = 0;
    for (let i = 0; i < first.length; i++) {
        total += first[i] * second[i];
    }
    return total;
}
function sub(first, second) {
    let difference = [];
    for (let i = 0; i < first.length; i++) {
        difference.push(first[i] - second[i]);
    }
    return difference;
}
function fresh(reading) {
    if (disconnected || !reading || !reading.valid) {
        return false;
    }
    let lastUpdate = reading.received;
    if (reading.updated !== undefined) {
        lastUpdate = reading.updated;
    }
    return state.now - lastUpdate < 5;
}
function sources() {
    const sbg = state.sources.Boat || {};
    const nmea = state.sources.NMEA || {};
    const boat = Object.assign({}, sbg);
    const fields = ['position', 'heading', 'attitude'];

    for (let i = 0; i < fields.length; i++) {
        const field = fields[i];
        if (!fresh(sbg[field]) && fresh(nmea[field])) {
            boat[field] = nmea[field];
        }
    }

    return {
        Boat: boat,
        ROV: state.sources.ROV || {},
        Cage: state.sources.Cage || {}
    };
}

function offset(position, boatPosition, depth) {
    // Convert the small latitude/longitude difference into metres.
    const earthRadius = 6378137;
    const latitudeDifference = position.latitude - boatPosition.latitude;
    const longitudeDifference = position.longitude - boatPosition.longitude;
    const middleLatitude = (position.latitude + boatPosition.latitude) / 2;
    const east = longitudeDifference * rad * earthRadius * Math.cos(middleLatitude * rad);
    const north = latitudeDifference * rad * earthRadius;
    return [east, north, -depth];
}
function retainedTarget(target) {
    let position = target.last_valid_position;
    let depth = target.last_valid_depth;

    if (target.position && target.position.valid) {
        position = target.position;
    }
    if (target.depth && target.depth.valid) {
        depth = target.depth;
    }

    return {
        position: position,
        depth: depth,
        stale: !fresh(target.position) || !fresh(target.depth)
    };
}

function floorDepth() {
    const input = document.getElementById('floor-depth');
    if (input.value.trim() === '') {
        return NaN;
    }
    const depth = Number(input.value);
    if (!Number.isFinite(depth) || depth < 0) {
        return NaN;
    }
    return depth;
}
let planeSpan=40;
function planeExtent(s) {
    if(fresh(s.Boat.position))for(const name of ['ROV', 'Cage']) {
        const v=s[name];
        if(fresh(v.position)) {
            const point=offset(v.position, s.Boat.position, 0);
            planeSpan=Math.max(planeSpan, Math.ceil((Math.max(Math.abs(point[0]), Math.abs(point[1]))+10)/10)*10);
        }
    }
    return planeSpan;
}
function boatTransform(point, heading, roll, pitch) {
    // Rotate the boat around each of its three axes.
    let x = point[0];
    let y = point[1];
    let z = point[2];
    let rolledX = x * Math.cos(roll) + z * Math.sin(roll);
    let rolledZ = -x * Math.sin(roll) + z * Math.cos(roll);
    x = rolledX;
    z = rolledZ;
    let pitchedY = y * Math.cos(pitch) - z * Math.sin(pitch);
    let pitchedZ = y * Math.sin(pitch) + z * Math.cos(pitch);
    y = pitchedY;
    z = pitchedZ;
    let east = x * Math.cos(heading) + y * Math.sin(heading);
    let north = -x * Math.sin(heading) + y * Math.cos(heading);
    return [east, north, z];
}
function camera() {
    return {
        target:[0, 0, -Math.min(floorDepth()||0, 50)/3], back:[Math.cos(elevation)*Math.sin(azimuth), Math.cos(elevation)*Math.cos(azimuth), Math.sin(elevation)], right:[-Math.cos(azimuth), Math.sin(azimuth), 0], up:[-Math.sin(elevation)*Math.sin(azimuth), -Math.sin(elevation)*Math.cos(azimuth), Math.cos(elevation)]
    };
}
function project(v, c) {
    const rel=sub(v, c.target), depth=distance-dot(rel, c.back);
    if(depth<.5)return null;
    const scale=Math.min(canvas.clientWidth, canvas.clientHeight)*1.2/depth;
    return [canvas.clientWidth/2+dot(rel, c.right)*scale, canvas.clientHeight/2-dot(rel, c.up)*scale, depth];
}
function draw() {
    if(!dirty) {
        requestAnimationFrame(draw);
        return;
    }
    dirty=false;
    const w=canvas.clientWidth, h=canvas.clientHeight, dpr=window.devicePixelRatio||1;
    if(canvas.width!==Math.round(w*dpr)||canvas.height!==Math.round(h*dpr)) {
        canvas.width=Math.round(w*dpr);
        canvas.height=Math.round(h*dpr);
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    const c=camera(), objects=[], depth=floorDepth(), floorValid=Number.isFinite(depth)&&depth>=0;
    function face(vertices, fill, stroke) {
        const pts=vertices.map(v=>project(v, c));
        if(pts.some(p=>!p))return;
        objects.push( {
            z:pts.reduce((s, p)=>s+p[2], 0)/pts.length, draw() {
                ctx.beginPath();pts.forEach((p, i)=>i?ctx.lineTo(p[0], p[1]):ctx.moveTo(p[0], p[1]));ctx.closePath();ctx.fillStyle=fill;ctx.fill();ctx.strokeStyle=stroke;ctx.stroke();
            }
        });
    }
    const span=planeExtent(sources());
    face([[-span, -span, 0], [span, -span, 0], [span, span, 0], [-span, span, 0]], 'rgba(110,180,210,0.08)', 'rgba(80,140,170,.35)');
    if(floorValid)face([[-span, -span, -depth], [span, -span, -depth], [span, span, -depth], [-span, span, -depth]], 'rgba(160,145,110,.18)', '#aaa');
    // Arrow stays in the water plane and points along world north (+y).
    const ax=c.back[0]>0?-span+8:span-8, ay=c.back[1]>0?-span+6:span-16;
    const arrow=[[ax, ay, .05], [ax, ay+8, .05], [ax-2, ay+5, .05], [ax, ay+8, .05], [ax+2, ay+5, .05]].map(v=>project(v, c));
    if(arrow.every(Boolean))objects.push( {
        z:arrow[0][2], draw() {
            ctx.beginPath();ctx.moveTo(arrow[0][0], arrow[0][1]);ctx.lineTo(arrow[1][0], arrow[1][1]);ctx.lineTo(arrow[2][0], arrow[2][1]);ctx.moveTo(arrow[3][0], arrow[3][1]);ctx.lineTo(arrow[4][0], arrow[4][1]);ctx.strokeStyle='#333';ctx.stroke();ctx.fillStyle='#333';ctx.font='14px Arial';ctx.fillText('N', arrow[1][0]+5, arrow[1][1]-4);
        }
    });
    const s=sources(), b=s.Boat, a=b.attitude, heading=b.heading, hd=fresh(heading)?heading.degrees*rad:0, r=fresh(a)?a.roll*rad:0, p=fresh(a)?a.pitch*rad:0;
    // Simplified trawler: 80 ft hull, forward wheelhouse, open aft working deck.
    function transform(point) {
        return boatTransform(point, hd, r, p);
    }
    function boatFace(vertices, color) {
        const pts=vertices.map(v=>project(transform(v), c));
        if(pts.some(p=>!p))return;
        objects.push( {
            wire:true, z:0, draw() {
                ctx.beginPath();pts.forEach((p, i)=>i?ctx.lineTo(p[0], p[1]):ctx.moveTo(p[0], p[1]));ctx.closePath();ctx.strokeStyle='rgba(60,60,60,0.55)';ctx.lineWidth=1;ctx.stroke();
            }
        });
    }
    function box(x0, x1, y0, y1, z0, z1, color) {
        const lo=[[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0]], hi=lo.map(([x, y])=>[x, y, z1]);
        boatFace(hi, color);
        for(let i=0;i<4;i++)boatFace([lo[i], lo[(i+1)%4], hi[(i+1)%4], hi[i]], color);
    }
    const outline=[[-2.8, -12.192], [2.8, -12.192], [3.35, -8], [3.35, 4], [2.7, 8], [0, 12.192], [-2.7, 8], [-3.35, 4], [-3.35, -8]];
    const lower=outline.map(([x, y])=>[x*.72, y*.94, -2]), upper=outline.map(([x, y])=>[x, y, 1.1]);
    boatFace(lower, '#777');
    boatFace(upper, '#e8e4dc');
    for(let i=0;i<outline.length;i++) {
        const j=(i+1)%outline.length;
        boatFace([lower[i], lower[j], upper[j], upper[i]], '#999');
    }
    // The cabin is 11.22 metres long. It has no window.
    box(-2.1, 2.1, -5.92, 5.3, 1.1, 3.7, '#e5e5e5');
    function label(v, text, color, point=false) {
        const pt=project(v, c);
        if(!pt)return;
        objects.push( {
            z:pt[2], draw() {
                ctx.fillStyle=color;if(point) {
                    ctx.beginPath();ctx.arc(pt[0], pt[1], 5, 0, Math.PI*2);ctx.fill();
                }
                ctx.font='14px Arial';ctx.fillText(text, pt[0]+8, pt[1]-8);
            }
        });
    }
    label(boatTransform([0, 0, 2], hd, r, p), 'Boat', '#111');
    for(const[name, color]of[['ROV', '#c22'], ['Cage', '#2255b0']]) {
        const v=s[name], held=retainedTarget(v), bp=b.position?.valid?b.position:b.last_valid_position;
        if(name==='ROV'&&bp&&held.position&&held.depth)label(offset(held.position, bp, held.depth.meters), name, color, true);
        else if(fresh(b.position)&&fresh(v.position)&&fresh(v.depth))label(offset(v.position, b.position, v.depth.meters), name, color, true);
    }
    objects.sort((a, b)=>Number(!!a.wire)-Number(!!b.wire)||b.z-a.z).forEach(o=>o.draw());
    requestAnimationFrame(draw);
}
function cell(value, decimalPlaces) {
    if (value === null || value === undefined) {
        return '—';
    }
    return Number(value).toFixed(decimalPlaces);
}
const dataRows= {
};
function readings() {
    const body=document.getElementById('data');
    for(const[name, v]of Object.entries(sources())) {
        const held=retainedTarget(v), p=name==='ROV'?held.position:(fresh(v.position)?v.position:null), d=name==='ROV'?held.depth:(fresh(v.depth)?v.depth:null), a=fresh(v.attitude)?v.attitude:null, h=fresh(v.heading)?v.heading:null;
        let row=dataRows[name];
        if(!row) {
            row=document.createElement('tr');
            for(let i=0;i<7;i++)row.append(document.createElement('td'));
            body.append(row);
            dataRows[name]=row;
        }
        const stale=name==='ROV'&&held.stale;
        row.className=stale?'stale':'';
        row.title=stale?'Last known data — ROV stream unavailable, invalid, or stale':'';
        [name, cell(p?.latitude, 7), cell(p?.longitude, 7), cell(d?.meters, 2), cell(h?.degrees, 1), cell(a?.roll, 2), cell(a?.pitch, 2)].forEach((value, i)=> {
            if(row.children[i].textContent!==value)row.children[i].textContent=value;
        });
    }
}
async function poll() {
    try {
        const response = await fetch('/api/state');
        if (!response.ok) {
            throw new Error('Receiver did not respond');
        }

        state = await response.json();
        disconnected = false;

        let statusText = '';
        if (state.demo) {
            statusText = 'DEMO';
        }
        document.getElementById('status').textContent = statusText;
    } catch (error) {
        disconnected = true;
        document.getElementById('status').textContent = 'Disconnected';
    }

    const velocity = state.cage_velocity;
    let velocityText = '— m/s';
    if (fresh(velocity)) {
        // The receiver uses positive down. The display uses negative down.
        velocityText = (-velocity.meters_per_second).toFixed(2) + ' m/s';
    }
    document.getElementById('cage-velocity').textContent = velocityText;

    readings();
    dirty = true;
    setTimeout(poll, 200);
}

canvas.addEventListener('pointerdown', e=> {
    dragging=[e.clientX, e.clientY];canvas.setPointerCapture(e.pointerId);
});
canvas.addEventListener('pointermove', e=> {
    if(!dragging)return;dirty=true;azimuth-=(e.clientX-dragging[0])*.008;elevation=Math.max(-1.4, Math.min(1.4, elevation+(e.clientY-dragging[1])*.008));dragging=[e.clientX, e.clientY];
});
canvas.addEventListener('pointerup', ()=>dragging=null);
canvas.addEventListener('pointercancel', ()=>dragging=null);
canvas.addEventListener('wheel', e=> {
    e.preventDefault();dirty=true;distance=Math.max(15, Math.min(5000, distance*Math.exp(e.deltaY*.001)));
}, {
    passive:false
});
canvas.addEventListener('keydown', e=> {
    if(!['ArrowLeft', 'ArrowRight', 'ArrowUp', 'ArrowDown', '+', '-'].includes(e.key))return;e.preventDefault();dirty=true;if(e.key==='ArrowLeft')azimuth-=.1;if(e.key==='ArrowRight')azimuth+=.1;if(e.key==='ArrowUp')elevation=Math.min(1.4, elevation+.1);if(e.key==='ArrowDown')elevation=Math.max(-1.4, elevation-.1);if(e.key==='+')distance=Math.max(15, distance*.9);if(e.key==='-')distance=Math.min(5000, distance*1.1);
});
poll();
draw();

// Redraw the floor when the operator changes its depth.
document.getElementById('floor-depth').addEventListener('input', function () {
    dirty = true;
});


