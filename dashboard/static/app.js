"use strict";
/* Arm Hub front end. Plain JavaScript, no build step.
   To add a page: write a mount(root) function that returns { update(), packet(p), board(b), log(l), frame() }
   (all optional) and add one entry to PAGES. The hub streams everything over one EventSource. */

/* ===================================================================== helpers */
const $ = (s, r = document) => r.querySelector(s);
const esc = (s) => String(s).replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
const icon = (n, c = "") => `<svg class="ic ${c}"><use href="#i-${n}"/></svg>`;
const r3 = (v) => Math.round(v * 1000) / 1000;
const f1 = (v, d = 1) => (v == null || Number.isNaN(v) ? "-" : Number(v).toFixed(d));
const timeStr = (t) => {
  const d = new Date(t * 1000);
  return d.toTimeString().slice(0, 8) + "." + String(d.getMilliseconds()).padStart(3, "0");
};
const ageStr = (s) => (s == null ? "no data" : s < 10 ? s.toFixed(1) + " s" : s < 90 ? Math.round(s) + " s" : s < 5400 ? Math.round(s / 60) + " min" : Math.round(s / 3600) + " h");
const BUTTONS = ["a", "b", "x", "y", "lb", "rb", "back", "start", "ls", "rs"];
const neutral = () => ({ lx: 0, ly: 0, rx: 0, ry: 0, lt: 0, rt: 0, buttons: Object.fromEntries(BUTTONS.map((k) => [k, false])), dpad: [0, 0] });

async function api(path, body) {
  try {
    const r = await fetch(path, { method: "POST", headers: { "Content-Type": "application/json", "X-Hub": "1" }, body: JSON.stringify(body || {}) });
    const j = await r.json().catch(() => ({}));
    return { ...j, ok: r.ok && j.ok !== false };
  } catch (e) {
    return { ok: false, error: "The hub is not reachable" };
  }
}
function toast(msg, kind = "") {
  const t = document.createElement("div");
  t.className = "toast " + kind;
  t.textContent = msg;
  $("#toasts").appendChild(t);
  setTimeout(() => t.remove(), kind === "bad" ? 7000 : 4000);
}

/* ===================================================================== state */
const S = { st: null, packets: [], board: [], logs: {}, ok: false, lastState: null, ppsHist: [], pose: null };
const cfg = { rate: 30 };   // damper: fastest joint speed in deg/s, mirrored from the hub's virtual arm
const LOG_SRC = [["hub", "Hub"], ["server_docker", "Server (Docker)"], ["server_python", "Server (Python)"], ["gui", "Controller GUI"], ["docker_engine", "Docker Desktop"], ["firmware", "Firmware"]];

/* ===================================================================== engine
   One loop for the whole app: reads the chosen controller source, draws it, and sends input while ARMED.
   The arm itself is the virtual arm in the hub (sim/armsim.py); this page only draws the pose it streams. */
const E = {
  source: "server", virt: neutral(), active: neutral(), idle: false,
  last: performance.now(), lastSend: 0, lastJson: "", inflight: false, sending: false, wasSending: false, gpName: null,
  userRateT: -1e9, lastLive: 0, sticky: {},
};
function readGamepad() {
  const pads = (navigator.getGamepads && navigator.getGamepads()) || [];
  let p = null;
  for (const g of pads) if (g && g.connected) { p = g; break; }
  E.gpName = p ? p.id : null;
  E.gpStd = !!(p && p.mapping === "standard");   // Xbox and PS5 pads report the standard layout in Chrome and Edge
  if (!p) return neutral();
  const dz = (v) => (Math.abs(v) < 0.12 ? 0 : (Math.sign(v) * (Math.abs(v) - 0.12)) / 0.88);
  const b = (i) => !!(p.buttons[i] && p.buttons[i].pressed);
  const tv = (i) => (p.buttons[i] ? p.buttons[i].value : 0);
  return {
    lx: r3(dz(p.axes[0] || 0)), ly: r3(dz(p.axes[1] || 0)), rx: r3(dz(p.axes[2] || 0)), ry: r3(dz(p.axes[3] || 0)), lt: r3(tv(6)), rt: r3(tv(7)),
    buttons: { a: b(0), b: b(1), x: b(2), y: b(3), lb: b(4), rb: b(5), back: b(8), start: b(9), ls: b(10), rs: b(11) },
    dpad: [(b(15) ? 1 : 0) - (b(14) ? 1 : 0), (b(12) ? 1 : 0) - (b(13) ? 1 : 0)],
  };
}
function readSource(ts) {
  E.idle = false;
  if (E.source === "gamepad") return readGamepad();
  if (E.source === "virtual") return E.virt;
  if (E.source === "demo") {
    const t = ts / 1000, s = neutral();
    s.lx = r3(Math.sin(t * 0.8)); s.ly = r3(Math.cos(t * 0.6)); s.rx = r3(Math.sin(t * 0.5 + 1)); s.ry = r3(Math.cos(t * 0.9));
    return s;
  }
  const age = S.st ? S.st.last_packet_age : null;      // "server": whatever the local server is receiving
  if (S.lastState && age != null && age < 1.5) return S.lastState;
  E.idle = true;
  return neutral();
}
function engine(ts) {
  const dt = Math.min(0.1, (ts - E.last) / 1000);
  E.last = ts;
  const st = (E.active = readSource(ts));
  maybeSend(ts, st);
  if (cur && cur.frame) cur.frame(st);
  requestAnimationFrame(engine);
}
function maybeSend(ts, st) {
  const should = E.source !== "server" && S.st && (S.st.armed || S.st.emu_input) && S.ok;
  for (const b of BUTTONS) if (st.buttons[b]) E.sticky[b] = true;      // a tap shorter than one request still gets sent
  if (should) {
    const buttons = Object.assign({}, st.buttons, E.sticky);
    const body = JSON.stringify({ lx: st.lx, ly: st.ly, rx: st.rx, ry: st.ry, lt: st.lt, rt: st.rt, buttons, dpad: st.dpad });
    if (!E.inflight && (body !== E.lastJson || ts - E.lastSend > 500)) {
      E.inflight = true; E.lastJson = body; E.lastSend = ts; E.sticky = {};
      fetch("/api/input", { method: "POST", headers: { "Content-Type": "application/json", "X-Hub": "1" }, body })
        .then((r) => r.json()).then((j) => { E.sending = !!j.sent; }).catch(() => { E.sending = false; })
        .finally(() => { E.inflight = false; });
    }
    E.wasSending = true;
  } else {
    E.sticky = {};
    if (E.wasSending && S.ok) api("/api/input", neutral());   // leaving: tell the arm to stop
    E.wasSending = false; E.sending = false; E.lastJson = "";
  }
}
function setSource(s) {
  E.source = s;
  E.virt = neutral();
  if (cur && cur.sourceChanged) cur.sourceChanged();
}

/* ===================================================================== damper sync */
function syncCfg() {
  const sim = S.st && S.st.sim;
  if (sim && performance.now() - E.userRateT > 2500) cfg.rate = sim.rate;
}

/* ===================================================================== pages: overview */
const NODE_W = 160, NODE_H = 104, NODE_Y = 40;
const MAP_NODES = [
  { id: "input", x: 0, icon: "game-controller", name: "Input" },
  { id: "local", x: 210, icon: "cube", name: "Local server" },
  { id: "end", x: 420, icon: "cloud", name: "End server" },
  { id: "board", x: 630, icon: "cpu", name: "ESP32 board" },
  { id: "arm", x: 840, icon: "robot", name: "Arm and servos" },
];
function buildMap(svg) {
  let s = "";
  const my = NODE_Y + NODE_H / 2;
  for (let i = 0; i < 4; i++) {
    const x1 = MAP_NODES[i].x + NODE_W, x2 = MAP_NODES[i + 1].x;
    s += `<line class="link" id="lk${i}" x1="${x1}" y1="${my}" x2="${x2 - 8}" y2="${my}"/><polygon class="tip" id="tp${i}" points="${x2},${my} ${x2 - 9},${my - 5} ${x2 - 9},${my + 5}"/>`;
  }
  const ux1 = MAP_NODES[1].x + NODE_W / 2, ux2 = MAP_NODES[3].x + NODE_W / 2, uy = NODE_Y + NODE_H;
  s += `<path class="link" id="lkU" d="M${ux1} ${uy} V${uy + 62} H${ux2} V${uy + 9}"/><polygon class="tip" id="tpU" points="${ux2},${uy} ${ux2 - 5},${uy + 9} ${ux2 + 5},${uy + 9}"/>`;
  s += `<text class="lbl" x="${(ux1 + ux2) / 2}" y="${uy + 82}" text-anchor="middle">USB cable, direct to the board</text>`;
  s += `<text class="lbl" x="${(MAP_NODES[1].x + NODE_W + MAP_NODES[2].x) / 2}" y="${my - 12}" text-anchor="middle">sends</text>`;
  s += `<text class="lbl" x="${(MAP_NODES[2].x + NODE_W + MAP_NODES[3].x) / 2}" y="${my - 12}" text-anchor="middle">WiFi</text>`;
  s += `<text class="lbl" x="${(MAP_NODES[3].x + NODE_W + MAP_NODES[4].x) / 2}" y="${my - 12}" text-anchor="middle">PWM</text>`;
  for (const n of MAP_NODES) {
    s += `<g class="node idle" id="nd-${n.id}"><rect x="${n.x}" y="${NODE_Y}" width="${NODE_W}" height="${NODE_H}"/>
      <use class="ico" href="#i-${n.icon}" x="${n.x + 12}" y="${NODE_Y + 12}" width="22" height="22"/>
      <text class="nm" x="${n.x + 42}" y="${NODE_Y + 29}">${n.name}</text>
      <text class="ds" id="ds-${n.id}" x="${n.x + 14}" y="${NODE_Y + 60}"></text>
      <text class="stt" id="stt-${n.id}" x="${n.x + 14}" y="${NODE_Y + 86}"></text></g>`;
  }
  svg.innerHTML = s;
}
function setNode(svg, id, level, desc, status) {
  const g = svg.querySelector("#nd-" + id);
  g.setAttribute("class", "node " + level);
  svg.querySelector("#ds-" + id).textContent = desc;
  svg.querySelector("#stt-" + id).textContent = status;
}
function setLink(svg, id, on) {
  svg.querySelector("#lk" + id).classList.toggle("on", !!on);
  svg.querySelector("#tp" + id).classList.toggle("on", !!on);
}
function attention(st) {
  const items = [];
  const sv = st.services, usb = st.usb;
  if (sv.docker_engine.state === "stopped") items.push({ t: "Docker Desktop is not running, so the Docker server cannot start.", a: { label: "Start Docker", act: "svc", name: "docker_engine", action: "start" } });
  if (!st.server.health) items.push({ t: "The local server is not running. Nothing can be received until it is.", a: sv.docker_engine.state === "running" ? { label: "Start server", act: "svc", name: "server_docker", action: "start" } : { label: "Open Services", act: "go", page: "services" } });
  if (st.forward_control === false) items.push({ t: "The server image cannot pause forwarding yet, so SAFE cannot stop the network path. Rebuild it.", a: { label: "Rebuild server", act: "svc", name: "server_docker", action: "restart" } });
  if (!st.config.forward_url_set) items.push({ t: "No FORWARD_URL in .env: the local server will not forward to an end server.", a: { label: "Open Project", act: "go", page: "project" } });
  if (usb.phase === "busy") items.push({ t: "The board's COM port is held by another program (the GUI or the Arduino Serial Monitor).", a: { label: "Open Services", act: "go", page: "services" } });
  if (usb.phase === "waiting" && usb.enabled) items.push({ t: "No ESP32 found on USB. Plug it in with a data cable, or work on the preview.", a: null });
  if (!st.firmware.secrets_present) items.push({ t: "secrets.h is missing, so the firmware cannot be built.", a: { label: "Open Firmware", act: "go", page: "firmware" } });
  return items;
}
function mountOverview(root) {
  root.innerHTML = `
  <div class="strip">
    <div><div class="k">Packets per second</div><div class="v" id="o-pps">-</div><div class="sub" id="o-pps-s"></div></div>
    <div><div class="k">Last packet</div><div class="v" id="o-age">-</div><div class="sub" id="o-age-s"></div></div>
    <div><div class="k">End server</div><div class="v s" id="o-fwd">-</div><div class="sub" id="o-fwd-s"></div></div>
    <div><div class="k">USB link</div><div class="v s" id="o-usb">-</div><div class="sub" id="o-usb-s"></div></div>
    <div><div class="k">Joint speed limit</div><div class="v" id="o-rate">-</div><div class="sub" id="o-rate-s"></div></div>
  </div>
  <section class="sec"><h2>System map</h2>
    <p class="lead">How input travels today. Lines move only while packets are flowing, and only the paths that are allowed through are lit.</p>
    <div class="mapwrap"><svg id="map" class="map" viewBox="0 0 1000 250" role="img" aria-label="System map from input to arm"></svg></div>
  </section>
  <div class="cols">
    <section class="sec"><h2>Needs attention</h2><div class="todo" id="o-todo"><div class="skel"></div></div></section>
    <section class="sec"><h2>Launch</h2><div class="rows" id="o-launch">
      <div style="grid-template-columns:1fr auto"><div><div>Local server</div><div class="note">Starts the Docker container that receives input.</div></div><button class="btn" data-act="svc" data-name="server_docker" data-action="start">${icon("play", "sm")}Start</button></div>
      <div style="grid-template-columns:1fr auto"><div><div>Controller window</div><div class="note">The pygame window for a physical controller.</div></div><button class="btn" data-act="svc" data-name="gui" data-action="start">${icon("game-controller", "sm")}Open</button></div>
      <div style="grid-template-columns:1fr auto"><div><div>Drive from the browser</div><div class="note">Virtual sticks, browser gamepad and preview.</div></div><button class="btn" data-act="go" data-page="control">${icon("sliders-horizontal", "sm")}Open</button></div>
      <div style="grid-template-columns:1fr auto"><div><div>Build the firmware</div><div class="note">Compile or upload the ESP32 sketch.</div></div><button class="btn" data-act="go" data-page="firmware">${icon("hammer", "sm")}Open</button></div>
    </div></section>
  </div>`;
  const svg = $("#map", root);
  buildMap(svg);
  return {
    update() {
      const st = S.st; if (!st) return;
      const sv = st.server, fw = sv.forward, usb = st.usb;
      $("#o-pps", root).textContent = f1(st.pps);
      $("#o-pps-s", root).textContent = sv.health ? "from the local server" : "server is down";
      $("#o-age", root).textContent = st.last_packet_age == null ? "none yet" : ageStr(st.last_packet_age);
      $("#o-age-s", root).textContent = st.last_packet_age != null && st.last_packet_age < 1.5 ? "live" : "idle";
      const fwdText = !sv.health ? "unknown" : !fw.configured ? "not set" : sv.forward_enabled === false ? "paused" : fw.ok === true ? "forwarding" : fw.ok === false ? "failing" : "waiting";
      $("#o-fwd", root).textContent = fwdText;
      $("#o-fwd-s", root).textContent = fw.sent != null ? `${fw.sent} sent, ${fw.failed || 0} failed` : (sv.forward_enabled === false ? "SAFE pauses it" : "");
      $("#o-usb", root).textContent = usb.port || "no board";
      $("#o-usb-s", root).textContent = { open: "open", busy: "port busy", waiting: "waiting", off: "link off", suspended: "flashing" }[usb.phase] || usb.phase;
      const sim = st.sim;
      $("#o-rate", root).innerHTML = sim ? `${sim.rate > 0 ? sim.rate : 180}<span class="u">deg/s</span>` : "-";
      $("#o-rate-s", root).textContent = !sim ? "" : (sim.rate > 0 ? "damper on the virtual arm" : "no limit (capped at 180)");
      // map
      const live = st.pps > 0, armed = st.armed;
      const lvl = (c) => c;
      setNode(svg, "input", live ? "ok" : "idle", "controller or browser", live ? `${f1(st.pps)} pkt/s` : "idle");
      setNode(svg, "local", sv.health ? "ok" : "bad", { docker: "Docker", python: "Python", external: "external", none: "not running" }[sv.mode] || "", sv.health ? "up" : "down");
      const endLevel = !sv.health || !fw.configured ? "idle" : sv.forward_enabled === false ? "idle" : fw.ok === false ? "bad" : fw.ok ? "ok" : "warn";
      setNode(svg, "end", lvl(endLevel), "forwarding target", !fw.configured ? "not set" : sv.forward_enabled === false ? "paused (SAFE)" : fw.ok === false ? "failing" : fw.ok ? "forwarding" : "waiting");
      const bl = usb.phase === "open" ? "ok" : usb.phase === "busy" || usb.phase === "suspended" ? "warn" : "idle";
      setNode(svg, "board", bl, st.board.source ? `input via ${st.board.source === "usb" ? "USB" : "WiFi"}` : (usb.port ? "on " + usb.port : "USB or WiFi"), { open: "USB open", busy: "port busy", waiting: "not on USB", off: "USB link off", suspended: "flashing" }[usb.phase]);
      setNode(svg, "arm", "idle", "no live feedback", st.board.pos ? "last pos reported" : "no feedback");
      const fwdOn = armed && live && sv.forward_enabled !== false && fw.ok !== false && fw.configured;
      setLink(svg, 0, live); setLink(svg, 1, fwdOn); setLink(svg, 2, fwdOn); setLink(svg, 3, armed && live);
      const u = svg.querySelector("#lkU"), t = svg.querySelector("#tpU");
      u.classList.toggle("on", armed && live && usb.phase === "open"); t.classList.toggle("on", armed && live && usb.phase === "open");
      const lb = (name) => root.querySelector(`#o-launch [data-name="${name}"]`);
      for (const n of ["server_docker", "gui"]) { const b = lb(n), sv2 = st.services[n].state; if (b) { b.disabled = sv2 === "running" || sv2 === "busy"; b.lastChild.textContent = sv2 === "running" ? "Running" : (n === "gui" ? "Open" : "Start"); } }
      // attention list
      const items = attention(st);
      $("#o-todo", root).innerHTML = (items.length ? items : [{ good: 1, t: "Nothing needs attention.", a: null }]).concat(
        [{ good: 1, t: armed ? "Output is ARMED: input can reach the arm. Press Escape to stop." : "Output is SAFE: nothing is sent to the arm.", a: null, info: 1 }]).map((i) =>
        `<div class="${i.good ? "good" : ""}">${icon(i.good ? "check-circle" : "warning")}<span>${esc(i.t)}</span>${i.a ? (i.a.act === "go" ? `<button class="btn sm" data-act="go" data-page="${i.a.page}">${esc(i.a.label)}</button>` : `<button class="btn sm" data-act="svc" data-name="${i.a.name}" data-action="${i.a.action}">${esc(i.a.label)}</button>`) : "<span></span>"}</div>`).join("");
    },
  };
}

/* ===================================================================== pages: control */
const SOURCES = [
  ["server", "Server stream", "Shows what the local server is receiving, from the pygame window or any other client. Nothing is sent from this page."],
  ["gamepad", "Browser gamepad", "Reads a controller plugged into this computer through the browser. Press any button once so the browser reports it. Sends while ARMED."],
  ["virtual", "Virtual sticks", "Drag the sticks and press the buttons with the mouse or touch. Sticks spring back to centre like real ones. Sends while ARMED."],
  ["demo", "Demo wave", "Slow automatic stick movement for checking the preview and the data path. Sends while ARMED, so only use it when the arm is clear."],
];
function padSVG() {
  const cross = (cx, cy) => `<g id="p-dpad"><rect class="part" id="p-du" x="${cx - 12}" y="${cy - 36}" width="24" height="28"/><rect class="part" id="p-dd" x="${cx - 12}" y="${cy + 8}" width="24" height="28"/>
    <rect class="part" id="p-dl" x="${cx - 36}" y="${cy - 12}" width="28" height="24"/><rect class="part" id="p-dr" x="${cx + 8}" y="${cy - 12}" width="28" height="24"/><rect class="part" x="${cx - 8}" y="${cy - 8}" width="16" height="16" style="stroke:none"/></g>`;
  const face = (id, x, y, l) => `<g class="btnp" data-b="${id}"><circle class="part" id="p-${id}" cx="${x}" cy="${y}" r="17"/><text class="lab" x="${x}" y="${y}">${l}</text></g>`;
  return `<svg class="pad" id="pad" viewBox="0 0 640 400" role="img" aria-label="Controller">
    <g class="btnp" data-t="lt"><rect class="well" x="92" y="6" width="84" height="26"/><rect class="fill" id="p-ltf" x="92" y="6" width="0" height="26"/><text class="lab" x="134" y="19">LT</text></g>
    <g class="btnp" data-t="rt"><rect class="well" x="464" y="6" width="84" height="26"/><rect class="fill" id="p-rtf" x="464" y="6" width="0" height="26"/><text class="lab" x="506" y="19">RT</text></g>
    <g class="btnp" data-b="lb"><rect class="part" id="p-lb" x="70" y="42" width="120" height="22"/><text class="lab" x="130" y="53">LB</text></g>
    <g class="btnp" data-b="rb"><rect class="part" id="p-rb" x="450" y="42" width="120" height="22"/><text class="lab" x="510" y="53">RB</text></g>
    <path class="body" d="M120 84 H520 C600 84 622 170 604 270 C590 350 534 382 484 342 L444 312 H196 L156 342 C106 382 50 350 36 270 C18 170 40 84 120 84 Z"/>
    <g class="stick" data-s="l" transform="translate(170 170)"><circle class="well" r="50"/><circle class="knob" id="p-lk" r="22"/></g>
    <g class="stick" data-s="r" transform="translate(400 282)"><circle class="well" r="50"/><circle class="knob" id="p-rk" r="22"/></g>
    ${cross(265, 282)}
    ${face("y", 480, 134, "Y")}${face("a", 480, 206, "A")}${face("x", 444, 170, "X")}${face("b", 516, 170, "B")}
    <g class="btnp" data-b="back"><circle class="part" id="p-back" cx="292" cy="170" r="10"/></g><text class="lab" x="292" y="192">Back</text>
    <g class="btnp" data-b="start"><circle class="part" id="p-start" cx="348" cy="170" r="10"/></g><text class="lab" x="348" y="192">Start</text>
  </svg>`;
}
const OUTS = [["OUT1", 25, "LX", "Left stick, left and right"], ["OUT2", 26, "LY", "Left stick, up and down"], ["OUT3", 27, "RX", "Right stick, left and right"], ["OUT4", 32, "RY", "Right stick, up and down"]];   // the USB jog sketch (Arm and board page)
/* The virtual arm: two views drawn from the pose the hub streams (the sim in sim/armsim.py runs in the hub, not here). */
function armViewsHTML() {
  const dots = (p, r) => [0, 1, 2, 3, 4].map((i) => `<circle class="jnt${i === 4 ? " tip" : ""}" id="av-${p}${i}" r="${i === 4 ? r + 2 : r}" cx="0" cy="0"/>`).join("");
  return `<div class="armviews">
    <figure><svg viewBox="-110 -470 620 520" role="img" aria-label="Arm, side view"><g transform="scale(1,-1)">
      <line class="ground" x1="-110" y1="0" x2="510" y2="0"/><polyline class="link" id="av-sl" points=""/>${dots("s", 9)}</g></svg><figcaption>Side view</figcaption></figure>
    <figure><svg viewBox="-420 -420 840 840" role="img" aria-label="Arm, top view"><g transform="scale(1,-1)">
      <circle class="reach" id="av-reach" r="392" cx="0" cy="0"/><line class="ground" x1="0" y1="-420" x2="0" y2="420"/><polyline class="link" id="av-tl" points=""/>${dots("t", 11)}</g></svg><figcaption>Top view (arm forward is up)</figcaption></figure>
  </div>`;
}
const MODE_TEXT = { idle: "holding", manual: "jogging", home: "going home", replay: "replaying" };
const JOINT_NAMES = ["Base", "Shoulder", "Elbow", "Wrist"];
function pushRate(v, force) {
  const now = performance.now();
  if (!force && now - E.lastLive < 100) return;
  E.lastLive = now;
  api("/api/sim", { rate: v });
}
function mountControl(root) {
  root.innerHTML = `
  <div class="cols">
    <div>
      <div class="tools"><div class="tabs" role="tablist" id="c-tabs">${SOURCES.map((s) => `<button type="button" role="tab" data-src="${s[0]}" aria-selected="${E.source === s[0]}">${s[1]}</button>`).join("")}</div>
        <span class="st idle" id="c-send" role="status"></span></div>
      <p class="note" id="c-desc" style="margin-bottom:12px"></p>
      <div id="c-pad">${padSVG()}</div>
      <p class="note" id="c-info" style="margin-top:8px"></p>
    </div>
    <div>
      <section class="sec" style="border-top:0;padding-top:0"><h2>Virtual arm</h2>
        <p class="lead">The UWE-RArm movement logic, run on this computer: sticks move the tool through inverse kinematics, A saves a position, holding A replays them, B goes home. It always runs, including while SAFE, and needs no hardware.</p>
        ${armViewsHTML()}
        <div class="strip" style="margin-top:12px">
          <div><div class="k">Tool position</div><div class="v s" id="a-xyz">-</div><div class="sub" id="a-ang"></div></div>
          <div><div class="k">State</div><div class="v s" id="a-mode">-</div><div class="sub" id="a-status"></div></div>
          <div><div class="k">Reach</div><div class="v s" id="a-ik">-</div><div class="sub" id="a-ik-s"></div></div>
          <div><div class="k">Saved positions</div><div class="v" id="a-saved">0</div><div class="sub" id="a-replay"></div></div>
        </div>
        <table class="tbl" style="margin-top:12px"><thead><tr><th>Joint</th><th>Angle</th><th>Limits</th><th>Servo</th></tr></thead><tbody>
          ${JOINT_NAMES.map((n, i) => `<tr><td>${i + 1} ${n}</td><td id="j-a${i}">-</td><td id="j-l${i}">-</td><td id="j-s${i}">-</td></tr>`).join("")}</tbody></table>
        <div class="inline" style="margin-top:12px"><button class="btn sm" id="a-home" type="button">${icon("arrows-clockwise", "sm")}Go home</button>
          <button class="btn sm" id="a-clear" type="button">${icon("trash", "sm")}Clear saved positions</button></div>
      </section>
      <section class="sec"><h2>Firmware to simulate</h2>
        <p class="lead">Which control logic the virtual arm runs. Stock is the UWE-RArm firmware exactly as it is, quirks included; the patched version adds only the damper and a few failsafes.</p>
        <div class="field"><select id="c-model" aria-label="Firmware to simulate">
          <option value="stock-patch">Our firmware: stock + patch (default)</option>
          <option value="stock">Stock UWE-RArm firmware, as shipped</option>
          <option value="improved">Reference (this repo's model)</option></select>
          <div class="note" id="c-model-n"></div></div>
      </section>
      <section class="sec" id="emu-sec"><h2>Emulated ESP32 and network</h2>
        <p class="lead">Runs the real firmware code on this computer as an ESP32 that polls the local server over a simulated network. While it runs, the arm drawn above is the emulated servos, and the sticks on this page travel over the controller link (even while output is SAFE: nothing reaches the real arm). The link numbers are assumptions until calibrated against the board.</p>
        <div class="inline"><label class="muted" for="e-fw">Firmware</label><select id="e-fw"><option value="ours">Our firmware (stock + patches, magnet and sequences)</option><option value="patched">Stock + damper patch only</option><option value="stock">Stock UWE-RArm, as shipped</option><option value="rarm">Rewrite (firmware/rarm)</option></select>
          <label class="muted" for="e-preset">Network</label><select id="e-preset"></select>
          <button class="btn sm" id="e-go" type="button">Start emulator</button><span class="st idle" id="e-state">Off</span></div>
        <div class="note" id="e-err" style="color:var(--bad)"></div>
        <div id="e-live" hidden>
          <div class="inline" style="margin-top:14px"><label class="muted" for="e-path">Link</label><select id="e-path"><option value="esp">ESP32 to server</option><option value="ctl">Controller to server</option><option value="both">Both</option></select>
            <select id="e-lp" aria-label="Link preset"></select>
            <label class="muted" for="e-rtt">Round trip</label><input type="number" id="e-rtt" min="0" max="2000" step="5" style="width:72px"><span class="muted">ms</span>
            <label class="muted" for="e-jit">Jitter</label><input type="number" id="e-jit" min="0" max="2000" step="5" style="width:72px"><span class="muted">ms</span>
            <label class="muted" for="e-loss">Loss</label><input type="number" id="e-loss" min="0" max="50" step="0.5" style="width:64px"><span class="muted">%</span>
            <label class="muted"><input type="checkbox" id="e-keep"> reuse connection</label>
            <button class="btn sm" id="e-apply" type="button">Apply</button><button class="btn sm" id="e-drop" type="button">Drop link 3 s</button></div>
          <div class="strip" style="margin-top:14px">
            <div><div class="k">ESP32 polls</div><div class="v" id="e-polls">-</div><div class="sub" id="e-polls-s"></div></div>
            <div><div class="k">Poll time</div><div class="v" id="e-ms">-</div><div class="sub" id="e-ms-s"></div></div>
            <div><div class="k">Longest stall</div><div class="v" id="e-gap">-</div><div class="sub">longest gap between loop passes</div></div>
            <div><div class="k">Links</div><div class="v s" id="e-links">-</div><div class="sub" id="e-links-s"></div></div>
          </div>
        </div>
      </section>
      <section class="sec"><h2>Damper</h2>
        <p class="lead">The fastest any joint may move, in degrees per second. It applies to the sticks, Go home and replay. All four joints are slowed together, so the tool keeps to its path. A joint is never faster than this.</p>
        <div class="field"><div class="inline"><input type="range" id="c-rate" min="0" max="60" step="1" aria-label="Fastest joint speed in degrees per second"><span class="mono" id="c-rate-v" style="min-width:90px;text-align:right"></span></div>
          <div class="note" id="c-rate-n"></div></div>
      </section>
      <section class="sec"><h2>Move speed (sequences)</h2>
        <p class="lead">Sequence poses are reached on a smooth straight line through space, with the speed easing in and out (the linear move from Sam's UWE-RArm repo). This is the peak tool speed along that line, in millimetres per second; 0 goes back to every joint heading straight for the pose. The damper above still caps the joints underneath.</p>
        <div class="field"><div class="inline"><input type="range" id="c-move" min="0" max="300" step="10" aria-label="Peak tool speed of straight-line moves in millimetres per second"><span class="mono" id="c-move-v" style="min-width:90px;text-align:right"></span></div></div>
      </section>
      <section class="sec"><h2>Stick speed</h2>
        <p class="lead">How fast a full stick moves the tool tip, in millimetres per second. The firmware's own value is 20, which is slow, and the damper above cannot speed it up: it only limits joint speed. Raise this for a quicker arm; the joint speed limit still applies on top.</p>
        <div class="field"><div class="inline"><input type="range" id="c-stick" min="10" max="150" step="5" aria-label="Tool tip speed at full stick in millimetres per second"><span class="mono" id="c-stick-v" style="min-width:90px;text-align:right"></span></div></div>
      </section>
      <section class="sec"><h2>Calibration and tool</h2>
        <p class="lead">Joint limits and homes as trained on the real arm: whole degrees 0 to 180, and a min above the max marks an inverted servo. Applying them moves the virtual arm home, clears the saved positions and posts the values to the server's /calibration, which the firmware reads at boot.</p>
        <table class="tbl"><thead><tr><th>Joint</th><th>Min</th><th>Max</th><th>Home</th></tr></thead><tbody>
          ${[0, 1, 2, 3].map((j) => `<tr><td>${j + 1} ${JOINT_NAMES[j]}</td>${["min", "max", "home"].map((k) => `<td><input type="number" class="cal" data-j="${j}" data-k="${k}" min="0" max="180" step="1" style="width:72px" aria-label="Joint ${j + 1} ${k}"></td>`).join("")}</tr>`).join("")}</tbody></table>
        <div class="inline" style="margin-top:12px"><label class="muted" for="c-tool">Tool length</label><input type="number" id="c-tool" min="0" max="300" step="1" style="width:80px"><span class="muted">mm (63 with the test ball, 53 without)</span>
          <button class="btn sm" id="c-cal-go" type="button">Apply</button></div>
      </section>
    </div>
  </div>`;
  const $r = (s) => $(s, root);
  const pad = $r("#pad");
  // source tabs
  $r("#c-tabs").addEventListener("click", (e) => {
    const b = e.target.closest("[data-src]"); if (!b) return;
    setSource(b.dataset.src);
  });
  // virtual interaction
  const setStick = (which, ev, g) => {
    const m = g.getScreenCTM(); if (!m) return;
    const p = new DOMPoint(ev.clientX, ev.clientY).matrixTransform(m.inverse());
    let vx = p.x / 50, vy = p.y / 50; const mag = Math.hypot(vx, vy); if (mag > 1) { vx /= mag; vy /= mag; }
    E.virt[which + "x"] = r3(vx); E.virt[which + "y"] = r3(vy);
  };
  pad.addEventListener("pointerdown", (ev) => {
    if (E.source !== "virtual") return;
    const st = ev.target.closest(".stick"), bt = ev.target.closest(".btnp");
    if (st) { try { st.setPointerCapture(ev.pointerId); } catch (e) { /* synthetic pointer */ } st._drag = true; setStick(st.dataset.s, ev, st); }
    else if (bt) {
      try { bt.setPointerCapture(ev.pointerId); } catch (e) { /* synthetic pointer */ }
      if (bt.dataset.b) E.virt.buttons[bt.dataset.b] = true; else E.virt[bt.dataset.t] = 1;
    }
  });
  pad.addEventListener("pointermove", (ev) => { const st = ev.target.closest(".stick"); if (E.source === "virtual" && st && st._drag) setStick(st.dataset.s, ev, st); });
  const release = (ev) => {
    if (E.source !== "virtual") return;
    const st = ev.target.closest(".stick"), bt = ev.target.closest(".btnp");
    if (st) { st._drag = false; E.virt[st.dataset.s + "x"] = 0; E.virt[st.dataset.s + "y"] = 0; }
    if (bt) { if (bt.dataset.b) E.virt.buttons[bt.dataset.b] = false; else E.virt[bt.dataset.t] = 0; }
  };
  pad.addEventListener("pointerup", release); pad.addEventListener("pointercancel", release);
  // damper
  const slider = $r("#c-rate");
  slider.value = cfg.rate;
  slider.addEventListener("input", () => { cfg.rate = +slider.value; E.userRateT = performance.now(); pushRate(cfg.rate); });
  slider.addEventListener("change", () => { E.userRateT = performance.now(); pushRate(+slider.value, true); });
  // stick speed
  const stickSl = $r("#c-stick");
  let stickT = 0;
  stickSl.addEventListener("input", () => { stickT = performance.now(); $r("#c-stick-v").textContent = stickSl.value + " mm/s"; });
  stickSl.addEventListener("change", async () => {
    stickT = performance.now();
    const r = await api("/api/sim", { stick_speed: +stickSl.value });
    if (!r.ok) toast(r.error || "The hub is not reachable", "bad");
  });
  // move speed
  const moveSl = $r("#c-move");
  let moveT = 0;
  const moveLabel = (v) => (+v === 0 ? "off (joint moves)" : v + " mm/s");
  moveSl.addEventListener("input", () => { moveT = performance.now(); $r("#c-move-v").textContent = moveLabel(moveSl.value); });
  moveSl.addEventListener("change", async () => {
    moveT = performance.now();
    const v = +moveSl.value, r = await api("/api/sim", { move_speed: v === 0 ? 0 : Math.max(10, v) });
    if (!r.ok) toast(r.error || "The hub is not reachable", "bad");
  });
  // firmware model
  const modelSel = $r("#c-model");
  modelSel.addEventListener("change", async () => {
    const v = modelSel.value, r = await api("/api/sim", { model: v === "improved" ? "improved" : "stock", patch: v === "stock-patch" });
    if (!r.ok) toast(r.error || "The hub is not reachable", "bad");
  });
  // emulated ESP32 and network
  const eFw = $r("#e-fw"), ePre = $r("#e-preset"), eLp = $r("#e-lp"), ePath = $r("#e-path");
  const emu = async (body) => { const r = await api("/api/emu", body); if (!r.ok) toast(r.error || "The hub is not reachable", "bad"); return r; };
  $r("#e-go").addEventListener("click", async () => {
    const e = S.st && S.st.emu;
    if (e && (e.state === "running" || e.state === "building")) await emu({ action: "stop" });
    else await emu({ action: "start", firmware: eFw.value, preset: ePre.value });
  });
  $r("#e-apply").addEventListener("click", async () => {
    const preset = eLp.value, body = { action: "link", path: ePath.value };
    if (preset) body.preset = preset;
    else Object.assign(body, { rtt_ms: +$r("#e-rtt").value, jitter_ms: +$r("#e-jit").value, loss: +$r("#e-loss").value / 100, keepalive: $r("#e-keep").checked });
    await emu(body);
  });
  $r("#e-drop").addEventListener("click", () => emu({ action: "outage", path: ePath.value, seconds: 3 }));
  let eShown = "";
  const emuUpdate = (e) => {
    const go = $r("#e-go"), state = $r("#e-state"), busy = e && (e.state === "running" || e.state === "building");
    if (e && !ePre.options.length) {
      const names = Object.keys(e.presets);
      ePre.innerHTML = names.map((n) => `<option>${n}</option>`).join(""); ePre.value = "home_wifi";
      eLp.innerHTML = `<option value="">custom values</option>` + names.map((n) => `<option>${n}</option>`).join("");
    }
    go.textContent = busy ? "Stop emulator" : "Start emulator";
    eFw.disabled = ePre.disabled = !!busy;
    state.className = "st " + (!e || e.state === "off" ? "idle" : e.state === "error" ? "bad" : e.state === "building" ? "warn" : "ok");
    state.textContent = !e ? "Off" : { off: "Off", building: "Compiling the firmware...", running: `Running ${e.firmware} for ${Math.round(e.uptime)} s`, error: "Stopped" }[e.state];
    $r("#e-err").textContent = e ? e.error : "";
    $r("#e-live").hidden = !(e && e.state === "running");
    if (!e || e.state !== "running") { eShown = ""; return; }
    const n = e.net, L = e.links;
    $r("#e-polls").textContent = n ? n.polls : "-";
    $r("#e-polls-s").textContent = n ? `${n.poll_fails} failed, ${n.connects} connections` : "waiting for the first poll";
    $r("#e-ms").textContent = n ? `${f1(n.poll_ms_mean, 0)} ms` : "-";
    $r("#e-ms-s").textContent = n ? `95% under ${f1(n.poll_ms_p95, 0)} ms, worst ${f1(n.poll_ms_max, 0)} ms` : "";
    $r("#e-gap").textContent = n ? `${f1(n.loop_gap_ms_max, 0)} ms` : "-";
    const lk = (k) => `${L[k].preset}${L[k].outage_s > 0 ? " (down)" : ""}`;
    $r("#e-links").textContent = `ESP32 ${lk("esp")}, controller ${lk("ctl")}`;
    $r("#e-links-s").textContent = `last request: ESP32 ${f1(L.esp.stats.last_delay_ms, 0)} ms, controller ${f1(L.ctl.stats.last_delay_ms, 0)} ms`;
    const L0 = L[ePath.value === "both" ? "esp" : ePath.value], sig = JSON.stringify([ePath.value, L0.profile]);
    if (sig !== eShown && !root.querySelector("#e-live").contains(document.activeElement)) {
      eShown = sig;
      eLp.value = L0.preset in e.presets ? L0.preset : "";
      $r("#e-rtt").value = L0.profile.rtt_ms; $r("#e-jit").value = L0.profile.jitter_ms;
      $r("#e-loss").value = +(L0.profile.loss * 100).toFixed(2); $r("#e-keep").checked = !!L0.profile.keepalive;
    }
  };
  ePath.addEventListener("change", () => { eShown = ""; });
  eLp.addEventListener("change", () => { if (eLp.value && S.st && S.st.emu && S.st.emu.presets[eLp.value]) {
    const p = S.st.emu.presets[eLp.value];
    $r("#e-rtt").value = p.rtt_ms; $r("#e-jit").value = p.jitter_ms; $r("#e-loss").value = +(p.loss * 100).toFixed(2); $r("#e-keep").checked = !!p.keepalive; } });
  for (const id of ["#e-rtt", "#e-jit", "#e-loss", "#e-keep"]) $r(id).addEventListener("input", () => { eLp.value = ""; });
  // actions
  $r("#a-home").addEventListener("click", async () => { const r = await api("/api/sim", { home: true }); if (!r.ok) toast(r.error || "The hub is not reachable", "bad"); });
  $r("#a-clear").addEventListener("click", async () => { const r = await api("/api/sim", { clear_saved: true }); toast(r.ok ? "Saved positions cleared" : (r.error || "The hub is not reachable"), r.ok ? "ok" : "bad"); });
  // calibration and tool length
  let dirty = false;
  const cal = root.querySelectorAll("input.cal"), tool = $r("#c-tool");
  const fill = (s) => {
    cal.forEach((i) => { const j = +i.dataset.j; i.value = i.dataset.k === "home" ? s.homes[j] : s.limits[j][i.dataset.k === "min" ? 0 : 1]; });
    tool.value = s.tool;
  };
  root.addEventListener("input", (e) => { if (e.target.matches("input.cal, #c-tool")) dirty = true; });
  $r("#c-cal-go").addEventListener("click", async () => {
    const s = S.st.sim, lim = [[0, 0], [0, 0], [0, 0], [0, 0]], homes = [0, 0, 0, 0];
    cal.forEach((i) => { const j = +i.dataset.j, v = Math.round(+i.value); if (i.dataset.k === "home") homes[j] = v; else lim[j][i.dataset.k === "min" ? 0 : 1] = v; });
    const body = { tool: +tool.value };
    const changed = JSON.stringify(lim) !== JSON.stringify(s.limits) || JSON.stringify(homes) !== JSON.stringify(s.homes);
    if (changed) body.calibration = { jointLimits: lim, jointHomes: homes };
    const r = await api("/api/sim", body);
    if (r.ok) { dirty = false; fill(r.sim); toast(changed ? "Calibration applied: the arm is at its new home" : "Tool length applied", "ok"); }
    else toast(r.error || "The hub is not reachable", "bad");
  });
  const upd = {
    sourceChanged() {
      root.querySelectorAll("#c-tabs button").forEach((b) => b.setAttribute("aria-selected", b.dataset.src === E.source));
      pad.classList.toggle("virtual", E.source === "virtual");
      $r("#c-desc").textContent = SOURCES.find((s) => s[0] === E.source)[2];
    },
    update() {
      const st = S.st; if (!st) return;
      const armed = st.armed;
      const el = $r("#c-send");
      if (E.source === "server") { el.className = "st idle"; el.textContent = "Not sending from this page"; }
      else if (!armed && st.emu_input) { el.className = "st ok"; el.textContent = E.sending ? "Sending to the emulated ESP32 (output is SAFE)" : "Emulator ready, waiting to send"; }
      else if (!armed) { el.className = "st warn"; el.textContent = "Virtual arm only: output is SAFE"; }
      else if (E.sending) { el.className = "st bad"; el.textContent = "Sending to the arm"; }
      else { el.className = "st warn"; el.textContent = "ARMED, waiting to send"; }
      $r("#c-info").textContent = E.source === "gamepad" ? (E.gpName ? `Controller found: ${E.gpName}${E.gpStd ? "" : ". Not the standard layout, so buttons may be mapped wrongly: use the pygame window instead."}` : "No controller reported yet. Press a button on it.")
        : E.source === "server" && E.idle ? "No live input at the local server right now." : "";
      const sm = st.sim;
      if (sm && document.activeElement !== modelSel) modelSel.value = sm.model === "stock" ? (sm.patch ? "stock-patch" : "stock") : "improved";
      $r("#c-model-n").textContent = !sm ? "" : sm.model !== "stock"
        ? "Single source of truth for the pose: moving after Go home starts from home."
        : sm.patch ? "Stock logic plus: speed limit on the servo output, input capped at 100 ms per packet, stick input pauses while the arm lags its target by 25 degrees, Go home waits for arrival and re-syncs the target."
        : `As shipped: no speed limit, the XYZ target is separate from the arm (target ${sm.target ? sm.target.join(", ") : "-"}), Go home gives up after 3 s, and once it ends the arm swings back to the pre-home target.`;
      slider.value = cfg.rate;
      if (sm && sm.move_speed != null && performance.now() - moveT > 1500) { moveSl.value = sm.move_speed; $r("#c-move-v").textContent = moveLabel(sm.move_speed); }
      if (sm && sm.stick_speed != null && performance.now() - stickT > 1500) { stickSl.value = sm.stick_speed; $r("#c-stick-v").textContent = sm.stick_speed + " mm/s"; }
      $r("#c-rate-v").textContent = cfg.rate > 0 ? `${cfg.rate} deg/s` : "no limit";
      $r("#c-rate-n").textContent = sm && sm.model === "stock" && !sm.patch ? "The stock firmware has no speed limiter: this setting is ignored in this mode." : cfg.rate > 0 ? `A 90 degree joint move takes at least ${Math.round(90 / cfg.rate)} s.` : "No limit: joints are capped at 180 deg/s.";
      if (!dirty && st.sim) fill(st.sim);
      emuUpdate(st.emu);
    },
    frame(st) {
      const k = (id) => $r("#p-" + id);
      const kn = (id, vx, vy) => { const n = k(id); n.setAttribute("cx", r3(vx * 28)); n.setAttribute("cy", r3(vy * 28)); };
      kn("lk", st.lx, st.ly); kn("rk", st.rx, st.ry);
      for (const b of BUTTONS) { const n = k(b); if (n) n.classList.toggle("on", !!st.buttons[b]); }
      k("ltf").setAttribute("width", r3(84 * st.lt)); k("rtf").setAttribute("width", r3(84 * st.rt));
      k("du").classList.toggle("on", st.dpad[1] === 1); k("dd").classList.toggle("on", st.dpad[1] === -1);
      k("dl").classList.toggle("on", st.dpad[0] === -1); k("dr").classList.toggle("on", st.dpad[0] === 1);
    },
    pose(p) {
      if (!p || !p.points) return;
      const P = p.points, tip = P[4], m = Math.hypot(tip[0], tip[1]), d = m > 1 ? [tip[0] / m, tip[1] / m] : [0, 1];
      const side = P.map((q) => [q[0] * d[0] + q[1] * d[1], q[2]]), pts = (a) => a.map((q) => q[0].toFixed(1) + "," + q[1].toFixed(1)).join(" ");
      $r("#av-sl").setAttribute("points", pts(side));
      $r("#av-tl").setAttribute("points", pts(P.map((q) => [q[0], q[1]])));
      for (let i = 0; i < 5; i++) {
        const a = $r("#av-s" + i), b = $r("#av-t" + i);
        a.setAttribute("cx", side[i][0]); a.setAttribute("cy", side[i][1]); b.setAttribute("cx", P[i][0]); b.setAttribute("cy", P[i][1]);
      }
      $r("#av-reach").setAttribute("r", 145.95 + 183.7 + p.tool);
      $r("#a-xyz").textContent = `${f1(p.xyz[0], 0)}, ${f1(p.xyz[1], 0)}, ${f1(p.xyz[2], 0)}`;
      $r("#a-ang").textContent = `x, y, z mm · tool ${f1(p.tool_angle, 0)}°`;
      $r("#a-mode").textContent = p.attached ? MODE_TEXT[p.mode] || p.mode : "servos released";
      $r("#a-status").textContent = p.status;
      const ik = $r("#a-ik"); ik.textContent = p.ik_ok ? "reachable" : "out of reach"; ik.style.color = p.ik_ok ? "" : "var(--bad)";
      $r("#a-ik-s").textContent = p.ik_ok ? "" : "holding position";
      $r("#a-saved").textContent = p.saved;
      $r("#a-replay").textContent = p.mode === "replay" ? `next: #${p.replay_idx + 1}` : "";
      for (let i = 0; i < 4; i++) {
        const [lo, hi] = p.limits[i];
        $r("#j-a" + i).textContent = f1(p.angles[i]) + "°";
        $r("#j-l" + i).textContent = `${lo}° to ${hi}°${lo > hi ? " (inverted)" : ""}`;
        $r("#j-s" + i).textContent = p.servo[i] + "°";
      }
    },
  };
  upd.sourceChanged();
  upd.pose(S.pose || (S.st && S.st.sim));
  return upd;
}

/* ===================================================================== pages: packets */
function rawHttp(s) {
  const body = JSON.stringify(s);
  return `POST /input HTTP/1.1\nHost: localhost:8000\nContent-Type: application/json\nContent-Length: ${new TextEncoder().encode(body).length}\n\n${body}`;
}
function mountPackets(root) {
  root.innerHTML = `
  <div class="tools"><button class="btn" id="k-pause" type="button">${icon("pause", "sm")}<span>Pause</span></button>
    <button class="btn" id="k-clear" type="button">${icon("trash", "sm")}Clear</button>
    <input type="text" id="k-filter" placeholder="Filter changes, e.g. LX or A down" aria-label="Filter packets" style="width:260px">
    <span style="margin-left:auto" class="inline"><svg class="spark" id="k-spark" viewBox="0 0 160 36" aria-hidden="true"><polyline points=""/></svg>
      <span><span class="mono" id="k-pps" style="font-size:20px;font-weight:600">0.0</span> <span class="muted">packets per second</span></span></span></div>
  <div class="cols">
    <div class="scroll"><table class="tbl"><thead><tr><th>Time</th><th>#</th><th>LX</th><th>LY</th><th>RX</th><th>RY</th><th>Change</th></tr></thead><tbody id="k-body"></tbody></table>
      <div id="k-empty" class="empty" style="margin:14px">No packets yet. Start the local server and move a controller, or send some from the Control page once the output is ARMED.
        <div class="inline"><button class="btn sm" data-act="go" data-page="services">Open Services</button><button class="btn sm" data-act="go" data-page="control">Open Control</button></div></div></div>
    <div><div class="inline" style="margin-bottom:8px"><div class="tabs" id="k-tabs"><button type="button" data-v="http" aria-selected="true">As sent</button><button type="button" data-v="json" aria-selected="false">JSON</button></div>
      <button class="btn sm" id="k-copy" type="button">${icon("copy", "sm")}Copy</button></div>
      <div class="raw" id="k-raw"><span class="muted">Select a packet to see exactly what was sent.</span></div>
      <p class="note" style="margin-top:8px">Packets are listed when the state changes. Unchanged heartbeats are not shown.</p></div>
  </div>`;
  const $r = (s) => $(s, root);
  let paused = false, selected = null, view = "http", missed = 0;
  const body = $r("#k-body");
  const rowHtml = (p) => `<td>${timeStr(p.t)}</td><td>${p.n}</td><td>${f1(p.s.lx, 2)}</td><td>${f1(p.s.ly, 2)}</td><td>${f1(p.s.rx, 2)}</td><td>${f1(p.s.ry, 2)}</td><td class="chg">${esc(p.d.join(", ") || "no change")}</td>`;
  const show = () => {
    const p = S.packets.find((x) => x.n === selected);
    $r("#k-raw").textContent = p ? (view === "http" ? rawHttp(p.w || p.s) : JSON.stringify(p.w || p.s, null, 2)) : "Select a packet to see exactly what was sent.";
    body.querySelectorAll("tr").forEach((r) => r.setAttribute("aria-selected", String(+r.dataset.n === selected)));
  };
  const match = (p) => { const f = $r("#k-filter").value.trim().toLowerCase(); return !f || p.d.join(" ").toLowerCase().includes(f); };
  const add = (p) => {
    if (!match(p)) return;
    const tr = document.createElement("tr"); tr.dataset.n = p.n; tr.innerHTML = rowHtml(p);
    body.prepend(tr);
    while (body.rows.length > 300) body.deleteRow(-1);
    $r("#k-empty").hidden = true;
  };
  const rebuild = () => { body.innerHTML = ""; S.packets.slice(-300).forEach(add); $r("#k-empty").hidden = body.rows.length > 0; show(); };
  body.addEventListener("click", (e) => { const tr = e.target.closest("tr[data-n]"); if (tr) { selected = +tr.dataset.n; show(); } });
  $r("#k-tabs").addEventListener("click", (e) => { const b = e.target.closest("[data-v]"); if (!b) return; view = b.dataset.v; root.querySelectorAll("#k-tabs button").forEach((x) => x.setAttribute("aria-selected", x === b)); show(); });
  $r("#k-pause").addEventListener("click", () => { paused = !paused; $r("#k-pause span").textContent = paused ? "Resume" : "Pause"; });
  $r("#k-clear").addEventListener("click", () => { body.innerHTML = ""; $r("#k-empty").hidden = false; });
  $r("#k-filter").addEventListener("input", rebuild);
  $r("#k-copy").addEventListener("click", async () => { try { await navigator.clipboard.writeText($r("#k-raw").textContent); toast("Copied", "ok"); } catch { toast("Copy was blocked by the browser", "bad"); } });
  rebuild();
  return {
    packet(p) { if (paused) { return; } add(p); },
    update() {
      const pts = S.ppsHist.slice(-60), mx = Math.max(1, ...pts);
      $r("#k-spark polyline").setAttribute("points", pts.map((v, i) => `${(i * 160) / 59},${34 - (v / mx) * 32}`).join(" "));
      $r("#k-pps").textContent = f1(S.st && S.st.pps);
    },
  };
}

/* ===================================================================== pages: board */
const lineClass = (l) => /HOLD|lost|busy|not connected|failed|error/i.test(l) ? "w" : /WiFi OK|input active|Time OK|settings:/i.test(l) ? "g" : /^(JOG|#\d)/.test(l) ? "m" : "";
function termAppend(term, t, line, cls) {
  const hint = term.querySelector(":scope > .m:only-child");
  if (hint) hint.remove();
  const near = term.scrollHeight - term.scrollTop - term.clientHeight < 40;
  const d = document.createElement("div");
  d.innerHTML = `<span class="m">${timeStr(t).slice(0, 8)}</span>  <span class="${cls}">${esc(line)}</span>`;
  term.appendChild(d);
  while (term.childElementCount > 400) term.firstChild.remove();
  if (near) term.scrollTop = term.scrollHeight;
}
function mountBoard(root) {
  root.innerHTML = `
  <div class="strip">
    <div><div class="k">USB port</div><div class="v s" id="b-port">-</div><div class="sub" id="b-port-s"></div></div>
    <div><div class="k">Input source</div><div class="v s" id="b-src">-</div><div class="sub">what the board is obeying</div></div>
    <div><div class="k">Board WiFi</div><div class="v s" id="b-wifi">-</div><div class="sub">from its log</div></div>
    <div><div class="k">Speed at full stick</div><div class="v s" id="b-rate">-</div><div class="sub" id="b-rate-s"></div></div>
    <div><div class="k">Servo travel</div><div class="v s" id="b-range">-</div><div class="sub">assumed range of the pulse span</div></div>
  </div>
  <div id="b-msg"></div>
  <div class="cols">
    <section class="sec" style="border-top:0"><h2>Board log</h2><p class="lead">Everything the ESP32 prints over USB, live.</p>
      <div class="tools"><button class="btn sm" id="b-pause" type="button">${icon("pause", "sm")}<span>Pause</span></button><button class="btn sm" id="b-clear" type="button">${icon("trash", "sm")}Clear</button>
        <button class="btn sm" id="b-show" type="button">Read settings</button></div>
      <div class="term tall" id="b-term" role="log" aria-live="off"></div></section>
    <div>
      <section class="sec" style="border-top:0"><h2>Mirror the virtual arm</h2>
        <p class="lead">The real arm follows the virtual one: sticks, home (B), sequences and their speeds all run in the virtual arm with its inverse kinematics, and the hub streams the joint angles to the board over USB. It only moves while the output is ARMED, never faster than 60 deg/s, and it holds still if the stream stops. Off until you switch it on.</p>
        <div class="inline" style="flex-wrap:wrap;gap:8px"><button class="btn" id="m-on" type="button">Mirror: off</button><span class="st idle" id="m-st">off</span></div>
        <p class="note" style="margin-top:10px">Before trusting a sequence, find out which joint is which: arm the output with mirror on, press a joint, and watch which part of the real arm nudges (about 8 degrees, slowly). Joint 1 is the base, then shoulder, elbow, wrist. If the wrong part moves, change the map below.</p>
        <div class="inline" style="flex-wrap:wrap;gap:8px" id="m-test">${[1, 2, 3, 4].map((j) => `<button class="btn sm" data-j="${j}" type="button">Nudge joint ${j}</button>`).join("")}</div>
        <div class="inline" style="flex-wrap:wrap;gap:8px;margin-top:10px"><label class="muted">Joint to channel map <input id="m-map" value="3,2,1,0" style="width:90px" aria-label="Joint to channel map"></label><button class="btn sm" id="m-map-set" type="button">Send to board</button></div>
        <div class="inline" style="flex-wrap:wrap;gap:8px;margin-top:10px"><label class="muted">Pulse range (us) <input id="m-pulse" value="500,2400" style="width:110px" aria-label="Pulse width at 0 and at 180 degrees"></label><button class="btn sm" id="m-pulse-set" type="button">Send to board</button></div>
        <p class="note">Mirror mode uses Sam's servo mapping: 500 us at 0 degrees to 2400 us at 180 degrees, 90 degrees at 1450 us, because the model's joint angles and the trained limits and homes are in that convention. If your servos need a different span, change it here; the board remembers it.</p>
        <p class="note" id="m-cal" style="color:var(--warn)" hidden>The joint limits are still the defaults (0 to 180 for every joint). Set what each joint can safely do on the Control page before the real arm moves: with the defaults the arm can be told to go to the very end of a servo's travel.</p>
        <p class="note">Channels are the board's outputs in the order of the pin map below (0 = OUT1 GPIO25 ... 3 = OUT4 GPIO32). The default 3,2,1,0 is how the arm is wired for Sam's firmware: joint 1 on GPIO32, 2 on GPIO27, 3 on GPIO26, 4 on GPIO25. The board remembers it.</p></section>
      <section class="sec"><h2>USB link</h2>
        <p class="lead">The hub opens the board's COM port to read its log and send settings. Only one program can hold the port.</p>
        <div class="inline"><button class="btn" id="b-usb" type="button"></button><span class="note" id="b-usb-n"></span></div></section>
      <section class="sec"><h2>Outputs</h2><p class="lead">Last positions the board reported. They update when the input changes.</p><div id="b-outs"></div>
        <p class="note">Position lives on the board: a reset or power cycle returns every output to centre.</p></section>
      <section class="sec"><h2>Pin map</h2><table class="pins"><thead><tr><th>Output</th><th>Pin</th><th>Stick</th></tr></thead><tbody>${OUTS.map((o) => `<tr><td>${o[0]}</td><td>GPIO${o[1]}</td><td>${o[2]}</td></tr>`).join("")}</tbody></table>
        <p class="note" style="margin-top:8px">Servo pulses: 50 Hz, 1000 to 2000 us, centre 1500 us.</p></section>
    </div>
  </div>`;
  const $r = (s) => $(s, root), term = $r("#b-term");
  let paused = false;
  S.board.slice(-200).forEach((b) => termAppend(term, b.t, b.line, lineClass(b.line)));
  if (!term.childElementCount) term.innerHTML = '<span class="m">No board output yet. Lines appear here as soon as the ESP32 is on USB and prints something.</span>';
  $r("#b-pause").addEventListener("click", () => { paused = !paused; $r("#b-pause span").textContent = paused ? "Resume" : "Pause"; });
  $r("#b-clear").addEventListener("click", () => { term.innerHTML = ""; });
  $r("#b-show").addEventListener("click", async () => { const r = await api("/api/board", { cmd: "!show" }); if (!r.ok) toast("The USB link is not open", "bad"); });
  $r("#b-usb").addEventListener("click", async () => { await api("/api/usb", { enabled: !(S.st && S.st.usb.enabled) }); });
  $r("#m-on").addEventListener("click", async () => {
    const on = !(S.st && S.st.mirror && S.st.mirror.on), r = await api("/api/mirror", { action: on ? "on" : "off" });
    if (!r.ok) toast(r.error || "The hub is not reachable", "bad");
  });
  $r("#m-test").addEventListener("click", async (e) => {
    const b = e.target.closest("[data-j]"); if (!b) return;
    const r = await api("/api/mirror", { action: "test", joint: Number(b.dataset.j) });
    if (!r.ok) toast(r.error || "The hub is not reachable", "bad");
  });
  $r("#m-pulse-set").addEventListener("click", async () => {
    const [lo, hi] = $r("#m-pulse").value.split(",").map((v) => Number(v.trim()));
    const r = await api("/api/mirror", { action: "pulse", lo, hi });
    if (!r.ok) toast(r.error || "The hub is not reachable", "bad"); else toast("Pulse range sent to the board", "ok");
  });
  $r("#m-map-set").addEventListener("click", async () => {
    const r = await api("/api/mirror", { action: "map", map: $r("#m-map").value.split(",").map((v) => Number(v.trim())) });
    if (!r.ok) toast(r.error || "The hub is not reachable", "bad"); else toast("Map sent to the board", "ok");
  });
  $r("#b-outs").innerHTML = OUTS.map((o, i) => `<div class="meter"><span>${o[0]}</span><div class="bar mid"><i id="b-o${i}" style="width:0"></i></div><span class="n" id="b-on${i}">-</span></div>`).join("");
  return {
    board(b) { if (!paused) termAppend(term, b.t, b.line, lineClass(b.line)); },
    update() {
      const st = S.st; if (!st) return; const u = st.usb;
      $r("#b-port").textContent = u.port || "none";
      $r("#b-port-s").textContent = { open: "open", busy: "held by another program", waiting: "not detected", off: "link switched off", suspended: "released for upload" }[u.phase];
      $r("#b-src").textContent = st.board.source === "usb" ? "USB" : st.board.source === "wifi" ? "WiFi" : "unknown";
      $r("#b-wifi").textContent = st.board.wifi === true ? "connected" : st.board.wifi === false ? "not connected" : "unknown";
      $r("#b-rate").textContent = u.rate == null ? "-" : u.rate + " deg/s";
      $r("#b-rate-s").textContent = u.rate === 0 ? "no limit" : "";
      $r("#b-range").textContent = u.range == null ? "-" : u.range + " deg";
      const msg = { waiting: "No board detected. Plug the ESP32 in with a data cable. The page keeps looking by itself.",
        busy: "The COM port is held by another program. Close the Arduino Serial Monitor, or stop the controller window under Services.",
        suspended: "The port is released while firmware uploads.", off: "The USB link is switched off, so the board is not being read." }[u.phase];
      $r("#b-msg").innerHTML = msg ? `<div class="err" style="margin-bottom:16px">${esc(msg)}</div>` : "";
      $r("#b-usb").textContent = u.enabled ? "Turn link off" : "Turn link on";
      const mi = st.mirror || {};
      $r("#m-on").textContent = mi.on ? "Mirror: on (press to switch off)" : "Mirror: off (press to switch on)";
      $r("#m-st").className = "st " + (mi.active ? "run" : mi.on ? "warn" : "idle");
      $r("#m-st").textContent = mi.testing ? "nudging joint " + mi.testing : mi.active ? "the real arm is following" : mi.on ? "on, waiting for ARM and the USB board" : "off";
      root.querySelectorAll("#m-test button").forEach((b) => { b.disabled = !mi.active; });
      const lim = st.sim && st.sim.limits;
      $r("#m-cal").hidden = !(mi.on && lim && lim.every((l) => l[0] === 0 && l[1] === 180));
      $r("#b-usb-n").textContent = u.enabled ? "On: opens the port when a board is plugged in." : "Off: the port is free for other tools.";
      const pos = st.board.pos;
      OUTS.forEach((o, i) => {
        const bar = $r("#b-o" + i), n = $r("#b-on" + i);
        if (pos) { const v = (pos[i] - 1500) / 500; bar.style.left = (v >= 0 ? 50 : 50 + v * 50) + "%"; bar.style.width = Math.abs(v) * 50 + "%"; n.textContent = Math.round(pos[i]) + " us"; }
        else { bar.style.width = "0"; n.textContent = "-"; }
      });
    },
  };
}

/* ===================================================================== pages: firmware */
function mountFirmware(root) {
  root.innerHTML = `
  <div class="cols">
    <div>
      <section class="sec" style="border-top:0;padding-top:0"><h2>Swap the firmware on the board</h2>
        <p class="lead">Flash Sam's firmware, ours, or the USB jog sketch onto the ESP32. Each one is built once and kept, so swapping back is just an upload. Sam's and ours poll the end server over WiFi themselves; the USB sketch does not.</p>
        <div class="rows" id="f-profiles"></div>
        <p class="note" id="f-pnote" style="margin-top:8px"></p></section>
      <section class="sec"><h2>ESP32 sketch (USB jog)</h2>
        <p class="lead">arduino/arm_receiver, built for the ESP32 Dev Module with arduino-cli.</p>
        <div class="rows">
          <div style="grid-template-columns:150px 1fr"><span class="muted">Port</span><span class="mono" id="f-port">-</span></div>
          <div style="grid-template-columns:150px 1fr"><span class="muted">WiFi and URL file</span><span id="f-sec"></span></div>
          <div style="grid-template-columns:150px 1fr"><span class="muted">Last job</span><span id="f-last"></span></div>
        </div>
        <div class="inline" style="margin-top:14px"><button class="btn primary" id="f-compile" type="button">${icon("hammer", "sm")}Compile</button>
          <button class="btn" id="f-upload" type="button">${icon("upload-simple", "sm")}Upload</button><span class="note" id="f-why"></span></div>
        <div id="f-prog" class="bar" style="margin-top:14px" hidden><i style="width:35%;animation:shimmer 1.2s linear infinite"></i></div>
      </section>
      <section class="sec"><h2>Size</h2>
        <div class="meter"><span>Flash</span><div class="bar"><i id="f-flash" style="width:0"></i></div><span class="n" id="f-flash-n">-</span></div>
        <div class="meter"><span>Memory</span><div class="bar"><i id="f-ram" style="width:0"></i></div><span class="n" id="f-ram-n">-</span></div>
        <p class="note">Shown after a successful compile.</p></section>
      <section class="sec"><h2>Before you upload</h2>
        <p class="note">Uploading resets the board. Outputs return to centre, so a connected arm will move there. The hub releases the COM port for the upload and takes it back afterwards. Upload is only offered while the output is SAFE.</p></section>
    </div>
    <section class="sec" style="border-top:0;padding-top:0"><h2>Build output</h2><div class="term tall" id="f-term" role="log" aria-live="off"></div></section>
  </div>`;
  const $r = (s) => $(s, root), term = $r("#f-term");
  (S.logs.firmware || []).forEach((l) => termAppend(term, l.t, l.line, /error|fail/i.test(l.line) ? "w" : ""));
  if (!term.childElementCount) term.innerHTML = '<span class="m">No build output yet. Press Compile to see it here.</span>';
  const run = async (a) => { const r = await api("/api/firmware", { action: a }); if (!r.ok) toast(r.message || r.error, "bad"); };
  const runP = async (a, profile) => { const r = await api("/api/firmware", { action: a, profile }); if (!r.ok) toast(r.message || r.error, "bad"); };
  $r("#f-profiles").addEventListener("click", (e) => { const b = e.target.closest("button[data-fa]"); if (b && !b.disabled) runP(b.dataset.fa, b.dataset.fp); });
  let pSig = "";
  $r("#f-compile").addEventListener("click", () => run("compile"));
  $r("#f-upload").addEventListener("click", () => run("upload"));
  return {
    log(l) { if (l.src === "firmware") termAppend(term, l.t, l.line, /error|fail/i.test(l.line) ? "w" : ""); },
    update() {
      const st = S.st; if (!st) return; const fw = st.firmware, u = st.usb, running = fw.state === "running";
      $r("#f-port").textContent = u.port || "not detected";
      $r("#f-sec").innerHTML = fw.secrets_present ? `<span class="st ok">secrets.h found</span>` : `<span class="st bad">secrets.h missing: copy secrets.example.h and fill it in</span>`;
      $r("#f-last").innerHTML = running ? `<span class="st run">${esc(fw.action)} running</span>` : fw.finished_t ? `<span class="st ${fw.state === "ok" ? "ok" : "bad"}">${esc(fw.action)} ${fw.state === "ok" ? "succeeded" : "failed"}, ${ageStr(fw.age)} ago</span>` : `<span class="st idle">nothing run yet</span>`;
      $r("#f-prog").hidden = !running;
      const profs = fw.profiles || [], busyP = running ? fw.profile : null;
      const canFlash = !running && !st.armed && !!u.port;
      const sig = JSON.stringify([profs, running, busyP, canFlash]);
      if (sig !== pSig) {
        pSig = sig;
        $r("#f-profiles").innerHTML = profs.map((p) => {
          const badge = p.on_board ? `<span class="st ok">on the board</span> ` : "";
          const built = !p.built ? `<span class="st idle">not built</span>` : p.stale ? `<span class="st bad">changed, needs rebuild</span>` : `<span class="st ok">built${p.flash_pct != null ? `, ${p.flash_pct}% flash` : ""}</span>`;
          const sp = busyP === p.id ? ` <span class="st run">${esc(fw.action)} running</span>` : "";
          return `<div style="grid-template-columns:1fr auto"><div><div>${esc(p.label)} ${badge}${built}${sp}</div><div class="note">${esc(p.desc)}</div></div>
            <div class="inline"><button class="btn" type="button" data-fa="compile" data-fp="${esc(p.id)}"${running ? " disabled" : ""}>${icon("hammer", "sm")}Build</button>
            <button class="btn primary" type="button" data-fa="swap" data-fp="${esc(p.id)}"${canFlash ? "" : " disabled"}>${icon("upload-simple", "sm")}${p.on_board ? "Reflash" : "Flash"}</button></div></div>`;
        }).join("") || '<span class="muted">No firmware profiles reported. Restart the hub.</span>';
      }
      $r("#f-pnote").textContent = running ? "A firmware job is running." : st.armed ? "Switch the output to SAFE to flash." : !u.port ? "Plug the board in to flash." : "Flashing resets the board: the servos go to their start pose.";
      const canUp = !running && fw.secrets_present && !st.armed && !!u.port;
      $r("#f-compile").disabled = running || !fw.secrets_present;
      $r("#f-upload").disabled = !canUp;
      $r("#f-why").textContent = !fw.secrets_present ? "Compile needs secrets.h." : st.armed ? "Switch to SAFE to upload." : !u.port ? "Plug the board in to upload." : "";
      if (fw.flash_pct != null) { $r("#f-flash").style.width = fw.flash_pct + "%"; $r("#f-flash-n").textContent = fw.flash_pct + "%"; }
      if (fw.ram_pct != null) { $r("#f-ram").style.width = fw.ram_pct + "%"; $r("#f-ram-n").textContent = fw.ram_pct + "%"; }
    },
  };
}

/* ===================================================================== pages: services */
const SVC = [
  ["docker_engine", "Docker Desktop", "The engine the local server runs on.", ["start"]],
  ["server_docker", "Local server (Docker)", "Receives controller packets and forwards them. The normal way to run it.", ["start", "stop", "restart"]],
  ["server_python", "Local server (Python)", "The same server without Docker. Uses port 8000, so run only one of the two.", ["start", "stop", "restart"]],
  ["gui", "Controller window", "The pygame app for a physical Xbox controller. It also holds the board's COM port.", ["start", "stop"]],
];
function mountServices(root) {
  let logSrc = "hub";
  root.innerHTML = `
  <section class="sec" style="border-top:0;padding-top:0"><h2>Services</h2><div class="rows" id="s-rows"></div></section>
  <div class="cols even">
    <section class="sec"><h2>Configuration</h2><div class="rows" id="s-cfg"></div>
      <p class="note" style="margin-top:8px">Values stay in git-ignored files. This page only shows whether they exist.</p></section>
    <section class="sec"><h2>End-server forwarding</h2><div class="rows" id="s-fwd"></div></section>
  </div>
  <section class="sec"><h2>Logs</h2><div class="tools"><div class="tabs" id="s-tabs">${LOG_SRC.map((s) => `<button type="button" data-src="${s[0]}" aria-selected="${s[0] === logSrc}">${s[1]}</button>`).join("")}</div></div>
    <div class="term" id="s-term" role="log" aria-live="off"></div></section>`;
  const $r = (s) => $(s, root), term = $r("#s-term");
  const fill = () => { term.innerHTML = ""; (S.logs[logSrc] || []).slice(-120).forEach((l) => termAppend(term, l.t, l.line, /error|fail|not found/i.test(l.line) ? "w" : "")); if (!term.childElementCount) term.innerHTML = '<span class="m">No output yet for this service.</span>'; };
  $r("#s-tabs").addEventListener("click", (e) => { const b = e.target.closest("[data-src]"); if (!b) return; logSrc = b.dataset.src; root.querySelectorAll("#s-tabs button").forEach((x) => x.setAttribute("aria-selected", x === b)); fill(); });
  fill();
  const lab = { start: ["play", "Start"], stop: ["stop", "Stop"], restart: ["arrows-clockwise", "Rebuild"] };
  return {
    log(l) { if (l.src === logSrc) { if (term.querySelector(".m:only-child")) term.innerHTML = ""; termAppend(term, l.t, l.line, /error|fail|not found/i.test(l.line) ? "w" : ""); } },
    update() {
      const st = S.st; if (!st) return;
      $r("#s-rows").innerHTML = SVC.map(([id, name, desc, acts]) => {
        const s = st.services[id], lvl = s.state === "running" ? "ok" : s.state === "busy" ? "run" : "idle";
        const btns = acts.map((a) => {
          const dis = s.state === "busy" || (a === "start" && s.state === "running") || ((a === "stop" || a === "restart") && s.state === "stopped" && id !== "docker_engine");
          return `<button class="btn sm" data-act="svc" data-name="${id}" data-action="${a}" ${dis ? "disabled" : ""}>${icon(lab[a][0], "sm")}${id === "server_python" && a === "restart" ? "Restart" : lab[a][1]}</button>`;
        }).join("");
        return `<div class="svc"><div><div class="name">${name}</div><div class="desc">${esc(desc)}</div></div><span class="st ${lvl}">${s.state}</span><span class="note">${esc(s.detail || "")}</span><div class="actions">${btns}</div></div>`;
      }).join("");
      const c = st.config, f = st.server.forward;
      const yn = (b, t, f2) => `<span class="st ${b ? "ok" : "warn"}">${b ? t : f2}</span>`;
      $r("#s-cfg").innerHTML = `<div style="grid-template-columns:200px 1fr"><span class="muted">.env file</span>${yn(c.env_present, "found", "missing")}</div>
        <div style="grid-template-columns:200px 1fr"><span class="muted">End-server address</span>${yn(c.forward_url_set, "set (hidden)", "not set")}</div>
        <div style="grid-template-columns:200px 1fr"><span class="muted">Firmware secrets.h</span>${yn(st.firmware.secrets_present, "found", "missing")}</div>`;
      $r("#s-fwd").innerHTML = `<div style="grid-template-columns:200px 1fr"><span class="muted">State</span><span class="st ${st.server.forward_enabled === false ? "idle" : f.ok ? "ok" : f.ok === false ? "bad" : "warn"}">${st.server.forward_enabled === false ? "paused (SAFE)" : f.ok ? "forwarding" : f.ok === false ? "failing" : "waiting"}</span></div>
        <div style="grid-template-columns:200px 1fr"><span class="muted">Sent</span><span class="mono">${f.sent ?? "-"}</span></div>
        <div style="grid-template-columns:200px 1fr"><span class="muted">Failed</span><span class="mono">${f.failed ?? "-"}</span></div>
        <div style="grid-template-columns:200px 1fr"><span class="muted">Last error</span><span class="mono">${esc(f.error || "none")}</span></div>`;
    },
  };
}

/* ===================================================================== pages: AI and jobs */
const JOB_EXAMPLE = `{
  "job": "fetch_key",
  "key_slot": 7,
  "deposit": "collection_tray",
  "max_speed_deg_s": 5
}`;
function mountAI(root) {
  root.innerHTML = `
  <div class="cols">
    <div>
      <section class="sec" style="border-top:0;padding-top:0"><h2>Where this is heading</h2>
        <p class="lead">The front end comes first. These are the pieces the hub is being shaped around, none of them wired up yet.</p>
        <div class="rows">
          <div style="grid-template-columns:1fr auto"><div><div class="name" style="font-weight:600">AI decision support</div><div class="note">An AI assistant decides what the arm should do and passes its answer to the control software (project brief, section 4).</div></div><span class="st idle">planned</span></div>
          <div style="grid-template-columns:1fr auto"><div><div class="name" style="font-weight:600">Instruction packets</div><div class="note">The hub sends JSON instruction packets to the microcontroller, which performs the task. Today the board only accepts live stick input.</div></div><span class="st idle">planned</span></div>
          <div style="grid-template-columns:1fr auto"><div><div class="name" style="font-weight:600">Car hire key fetcher</div><div class="note">The likely use case: fetch the right key from a rack and drop it where the customer can collect it.</div></div><span class="st idle">planned</span></div>
        </div>
      </section>
      <section class="sec"><h2>Adding a page</h2>
        <p class="note">Each page is one function in dashboard/static/app.js plus one line in the PAGES list. The hub already streams packets, board output, logs and status to every page, so a job queue or an AI panel can plug in without touching the rest.</p></section>
    </div>
    <section class="sec" style="border-top:0;padding-top:0"><h2>Packet composer (dry run)</h2>
      <p class="lead">Check an instruction packet before there is anything to send it to. The example is a draft: its field names are not a fixed format.</p>
      <div class="field"><label for="j-text">Instruction packet (JSON)</label><textarea id="j-text" spellcheck="false">${esc(JOB_EXAMPLE)}</textarea></div>
      <div class="inline"><button class="btn primary" id="j-val" type="button">${icon("list-checks", "sm")}Validate</button>
        <button class="btn" id="j-fmt" type="button">Format</button><button class="btn" id="j-copy" type="button">${icon("copy", "sm")}Copy</button>
        <button class="btn" type="button" disabled title="The board does not accept instruction packets yet">Send to board</button></div>
      <div id="j-out" class="note" style="margin-top:12px" role="status"></div>
    </section>
  </div>`;
  const $r = (s) => $(s, root), ta = $r("#j-text"), out = $r("#j-out");
  const parse = () => { try { return [JSON.parse(ta.value), null]; } catch (e) { return [null, e.message]; } };
  $r("#j-val").addEventListener("click", () => {
    const [j, err] = parse();
    if (err) { out.innerHTML = `<div class="err">Not valid JSON: ${esc(err)}</div>`; return; }
    if (typeof j !== "object" || j === null || Array.isArray(j)) { out.innerHTML = `<div class="err">An instruction packet must be a JSON object.</div>`; return; }
    const keys = Object.keys(j);
    out.innerHTML = `<span class="st ok">Valid JSON</span> with ${keys.length} field${keys.length === 1 ? "" : "s"}: <span class="mono">${esc(keys.join(", "))}</span>. ${"job" in j ? "" : "No \"job\" field yet. "}Nothing was sent.`;
  });
  $r("#j-fmt").addEventListener("click", () => { const [j, err] = parse(); if (!err) ta.value = JSON.stringify(j, null, 2); else out.innerHTML = `<div class="err">Not valid JSON: ${esc(err)}</div>`; });
  $r("#j-copy").addEventListener("click", async () => { try { await navigator.clipboard.writeText(ta.value); toast("Copied", "ok"); } catch { toast("Copy was blocked by the browser", "bad"); } });
  return {};
}

/* ===================================================================== pages: project */
function mountChat(root) {
  document.body.classList.add("fill");
  root.innerHTML = `
  <div class="chat">
    <div class="chat-bar"><span id="ch-ai" class="st idle">AI</span><span id="ch-st" class="st idle">idle</span><span class="muted">Every message between you, the AI and the arm is shown here, including the AI's raw reply.</span></div>
    <div id="ch-log" class="chat-log" role="log" aria-live="polite"></div>
    <form id="ch-form" class="chat-in"><input id="ch-text" autocomplete="off" maxlength="500" placeholder="Tell the arm what to do, e.g. put the key from box 2 on the tray" aria-label="Message"><button class="btn" id="ch-send" type="submit">Send</button></form>
  </div>`;
  const $r = (q) => root.querySelector(q), log = $r("#ch-log");
  let since = 0, dead = false;
  const kv = (o) => Object.entries(o || {}).map(([k, v]) => `<span class="chip">${esc(k)}: <b>${esc(typeof v === "object" ? JSON.stringify(v) : v)}</b></span>`).join("");
  const bubble = (m) => {
    const t = new Date(m.t * 1000).toLocaleTimeString([], { hour: "2-digit", minute: "2-digit", second: "2-digit" });
    let who = { user: "You", ai: "AI", robot: "Arm", error: "Error", system: "Hub" }[m.role] || m.role, body = "";
    if (m.role === "ai") {
      const pick = m.sequence ? `Running <b class="mono">${esc(m.sequence)}</b>${m.reverse ? " <b>reversed</b>" : ""}` : "No sequence fits that request";
      body = `<div>${pick}</div>${m.text ? `<div class="muted">${esc(m.text)}</div>` : ""}<div class="meta">${kv({ by: m.by, model: m.model, ms: m.ms, tokens: m.tokens })}</div>`
        + (m.raw ? `<details><summary>Raw AI reply</summary><pre class="mono">${esc(m.raw)}</pre></details>` : "");
    } else if (m.role === "error") {
      body = `<b>${esc(m.text)}</b>${m.detail ? `<div>${esc(m.detail)}</div>` : ""}${m.hint ? `<div class="muted">${esc(m.hint)}</div>` : ""}${m.code ? `<div class="muted mono">${esc(m.code)}</div>` : ""}`;
    } else {
      body = `<div>${esc(m.text)}</div>`;
      if (m.result) body += `<div class="meta">${kv(m.result)}</div>`;
    }
    return `<div class="msg ${m.role}"><div class="who">${who}<span class="muted mono">${t}</span></div>${body}</div>`;
  };
  const poll = async () => {
    if (dead || !document.body.contains(log)) return;
    try {
      const j = await (await fetch("/api/chat?since=" + since)).json();
      const stick = log.scrollHeight - log.scrollTop - log.clientHeight < 60;
      if (j.messages.length) { if (!since) log.innerHTML = ""; log.insertAdjacentHTML("beforeend", j.messages.map(bubble).join("")); since = j.messages[j.messages.length - 1].id; if (stick) log.scrollTop = log.scrollHeight; }
      else if (!since && !log.children.length) log.innerHTML = '<div class="empty muted">No messages yet. Ask the arm to do something below.</div>';
      const ai = $r("#ch-ai"), st = $r("#ch-st"), busy = j.state === "working" || j.state === "running";
      ai.className = "st " + (j.ai === "groq" ? "ok" : "bad"); ai.textContent = j.ai === "groq" ? "AI: Groq" : "AI: keywords only (no GROQ_API_KEY in .env)";
      st.className = "st " + (busy ? "run" : j.state === "failed" ? "bad" : j.state === "done" ? "ok" : "idle"); st.textContent = j.state;
      $r("#ch-send").disabled = busy;
    } catch (e) { /* hub not reachable: try again */ }
    setTimeout(poll, 800);
  };
  $r("#ch-form").addEventListener("submit", async (e) => {
    e.preventDefault(); const inp = $r("#ch-text"), t = inp.value.trim(); if (!t) return;
    const r = await api("/api/ask", { request: t });
    if (!r.ok) return toast(r.error || "The hub is not reachable", "bad");
    inp.value = "";
  });
  poll();
  return { destroy() { dead = true; } };
}

function mountTune(root) {
  root.innerHTML = `
  <div class="cols"><div>
    <section class="sec" style="border-top:0;padding-top:0"><h2>Speed tuner</h2>
      <p class="lead">Learns a speed (deg/s) for every move of a sequence by running it again and again in a private copy of the virtual arm, keeping what scores better. Slow where a key is carried or a grab or release is coming, fast everywhere else. The tuning itself only ever runs in simulation; the section below it is what tries the result on the real arm.</p>
      <div class="inline" style="flex-wrap:wrap;gap:8px">
        <label class="muted">Sequence <select id="t-seq" aria-label="Sequence"></select></label>
        <label class="muted">Search length <input id="t-gen" type="number" min="5" max="150" value="40" style="width:70px" aria-label="Generations"></label>
        <label class="muted"><input id="t-cur" type="checkbox"> start from its saved speeds</label>
        <button class="btn" id="t-go" type="button">Tune</button><button class="btn" id="t-stop" type="button" hidden>Cancel</button>
        <span id="t-st" class="st idle">idle</span>
      </div>
      <details style="margin-top:8px"><summary class="muted" style="cursor:pointer">Limits the tuner must respect (estimates: adjust them after watching the real arm)</summary>
        <div class="inline" style="flex-wrap:wrap;gap:8px;margin-top:6px">
          <label class="muted">Key slips above (tip mm/s) <input id="t-slip" type="number" value="220" style="width:80px"></label>
          <label class="muted">Grab or release knocked above (mm/s) <input id="t-knock" type="number" value="110" style="width:80px"></label>
          <label class="muted">Safety margin <input id="t-margin" type="number" step="0.05" min="0.3" max="1" value="0.85" style="width:70px"></label>
        </div>
      </details>
      <div id="t-prog" class="note" style="margin-top:8px"></div>
    </section>
    <section class="sec" id="t-res" hidden><h2>Result</h2><div id="t-sum" class="lead"></div><div id="t-chart"></div><div class="rows" id="t-rows"></div>
      <div class="inline" style="margin-top:10px;gap:8px"><button class="btn" id="t-apply" type="button">Save speeds into the sequence</button><button class="btn" id="t-clear" type="button">Remove saved speeds</button></div>
      <p class="note" id="t-note"></p></section>
    <section class="sec" id="t-trial" hidden><h2>Try it on the real arm</h2>
      <p class="lead">The simulator has no inertia and its limits are estimates, so the tuned speeds are checked on the arm itself, a fraction of them first. Each rung only opens once the one below went well. What you see afterwards corrects the limits, and the next tuning respects them. <b>This moves the arm: keep clear. The SAFE button and Escape cut the output.</b></p>
      <div id="t-ready" class="rows"></div>
      <div class="inline" id="t-rungs" style="flex-wrap:wrap;gap:8px;margin-top:10px"></div>
      <div id="t-tstate" class="note" style="margin-top:8px"></div>
      <div class="inline" id="t-verdict" style="flex-wrap:wrap;gap:8px;margin-top:8px" hidden>
        <span class="muted">How did it go?</span>
        <button class="btn" data-v="ok" type="button">It went fine</button><button class="btn" data-v="dropped" type="button">The key dropped</button>
        <button class="btn" data-v="missed" type="button">A grab or release went wrong</button><button class="btn" data-v="rough" type="button">Too rough</button>
      </div>
      <button class="btn danger" id="t-abort" type="button" hidden style="margin-top:8px">Abort the run</button>
      <div id="t-learned" class="note" style="margin-top:10px"></div>
    </section>
  </div></div>`;
  const $r = (q) => root.querySelector(q);
  let dead = false, seqs = [], shown = null, limitsTouched = false, lastLimitsFor = "";
  const num = (id) => Number($r(id).value);
  const post = (body) => api("/api/tune", body);
  const limits = () => ({ slip_mm_s: num("#t-slip"), knock_mm_s: num("#t-knock"), margin: num("#t-margin") });
  const spark = (h) => {
    if (!h || h.length < 2) return "";
    const w = 360, ht = 70, mx = Math.max(...h), mn = Math.min(...h), r = mx - mn || 1;
    const pts = h.map((v, i) => `${(i / (h.length - 1) * w).toFixed(1)},${(ht - 6 - (v - mn) / r * (ht - 12)).toFixed(1)}`).join(" ");
    return `<svg viewBox="0 0 ${w} ${ht}" width="100%" style="max-width:${w}px" role="img" aria-label="Cost per generation, falling from ${mx} to ${mn}"><polyline points="${pts}" fill="none" stroke="var(--accent)" stroke-width="2"/></svg><div class="muted" style="font-size:12px">Best cost per generation: ${mx} s to ${mn} s</div>`;
  };
  const showResult = (t, seq) => {
    const res = t.result, sec = $r("#t-res");
    if (!res || !seq) { sec.hidden = true; return; }
    sec.hidden = false;
    const b = res.baseline, u = res.tuned, gain = b.time ? Math.round((1 - u.time / b.time) * 100) : 0;
    $r("#t-sum").innerHTML = `<b>${b.time} s</b> at ${b.speeds.every((v) => v === b.speeds[0]) ? b.speeds[0] + " deg/s everywhere" : "its saved speeds"} <b>→ ${u.time} s</b> tuned (${gain >= 0 ? gain + "% faster" : -gain + "% slower"}), ${u.failures ? `<span style="color:var(--bad)">${u.failures} failure(s)</span>` : "no failures"}. ${res.evals} simulated runs in ${res.seconds} s.`;
    $r("#t-chart").innerHTML = spark(res.history);
    const mx = Math.max(...res.speeds, 1), grid = "display:grid;grid-template-columns:34px 80px 1fr 150px;gap:8px;padding:4px 0";
    $r("#t-rows").innerHTML = `<div class="muted mono" style="${grid}"><span>#</span><span>action</span><span>tuned speed (deg/s)</span><span>tip start / end mm/s</span></div>` + res.tuned.segments.map((s, i) => {
      const p = (seq.poses || [])[i] || {};
      const note = !s.moves ? "no move" : s.holding ? "carrying key" : "";
      return `<div style="${grid};border-top:1px solid var(--line);align-items:center"><span class="mono">${i + 1}</span><span>${esc(p.action || "")}</span>
        <span style="display:flex;gap:8px;align-items:center"><span style="display:inline-block;height:8px;width:${Math.round(res.speeds[i] / mx * 160)}px;background:${s.moves ? "var(--accent)" : "var(--line-strong)"}"></span><span class="mono">${res.speeds[i]}</span><span class="muted">${note}</span></span>
        <span class="mono muted">${s.tip_start} / ${s.tip_end}</span></div>`;
    }).join("");
    $r("#t-note").textContent = u.failures ? "This result still fails in the simulation, so it cannot be saved." : "Saving writes a speed on each pose of the sequence file. The virtual arm then uses it every time the sequence runs, including from Chat, and so does the arm when it runs Our firmware (Sam's firmware has no speed per pose).";
    $r("#t-apply").disabled = !!u.failures || t.state === "running";
  };
  const loadSeq = async (name) => {
    try { shown = await (await fetch("/api/sequences/" + encodeURIComponent(name))).json(); } catch (e) { shown = null; }
  };
  const poll = async () => {
    if (dead || !document.body.contains($r("#t-go"))) return;
    try {
      const sel = $r("#t-seq").value, t = await (await fetch("/api/tune" + (sel ? "?name=" + encodeURIComponent(sel) : ""))).json(), busy = t.state === "running";
      const st = $r("#t-st");
      st.className = "st " + (busy ? "run" : t.state === "done" ? "ok" : t.state === "failed" ? "bad" : "idle");
      st.textContent = t.state;
      $r("#t-go").disabled = busy || !seqs.length; $r("#t-stop").hidden = !busy; $r("#t-seq").disabled = busy;
      $r("#t-prog").innerHTML = busy ? `Generation ${t.gen} of ${t.of}${t.best != null ? `, best cost so far ${t.best} s` : ""}<div style="height:6px;background:var(--line);margin-top:4px"><div style="height:6px;background:var(--accent);width:${Math.round(t.gen / Math.max(1, t.of) * 100)}%"></div></div>`
        : t.state === "failed" ? `<span style="color:var(--bad)">${esc(t.error || "failed")}</span>` : "";
      if (t.name && t.result && (shown == null || shown.name !== t.name)) await loadSeq(t.name);
      if (t.name && Array.from($r("#t-seq").options).some((o) => o.value === t.name) && !busy && !$r("#t-seq").dataset.touched) $r("#t-seq").value = t.name;
      showResult(t, shown && t.name === shown.name ? shown : null);
      fillLimits(t.calib); showTrial(t, !!t.ladder);
    } catch (e) { /* hub not reachable */ }
    setTimeout(poll, 700);
  };
  $r("#t-seq").addEventListener("change", () => { $r("#t-seq").dataset.touched = "1"; limitsTouched = false; lastLimitsFor = ""; });
  ["#t-slip", "#t-knock", "#t-margin"].forEach((id) => $r(id).addEventListener("input", () => { limitsTouched = true; }));
  const fillLimits = (c) => {
    if (!c || limitsTouched || lastLimitsFor === c.name + JSON.stringify(c.limits)) return;
    lastLimitsFor = c.name + JSON.stringify(c.limits);
    $r("#t-slip").value = c.limits.slip_mm_s; $r("#t-knock").value = c.limits.knock_mm_s; $r("#t-margin").value = c.limits.margin;
  };
  const showTrial = (t, haveSpeeds) => {
    const sec = $r("#t-trial"), tr = t.trial || {}, b = t.board || {}, busy = tr.state === "homing" || tr.state === "running";
    sec.hidden = !(haveSpeeds || tr.name);
    if (sec.hidden) return;
    const tick = (ok, yes, no) => `<div style="padding:3px 0"><span class="st ${ok ? "ok" : "bad"}">${ok ? yes : no}</span></div>`;
    const carried = b.mirror || (b.esp && b.pose_speeds);
    $r("#t-ready").innerHTML = tick(carried, b.mirror ? "Mirror mode is on: the real arm follows the virtual arm" : "Our firmware is reporting from the board",
        "Nothing is carrying the run out on a real arm: switch mirror mode on (Arm and board page, board on USB), or run Our firmware")
      + tick(b.armed, "The output is armed", "The output is SAFE: arm it (top right) when you are ready for the arm to move");
    const ready = carried && b.armed && !busy && t.state !== "running", lad = t.ladder || { allowed: 0.5, rungs: [0.5, 0.75, 1] };
    $r("#t-rungs").innerHTML = lad.rungs.map((r) => `<button class="btn" data-scale="${r}" type="button" ${ready && r <= lad.allowed + 1e-9 ? "" : "disabled"}>Run at ${Math.round(r * 100)}%${r <= lad.allowed + 1e-9 ? "" : " (locked)"}</button>`).join("");
    const names = { homing: "Getting the arm to home grip", running: "Running on the arm", verdict: "Finished: how did it go?", done: "Done", failed: "Did not run", aborted: "Aborted" };
    $r("#t-tstate").innerHTML = tr.state && tr.state !== "idle" ? `<b>${names[tr.state] || tr.state}</b>${tr.scale ? ` at ${Math.round(tr.scale * 100)}%` : ""}${tr.error ? ` <span style="color:var(--bad)">${esc(tr.error)}</span>` : ""}`
      + `${tr.predicted ? `<div class="muted">The simulator expects about ${tr.predicted.time} s, tool tip up to ${tr.predicted.hold_mm_s} mm/s while carrying the key and ${tr.predicted.land_mm_s} mm/s arriving at a grab or release.</div>` : ""}`
      + `${tr.note ? `<div>${esc(tr.note)}</div>` : ""}<div class="mono muted" style="font-size:12px">${(tr.log || []).map(esc).join("<br>")}</div>` : "";
    $r("#t-verdict").hidden = tr.state !== "verdict"; $r("#t-abort").hidden = !busy;
    const c = t.calib;
    $r("#t-learned").innerHTML = c ? `Limits learned from the arm for ${esc(c.name)}: key slips above <b>${c.limits.slip_mm_s}</b> mm/s, grab or release knocked above <b>${c.limits.knock_mm_s}</b> mm/s, margin <b>${c.limits.margin}</b>. ${(c.trials || []).length} real run(s) so far${(c.trials || []).length ? ": " + c.trials.map((x) => `${Math.round(x.scale * 100)}% ${x.outcome}`).join(", ") : ""}.` : "";
  };
  $r("#t-rungs").addEventListener("click", async (e) => {
    const b = e.target.closest("[data-scale]"); if (!b || b.disabled) return;
    const r = await post({ action: "trial", name: $r("#t-seq").value, scale: Number(b.dataset.scale) });
    if (r.error) toast(r.error, "bad");
  });
  $r("#t-verdict").addEventListener("click", async (e) => {
    const b = e.target.closest("[data-v]"); if (!b) return;
    const r = await post({ action: "verdict", outcome: b.dataset.v });
    if (r.error) toast(r.error, "bad");
  });
  $r("#t-abort").addEventListener("click", () => post({ action: "abort" }));
  $r("#t-go").addEventListener("click", async () => {
    const r = await post({ action: "start", name: $r("#t-seq").value, generations: num("#t-gen"), from_current: $r("#t-cur").checked, limits: limits() });
    if (r.error) toast(r.error, "bad");
  });
  $r("#t-stop").addEventListener("click", () => post({ action: "cancel" }));
  $r("#t-apply").addEventListener("click", async () => {
    const t = await (await fetch("/api/tune")).json(), r = await post({ action: "apply", name: t.name });
    if (r.error) return toast(r.error, "bad");
    toast("Speeds saved into " + t.name, "ok"); await loadSeq(t.name);
  });
  $r("#t-clear").addEventListener("click", async () => {
    const name = $r("#t-seq").value, r = await post({ action: "clear", name });
    if (r.error) return toast(r.error, "bad");
    toast("Speeds removed from " + name, "ok"); await loadSeq(name);
  });
  (async () => {
    try { seqs = (await (await fetch("/api/sequences")).json()).sequences || []; } catch (e) { seqs = []; }
    $r("#t-seq").innerHTML = seqs.length ? seqs.map((s) => `<option value="${esc(s.name)}">${esc(s.name)}</option>`).join("") : '<option value="">(no sequences yet: record one on the Robot page)</option>';
    poll();
  })();
  return { destroy() { dead = true; } };
}

function mountPoints(root) {
  document.body.classList.add("fill");
  const JOINTS = ["base", "shoulder", "elbow", "wrist"];
  root.innerHTML = `<div class="armstage" style="grid-template-columns:minmax(0,1fr) 470px">
    <section class="viewer" aria-label="Robot viewer">
      <canvas id="v3d" role="img" aria-label="3D view of the robot arm. Drag to turn, pinch or scroll to zoom, double tap to reset."></canvas>
      <div class="vhud tl"><div class="st idle" id="p-mode">-</div><div class="mono" id="p-xyz"></div><div class="mono" id="p-ang"></div></div>
      <div class="vhud tr"><button class="btn sm" type="button" data-view="iso">3D</button><button class="btn sm" type="button" data-view="top">Top</button><button class="btn sm" type="button" data-view="side">Side</button><button class="btn sm" type="button" id="v-clr">Clear path</button></div>
    </section>
    <aside class="side" aria-label="Points" style="overflow-y:auto">
      <p class="lead" style="margin-top:0">Five places for the tool tip to start with: the test points from Sam's UWE-RArm demo, which visits them in this order. Change any, switch some off, then move between them and watch the inverse kinematics work: the tip goes in a <b>straight line</b> while the four joints swing in curves to make it happen. The path in the viewer shows it.</p>
      <div id="p-cards">${[0, 1, 2, 3, 4].map((i) => `
        <div class="pcard" data-i="${i}" style="border-top:1px solid var(--line);padding:10px 0">
          <div class="inline" style="gap:10px;align-items:center"><label style="display:flex;gap:6px;align-items:center"><input type="checkbox" class="p-on" aria-label="Include this point"><b class="p-name"></b></label></div>
          <div class="inline" style="flex-wrap:wrap;gap:8px;margin:6px 0">
            ${[["x", "x mm"], ["y", "y mm"], ["z", "z mm"], ["angle", "tool angle"]].map(([k, l]) => `<label class="muted" style="font-size:12px">${l}<br><input class="p-f" data-k="${k}" type="number" step="5" style="width:78px"></label>`).join("")}
          </div>
          <div class="inline" style="gap:6px;flex-wrap:wrap"><button class="btn sm p-here" type="button">Use current position</button><button class="btn sm p-go" type="button">Go here</button></div>
          <div class="p-ik" style="margin-top:8px;font-size:12px;line-height:1.5"></div>
        </div>`).join("")}</div>
      <div class="inline" style="flex-wrap:wrap;gap:8px;margin:12px 0 6px"><button class="btn" id="p-loop" type="button">Loop between the points</button><button class="btn" id="p-stop" type="button">Stop</button></div>
      <div class="note" id="p-run" style="min-height:1.4em"></div>
      <details class="sidesec" style="margin-top:10px"><summary>How to read the numbers</summary>
        <p class="note">x is sideways, y is out from the base, z is up (mm, from the base). Tool angle is how the tool is tilted. <b>Base, shoulder, elbow and wrist</b> are the four joint angles the arm needs to put the tip there. There are usually two ways to bend the arm to reach a point, <b>elbow up</b> and <b>elbow down</b>; the arm takes the one that is within its joint limits and closest to where it is now. A point is refused when the two links cannot stretch that far, or every solution would push a joint past its limit. "Check" is the solution run forwards again: it should land on the point.</p>
        <p class="note">Moves between points are the straight-line move from Sam's UWE-RArm repo (peak speed on the Control page). With mirror mode on and the output armed, the real arm follows the virtual one.</p></details>
    </aside></div>`;
  const $r = (q) => root.querySelector(q), cards = Array.from(root.querySelectorAll(".pcard"));
  const viewer = makeViewer($r("#v3d"));
  let dead = false, last = null;
  const post = (body) => api("/api/points", body);
  const apply = (r) => { if (r && r.points) { last = r; render(r); } };
  const act = async (body) => { const r = await post(body); if (!r.ok && r.error) toast(r.error, "bad"); apply(r); };
  const ikHtml = (r) => {
    const fmt = (a) => JOINTS.map((n, i) => `${n} <b>${a[i].toFixed(1)}&deg;</b>`).join(", ");
    const lines = [];
    if (r.reachable) {
      const ch = r.branches.find((b) => b.branch === r.chosen) || r.branches.find((b) => b.ok);
      lines.push(`<div style="color:var(--ok)">Reachable. The arm takes <b>${ch.name}</b>: ${fmt(ch.angles)}.</div>`);
      lines.push(`<div class="muted">Check: forward kinematics lands within ${r.fk_error_mm} mm of the point.</div>`);
    } else lines.push('<div style="color:var(--bad)"><b>The arm cannot reach this point.</b></div>');
    r.branches.filter((b) => !(r.reachable && b.branch === r.chosen)).forEach((b) => {
      lines.push(`<div class="muted"><b>${b.name}</b>: ${b.ok ? `also possible: ${fmt(b.angles)}` : `not usable${b.why ? ": " + esc(b.why) : ""}${b.angles ? ` (${fmt(b.angles)})` : ""}`}</div>`);
    });
    return lines.join("");
  };
  const render = (st) => {
    cards.forEach((c, i) => {
      const p = st.points[i], ik = st.ik[i];
      c.querySelector(".p-name").textContent = "Point " + p.name;
      c.querySelector(".p-on").checked = !!p.on;
      c.querySelectorAll(".p-f").forEach((inp) => { if (document.activeElement !== inp) inp.value = p[inp.dataset.k]; });
      c.querySelector(".p-ik").innerHTML = ikHtml(ik);
      c.querySelector(".p-go").disabled = !ik.reachable;
    });
    const a = st.arm, lin = a.linear;
    $r("#p-loop").textContent = st.loop ? "Looping: press to stop" : "Loop between the points";
    $r("#p-run").textContent = st.loop ? (lin ? `Looping. Moving to (${lin.to.slice(0, 3).map((v) => Math.round(v)).join(", ")}): ${Math.round(lin.progress * 100)}%` : "Looping. Waiting for the arm to settle.")
      : lin ? `Moving to (${lin.to.slice(0, 3).map((v) => Math.round(v)).join(", ")}): ${Math.round(lin.progress * 100)}%` : "";
  };
  cards.forEach((c, i) => {
    c.querySelector(".p-on").addEventListener("change", (e) => act({ action: "enable", index: i, on: e.target.checked }));
    c.querySelectorAll(".p-f").forEach((inp) => inp.addEventListener("change", () => { if (inp.value !== "") act({ action: "set", index: i, [inp.dataset.k]: Number(inp.value) }); }));
    c.querySelector(".p-here").addEventListener("click", () => act({ action: "here", index: i }));
    c.querySelector(".p-go").addEventListener("click", () => act({ action: "go", index: i }));
  });
  $r("#p-loop").addEventListener("click", () => act({ action: "loop", on: !(last && last.loop) }));
  $r("#p-stop").addEventListener("click", () => act({ action: "stop" }));
  root.querySelectorAll("[data-view]").forEach((b) => b.addEventListener("click", () => viewer.view(b.dataset.view)));
  $r("#v-clr").addEventListener("click", () => viewer.clearTrail());
  const poll = async () => {
    if (dead || !document.body.contains($r("#p-loop"))) return;
    try { apply(await (await fetch("/api/points")).json()); } catch (e) { /* hub not reachable */ }
    setTimeout(poll, 700);
  };
  poll();
  const upd = {
    pose(p) {
      if (!p || !p.points) return;
      viewer.set(p);
      const m = $r("#p-mode");
      m.className = "st " + (p.attached ? "run" : "idle");
      m.textContent = p.attached ? MODE_TEXT[p.mode] || p.mode : "servos released";
      $r("#p-xyz").textContent = `tip x ${f1(p.xyz[0], 0)}  y ${f1(p.xyz[1], 0)}  z ${f1(p.xyz[2], 0)} mm`;
      $r("#p-ang").textContent = p.angles ? JOINTS.map((n, i) => `${n[0]} ${f1(p.angles[i], 0)}`).join("  ") + " deg" : "";
    },
    destroy() { dead = true; viewer.destroy(); },
  };
  upd.pose(S.pose || (S.st && S.st.sim));
  return upd;
}

function mountProject(root) {
  root.innerHTML = `<div id="p-body"><div class="skel" style="height:90px"></div></div>`;
  return {
    update() {
      const g = S.st && S.st.project; if (!g) return;
      if (!g.available) { $("#p-body", root).innerHTML = `<div class="err">Git is not available here, or this folder is not a repository.</div>`; return; }
      const repo = (g.url || "").replace(/^https:\/\/github\.com\//, "");
      $("#p-body", root).innerHTML = `
      <div class="strip">
        <div><div class="k">Repository</div><div class="v s">${esc(repo || "local")}</div><div class="sub">${g.url ? `<a href="${esc(g.url)}" target="_blank" rel="noopener">Open on GitHub</a>` : ""}</div></div>
        <div><div class="k">Branch</div><div class="v s">${esc(g.branch)}</div></div>
        <div><div class="k">Uncommitted files</div><div class="v">${g.dirty}</div><div class="sub">${g.dirty ? "work not saved to git yet" : "all committed"}</div></div>
        <div><div class="k">Not pushed</div><div class="v">${g.unpushed}</div><div class="sub">${g.unpushed ? "commits only on this PC" : "GitHub is up to date"}</div></div>
      </div>
      <div class="cols">
        <section class="sec" style="border-top:0"><h2>Recent commits</h2><div class="commits">${g.commits.map((c) => `<div><span class="h">${esc(c.h)}</span><span>${esc(c.s)}</span><span class="muted">${esc(c.when)}</span></div>`).join("") || '<div class="note">No commits.</div>'}</div></section>
        <section class="sec" style="border-top:0"><h2>Run the hub</h2>
          <p class="note">This dashboard is served by dashboard/hub.py. Start it from the repository folder:</p>
          <div class="raw" style="min-height:0;margin:10px 0">python dashboard/hub.py</div>
          <p class="note">It opens this page and starts in SAFE. The output must be armed here before anything reaches the arm.</p></section>
      </div>`;
    },
  };
}

/* ===================================================================== shell */
/* ===================================================================== the robot viewer (main view) + side panel */
function padFrame(root, st) {
  const k = (id) => root.querySelector("#p-" + id);
  const kn = (id, vx, vy) => { const n = k(id); n.setAttribute("cx", r3(vx * 28)); n.setAttribute("cy", r3(vy * 28)); };
  kn("lk", st.lx, st.ly); kn("rk", st.rx, st.ry);
  for (const b of BUTTONS) { const n = k(b); if (n) n.classList.toggle("on", !!st.buttons[b]); }
  k("ltf").setAttribute("width", r3(84 * st.lt)); k("rtf").setAttribute("width", r3(84 * st.rt));
  k("du").classList.toggle("on", st.dpad[1] === 1); k("dd").classList.toggle("on", st.dpad[1] === -1);
  k("dl").classList.toggle("on", st.dpad[0] === -1); k("dr").classList.toggle("on", st.dpad[0] === 1);
}
function wirePad(pad) {
  const setStick = (which, ev, g) => {
    const m = g.getScreenCTM(); if (!m) return;
    const p = new DOMPoint(ev.clientX, ev.clientY).matrixTransform(m.inverse());
    let vx = p.x / 50, vy = p.y / 50; const mag = Math.hypot(vx, vy); if (mag > 1) { vx /= mag; vy /= mag; }
    E.virt[which + "x"] = r3(vx); E.virt[which + "y"] = r3(vy);
  };
  pad.addEventListener("pointerdown", (ev) => {
    if (E.source !== "virtual") return;
    const st = ev.target.closest(".stick"), bt = ev.target.closest(".btnp");
    if (st) { try { st.setPointerCapture(ev.pointerId); } catch (e) { /* synthetic pointer */ } st._drag = true; setStick(st.dataset.s, ev, st); }
    else if (bt) {
      try { bt.setPointerCapture(ev.pointerId); } catch (e) { /* synthetic pointer */ }
      if (bt.dataset.b) E.virt.buttons[bt.dataset.b] = true; else E.virt[bt.dataset.t] = 1;
    }
  });
  pad.addEventListener("pointermove", (ev) => { const st = ev.target.closest(".stick"); if (E.source === "virtual" && st && st._drag) setStick(st.dataset.s, ev, st); });
  const release = (ev) => {
    if (E.source !== "virtual") return;
    const st = ev.target.closest(".stick"), bt = ev.target.closest(".btnp");
    if (st) { st._drag = false; E.virt[st.dataset.s + "x"] = 0; E.virt[st.dataset.s + "y"] = 0; }
    if (bt) { if (bt.dataset.b) E.virt.buttons[bt.dataset.b] = false; else E.virt[bt.dataset.t] = 0; }
  };
  pad.addEventListener("pointerup", release); pad.addEventListener("pointercancel", release);
}

/* A small 3D viewer on a canvas: orbit with one finger or the mouse, zoom with a pinch or the wheel, double tap to reset.
   It draws the pose the hub streams (arm links, joints, tool, the path the tool has taken, the reach circle). */
function makeViewer(canvas) {
  const ctx = canvas.getContext("2d");
  const V0 = { yaw: 0.75, pitch: 0.5, dist: 1150 };
  const v = Object.assign({}, V0), T = [150, 150, 140];
  let W = 0, H = 0, dpr = 1, pose = null, dirty = true, col = null, colT = 0, trail = [], reach = 330;
  const pointers = new Map();
  const colors = () => {
    const now = performance.now();
    if (!col || now - colT > 1000) {
      const cs = getComputedStyle(document.documentElement), g = (n) => cs.getPropertyValue(n).trim();
      col = { bg: g("--bg"), line: g("--line"), strong: g("--line-strong"), text: g("--text"), accent: g("--accent"), warn: g("--warn"), surf: g("--surface-2"), muted: g("--muted") };
      colT = now;
    }
    return col;
  };
  const resize = () => {
    const r = canvas.getBoundingClientRect();
    dpr = Math.min(2, window.devicePixelRatio || 1);
    W = Math.max(1, Math.round(r.width)); H = Math.max(1, Math.round(r.height));
    canvas.width = W * dpr; canvas.height = H * dpr; dirty = true;
  };
  const ro = new ResizeObserver(resize); ro.observe(canvas);
  const proj = (p) => {
    const cy = Math.cos(v.yaw), sy = Math.sin(v.yaw), cp = Math.cos(v.pitch), sp = Math.sin(v.pitch);
    const x = p[0] - T[0], y = p[1] - T[1], z = p[2] - T[2];
    const x1 = x * cy - y * sy, y1 = x * sy + y * cy;
    const depth = y1 * cp - z * sp + v.dist, up = y1 * sp + z * cp;
    const k = (Math.min(W, H * 1.25) * 1.15) / depth;
    return [W / 2 + x1 * k, H / 2 - up * k, depth, k];
  };
  const poly = (pts, close) => { ctx.beginPath(); pts.forEach((q, i) => (i ? ctx.lineTo(q[0], q[1]) : ctx.moveTo(q[0], q[1]))); if (close) ctx.closePath(); };
  function draw() {
    dirty = false;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    const c = colors();
    ctx.clearRect(0, 0, W, H);
    ctx.lineCap = "round"; ctx.lineJoin = "round";
    // ground grid
    ctx.strokeStyle = c.line; ctx.lineWidth = 1;
    for (let g = -500; g <= 500; g += 100) {
      poly([proj([g, -500, 0]), proj([g, 500, 0])]); ctx.stroke();
      poly([proj([-500, g, 0]), proj([500, g, 0])]); ctx.stroke();
    }
    ctx.strokeStyle = c.strong; ctx.setLineDash([6, 6]);
    const ring = []; for (let a = 0; a < 64; a++) ring.push(proj([reach * Math.cos((a / 64) * 6.2832), reach * Math.sin((a / 64) * 6.2832), 0]));
    poly(ring, true); ctx.stroke(); ctx.setLineDash([]);
    // base
    const disc = (r, z) => { const a = []; for (let i = 0; i < 28; i++) a.push(proj([r * Math.cos((i / 28) * 6.2832), r * Math.sin((i / 28) * 6.2832), z])); return a; };
    ctx.fillStyle = c.surf; ctx.strokeStyle = c.strong; ctx.lineWidth = 1.5;
    poly(disc(60, 0), true); ctx.fill(); ctx.stroke();
    if (!pose) return;
    const P = pose.points;
    // shadow of the arm on the ground
    ctx.strokeStyle = c.strong; ctx.globalAlpha = 0.5; ctx.lineWidth = 5;
    poly(P.map((q) => proj([q[0], q[1], 0]))); ctx.stroke(); ctx.globalAlpha = 1;
    // path of the tool
    if (trail.length > 1) {
      for (let i = 1; i < trail.length; i++) {
        ctx.globalAlpha = (i / trail.length) * 0.8; ctx.strokeStyle = c.warn; ctx.lineWidth = 2;
        poly([proj(trail[i - 1]), proj(trail[i])]); ctx.stroke();
      }
      ctx.globalAlpha = 1;
    }
    // the arm as solid parts (Lynxmotion AL5D style), painted far to near
    const g = [Math.sin(v.yaw) * Math.cos(v.pitch), Math.cos(v.yaw) * Math.cos(v.pitch), -Math.sin(v.pitch)];   // points away from the camera
    const items = [];
    for (const f of al5dParts(pose)) {
      if (f.sphere) {
        const q = proj(f.sphere), r = f.r * q[3];
        items.push({ d: q[2], f: () => {
          const gr = ctx.createRadialGradient(q[0] - r * 0.35, q[1] - r * 0.35, r * 0.1, q[0], q[1], r);
          gr.addColorStop(0, "#fff3d0"); gr.addColorStop(1, c.warn);
          ctx.beginPath(); ctx.arc(q[0], q[1], r, 0, 6.2832); ctx.fillStyle = gr; ctx.fill(); ctx.strokeStyle = "rgba(0,0,0,.45)"; ctx.lineWidth = 1; ctx.stroke();
        } });
        continue;
      }
      if (f.n[0] * g[0] + f.n[1] * g[1] + f.n[2] * g[2] >= 0) continue;      // faces turned away
      const q = f.pts.map(proj), d = q.reduce((t, x) => t + x[2], 0) / q.length;
      const lit = 0.5 + 0.5 * Math.max(0, f.n[0] * -0.35 + f.n[1] * -0.3 + f.n[2] * 0.89);
      items.push({ d, f: () => {
        ctx.fillStyle = shade(f.color, lit); poly(q, true); ctx.fill();
        ctx.strokeStyle = "rgba(0,0,0,.35)"; ctx.lineWidth = 1; ctx.stroke();
      } });
    }
    items.sort((p, q) => q.d - p.d).forEach((it) => it.f());
    if (pose.world) {
      const w = pose.world;
      ctx.font = "600 13px system-ui, sans-serif"; ctx.textAlign = "center"; ctx.fillStyle = c.text;
      w.boxes.forEach(([cx, cy], i) => { const q = proj([cx, cy, w.box_height + 26]); ctx.fillText("Box " + (i + 1), q[0], q[1]); });
      const q = proj([w.tray[0], w.tray[1], w.tray_top + 26]); ctx.fillText("Tray", q[0], q[1]);
    }
  }
  const loop = () => { if (dirty) draw(); requestAnimationFrame(loop); };
  requestAnimationFrame(loop);
  const centroid = () => { let x = 0, y = 0; pointers.forEach((p) => { x += p.x; y += p.y; }); return [x / pointers.size, y / pointers.size]; };
  const spread = () => { const a = [...pointers.values()]; return a.length < 2 ? 0 : Math.hypot(a[0].x - a[1].x, a[0].y - a[1].y); };
  let lastC = null, lastS = 0;
  canvas.addEventListener("pointerdown", (e) => { try { canvas.setPointerCapture(e.pointerId); } catch (x) { /* synthetic */ } pointers.set(e.pointerId, { x: e.clientX, y: e.clientY }); lastC = centroid(); lastS = spread(); });
  canvas.addEventListener("pointermove", (e) => {
    if (!pointers.has(e.pointerId)) return;
    pointers.set(e.pointerId, { x: e.clientX, y: e.clientY });
    const c = centroid(), s = spread();
    if (lastC) { v.yaw -= (c[0] - lastC[0]) * 0.008; v.pitch = Math.max(-0.05, Math.min(1.5, v.pitch + (c[1] - lastC[1]) * 0.008)); }
    if (pointers.size > 1 && lastS > 0 && s > 0) v.dist = Math.max(450, Math.min(2600, v.dist * (lastS / s)));
    lastC = c; lastS = s; dirty = true;
  });
  const up = (e) => { pointers.delete(e.pointerId); lastC = pointers.size ? centroid() : null; lastS = spread(); };
  canvas.addEventListener("pointerup", up); canvas.addEventListener("pointercancel", up);
  canvas.addEventListener("wheel", (e) => { e.preventDefault(); v.dist = Math.max(450, Math.min(2600, v.dist * Math.exp(e.deltaY * 0.001))); dirty = true; }, { passive: false });
  canvas.addEventListener("dblclick", () => { Object.assign(v, V0); dirty = true; });
  return {
    set(p) {
      pose = p; reach = 145.95 + 183.7 + (p.tool || 0);
      const tip = p.points[4], last = trail[trail.length - 1];
      if (!last || Math.hypot(tip[0] - last[0], tip[1] - last[1], tip[2] - last[2]) > 3) { trail.push(tip.slice()); if (trail.length > 160) trail.shift(); }
      dirty = true;
    },
    view(name) {
      const az = pose ? ((pose.angles[0] - 90) * Math.PI) / 180 : 0;      // turn the view with the arm's base angle
      Object.assign(v, name === "top" ? { yaw: az, pitch: 1.5, dist: 1250 } : name === "side" ? { yaw: az - Math.PI / 2, pitch: 0.02, dist: 1150 } : V0); dirty = true;
    },
    clearTrail() { trail = []; dirty = true; },
    destroy() { ro.disconnect(); },
  };
}
/* ---- Lynxmotion AL5D-style solid model, simplified from the published dimensions (no CAD). The last joint and gripper are removed:
   the wrist ends in a short rod and the test ball. Drawn as boxes and prisms: arrays of {pts, n, color} faces plus one sphere. ---- */
const shade = (hex, k) => { const n = parseInt(hex.slice(1), 16); return `rgb(${Math.round(((n >> 16) & 255) * k)},${Math.round(((n >> 8) & 255) * k)},${Math.round((n & 255) * k)})`; };
const vadd = (a, b, s = 1) => [a[0] + b[0] * s, a[1] + b[1] * s, a[2] + b[2] * s];
const vsub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const vlen = (a) => Math.hypot(a[0], a[1], a[2]);
const vnorm = (a) => { const m = vlen(a) || 1; return [a[0] / m, a[1] / m, a[2] / m]; };
const vcross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
function boxFaces(out, c, ax, h, color) {         // c centre, ax three unit axes, h half sizes
  for (let i = 0; i < 3; i++) for (const sg of [-1, 1]) {
    const j = (i + 1) % 3, k = (i + 2) % 3, mid = vadd(c, ax[i], sg * h[i]);
    const corner = (a, b) => vadd(vadd(mid, ax[j], a * h[j]), ax[k], b * h[k]);
    out.push({ pts: [corner(-1, -1), corner(1, -1), corner(1, 1), corner(-1, 1)], n: ax[i].map((x) => x * sg), color });
  }
}
function prismFaces(out, c, r, h, ax, color, sides = 22) {     // upright cylinder: axis ax[2] is its length axis
  const ring = (z) => { const a = []; for (let i = 0; i < sides; i++) { const t = (i / sides) * 6.2832; a.push(vadd(vadd(vadd(c, ax[0], r * Math.cos(t)), ax[1], r * Math.sin(t)), ax[2], z)); } return a; };
  const lo = ring(-h), hi = ring(h);
  out.push({ pts: hi, n: ax[2], color }); out.push({ pts: lo.slice().reverse(), n: ax[2].map((x) => -x), color });
  for (let i = 0; i < sides; i++) {
    const j = (i + 1) % sides, t = ((i + 0.5) / sides) * 6.2832;
    out.push({ pts: [lo[i], lo[j], hi[j], hi[i]], n: vadd(ax[0].map((x) => x * Math.cos(t)), ax[1], Math.sin(t)), color });
  }
}
function al5dParts(p) {
  const out = [], P = p.points, base = ((p.angles[0] - 90) * Math.PI) / 180;
  const d = [Math.sin(base), Math.cos(base), 0], l = [Math.cos(base), -Math.sin(base), 0], up = [0, 0, 1];
  const W = [[1, 0, 0], [0, 1, 0], [0, 0, 1]], AL = "#b4bcc8", AL2 = "#9aa3b0", SV = "#2b313b", BLK = "#1d2128";
  boxFaces(out, [0, 0, 3], W, [70, 52, 3], AL2);                                   // base plate
  prismFaces(out, [0, 0, 22], 40, 16, [d, l, up], BLK);                            // turret
  boxFaces(out, vadd([0, 0, 24], d, 36), [d, l, up], [8, 7, 8], AL);               // marker showing the base direction
  for (const sg of [-1, 1]) boxFaces(out, vadd([0, 0, 68], l, 24 * sg), [d, l, up], [28, 2, 30], AL);   // shoulder bracket plates
  boxFaces(out, vadd([0, 0, 72], l, 36), [d, l, up], [26, 12, 24], SV);            // shoulder servo
  const u1 = vnorm(vsub(P[2], P[1])), u2 = vnorm(vsub(P[3], P[2])), u3 = vnorm(vsub(P[4], P[3]));
  const frame = (u) => [u, l, vnorm(vcross(u, l))];
  const L1 = vlen(vsub(P[2], P[1])), L2 = vlen(vsub(P[3], P[2])), L3 = vlen(vsub(P[4], P[3]));
  for (const sg of [-1, 1]) {
    boxFaces(out, vadd(vadd(P[1], P[2]), [0, 0, 0]).map((x) => x / 2).map((x, i) => x + l[i] * 15 * sg), frame(u1), [L1 / 2, 2, 8], AL);   // upper arm bars
    boxFaces(out, vadd(vadd(P[2], P[3]), [0, 0, 0]).map((x) => x / 2).map((x, i) => x + l[i] * 12 * sg), frame(u2), [L2 / 2, 2, 7], AL);   // forearm bars
  }
  boxFaces(out, vadd(vadd(P[2], u1, -12), l, -28), frame(u1), [24, 12, 20], SV);   // elbow servo
  boxFaces(out, vadd(vadd(P[3], u2, -12), l, 16), frame(u2), [17, 8, 14], SV);    // wrist servo
  const on = !!(p.world && p.world.magnet), rodLen = Math.max(2, L3 - 22);
  boxFaces(out, vadd(P[3], u3, rodLen / 2), frame(u3), [rodLen / 2, 4, 4], AL);   // rod to the electromagnet
  const fr = frame(u3), mag = [fr[1], vnorm(vcross(u3, fr[1])), u3];
  prismFaces(out, vadd(P[4], u3, -11), 12, 11, mag, "#b87333");                    // the electromagnet: copper coil
  prismFaces(out, vadd(P[4], u3, 0.5), 12.5, 1.5, mag, on ? "#5ec8ff" : "#4a4f58");   // its face: lit when energised
  if (p.world) out.push(...worldParts(p.world));
  return out;
}

/* ---- the work cell: three boxes with a key each, a tray, drawn from the world the hub streams ---- */
function worldParts(w) {
  const out = [], W = [[1, 0, 0], [0, 1, 0], [0, 0, 1]], CARD = "#a8825a", CARD2 = "#8d6c47", TRAY = "#56657a", TRAY2 = "#6e7f96";
  const h = w.box_inner, t = w.box_wall, fl = w.box_floor, ht = w.box_height;
  for (const [cx, cy] of w.boxes) {
    boxFaces(out, [cx, cy, fl / 2], W, [h + t, h + t, fl / 2], CARD2);
    for (const sg of [-1, 1]) {
      boxFaces(out, [cx, cy + sg * (h + t / 2), ht / 2], W, [h + t, t / 2, ht / 2], CARD);
      boxFaces(out, [cx + sg * (h + t / 2), cy, ht / 2], W, [t / 2, h, ht / 2], CARD);
    }
  }
  const [tx, ty] = w.tray, [hx, hy] = w.tray_half, tt = w.tray_top;
  boxFaces(out, [tx, ty, tt / 2], W, [hx, hy, tt / 2], TRAY);
  for (const sg of [-1, 1]) {
    boxFaces(out, [tx, ty + sg * (hy - 1.5), tt + 3], W, [hx, 1.5, 3], TRAY2);
    boxFaces(out, [tx + sg * (hx - 1.5), ty, tt + 3], W, [1.5, hy - 3, 3], TRAY2);
  }
  for (const k of w.keys) {
    const y = (k.yaw * Math.PI) / 180, a = [Math.cos(y), Math.sin(y), 0], b = [-Math.sin(y), Math.cos(y), 0], up = [0, 0, 1];
    const col = w.held === k.id ? "#ffd666" : "#d8b04a";
    boxFaces(out, vadd(k.pos, a, 6), [a, b, up], [15, 3.5, 1.5], col);                  // blade
    boxFaces(out, vadd(k.pos, a, 13), [a, b, up], [2.5, 6, 1.5], col);                  // teeth
    prismFaces(out, vadd(k.pos, a, -14), 8, 1.5, [a, b, up], col, 14);                  // bow
  }
  return out;
}

function mountArm(root) {
  document.body.classList.add("fill");
  root.innerHTML = `<div class="armstage">
    <section class="viewer" aria-label="Robot viewer">
      <canvas id="v3d" role="img" aria-label="3D view of the robot arm. Drag to turn, pinch or scroll to zoom, double tap to reset."></canvas>
      <div class="vhud tl"><div class="st idle" id="v-mode">-</div><div class="mono" id="v-xyz"></div><div class="note" id="v-status"></div><div class="note mono" id="v-saved"></div><div class="note mono" id="v-mag"></div></div>
      <div class="vhud tr"><button class="btn sm" type="button" data-view="iso">3D</button><button class="btn sm" type="button" data-view="top">Top</button><button class="btn sm" type="button" data-view="side">Side</button><button class="btn sm" type="button" id="v-clr">Clear path</button></div>
      <div class="vhud bl" id="v-joints"></div>
    </section>
    <aside class="side" aria-label="Controls and status">
      <div class="tabs" role="tablist" id="c-tabs">${SOURCES.map((s) => `<button type="button" role="tab" data-src="${s[0]}" aria-selected="${E.source === s[0]}">${s[1].replace("Browser ", "").replace(" stream", "")}</button>`).join("")}</div>
      <div class="inline" style="margin:8px 0"><span class="st idle" id="c-send" role="status"></span></div>
      <div id="c-pad">${padSVG()}</div>
      <div class="note" id="c-desc" style="margin:6px 0 10px"></div>
      <div class="inline"><button class="btn sm" id="a-home" type="button">${icon("arrows-clockwise", "sm")}Go home</button>
        <button class="btn sm" id="a-play" type="button" aria-pressed="false">Play saved</button>
        <select id="a-model" aria-label="Firmware to simulate"><option value="stock-patch">Ours (stock + patch)</option><option value="stock">Stock, as shipped</option><option value="improved">Reference</option></select></div>
      <details class="sidesec legend" open><summary>What the controls do</summary>
        <dl>
          <dt>Left stick</dt><dd>Moves the tool sideways (x) and in or out (y), about 20 mm/s at full tilt.</dd>
          <dt>Right stick up / down</dt><dd>Moves the tool up and down (z).</dd>
          <dt>Right stick left / right</dt><dd>Tilts the tool angle, about 20 degrees/s.</dd>
          <dt>A (tap)</dt><dd>Saves the current position and whether the simulated magnet is on. Up to 20 are kept; a 21st is ignored. In a replay the magnet is set to the saved state when the arm gets there.</dd>
          <dt>Y, or the Play saved button</dt><dd>Starts the replay and keeps it going with nothing held; press again to stop. Simulation only: the real firmware only replays while A is held.</dd>
          <dt>A (hold 0.75 s)</dt><dd>Replays the saved positions in order, one every 1.5 s, looping. Let go of A to stop. The stock firmware also saves one extra position (where the arm was) every time you start a replay; the reference model and the patch undo that save.</dd>
          <dt>B</dt><dd>Go home: every joint returns to its home angle, then the servos are released.</dd>
          <dt>X</dt><dd>Switches the simulated electromagnet on or off (each press toggles it). The real firmware ignores X.</dd>
          <dt>Triggers, bumpers, d-pad</dt><dd>Not used.</dd>
          <dt>Hold A while the ESP32 starts</dt><dd>Calibration mode: the right stick moves one joint, A records its minimum, maximum and home in turn, for all four joints.</dd>
        </dl>
        <p class="note">Sticks have a 20% dead zone. Saved positions live in the ESP32's memory only: they are lost when it restarts.</p></details>
      <div class="sidesec"><h2>Electromagnet and keys</h2>
        <div class="inline"><button class="btn sm" id="w-mag" type="button" aria-pressed="false">Magnet off</button><button class="btn sm" id="w-reset" type="button">Reset keys</button></div>
        <div class="inline" style="margin-top:8px"><input id="q-ask" placeholder="Ask the arm: put the key on the tray" maxlength="300" aria-label="Ask the arm" style="flex:1"><button class="btn sm" id="q-go" type="button">Do it</button></div>
        <div class="note"><span class="st idle" id="q-ai" role="status">AI: checking</span></div>
        <div class="note" id="q-log" role="status"></div>
        <details class="sidesec" id="s-lib"><summary>Sequences</summary>
          <div class="inline" style="margin-top:6px"><input id="s-name" placeholder="name, e.g. box2_to_tray" maxlength="48" aria-label="Sequence name"><button class="btn sm" id="s-send" type="button">Save to host</button></div>
          <input id="s-desc" placeholder="what it does" maxlength="400" aria-label="Sequence description" style="width:100%;margin-top:4px">
          <div class="inline" style="margin-top:4px"><input id="s-req" placeholder="needs key in (box2)" maxlength="20" aria-label="Key must start in" size="14"><input id="s-end" placeholder="leaves key on (tray)" maxlength="20" aria-label="Key ends on" size="14"></div>
          <div id="s-list" class="seqlist" role="list" aria-label="Saved sequences"></div>
          <div class="inline" style="margin-top:6px"><button class="btn sm" id="s-copy" type="button">Copy JSON</button><button class="btn sm" id="s-load" type="button">Load JSON</button></div>
          <div class="note">Saved positions become poses; the magnet is stored as grab / release so a sequence can be run backwards. Files go in sequences/ on the host.</div>
        </details>
        <div class="note" style="margin-top:6px">Hold the tip within 2 cm of a key and switch the magnet on to pick it up. Switch it off to let go: it falls onto whatever is below. The X button does the same.</div>
        <div class="note mono" id="w-keys"></div></div>
      <div class="sidesec"><h2>Emulated ESP32</h2>
        <div class="inline"><select id="m-fw" aria-label="Firmware"><option value="ours">Ours (stock + patches)</option><option value="patched">Stock + damper patch only</option><option value="stock">Stock, as shipped</option><option value="rarm">Rewrite</option></select>
          <select id="m-net" aria-label="Network"></select><button class="btn sm" id="m-go" type="button">Start</button></div>
        <div class="inline" style="margin-top:8px"><span class="st idle" id="m-state">Off</span><button class="btn sm" id="m-drop" type="button" hidden>Drop link 3 s</button></div>
        <div class="note" id="m-net-s"></div><div class="note" id="m-err" style="color:var(--bad)"></div></div>
      <div class="sidesec"><h2>Input</h2><div class="note mono" id="s-in"></div></div>
      <div class="sidesec" id="s-phone" hidden><h2>On your phone</h2><div class="note">Same WiFi. Open this once, then Add to Home Screen.</div>
        <div class="inline"><input id="s-url" readonly aria-label="Phone link" style="flex:1;min-width:0"><button class="btn sm" id="s-copy" type="button">Copy</button></div></div>
      <a class="note" href="#/control">More: calibration, damper, network details</a>
    </aside></div>`;
  const $r = (q) => root.querySelector(q);
  const viewer = makeViewer($r("#v3d"));
  const pad = $r("#pad"); wirePad(pad);
  $r("#c-tabs").addEventListener("click", (e) => { const b = e.target.closest("[data-src]"); if (b) setSource(b.dataset.src); });
  root.querySelectorAll("[data-view]").forEach((b) => b.addEventListener("click", () => viewer.view(b.dataset.view)));
  $r("#v-clr").addEventListener("click", () => viewer.clearTrail());
  $r("#a-home").addEventListener("click", async () => { const r = await api("/api/sim", { home: true }); if (!r.ok) toast(r.error || "The hub is not reachable", "bad"); });
  $r("#a-play").addEventListener("click", async () => { const r = await api("/api/sim", { replay: "toggle" }); if (!r.ok) toast(r.error || "The hub is not reachable", "bad"); });
  const model = $r("#a-model");
  model.addEventListener("change", async () => { const v = model.value; await api("/api/sim", { model: v === "improved" ? "improved" : "stock", patch: v === "stock-patch" }); });
  const askIt = async () => { const t = $r("#q-ask").value.trim(); if (!t) return;
    const r = await api("/api/ask", { request: t }); if (!r.ok) toast(r.error || "The hub is not reachable", "bad"); };
  $r("#q-go").addEventListener("click", askIt);
  $r("#q-ask").addEventListener("keydown", (e) => { if (e.key === "Enter") askIt(); });
  const showTask = async () => { try {
    const s = await (await fetch("/api/state")).json(), t = s.task || {}, ai = $r("#q-ai");
    if (s.ai === "groq") { ai.className = "st ok"; ai.textContent = "AI: Groq"; }
    else { ai.className = "st bad"; ai.textContent = "AI: keywords only (no GROQ_API_KEY in .env)"; }
    const lg = $r("#q-log"), e = t.error;
    if (e) { lg.innerHTML = `<div class="taskerr" role="alert" style="border-left:3px solid var(--bad);padding:6px 8px;margin-top:4px"><b style="color:var(--bad)">${esc(e.title)}</b>${e.detail ? `<div>${esc(e.detail)}</div>` : ""}${e.hint ? `<div class="muted">${esc(e.hint)}</div>` : ""}<div class="muted mono">${esc(e.code)}</div></div>`; }
    else lg.textContent = t.log && t.log.length ? `${t.state}: ${t.log[t.log.length - 1]}` : "";
  } catch (e) { /* hub not reachable */ } };
  showTask();
  const taskTimer = setInterval(() => { if (!document.body.contains($r("#q-log"))) return clearInterval(taskTimer); showTask(); }, 1500);
  $r("#w-mag").addEventListener("click", async () => { const r = await api("/api/world", { magnet: "toggle" }); if (!r.ok) toast(r.error || "The hub is not reachable", "bad"); });
  const refreshSeqs = async () => {
    try {
      const j = await (await fetch("/api/sequences")).json(); const list = $r("#s-list");
      list.innerHTML = (j.sequences || []).map((q) => `<div class="seqrow" role="listitem" data-name="${esc(q.name)}">
        <div class="seqname"><b>${esc(q.name)}</b> <span class="muted">${q.count} poses</span><div class="muted">${esc(q.description || "")}</div></div>
        <div class="inline"><button class="btn sm" data-act="run" type="button">Run</button><button class="btn sm" data-act="rev" type="button">Reverse</button>
        <button class="btn sm" data-act="ren" type="button" aria-label="Rename ${esc(q.name)}">Rename</button><button class="btn sm danger" data-act="del" type="button" aria-label="Delete ${esc(q.name)}">Delete</button></div></div>`).join("") || '<div class="muted">No sequences yet. Save the current positions above.</div>';
    } catch (e) { /* hub not reachable */ }
  };
  refreshSeqs();
  $r("#s-list").addEventListener("click", async (e) => {
    const btn = e.target.closest("button[data-act]"); if (!btn) return;
    const name = btn.closest(".seqrow").dataset.name, act = btn.dataset.act;
    if (act === "run" || act === "rev") {
      const r = await api("/api/sequence", { name, use: true, run: true, reverse: act === "rev" });
      toast(r.ok ? `Running ${r.name}${act === "rev" ? " reversed" : ""}` : (r.error || "The hub is not reachable"), r.ok ? "ok" : "bad");
    } else if (act === "del") {
      if (!window.confirm(`Delete the sequence "${name}"?`)) return;
      const r = await api("/api/sequence", { delete: name }); toast(r.ok ? `Deleted ${name}` : (r.error || "Could not delete"), r.ok ? "ok" : "bad"); refreshSeqs();
    } else if (act === "ren") {
      const to = (window.prompt(`New name for "${name}"`, name) || "").trim(); if (!to || to === name) return;
      const r = await api("/api/sequence", { rename: { from: name, to } }); toast(r.ok ? `Renamed to ${to}` : (r.error || "Could not rename"), r.ok ? "ok" : "bad"); refreshSeqs();
    }
  });
  $r("#s-send").addEventListener("click", async () => {
    const r = await api("/api/sequence", { name: $r("#s-name").value.trim(), description: $r("#s-desc").value.trim(),
      requires: $r("#s-req").value.trim() ? { key_in: $r("#s-req").value.trim() } : {}, ends: $r("#s-end").value.trim() ? { key_on: $r("#s-end").value.trim() } : {} });
    toast(r.ok ? `Saved ${r.packet.name} (${r.packet.count} poses) on the host: ${r.path}` : (r.error || "The hub is not reachable"), r.ok ? "ok" : "bad"); refreshSeqs(); });
  $r("#s-copy").addEventListener("click", async () => {
    try { const j = JSON.stringify(await (await fetch("/api/sequence")).json(), null, 2); await navigator.clipboard.writeText(j); toast("Sequence JSON copied", "ok"); }
    catch (e) { toast("Could not copy; use Send sequence to host", "bad"); } });
  $r("#s-load").addEventListener("click", async () => {
    const t = window.prompt("Paste a sequence JSON packet"); if (!t) return;
    let pkt; try { pkt = JSON.parse(t); } catch (e) { return toast("That is not valid JSON", "bad"); }
    const r = await api("/api/sequence", { load: pkt }); toast(r.ok ? `Loaded ${r.loaded} positions` : (r.error || "The hub is not reachable"), r.ok ? "ok" : "bad"); });
  $r("#w-reset").addEventListener("click", () => api("/api/world", { reset: true }));
  $r("#s-copy").addEventListener("click", async () => { try { await navigator.clipboard.writeText($r("#s-url").value); toast("Link copied", "ok"); } catch (e) { $r("#s-url").select(); } });
  const emu = async (b) => { const r = await api("/api/emu", b); if (!r.ok) toast(r.error || "The hub is not reachable", "bad"); };
  const fw = $r("#m-fw"), net = $r("#m-net"), go = $r("#m-go");
  go.addEventListener("click", () => { const e = S.st && S.st.emu; (e && (e.state === "running" || e.state === "building")) ? emu({ action: "stop" }) : emu({ action: "start", firmware: fw.value, preset: net.value }); });
  $r("#m-drop").addEventListener("click", () => emu({ action: "outage", path: "esp", seconds: 3 }));
  const joints = $r("#v-joints");
  joints.innerHTML = JOINT_NAMES.map((n, i) => `<div class="jr"><span>${n}</span><span class="jb"><i id="vj-b${i}"></i></span><span class="mono" id="vj-a${i}">-</span></div>`).join("");
  const upd = {
    sourceChanged() {
      root.querySelectorAll("#c-tabs button").forEach((b) => b.setAttribute("aria-selected", b.dataset.src === E.source));
      pad.classList.toggle("virtual", E.source === "virtual");
      $r("#c-desc").textContent = SOURCES.find((s) => s[0] === E.source)[2];
    },
    update() {
      const st = S.st; if (!st) return;
      const el = $r("#c-send");
      if (E.source === "server") { el.className = "st idle"; el.textContent = "Not sending from this page"; }
      else if (!st.armed && st.emu_input) { el.className = "st ok"; el.textContent = E.sending ? "Sending to the emulated ESP32" : "Emulator ready"; }
      else if (!st.armed) { el.className = "st warn"; el.textContent = "Virtual arm only: SAFE"; }
      else if (E.sending) { el.className = "st bad"; el.textContent = "Sending to the arm"; }
      else { el.className = "st warn"; el.textContent = "ARMED, waiting"; }
      const sm = st.sim;
      if (sm && document.activeElement !== model) model.value = sm.model === "stock" ? (sm.patch ? "stock-patch" : "stock") : "improved";
      const e = st.emu, busy = e && (e.state === "running" || e.state === "building");
      if (e && !net.options.length) { net.innerHTML = Object.keys(e.presets).map((n) => `<option>${n}</option>`).join(""); net.value = "home_wifi"; }
      go.textContent = busy ? "Stop" : "Start"; fw.disabled = net.disabled = !!busy;
      const ms = $r("#m-state"); ms.className = "st " + (!e || e.state === "off" ? "idle" : e.state === "error" ? "bad" : e.state === "building" ? "warn" : "ok");
      ms.textContent = !e ? "Off" : { off: "Off", building: "Compiling...", running: "Running " + e.firmware, error: "Stopped" }[e.state];
      $r("#m-drop").hidden = !(e && e.state === "running");
      $r("#m-err").textContent = e ? e.error : "";
      $r("#m-net-s").textContent = e && e.state === "running" && e.net ? `poll ${f1(e.net.poll_ms_mean, 0)} ms avg, worst stall ${f1(e.net.loop_gap_ms_max, 0)} ms, ${e.net.poll_fails} failed` : "";
      const age = st.last_packet_age;
      $r("#s-in").textContent = `${f1(st.pps, 0)} packets/s, ${age == null ? "none yet" : "last " + f1(age, 1) + " s ago"}`;
      $r("#s-phone").hidden = !st.phone_url;
      if (st.phone_url && $r("#s-url").value !== st.phone_url) $r("#s-url").value = st.phone_url;
    },
    frame(st) { padFrame(root, st); },
    pose(p) {
      if (!p || !p.points) return;
      viewer.set(p);
      const m = $r("#v-mode");
      m.className = "st " + (p.emulated ? "warn" : p.attached ? "run" : "idle");
      m.textContent = p.emulated ? "Emulated ESP32" : p.attached ? MODE_TEXT[p.mode] || p.mode : "servos released";
      $r("#v-xyz").textContent = `x ${f1(p.xyz[0], 0)}  y ${f1(p.xyz[1], 0)}  z ${f1(p.xyz[2], 0)} mm`;
      $r("#v-status").textContent = p.ik_ok ? p.status : "out of reach: holding position";
      $r("#v-saved").textContent = p.emulated ? "" : `saved positions: ${p.saved}/20${p.mode === "replay" ? ", replaying #" + (p.replay_idx + 1) : ""}${p.world && p.world.saved_magnet.some(Boolean) ? "  (magnet on at " + p.world.saved_magnet.map((m, i) => (m ? "#" + (i + 1) : "")).filter(Boolean).join(", ") + ")" : ""}`;
      const pb = $r("#a-play"); pb.textContent = p.replaying ? "Stop replay" : "Play saved"; pb.setAttribute("aria-pressed", String(!!p.replaying));
      const w = p.world;
      if (w) {
        const mb = $r("#w-mag"); mb.textContent = w.magnet ? "Magnet ON" : "Magnet off"; mb.setAttribute("aria-pressed", String(w.magnet));
        $r("#v-mag").textContent = `magnet ${w.magnet ? "ON" : "off"}${w.held ? ", holding key " + w.held : ""}`;
        $r("#w-keys").textContent = w.keys.map((k) => `Key ${k.id}: ${k.where}`).join("\n");
      }
      for (let i = 0; i < 4; i++) {
        const [lo, hi] = p.limits[i], a = Math.min(lo, hi), b = Math.max(lo, hi);
        $r("#vj-a" + i).textContent = f1(p.angles[i], 0) + "°";
        $r("#vj-b" + i).style.width = Math.max(0, Math.min(100, ((p.angles[i] - a) / Math.max(1, b - a)) * 100)) + "%";
      }
    },
    destroy() { viewer.destroy(); },
  };
  upd.sourceChanged();
  upd.pose(S.pose || (S.st && S.st.sim));
  return upd;
}

const PAGES = [
  { id: "arm", group: "", title: "Robot", sub: "The arm, live", icon: "robot", mount: mountArm },
  { id: "points", group: "", title: "Points", sub: "Move between fixed points and watch the inverse kinematics solve them", icon: "pin", mount: mountPoints },
  { id: "chat", group: "", title: "Chat", sub: "Talk to the arm: what you asked, what the AI answered, what the arm did", icon: "chat", mount: mountChat },
  { id: "overview", group: "Monitor", title: "Overview", sub: "Everything at a glance", icon: "squares-four", mount: mountOverview },
  { id: "control", group: "", title: "Control", sub: "Drive from the browser and watch the virtual arm", icon: "game-controller", mount: mountControl },
  { id: "packets", group: "", title: "Packets", sub: "Every controller packet, exactly as sent", icon: "pulse", mount: mountPackets },
  { id: "board", group: "Hardware", title: "Arm and board", sub: "The ESP32 over USB: log, settings and outputs", icon: "cpu", mount: mountBoard },
  { id: "firmware", group: "", title: "Firmware", sub: "Compile and upload the ESP32 sketch", icon: "upload-simple", mount: mountFirmware },
  { id: "services", group: "System", title: "Services", sub: "Start and stop what the system runs on", icon: "plugs-connected", mount: mountServices },
  { id: "tune", group: "Plan", title: "Tune", sub: "Learn a speed for every move of a sequence, then try it on the arm", icon: "gauge", mount: mountTune },
  { id: "ai", group: "", title: "AI and jobs", sub: "Where instruction packets and the AI assistant will plug in", icon: "brain", mount: mountAI },
  { id: "project", group: "", title: "Project", sub: "Repository, commits and how to run the hub", icon: "git-branch", mount: mountProject },
];
let cur = null, curId = null;
function buildNav() {
  let html = "";
  PAGES.forEach((p, i) => {
    if (p.group) html += `<div class="grp">${p.group}</div>`;
    html += `<a href="#/${p.id}" data-id="${p.id}" title="${p.title} (${i + 1})">${icon(p.icon)}<span>${p.title}</span><span class="key">${i + 1}</span></a>`;
  });
  $("#nav").innerHTML = html;
}
function route() {
  const id = (location.hash.replace(/^#\//, "") || "arm");
  const p = PAGES.find((x) => x.id === id) || PAGES[0];
  curId = p.id;
  document.querySelectorAll("#nav a").forEach((a) => (a.dataset.id === p.id ? a.setAttribute("aria-current", "page") : a.removeAttribute("aria-current")));
  $("#title").textContent = p.title; $("#subtitle").textContent = p.sub;
  document.title = `${p.title} - Arm Hub`;
  const root = $("#main");
  if (cur && cur.destroy) cur.destroy();
  document.body.classList.remove("fill");
  root.innerHTML = "";
  if (!S.st) { root.innerHTML = '<div class="skel" style="height:110px;margin-bottom:12px"></div><div class="skel" style="height:260px"></div>'; cur = null; return; }
  cur = p.mount(root) || {};
  if (cur.update) cur.update();
}
function updateArm() {
  const st = S.st; if (!st) return;
  const b = $("#armBtn"), armed = st.armed;
  b.setAttribute("aria-pressed", String(armed));
  b.querySelector("span").textContent = armed ? "ARMED" : "SAFE";
  const want = `#i-${armed ? "shield-warning" : "shield-check"}`, u = b.querySelector("use");
  if (u.getAttribute("href") !== want) u.setAttribute("href", want);
  b.setAttribute("aria-label", armed ? "Output is armed. Press to return to safe." : "Output is safe. Press to arm.");
  let note = armed ? "Input can reach the arm. Escape stops it." : "SAFE: nothing is sent to the arm.";
  if (st.forward_control === false) note += " The server cannot pause forwarding yet: rebuild it under Services.";
  $("#armNote").textContent = note;
}
function onStatus() {
  syncCfg();
  S.ppsHist.push(S.st.pps); if (S.ppsHist.length > 120) S.ppsHist.shift();
  updateArm();
  const h = $("#hubState");
  h.className = "st " + (S.ok ? "ok" : "bad"); h.textContent = S.ok ? "Hub connected" + (S.st.build ? " (build " + S.st.build + ")" : "") : "Hub offline";
  const bn = $("#banner");
  bn.hidden = S.ok;
  if (!S.ok) bn.textContent = "The hub is not reachable. Start it with: python dashboard/hub.py. This page reconnects by itself.";
  if (cur && cur.update) cur.update();
}
function connect() {
  const es = new EventSource("/api/events");
  es.addEventListener("hello", (e) => {
    const d = JSON.parse(e.data);
    S.st = d.status; S.packets = d.packets; S.board = d.board; S.logs = d.logs; S.ok = true; S.pose = d.status.sim || null;
    S.lastState = d.packets.length ? d.packets[d.packets.length - 1].s : null;
    route(); onStatus();
  });
  es.addEventListener("status", (e) => { S.st = JSON.parse(e.data); S.ok = true; onStatus(); });
  es.addEventListener("pose", (e) => { S.pose = JSON.parse(e.data); if (cur && cur.pose) cur.pose(S.pose); });
  es.addEventListener("packet", (e) => { const p = JSON.parse(e.data); S.packets.push(p); if (S.packets.length > 500) S.packets.shift(); S.lastState = p.s; if (cur && cur.packet) cur.packet(p); });
  es.addEventListener("board", (e) => { const b = JSON.parse(e.data); S.board.push(b); if (S.board.length > 400) S.board.shift(); if (cur && cur.board) cur.board(b); });
  es.addEventListener("log", (e) => { const l = JSON.parse(e.data); (S.logs[l.src] = S.logs[l.src] || []).push(l); if (S.logs[l.src].length > 200) S.logs[l.src].shift(); if (cur && cur.log) cur.log(l); });
  es.onerror = () => { S.ok = false; if (S.st) onStatus(); else { const h = $("#hubState"); h.className = "st bad"; h.textContent = "Hub offline"; } };
}

/* ---- global actions (buttons carry data-act) ---- */
document.addEventListener("click", async (e) => {
  const b = e.target.closest("[data-act]"); if (!b || b.disabled) return;
  const d = b.dataset;
  if (d.act === "go") location.hash = "#/" + d.page;
  else if (d.act === "svc") { const r = await api("/api/service", { name: d.name, action: d.action }); toast(r.message || r.error || "Done", r.ok ? "ok" : "bad"); }
});
const dlg = $("#armDlg");
$("#armBtn").addEventListener("click", async () => {
  if (!S.st) return;
  if (S.st.armed) { const r = await api("/api/arm", { armed: false }); toast(r.ok ? "Output is SAFE" : "Could not reach the hub", r.ok ? "ok" : "bad"); }
  else dlg.showModal();
});
$("#armCancel").addEventListener("click", () => dlg.close());
$("#armConfirm").addEventListener("click", async () => { dlg.close(); const r = await api("/api/arm", { armed: true }); toast(r.ok ? "Output is ARMED" : "Could not reach the hub", r.ok ? "" : "bad"); });
document.addEventListener("keydown", (e) => {
  const t = e.target;
  if (e.key === "Escape" && S.st && S.st.armed && !dlg.open) { api("/api/arm", { armed: false }); toast("Output is SAFE", "ok"); return; }
  if (t.closest && t.closest("input, textarea, select")) return;
  if (e.altKey || e.ctrlKey || e.metaKey) return;
  const n = parseInt(e.key, 10);
  if (n >= 1 && n <= PAGES.length) location.hash = "#/" + PAGES[n - 1].id;
});
/* ---- theme ---- */
(function theme() {
  const root = document.documentElement;
  try { const t = localStorage.getItem("hub-theme"); if (t) root.dataset.theme = t; } catch (e) { /* storage blocked: follow the system */ }
  $("#themeBtn").addEventListener("click", () => {
    const dark = root.dataset.theme ? root.dataset.theme === "dark" : !matchMedia("(prefers-color-scheme: light)").matches;
    root.dataset.theme = dark ? "light" : "dark";
    try { localStorage.setItem("hub-theme", root.dataset.theme); } catch (e) { /* ignore */ }
  });
})();

buildNav();
window.addEventListener("hashchange", route);
route();
connect();
requestAnimationFrame(engine);
