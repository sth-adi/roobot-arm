"use strict";
/* Arm Hub front end. Plain JavaScript, no build step.
   To add a page: write a mount(root) function that returns { update(), packet(p), board(b), log(l), frame() }
   (all optional) and add one entry to PAGES. The hub streams everything over one EventSource. */

/* ===================================================================== helpers */
const $ = (s, r = document) => r.querySelector(s);
const esc = (s) => String(s).replace(/[&<>"']/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
const icon = (n, c = "") => `<svg class="ic ${c}"><use href="#i-${n}"/></svg>`;
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));
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
const S = { st: null, packets: [], board: [], logs: {}, ok: false, lastState: null, ppsHist: [] };
const cfg = { rate: 5, range: 180 };
const LOG_SRC = [["hub", "Hub"], ["server_docker", "Server (Docker)"], ["server_python", "Server (Python)"], ["gui", "Controller GUI"], ["docker_engine", "Docker Desktop"], ["firmware", "Firmware"]];

/* ===================================================================== engine
   One loop for the whole app: reads the chosen controller source, runs a "preview arm" that mirrors the
   firmware's jog logic (stick = velocity, position holds on release), and sends input while ARMED. */
const E = {
  source: "server", virt: neutral(), active: neutral(), idle: false, twin: [1500, 1500, 1500, 1500],
  last: performance.now(), lastSend: 0, lastJson: "", inflight: false, sending: false, wasSending: false, gpName: null,
  userRateT: 0, lastLive: 0,
};
function readGamepad() {
  const pads = (navigator.getGamepads && navigator.getGamepads()) || [];
  let p = null;
  for (const g of pads) if (g && g.connected) { p = g; break; }
  E.gpName = p ? p.id : null;
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
  // preview arm: same maths as tickPwm() in the firmware
  const rate = cfg.rate > 0 ? cfg.rate : 180, usPerDeg = 1000 / cfg.range;
  const cmd = [st.lx, -st.ly, st.rx, -st.ry];
  for (let i = 0; i < 4; i++) E.twin[i] = clamp(E.twin[i] + cmd[i] * rate * usPerDeg * dt, 1000, 2000);
  maybeSend(ts, st);
  if (cur && cur.frame) cur.frame(st);
  requestAnimationFrame(engine);
}
function maybeSend(ts, st) {
  const should = E.source !== "server" && S.st && S.st.armed && S.ok;
  if (should) {
    const body = JSON.stringify({ lx: st.lx, ly: st.ly, rx: st.rx, ry: st.ry, lt: st.lt, rt: st.rt, buttons: st.buttons, dpad: st.dpad });
    if (!E.inflight && (body !== E.lastJson || ts - E.lastSend > 500)) {
      E.inflight = true; E.lastJson = body; E.lastSend = ts;
      fetch("/api/input", { method: "POST", headers: { "Content-Type": "application/json", "X-Hub": "1" }, body })
        .then((r) => r.json()).then((j) => { E.sending = !!j.sent; }).catch(() => { E.sending = false; })
        .finally(() => { E.inflight = false; });
    }
    E.wasSending = true;
  } else {
    if (E.wasSending && S.ok) api("/api/input", neutral());   // leaving: tell the arm to stop
    E.wasSending = false; E.sending = false; E.lastJson = "";
  }
}
function setSource(s) {
  E.source = s;
  E.virt = neutral();
  if (cur && cur.sourceChanged) cur.sourceChanged();
}

/* ===================================================================== board settings */
function pushRateLive(v) {
  const now = performance.now();
  if (now - E.lastLive < 100) return;
  E.lastLive = now;
  api("/api/board", { cmd: `!live ${v}` });
}
async function saveRate(v) {
  const r = await api("/api/board", { cmd: `!rate ${v}` });
  toast(r.ok ? `Speed at full stick saved on the board: ${v} deg/s` : "The USB link is not open: only the preview uses this speed", r.ok ? "ok" : "");
}
function syncCfg() {
  const u = S.st && S.st.usb;
  if (!u) return;
  if (u.range) cfg.range = u.range;
  if (u.rate != null && performance.now() - E.userRateT > 2500) cfg.rate = u.rate;
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
    <div><div class="k">Speed at full stick</div><div class="v" id="o-rate">-</div><div class="sub" id="o-rate-s"></div></div>
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
      $("#o-rate", root).innerHTML = usb.rate == null ? "-" : `${usb.rate}<span class="u">deg/s</span>`;
      $("#o-rate-s", root).textContent = usb.rate == null ? "board not reporting" : (usb.rate > 0 ? `${cfg.range} deg in ${Math.round(cfg.range / usb.rate)} s at full stick` : "no limit");
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
const OUTS = [["OUT1", 25, "LX", "Left stick, left and right"], ["OUT2", 26, "LY", "Left stick, up and down"], ["OUT3", 27, "RX", "Right stick, left and right"], ["OUT4", 32, "RY", "Right stick, up and down"]];
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
function dialSVG(i) {
  let t = "";
  for (const a of [-90, -45, 0, 45, 90]) {
    const rad = (a * Math.PI) / 180, x1 = 80 + 66 * Math.sin(rad), y1 = 90 - 66 * Math.cos(rad), x2 = 80 + 72 * Math.sin(rad), y2 = 90 - 72 * Math.cos(rad);
    t += `<line class="tick" x1="${x1}" y1="${y1}" x2="${x2}" y2="${y2}"/>`;
  }
  return `<svg viewBox="0 0 160 100" role="img" aria-label="${OUTS[i][0]} position"><path class="arc" d="M20 90 A60 60 0 0 1 140 90"/>${t}
    <line class="needle" id="nd${i}" x1="80" y1="90" x2="80" y2="38"/><circle class="bmark" id="bm${i}" r="4" cx="80" cy="30" style="display:none"/><circle class="hub" cx="80" cy="90" r="5"/></svg>`;
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
      <section class="sec" style="border-top:0;padding-top:0"><h2>Preview arm</h2>
        <p class="lead">Mirrors the firmware: a stick moves the output, and the output stays put when the stick is released. It runs in this browser only.</p>
        <div class="dials">${OUTS.map((o, i) => `<div class="dial"><div class="hd"><span>${o[0]} <span class="muted">${o[2]}</span></span><span class="muted mono">GPIO${o[1]}</span></div>${dialSVG(i)}
          <div class="hd"><span class="rd" id="d-us${i}">1500 us</span><span class="mono muted" id="d-deg${i}">0 deg</span></div>
          <div class="bar mid" style="margin-top:6px"><i id="d-cmd${i}" style="left:50%;width:0"></i></div></div>`).join("")}</div>
        <div class="inline" style="margin-top:14px"><button class="btn sm" id="c-centre" type="button">${icon("arrows-clockwise", "sm")}Centre preview</button>
          <span class="note" id="c-board"></span></div>
      </section>
      <section class="sec"><h2>Speed at full stick</h2>
        <p class="lead">The damper: how fast an output moves when the stick is pushed all the way. Partial pushes go proportionally slower.</p>
        <div class="field"><div class="inline"><input type="range" id="c-rate" min="0" max="60" step="1" aria-label="Speed at full stick in degrees per second"><span class="mono" id="c-rate-v" style="min-width:90px;text-align:right"></span></div>
          <div class="note" id="c-rate-n"></div></div>
        <div class="inline"><label class="muted" for="c-range">Servo travel</label><input type="number" id="c-range" min="10" max="720" step="1"><span class="muted">degrees</span>
          <button class="btn sm" id="c-range-go" type="button">Apply</button></div>
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
  // settings
  const slider = $r("#c-rate");
  slider.value = cfg.rate;
  slider.addEventListener("input", () => { cfg.rate = +slider.value; E.userRateT = performance.now(); pushRateLive(cfg.rate); });
  slider.addEventListener("change", () => { E.userRateT = performance.now(); saveRate(+slider.value); });
  $r("#c-range").value = cfg.range;
  $r("#c-range-go").addEventListener("click", async () => {
    const v = clamp(+$r("#c-range").value || 180, 10, 720); cfg.range = v;
    const r = await api("/api/board", { cmd: `!range ${v}` });
    toast(r.ok ? `Servo travel saved on the board: ${v} degrees` : "The USB link is not open: only the preview uses this travel", r.ok ? "ok" : "");
  });
  $r("#c-centre").addEventListener("click", () => { E.twin = [1500, 1500, 1500, 1500]; });
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
      else if (!armed) { el.className = "st warn"; el.textContent = "Preview only: output is SAFE"; }
      else if (E.sending) { el.className = "st bad"; el.textContent = "Sending to the arm"; }
      else { el.className = "st warn"; el.textContent = "ARMED, waiting to send"; }
      $r("#c-info").textContent = E.source === "gamepad" ? (E.gpName ? `Controller found: ${E.gpName}` : "No controller reported yet. Press a button on it.")
        : E.source === "server" && E.idle ? "No live input at the local server right now." : "";
      const u = st.usb;
      $r("#c-board").textContent = u.phase === "open" ? "The board is connected: speed and travel changes reach it." : "No board on USB: speed and travel change the preview only.";
      if (document.activeElement !== $r("#c-range") && u.range) $r("#c-range").value = cfg.range;
      slider.value = cfg.rate;
    },
    frame(st) {
      // controller drawing
      const k = (id) => $r("#p-" + id);
      const kn = (id, vx, vy) => { const n = k(id); n.setAttribute("cx", r3(vx * 28)); n.setAttribute("cy", r3(vy * 28)); };
      kn("lk", st.lx, st.ly); kn("rk", st.rx, st.ry);
      for (const b of BUTTONS) { const n = k(b); if (n) n.classList.toggle("on", !!st.buttons[b]); }
      k("ltf").setAttribute("width", r3(84 * st.lt)); k("rtf").setAttribute("width", r3(84 * st.rt));
      k("du").classList.toggle("on", st.dpad[1] === 1); k("dd").classList.toggle("on", st.dpad[1] === -1);
      k("dl").classList.toggle("on", st.dpad[0] === -1); k("dr").classList.toggle("on", st.dpad[0] === 1);
      // preview arm
      const cmd = [st.lx, -st.ly, st.rx, -st.ry], bpos = S.st && S.st.board && S.st.board.pos;
      for (let i = 0; i < 4; i++) {
        const us = E.twin[i], ang = ((us - 1500) / 500) * 90, rad = (ang * Math.PI) / 180;
        const nd = $r("#nd" + i); nd.setAttribute("x2", r3(80 + 52 * Math.sin(rad))); nd.setAttribute("y2", r3(90 - 52 * Math.cos(rad)));
        $r("#d-us" + i).textContent = Math.round(us) + " us";
        $r("#d-deg" + i).textContent = ((us - 1500) / (1000 / cfg.range)).toFixed(1) + " deg";
        const c = $r("#d-cmd" + i), v = clamp(cmd[i], -1, 1);
        c.style.left = (v >= 0 ? 50 : 50 + v * 50) + "%"; c.style.width = Math.abs(v) * 50 + "%";
        const bm = $r("#bm" + i);
        if (bpos && bpos[i]) { const br = (((bpos[i] - 1500) / 500) * 90 * Math.PI) / 180; bm.style.display = ""; bm.setAttribute("cx", r3(80 + 60 * Math.sin(br))); bm.setAttribute("cy", r3(90 - 60 * Math.cos(br))); }
        else bm.style.display = "none";
      }
      const sv = $r("#c-rate-v");
      sv.textContent = cfg.rate > 0 ? `${cfg.rate} deg/s` : "no limit";
      $r("#c-rate-n").textContent = cfg.rate > 0 ? `Holding full stick crosses ${cfg.range} degrees in ${Math.round(cfg.range / cfg.rate)} s.` : "No limit means full stick moves at 180 deg/s.";
    },
  };
  upd.sourceChanged();
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
    $r("#k-raw").textContent = p ? (view === "http" ? rawHttp(p.s) : JSON.stringify(p.s, null, 2)) : "Select a packet to see exactly what was sent.";
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
      <section class="sec" style="border-top:0"><h2>USB link</h2>
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
      <section class="sec" style="border-top:0;padding-top:0"><h2>ESP32 sketch</h2>
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
const PAGES = [
  { id: "overview", group: "Monitor", title: "Overview", sub: "Everything at a glance", icon: "squares-four", mount: mountOverview },
  { id: "control", group: "", title: "Control", sub: "Drive from the browser and watch the preview arm", icon: "game-controller", mount: mountControl },
  { id: "packets", group: "", title: "Packets", sub: "Every controller packet, exactly as sent", icon: "pulse", mount: mountPackets },
  { id: "board", group: "Hardware", title: "Arm and board", sub: "The ESP32 over USB: log, settings and outputs", icon: "cpu", mount: mountBoard },
  { id: "firmware", group: "", title: "Firmware", sub: "Compile and upload the ESP32 sketch", icon: "upload-simple", mount: mountFirmware },
  { id: "services", group: "System", title: "Services", sub: "Start and stop what the system runs on", icon: "plugs-connected", mount: mountServices },
  { id: "ai", group: "Plan", title: "AI and jobs", sub: "Where instruction packets and the AI assistant will plug in", icon: "brain", mount: mountAI },
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
  const id = (location.hash.replace(/^#\//, "") || "overview");
  const p = PAGES.find((x) => x.id === id) || PAGES[0];
  curId = p.id;
  document.querySelectorAll("#nav a").forEach((a) => (a.dataset.id === p.id ? a.setAttribute("aria-current", "page") : a.removeAttribute("aria-current")));
  $("#title").textContent = p.title; $("#subtitle").textContent = p.sub;
  document.title = `${p.title} - Arm Hub`;
  const root = $("#main");
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
  h.className = "st " + (S.ok ? "ok" : "bad"); h.textContent = S.ok ? "Hub connected" : "Hub offline";
  const bn = $("#banner");
  bn.hidden = S.ok;
  if (!S.ok) bn.textContent = "The hub is not reachable. Start it with: python dashboard/hub.py. This page reconnects by itself.";
  if (cur && cur.update) cur.update();
}
function connect() {
  const es = new EventSource("/api/events");
  es.addEventListener("hello", (e) => {
    const d = JSON.parse(e.data);
    S.st = d.status; S.packets = d.packets; S.board = d.board; S.logs = d.logs; S.ok = true;
    S.lastState = d.packets.length ? d.packets[d.packets.length - 1].s : null;
    route(); onStatus();
  });
  es.addEventListener("status", (e) => { S.st = JSON.parse(e.data); S.ok = true; onStatus(); });
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
