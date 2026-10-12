// Controller input: gamepad, on-screen sticks and keyboard are merged into one state and sent to the hub while the output is
// ARMED (or the emulated ESP32 is running). Same body the original dashboard sent to /api/input.
import { getState } from "./store.js";

const BUTTONS = ["a", "b", "x", "y", "lb", "rb", "back", "start", "ls", "rs"];
const neutral = () => ({ lx: 0, ly: 0, rx: 0, ry: 0, lt: 0, rt: 0, buttons: Object.fromEntries(BUTTONS.map((k) => [k, false])), dpad: [0, 0] });

export const pad = { virt: neutral(), keys: new Set(), keyboard: true, gamepad: null, sending: false, listeners: new Set() };
const sticky = {};
let inflight = false, lastJson = "", lastSend = 0, wasSending = false;

const KEYMAP = { // keyboard -> controller
  left: { KeyA: [-1, 0], KeyD: [1, 0], KeyW: [0, -1], KeyS: [0, 1] },
  right: { ArrowLeft: [-1, 0], ArrowRight: [1, 0], ArrowUp: [0, -1], ArrowDown: [0, 1] },
  buttons: { Space: "a", KeyH: "b", KeyM: "x", KeyP: "y" },
};
export const KEY_HELP = [["W A S D", "Left stick: move the tool"], ["↑ ↓ ← →", "Right stick: height / tool angle"], ["Space", "A: save a pose"], ["H", "B: go home"], ["M", "X: magnet on / off"], ["P", "Y: play / stop saved poses"]];

function readGamepad() {
  const pads = (navigator.getGamepads && navigator.getGamepads()) || [];
  let p = null;
  for (const g of pads) if (g && g.connected) { p = g; break; }
  pad.gamepad = p ? p.id : null;
  if (!p) return null;
  const dz = (v) => (Math.abs(v) < 0.12 ? 0 : (Math.sign(v) * (Math.abs(v) - 0.12)) / 0.88);
  const b = (i) => !!(p.buttons[i] && p.buttons[i].pressed);
  const r3 = (v) => Math.round(v * 1000) / 1000;
  return {
    lx: r3(dz(p.axes[0] || 0)), ly: r3(dz(p.axes[1] || 0)), rx: r3(dz(p.axes[2] || 0)), ry: r3(dz(p.axes[3] || 0)), lt: 0, rt: 0,
    buttons: { a: b(0), b: b(1), x: b(2), y: b(3), lb: b(4), rb: b(5), back: b(8), start: b(9), ls: b(10), rs: b(11) },
    dpad: [(b(15) ? 1 : 0) - (b(14) ? 1 : 0), (b(12) ? 1 : 0) - (b(13) ? 1 : 0)],
  };
}

function merged() {
  const g = readGamepad();
  const v = pad.virt, out = neutral();
  const kb = pad.keyboard ? pad.keys : new Set();
  const axis = (map) => { let x = 0, y = 0; for (const k of kb) if (map[k]) { x += map[k][0]; y += map[k][1]; } return [Math.max(-1, Math.min(1, x)), Math.max(-1, Math.min(1, y))]; };
  const l = axis(KEYMAP.left), r = axis(KEYMAP.right);
  out.lx = g?.lx || v.lx || l[0]; out.ly = g?.ly || v.ly || l[1]; out.rx = g?.rx || v.rx || r[0]; out.ry = g?.ry || v.ry || r[1];
  for (const k of BUTTONS) out.buttons[k] = !!(g?.buttons[k] || v.buttons[k]);
  for (const k of kb) if (KEYMAP.buttons[k]) out.buttons[KEYMAP.buttons[k]] = true;
  if (g) out.dpad = g.dpad;
  return out;
}

export function startInput() {
  const typing = (e) => e.target && e.target.closest && e.target.closest("input, textarea, select, [contenteditable]");
  window.addEventListener("keydown", (e) => {
    if (typing(e) || e.ctrlKey || e.metaKey || e.altKey) return;
    if (KEYMAP.left[e.code] || KEYMAP.right[e.code] || KEYMAP.buttons[e.code]) { if (pad.keyboard) { pad.keys.add(e.code); e.preventDefault(); } }
  });
  window.addEventListener("keyup", (e) => pad.keys.delete(e.code));
  window.addEventListener("blur", () => pad.keys.clear());
  const tick = (ts) => {
    const st = merged();
    const s = getState().status;
    const should = !!(s && (s.armed || s.emu_input) && getState().online);
    for (const k of BUTTONS) if (st.buttons[k]) sticky[k] = true;        // a tap shorter than one request is still sent
    const active = st.lx || st.ly || st.rx || st.ry || Object.values(st.buttons).some(Boolean) || Object.keys(sticky).length;
    if (should && (active || wasSending)) {
      const buttons = { ...st.buttons, ...sticky };
      const body = JSON.stringify({ lx: st.lx, ly: st.ly, rx: st.rx, ry: st.ry, lt: st.lt, rt: st.rt, buttons, dpad: st.dpad });
      if (!inflight && ts - lastSend > 33 && (body !== lastJson || ts - lastSend > 500)) {
        inflight = true; lastJson = body; lastSend = ts;
        for (const k of Object.keys(sticky)) delete sticky[k];
        fetch("/api/input", { method: "POST", headers: { "Content-Type": "application/json", "X-Hub": "1" }, body })
          .then((r) => r.json()).then((j) => { pad.sending = !!j.sent; }).catch(() => { pad.sending = false; }).finally(() => { inflight = false; });
      }
      wasSending = !!active || (ts - lastSend < 600);
      if (!active && ts - lastSend > 600) wasSending = false;
    } else {
      for (const k of Object.keys(sticky)) delete sticky[k];
      if (wasSending) fetch("/api/input", { method: "POST", headers: { "Content-Type": "application/json", "X-Hub": "1" }, body: JSON.stringify(neutral()) }).catch(() => {});
      wasSending = false; pad.sending = false; lastJson = "";
    }
    pad.listeners.forEach((f) => f(st));
    requestAnimationFrame(tick);
  };
  requestAnimationFrame(tick);
}
export { neutral };
