// The System area: everything that used to live on the classic dashboard's pages, rebuilt as React pages.
import React, { useEffect, useMemo, useRef, useState } from "react";
import { api, feed, getJson, toast, useFeed } from "./store.js";
import { Btn, Field, Icon, Pill } from "./ui.jsx";

export const PAGES = [
  ["overview", "Overview", "pulse"], ["tuning", "Arm & tuning", "sliders"], ["packets", "Packets", "list"], ["board", "Board", "chip"],
  ["firmware", "Firmware", "hammer"], ["services", "Services", "server"], ["project", "Project", "git"], ["ai", "AI & jobs", "spark"],
];

const f1 = (v, d = 1) => (v == null || Number.isNaN(v) ? "-" : Number(v).toFixed(d));
const timeStr = (t) => { const d = new Date(t * 1000); return d.toTimeString().slice(0, 8) + "." + String(d.getMilliseconds()).padStart(3, "0"); };
const ageStr = (s) => (s == null ? "no data" : s < 10 ? s.toFixed(1) + " s" : s < 90 ? Math.round(s) + " s" : s < 5400 ? Math.round(s / 60) + " min" : Math.round(s / 3600) + " h");
const Stat = ({ k, v, sub, small }) => <div className="stat"><div className="k">{k}</div><div className={`v ${small ? "s" : ""}`}>{v}</div>{sub ? <div className="sub">{sub}</div> : null}</div>;
const Card = ({ title, aside, children, className = "" }) => <section className={`card ${className}`}><header className="cardhead"><h3>{title}</h3>{aside}</header>{children}</section>;
const call = async (path, body, okMsg) => { const r = await api(path, body); if (!r.ok) toast(r.message || r.error || "Failed", "bad"); else if (okMsg) toast(okMsg, "ok"); return r; };

/* ------------------------------------------------------------------ overview */
const NODES = [
  { id: "input", x: 0, name: "Input" }, { id: "local", x: 210, name: "Local server" }, { id: "end", x: 420, name: "End server" },
  { id: "board", x: 630, name: "ESP32 board" }, { id: "arm", x: 840, name: "Arm and servos" },
];
const NW = 160, NH = 104, NY = 40;

function attention(st) {
  const items = [], sv = st.services, usb = st.usb;
  if (sv.docker_engine.state === "stopped") items.push({ t: "Docker Desktop is not running, so the Docker server cannot start.", a: ["Start Docker", "svc", "docker_engine", "start"] });
  if (!st.server.health) items.push({ t: "The local server is not running. Nothing can be received until it is.", a: sv.docker_engine.state === "running" ? ["Start server", "svc", "server_docker", "start"] : ["Open Services", "go", "services"] });
  if (st.forward_control === false) items.push({ t: "The server image cannot pause forwarding yet, so SAFE cannot stop the network path. Rebuild it.", a: ["Rebuild server", "svc", "server_docker", "restart"] });
  if (!st.config.forward_url_set) items.push({ t: "No FORWARD_URL in .env: the local server will not forward to an end server.", a: ["Open Project", "go", "project"] });
  if (usb.phase === "busy") items.push({ t: "The board's COM port is held by another program (the GUI or the Arduino Serial Monitor).", a: ["Open Services", "go", "services"] });
  if (usb.phase === "waiting" && usb.enabled) items.push({ t: "No ESP32 found on USB. Plug it in with a data cable, or work on the preview.", a: null });
  if (!st.firmware.secrets_present) items.push({ t: "secrets.h is missing, so the firmware cannot be built.", a: ["Open Firmware", "go", "firmware"] });
  return items;
}

function SystemMap({ st }) {
  const sv = st.server, fw = sv.forward, usb = st.usb, live = st.pps > 0, armed = st.armed;
  const fwdOn = armed && live && sv.forward_enabled !== false && fw.ok !== false && fw.configured;
  const endLevel = !sv.health || !fw.configured ? "idle" : sv.forward_enabled === false ? "idle" : fw.ok === false ? "bad" : fw.ok ? "ok" : "warn";
  const bl = usb.phase === "open" ? "ok" : usb.phase === "busy" || usb.phase === "suspended" ? "warn" : "idle";
  const info = {
    input: [live ? "ok" : "idle", "controller or browser", live ? `${f1(st.pps)} pkt/s` : "idle"],
    local: [sv.health ? "ok" : "bad", { docker: "Docker", python: "Python", external: "external", none: "not running" }[sv.mode] || "", sv.health ? "up" : "down"],
    end: [endLevel, "forwarding target", !fw.configured ? "not set" : sv.forward_enabled === false ? "paused (SAFE)" : fw.ok === false ? "failing" : fw.ok ? "forwarding" : "waiting"],
    board: [bl, st.board.source ? `input via ${st.board.source === "usb" ? "USB" : "WiFi"}` : (usb.port ? "on " + usb.port : "USB or WiFi"), { open: "USB open", busy: "port busy", waiting: "not on USB", off: "USB link off", suspended: "flashing" }[usb.phase]],
    arm: ["idle", "no live feedback", st.board.pos ? "last pos reported" : "no feedback"],
  };
  const links = [live, fwdOn, fwdOn, armed && live];
  const my = NY + NH / 2, ux1 = NODES[1].x + NW / 2, ux2 = NODES[3].x + NW / 2, uy = NY + NH, usbOn = armed && live && usb.phase === "open";
  return (
    <div className="mapwrap">
      <svg className="map" viewBox="0 0 1000 250" role="img" aria-label="System map from input to arm">
        {links.map((on, i) => {
          const x1 = NODES[i].x + NW, x2 = NODES[i + 1].x;
          return <g key={i}><line className={`link ${on ? "on" : ""}`} x1={x1} y1={my} x2={x2 - 8} y2={my} /><polygon className={`tip ${on ? "on" : ""}`} points={`${x2},${my} ${x2 - 9},${my - 5} ${x2 - 9},${my + 5}`} /></g>;
        })}
        <path className={`link ${usbOn ? "on" : ""}`} d={`M${ux1} ${uy} V${uy + 62} H${ux2} V${uy + 9}`} />
        <polygon className={`tip ${usbOn ? "on" : ""}`} points={`${ux2},${uy} ${ux2 - 5},${uy + 9} ${ux2 + 5},${uy + 9}`} />
        <text className="lbl" x={(ux1 + ux2) / 2} y={uy + 82} textAnchor="middle">USB cable, direct to the board</text>
        {[["sends", 1], ["WiFi", 2], ["PWM", 3]].map(([t, i]) => <text key={t} className="lbl" x={(NODES[i].x + NW + NODES[i + 1].x) / 2} y={my - 12} textAnchor="middle">{t}</text>)}
        {NODES.map((n) => (
          <g key={n.id} className={`node ${info[n.id][0]}`}>
            <rect x={n.x} y={NY} width={NW} height={NH} rx="12" />
            <text className="nm" x={n.x + 14} y={NY + 29}>{n.name}</text>
            <text className="ds" x={n.x + 14} y={NY + 60}>{info[n.id][1]}</text>
            <text className="stt" x={n.x + 14} y={NY + 86}>{info[n.id][2]}</text>
          </g>
        ))}
      </svg>
    </div>
  );
}

function Overview({ st, go }) {
  const f = useFeed();
  const sv = st.server, fw = sv.forward, usb = st.usb, sim = st.sim;
  const fwdText = !sv.health ? "unknown" : !fw.configured ? "not set" : sv.forward_enabled === false ? "paused" : fw.ok === true ? "forwarding" : fw.ok === false ? "failing" : "waiting";
  const items = attention(st);
  const act = (a) => (a[1] === "go" ? go(a[2]) : call("/api/service", { name: a[2], action: a[3] }, "Done"));
  const svc = (n) => st.services[n].state;
  return (
    <div className="stack">
      <div className="stats">
        <Stat k="Packets per second" v={f1(st.pps)} sub={sv.health ? "from the local server" : "server is down"} />
        <Stat k="Last packet" v={st.last_packet_age == null ? "none yet" : ageStr(st.last_packet_age)} sub={st.last_packet_age != null && st.last_packet_age < 1.5 ? "live" : "idle"} />
        <Stat small k="End server" v={fwdText} sub={fw.sent != null ? `${fw.sent} sent, ${fw.failed || 0} failed` : sv.forward_enabled === false ? "SAFE pauses it" : ""} />
        <Stat small k="USB link" v={usb.port || "no board"} sub={{ open: "open", busy: "port busy", waiting: "waiting", off: "link off", suspended: "flashing" }[usb.phase] || usb.phase} />
        <Stat k="Joint speed limit" v={sim ? `${sim.rate > 0 ? sim.rate : 180}°/s` : "-"} sub={!sim ? "" : sim.rate > 0 ? "damper on the virtual arm" : "no limit (capped at 180)"} />
      </div>
      <Card title="System map" aside={<Spark data={f.ppsHist} />}>
        <p className="hint">How input travels today. Lines move only while packets are flowing, and only the paths that are allowed through are lit.</p>
        <SystemMap st={st} />
      </Card>
      <div className="grid2 top">
        <Card title="Needs attention">
          <ul className="todo">
            {items.map((i, k) => <li key={k} className="warn"><Icon n="bolt" size={16} /><span>{i.t}</span>{i.a ? <Btn size="sm" onClick={() => act(i.a)}>{i.a[0]}</Btn> : null}</li>)}
            {!items.length && <li className="good"><Icon n="check" size={16} /><span>Nothing needs attention.</span></li>}
            <li className="good"><Icon n="shield" size={16} /><span>{st.armed ? "Output is ARMED: input can reach the arm. Press Escape to stop." : "Output is SAFE: nothing is sent to the arm."}</span></li>
          </ul>
        </Card>
        <Card title="Launch">
          <div className="rowsl">
            <div><div><b>Local server</b><div className="hint">Starts the Docker container that receives input.</div></div><Btn size="sm" icon="play" disabled={svc("server_docker") === "running" || svc("server_docker") === "busy"} onClick={() => call("/api/service", { name: "server_docker", action: "start" }, "Starting")}>{svc("server_docker") === "running" ? "Running" : "Start"}</Btn></div>
            <div><div><b>Controller window</b><div className="hint">The pygame window for a physical controller.</div></div><Btn size="sm" icon="pad" disabled={svc("gui") === "running" || svc("gui") === "busy"} onClick={() => call("/api/service", { name: "gui", action: "start" }, "Opening")}>{svc("gui") === "running" ? "Running" : "Open"}</Btn></div>
            <div><div><b>Build the firmware</b><div className="hint">Compile or upload the ESP32 sketch.</div></div><Btn size="sm" icon="hammer" onClick={() => go("firmware")}>Open</Btn></div>
          </div>
        </Card>
      </div>
    </div>
  );
}

function Spark({ data }) {
  const pts = data.slice(-60), mx = Math.max(1, ...pts);
  return <svg className="spark" viewBox="0 0 160 36" aria-hidden="true"><polyline points={pts.map((v, i) => `${(i * 160) / 59},${34 - (v / mx) * 32}`).join(" ")} /></svg>;
}

/* ------------------------------------------------------------------ arm and tuning (virtual arm, model, damper, calibration, emulator link) */
const JOINTS = ["Base", "Shoulder", "Elbow", "Wrist"];

function Tuning({ st }) {
  const sim = st.sim, emu = st.emu;
  return (
    <div className="grid2 top">
      <div className="stack">
        <VirtualArm sim={sim} />
        <Model sim={sim} />
        <Damper sim={sim} />
      </div>
      <div className="stack">
        <Calibration sim={sim} />
        <Emulator emu={emu} />
      </div>
    </div>
  );
}

function VirtualArm({ sim }) {
  if (!sim) return <Card title="Virtual arm"><p className="hint">Waiting for the hub…</p></Card>;
  const modeText = { idle: "holding", manual: "jogging", home: "going home", replay: "replaying" };
  return (
    <Card title="Virtual arm" aside={<Pill tone={sim.ik_ok === false ? "bad" : "ok"}>{sim.ik_ok === false ? "out of reach" : "reachable"}</Pill>}>
      <p className="hint">The UWE-RArm movement logic run on this computer: sticks move the tool through inverse kinematics, A saves a position, holding A replays them, B goes home. It always runs, including while SAFE.</p>
      <div className="stats tight">
        <Stat small k="Tool position" v={(sim.xyz || []).map((v) => f1(v, 0)).join(", ")} sub={`x, y, z mm · tool ${f1(sim.tool_angle, 0)}°`} />
        <Stat small k="State" v={sim.attached === false ? "servos released" : modeText[sim.mode] || sim.mode} sub={sim.status} />
        <Stat k="Saved positions" v={sim.saved} sub={sim.mode === "replay" ? `next: #${(sim.replay_idx || 0) + 1}` : ""} />
      </div>
      <table className="tbl"><thead><tr><th>Joint</th><th>Angle</th><th>Limits</th><th>Servo</th></tr></thead><tbody>
        {JOINTS.map((n, i) => { const [lo, hi] = (sim.limits || [])[i] || [0, 0]; return <tr key={n}><td>{i + 1} {n}</td><td>{f1((sim.angles || [])[i])}°</td><td>{lo}° to {hi}°{lo > hi ? " (inverted)" : ""}</td><td>{(sim.servo || [])[i]}°</td></tr>; })}
      </tbody></table>
      <div className="row"><Btn size="sm" icon="home" onClick={() => call("/api/sim", { home: true })}>Go home</Btn><Btn size="sm" icon="trash" onClick={() => call("/api/sim", { clear_saved: true }, "Saved positions cleared")}>Clear saved positions</Btn></div>
    </Card>
  );
}

function Model({ sim }) {
  const val = !sim ? "stock-patch" : sim.model === "stock" ? (sim.patch ? "stock-patch" : "stock") : "improved";
  const note = !sim ? "" : sim.model !== "stock" ? "Single source of truth for the pose: moving after Go home starts from home."
    : sim.patch ? "Stock logic plus: speed limit on the servo output, input capped at 100 ms per packet, stick input pauses while the arm lags its target by 25 degrees, Go home waits for arrival and re-syncs the target."
      : `As shipped: no speed limit, the XYZ target is separate from the arm (target ${sim.target ? sim.target.join(", ") : "-"}), Go home gives up after 3 s, and once it ends the arm swings back to the pre-home target.`;
  return (
    <Card title="Firmware to simulate">
      <p className="hint">Which control logic the virtual arm runs. Stock is the UWE-RArm firmware exactly as it is, quirks included; the patched version adds only the damper and a few failsafes.</p>
      <Field label="Model">
        <select value={val} onChange={(e) => { const v = e.target.value; call("/api/sim", { model: v === "improved" ? "improved" : "stock", patch: v === "stock-patch" }); }}>
          <option value="stock-patch">Our firmware: stock + patch (default)</option><option value="stock">Stock UWE-RArm firmware, as shipped</option><option value="improved">Reference (this repo's model)</option>
        </select>
      </Field>
      <p className="hint">{note}</p>
    </Card>
  );
}

function Damper({ sim }) {
  const [v, setV] = useState(null), last = useRef(0);
  const shown = v ?? sim?.rate ?? 30;
  const push = (x, force) => { const now = performance.now(); if (!force && now - last.current < 100) return; last.current = now; api("/api/sim", { rate: x }); };
  const stockNoPatch = sim && sim.model === "stock" && !sim.patch;
  return (
    <Card title="Damper" aside={<span className="num">{shown > 0 ? `${shown}°/s` : "no limit"}</span>}>
      <p className="hint">The fastest any joint may move, in degrees per second. It applies to the sticks, Go home and replay. All four joints are slowed together, so the tool keeps to its path.</p>
      <input type="range" min="0" max="60" step="1" value={shown} aria-label="Fastest joint speed in degrees per second"
        onChange={(e) => { setV(+e.target.value); push(+e.target.value); }} onPointerUp={() => { push(shown, true); setV(null); }} onKeyUp={() => { push(shown, true); setV(null); }} />
      <p className="hint">{stockNoPatch ? "The stock firmware has no speed limiter: this setting is ignored in this mode." : shown > 0 ? `A 90 degree joint move takes at least ${Math.round(90 / shown)} s.` : "No limit: joints are capped at 180 deg/s."}</p>
    </Card>
  );
}

function Calibration({ sim }) {
  const [vals, setVals] = useState(null), [tool, setTool] = useState(""), dirty = useRef(false);
  useEffect(() => {
    if (!sim || dirty.current) return;
    setVals({ lim: (sim.limits || []).map((l) => [...l]), homes: [...(sim.homes || [])] }); setTool(String(sim.tool ?? ""));
  }, [sim]);
  if (!vals) return <Card title="Calibration and tool"><p className="hint">Waiting for the hub…</p></Card>;
  const edit = (fn) => { dirty.current = true; setVals((s) => { const n = { lim: s.lim.map((l) => [...l]), homes: [...s.homes] }; fn(n); return n; }); };
  const apply = async () => {
    const lim = vals.lim.map((l) => l.map((x) => Math.round(+x))), homes = vals.homes.map((x) => Math.round(+x));
    const changed = JSON.stringify(lim) !== JSON.stringify(sim.limits) || JSON.stringify(homes) !== JSON.stringify(sim.homes);
    const body = { tool: +tool }; if (changed) body.calibration = { jointLimits: lim, jointHomes: homes };
    const r = await api("/api/sim", body);
    if (r.ok) { dirty.current = false; toast(changed ? "Calibration applied: the arm is at its new home" : "Tool length applied", "ok"); } else toast(r.error || "The hub is not reachable", "bad");
  };
  const cell = (val, on) => <input type="number" min="0" max="180" step="1" value={val} onChange={(e) => on(e.target.value)} />;
  return (
    <Card title="Calibration and tool">
      <p className="hint">Joint limits and homes as trained on the real arm: whole degrees 0 to 180, and a min above the max marks an inverted servo. Applying moves the virtual arm home, clears saved positions and posts the values to the server's /calibration, which the firmware reads at boot.</p>
      <table className="tbl"><thead><tr><th>Joint</th><th>Min</th><th>Max</th><th>Home</th></tr></thead><tbody>
        {JOINTS.map((n, j) => <tr key={n}><td>{j + 1} {n}</td><td>{cell(vals.lim[j][0], (x) => edit((s) => { s.lim[j][0] = x; }))}</td><td>{cell(vals.lim[j][1], (x) => edit((s) => { s.lim[j][1] = x; }))}</td><td>{cell(vals.homes[j], (x) => edit((s) => { s.homes[j] = x; }))}</td></tr>)}
      </tbody></table>
      <div className="row"><Field label="Tool length (mm)"><input type="number" min="0" max="300" step="1" value={tool} onChange={(e) => { dirty.current = true; setTool(e.target.value); }} /></Field><span className="hint">63 with the test ball, 53 without</span><div className="grow" /><Btn kind="primary" onClick={apply}>Apply</Btn></div>
    </Card>
  );
}

function Emulator({ emu }) {
  const [fw, setFw] = useState("ours"), [preset, setPreset] = useState("home_wifi");
  const [path, setPath] = useState("esp"), [lp, setLp] = useState(""), [rtt, setRtt] = useState(0), [jit, setJit] = useState(0), [loss, setLoss] = useState(0), [keep, setKeep] = useState(false);
  const busy = emu && (emu.state === "running" || emu.state === "building"), running = emu && emu.state === "running";
  const names = Object.keys(emu?.presets || {});
  const L0 = running ? emu.links[path === "both" ? "esp" : path] : null, sig = L0 ? JSON.stringify([path, L0.profile]) : "";
  const touched = useRef(false);
  useEffect(() => { touched.current = false; }, [sig]);
  useEffect(() => {
    if (!L0 || touched.current) return;
    setLp(L0.preset in emu.presets ? L0.preset : ""); setRtt(L0.profile.rtt_ms); setJit(L0.profile.jitter_ms); setLoss(+(L0.profile.loss * 100).toFixed(2)); setKeep(!!L0.profile.keepalive);
  }, [sig]); // eslint-disable-line react-hooks/exhaustive-deps
  const pickPreset = (name) => { setLp(name); const p = emu?.presets?.[name]; if (p) { touched.current = true; setRtt(p.rtt_ms); setJit(p.jitter_ms); setLoss(+(p.loss * 100).toFixed(2)); setKeep(!!p.keepalive); } };
  const mark = (fn) => (e) => { touched.current = true; setLp(""); fn(e); };
  const e = (b) => call("/api/emu", b);
  const apply = () => e(lp ? { action: "link", path, preset: lp } : { action: "link", path, rtt_ms: +rtt, jitter_ms: +jit, loss: +loss / 100, keepalive: keep });
  const n = emu?.net, L = emu?.links, lk = (k) => `${L[k].preset}${L[k].outage_s > 0 ? " (down)" : ""}`;
  return (
    <Card title="Emulated ESP32 and network" aside={<Pill tone={!emu || emu.state === "off" ? "idle" : emu.state === "error" ? "bad" : emu.state === "building" ? "warn" : "ok"}>{!emu ? "Off" : { off: "Off", building: "Compiling…", running: `Running ${emu.firmware} · ${Math.round(emu.uptime)} s`, error: "Stopped" }[emu.state]}</Pill>}>
      <p className="hint">Runs the real firmware code on this computer as an ESP32 that polls the local server over a simulated network. While it runs, the arm is the emulated servos, and the sticks travel over the controller link (even while SAFE: nothing reaches the real arm). The link numbers are assumptions until calibrated against the board.</p>
      <div className="grid2">
        <Field label="Firmware"><select value={fw} onChange={(x) => setFw(x.target.value)} disabled={busy}><option value="ours">Ours: stock + patches (magnet, sequences)</option><option value="patched">Stock + damper patch only</option><option value="stock">Stock UWE-RArm, as shipped</option><option value="rarm">Rewrite (firmware/rarm)</option></select></Field>
        <Field label="Network"><select value={preset} onChange={(x) => setPreset(x.target.value)} disabled={busy}>{(names.length ? names : ["home_wifi"]).map((p) => <option key={p}>{p}</option>)}</select></Field>
      </div>
      <div className="row">{busy ? <Btn icon="stop" kind="danger" onClick={() => e({ action: "stop" })}>Stop emulator</Btn> : <Btn icon="chip" kind="primary" onClick={() => e({ action: "start", firmware: fw, preset })}>Start emulator</Btn>}</div>
      {emu?.error ? <p className="hint warn">{emu.error}</p> : null}
      {running && (
        <>
          <div className="grid2">
            <Field label="Link"><select value={path} onChange={(x) => { touched.current = false; setPath(x.target.value); }}><option value="esp">ESP32 to server</option><option value="ctl">Controller to server</option><option value="both">Both</option></select></Field>
            <Field label="Preset"><select value={lp} onChange={(x) => pickPreset(x.target.value)}><option value="">custom values</option>{names.map((p) => <option key={p}>{p}</option>)}</select></Field>
            <Field label="Round trip (ms)"><input type="number" min="0" max="2000" step="5" value={rtt} onChange={mark((x) => setRtt(x.target.value))} /></Field>
            <Field label="Jitter (ms)"><input type="number" min="0" max="2000" step="5" value={jit} onChange={mark((x) => setJit(x.target.value))} /></Field>
            <Field label="Loss (%)"><input type="number" min="0" max="50" step="0.5" value={loss} onChange={mark((x) => setLoss(x.target.value))} /></Field>
            <label className="switch"><input type="checkbox" checked={keep} onChange={mark((x) => setKeep(x.target.checked))} /><span />reuse connection</label>
          </div>
          <div className="row"><Btn size="sm" onClick={apply}>Apply</Btn><Btn size="sm" onClick={() => e({ action: "outage", path, seconds: 3 })}>Drop link 3 s</Btn></div>
          <div className="stats tight">
            <Stat k="ESP32 polls" v={n ? n.polls : "-"} sub={n ? `${n.poll_fails} failed, ${n.connects} connections` : "waiting for the first poll"} />
            <Stat k="Poll time" v={n ? `${f1(n.poll_ms_mean, 0)} ms` : "-"} sub={n ? `95% under ${f1(n.poll_ms_p95, 0)} ms, worst ${f1(n.poll_ms_max, 0)} ms` : ""} />
            <Stat k="Longest stall" v={n ? `${f1(n.loop_gap_ms_max, 0)} ms` : "-"} sub="longest gap between loop passes" />
            <Stat small k="Links" v={n && L ? `ESP32 ${lk("esp")}, controller ${lk("ctl")}` : "-"} sub={L ? `last request: ESP32 ${f1(L.esp.stats.last_delay_ms, 0)} ms, controller ${f1(L.ctl.stats.last_delay_ms, 0)} ms` : ""} />
          </div>
        </>
      )}
    </Card>
  );
}

/* ------------------------------------------------------------------ packets */
const rawHttp = (s) => { const body = JSON.stringify(s); return `POST /input HTTP/1.1\nHost: localhost:8000\nContent-Type: application/json\nContent-Length: ${new TextEncoder().encode(body).length}\n\n${body}`; };

function Packets({ go }) {
  const f = useFeed();
  const [paused, setPaused] = useState(false), [filter, setFilter] = useState(""), [sel, setSel] = useState(null), [view, setView] = useState("http");
  const [cleared, setCleared] = useState(0);
  const frozen = useRef([]);
  if (!paused) frozen.current = f.packets;
  const rows = useMemo(() => {
    const q = filter.trim().toLowerCase();
    return frozen.current.filter((p) => p.n > cleared && (!q || p.d.join(" ").toLowerCase().includes(q))).slice(-300).reverse();
  }, [frozen.current, filter, cleared, f.packets.length]); // eslint-disable-line react-hooks/exhaustive-deps
  const chosen = frozen.current.find((p) => p.n === sel);
  const raw = chosen ? (view === "http" ? rawHttp(chosen.w || chosen.s) : JSON.stringify(chosen.w || chosen.s, null, 2)) : "Select a packet to see exactly what was sent.";
  const copy = async () => { try { await navigator.clipboard.writeText(raw); toast("Copied", "ok"); } catch (e) { toast("Copy was blocked by the browser", "bad"); } };
  const last = f.ppsHist[f.ppsHist.length - 1];
  return (
    <div className="stack">
      <div className="row wrap">
        <Btn icon={paused ? "play" : "pause"} onClick={() => setPaused((p) => !p)}>{paused ? "Resume" : "Pause"}</Btn>
        <Btn icon="trash" onClick={() => setCleared(f.packets.length ? f.packets[f.packets.length - 1].n : 0)}>Clear</Btn>
        <input type="text" className="grow" placeholder="Filter changes, e.g. LX or A down" aria-label="Filter packets" value={filter} onChange={(e) => setFilter(e.target.value)} />
        <Spark data={f.ppsHist} /><span><b className="num big">{f1(last)}</b> <span className="muted">packets / s</span></span>
      </div>
      <div className="grid2 split">
        <div className="scroll">
          <table className="tbl"><thead><tr><th>Time</th><th>#</th><th>LX</th><th>LY</th><th>RX</th><th>RY</th><th>Change</th></tr></thead><tbody>
            {rows.map((p) => <tr key={p.n} aria-selected={p.n === sel} onClick={() => setSel(p.n)}><td>{timeStr(p.t)}</td><td>{p.n}</td><td>{f1(p.s.lx, 2)}</td><td>{f1(p.s.ly, 2)}</td><td>{f1(p.s.rx, 2)}</td><td>{f1(p.s.ry, 2)}</td><td className="chg">{p.d.join(", ") || "no change"}</td></tr>)}
          </tbody></table>
          {!rows.length && <div className="emptybox">No packets yet. Start the local server and move a controller, or send some with the sticks once the output is ARMED.<div className="row"><Btn size="sm" onClick={() => go("services")}>Open Services</Btn></div></div>}
        </div>
        <div className="stack">
          <div className="row"><div className="seg">{[["http", "As sent"], ["json", "JSON"]].map(([k, l]) => <button key={k} type="button" className={view === k ? "on" : ""} onClick={() => setView(k)}>{l}</button>)}</div><div className="grow" /><Btn size="sm" icon="copy" onClick={copy}>Copy</Btn></div>
          <pre className="raw">{raw}</pre>
          <p className="hint">Packets are listed when the state changes. Unchanged heartbeats are not shown.</p>
        </div>
      </div>
    </div>
  );
}

/* ------------------------------------------------------------------ terminal + board */
const lineClass = (l) => /HOLD|lost|busy|not connected|failed|error/i.test(l) ? "w" : /WiFi OK|input active|Time OK|settings:/i.test(l) ? "g" : /^(JOG|#\d)/.test(l) ? "m" : "";
function Term({ lines, empty, cls = lineClass, tall }) {
  const el = useRef(null), stick = useRef(true);
  useEffect(() => { const n = el.current; if (n && stick.current) n.scrollTop = n.scrollHeight; });
  return (
    <div className={`term ${tall ? "tall" : ""}`} role="log" aria-live="off" ref={el} onScroll={(e) => { const n = e.currentTarget; stick.current = n.scrollHeight - n.scrollTop - n.clientHeight < 40; }}>
      {lines.length ? lines.slice(-400).map((l, i) => <div key={i}><span className="m">{timeStr(l.t).slice(0, 8)}</span>  <span className={cls(l.line)}>{l.line}</span></div>) : <span className="m">{empty}</span>}
    </div>
  );
}

const OUTS = [["OUT1", 25, "LX", "Left stick, left and right"], ["OUT2", 26, "LY", "Left stick, up and down"], ["OUT3", 27, "RX", "Right stick, left and right"], ["OUT4", 32, "RY", "Right stick, up and down"]];
function Board({ st }) {
  const f = useFeed();
  const [paused, setPaused] = useState(false), [clearAt, setClearAt] = useState(0), frozen = useRef([]);
  if (!paused) frozen.current = f.board;
  const lines = frozen.current.filter((b) => b.t > clearAt);
  const u = st.usb, pos = st.board.pos;
  const msg = { waiting: "No board detected. Plug the ESP32 in with a data cable. The page keeps looking by itself.", busy: "The COM port is held by another program. Close the Arduino Serial Monitor, or stop the controller window under Services.", suspended: "The port is released while firmware uploads.", off: "The USB link is switched off, so the board is not being read." }[u.phase];
  return (
    <div className="stack">
      <div className="stats">
        <Stat small k="USB port" v={u.port || "none"} sub={{ open: "open", busy: "held by another program", waiting: "not detected", off: "link switched off", suspended: "released for upload" }[u.phase]} />
        <Stat small k="Input source" v={st.board.source === "usb" ? "USB" : st.board.source === "wifi" ? "WiFi" : "unknown"} sub="what the board is obeying" />
        <Stat small k="Board WiFi" v={st.board.wifi === true ? "connected" : st.board.wifi === false ? "not connected" : "unknown"} sub="from its log" />
        <Stat small k="Speed at full stick" v={u.rate == null ? "-" : u.rate + " deg/s"} sub={u.rate === 0 ? "no limit" : ""} />
        <Stat small k="Servo travel" v={u.range == null ? "-" : u.range + " deg"} sub="assumed range of the pulse span" />
      </div>
      {msg ? <div className="note bad">{msg}</div> : null}
      <div className="grid2 top">
        <Card title="Board log" aside={<div className="row"><Btn size="sm" icon={paused ? "play" : "pause"} onClick={() => setPaused((p) => !p)}>{paused ? "Resume" : "Pause"}</Btn><Btn size="sm" icon="trash" onClick={() => setClearAt(Date.now() / 1000)}>Clear</Btn><Btn size="sm" onClick={async () => { const r = await api("/api/board", { cmd: "!show" }); if (!r.ok) toast("The USB link is not open", "bad"); }}>Read settings</Btn></div>}>
          <p className="hint">Everything the ESP32 prints over USB, live.</p>
          <Term tall lines={lines} empty="No board output yet. Lines appear here as soon as the ESP32 is on USB and prints something." />
        </Card>
        <div className="stack">
          <Card title="USB link">
            <p className="hint">The hub opens the board's COM port to read its log and send settings. Only one program can hold the port.</p>
            <div className="row"><Btn onClick={() => api("/api/usb", { enabled: !u.enabled })}>{u.enabled ? "Turn link off" : "Turn link on"}</Btn><span className="hint">{u.enabled ? "On: opens the port when a board is plugged in." : "Off: the port is free for other tools."}</span></div>
          </Card>
          <Card title="Outputs">
            <p className="hint">Last positions the board reported. They update when the input changes. Position lives on the board: a reset returns every output to centre.</p>
            {OUTS.map((o, i) => { const v = pos ? (pos[i] - 1500) / 500 : 0; return <div key={o[0]} className="meter"><span>{o[0]}</span><div className="bar mid"><i style={{ left: (v >= 0 ? 50 : 50 + v * 50) + "%", width: Math.abs(v) * 50 + "%" }} /></div><span className="n">{pos ? Math.round(pos[i]) + " us" : "-"}</span></div>; })}
          </Card>
          <Card title="Pin map">
            <table className="tbl"><thead><tr><th>Output</th><th>Pin</th><th>Stick</th></tr></thead><tbody>{OUTS.map((o) => <tr key={o[0]}><td>{o[0]}</td><td>GPIO{o[1]}</td><td>{o[2]}</td></tr>)}</tbody></table>
            <p className="hint">Servo pulses: 50 Hz, 1000 to 2000 us, centre 1500 us.</p>
          </Card>
        </div>
      </div>
    </div>
  );
}

/* ------------------------------------------------------------------ firmware */
function Firmware({ st }) {
  const f = useFeed(), fw = st.firmware, u = st.usb, running = fw.state === "running";
  const run = (a) => call("/api/firmware", { action: a });
  const canUp = !running && fw.secrets_present && !st.armed && !!u.port;
  const why = !fw.secrets_present ? "Compile needs secrets.h." : st.armed ? "Switch to SAFE to upload." : !u.port ? "Plug the board in to upload." : "";
  return (
    <div className="grid2 top">
      <div className="stack">
        <Card title="ESP32 sketch">
          <p className="hint">arduino/arm_receiver, built for the ESP32 Dev Module with arduino-cli.</p>
          <dl className="kv"><dt>Port</dt><dd className="mono">{u.port || "not detected"}</dd>
            <dt>WiFi and URL file</dt><dd>{fw.secrets_present ? <Pill tone="ok">secrets.h found</Pill> : <Pill tone="bad">secrets.h missing: copy secrets.example.h and fill it in</Pill>}</dd>
            <dt>Last job</dt><dd>{running ? <Pill tone="warn">{fw.action} running</Pill> : fw.finished_t ? <Pill tone={fw.state === "ok" ? "ok" : "bad"}>{fw.action} {fw.state === "ok" ? "succeeded" : "failed"}, {ageStr(fw.age)} ago</Pill> : <Pill>nothing run yet</Pill>}</dd></dl>
          <div className="row"><Btn kind="primary" icon="hammer" disabled={running || !fw.secrets_present} onClick={() => run("compile")}>Compile</Btn><Btn icon="upload" disabled={!canUp} onClick={() => run("upload")}>Upload</Btn><span className="hint">{why}</span></div>
          {running ? <div className="bar"><i className="shimmer" /></div> : null}
        </Card>
        <Card title="Size">
          {[["Flash", fw.flash_pct], ["Memory", fw.ram_pct]].map(([n, p]) => <div key={n} className="meter"><span>{n}</span><div className="bar"><i style={{ width: (p || 0) + "%" }} /></div><span className="n">{p != null ? p + "%" : "-"}</span></div>)}
          <p className="hint">Shown after a successful compile.</p>
        </Card>
        <Card title="Before you upload"><p className="hint">Uploading resets the board. Outputs return to centre, so a connected arm will move there. The hub releases the COM port for the upload and takes it back afterwards. Upload is only offered while the output is SAFE.</p></Card>
      </div>
      <Card title="Build output"><Term tall lines={f.logs.firmware || []} cls={(l) => (/error|fail/i.test(l) ? "w" : "")} empty="No build output yet. Press Compile to see it here." /></Card>
    </div>
  );
}

/* ------------------------------------------------------------------ services */
const SVC = [
  ["docker_engine", "Docker Desktop", "The engine the local server runs on.", ["start"]],
  ["server_docker", "Local server (Docker)", "Receives controller packets and forwards them. The normal way to run it.", ["start", "stop", "restart"]],
  ["server_python", "Local server (Python)", "The same server without Docker. Uses port 8000, so run only one of the two.", ["start", "stop", "restart"]],
  ["gui", "Controller window", "The pygame app for a physical Xbox controller. It also holds the board's COM port.", ["start", "stop"]],
];
const LOG_SRC = [["hub", "Hub"], ["server_docker", "Server (Docker)"], ["server_python", "Server (Python)"], ["gui", "Controller GUI"], ["docker_engine", "Docker Desktop"], ["firmware", "Firmware"]];
const LAB = { start: ["play", "Start"], stop: ["stop", "Stop"], restart: ["rev", "Rebuild"] };

function Services({ st }) {
  const f = useFeed(), [src, setSrc] = useState("hub");
  const c = st.config, fw = st.server.forward, paused = st.server.forward_enabled === false;
  return (
    <div className="stack">
      <Card title="Services">
        <div className="rowsl">
          {SVC.map(([id, name, desc, acts]) => {
            const s = st.services[id], tone = s.state === "running" ? "ok" : s.state === "busy" ? "warn" : "idle";
            return (
              <div key={id}>
                <div><b>{name}</b><div className="hint">{desc}</div>{s.detail ? <div className="hint mono">{s.detail}</div> : null}</div>
                <Pill tone={tone}>{s.state}</Pill>
                <div className="row">{acts.map((a) => {
                  const dis = s.state === "busy" || (a === "start" && s.state === "running") || ((a === "stop" || a === "restart") && s.state === "stopped" && id !== "docker_engine");
                  return <Btn key={a} size="sm" icon={LAB[a][0]} disabled={dis} onClick={() => call("/api/service", { name: id, action: a }, "Done")}>{id === "server_python" && a === "restart" ? "Restart" : LAB[a][1]}</Btn>;
                })}</div>
              </div>
            );
          })}
        </div>
      </Card>
      <div className="grid2 top">
        <Card title="Configuration">
          <dl className="kv"><dt>.env file</dt><dd><Pill tone={c.env_present ? "ok" : "warn"}>{c.env_present ? "found" : "missing"}</Pill></dd><dt>End-server address</dt><dd><Pill tone={c.forward_url_set ? "ok" : "warn"}>{c.forward_url_set ? "set (hidden)" : "not set"}</Pill></dd><dt>Firmware secrets.h</dt><dd><Pill tone={st.firmware.secrets_present ? "ok" : "warn"}>{st.firmware.secrets_present ? "found" : "missing"}</Pill></dd></dl>
          <p className="hint">Values stay in git-ignored files. This page only shows whether they exist.</p>
        </Card>
        <Card title="End-server forwarding">
          <dl className="kv"><dt>State</dt><dd><Pill tone={paused ? "idle" : fw.ok ? "ok" : fw.ok === false ? "bad" : "warn"}>{paused ? "paused (SAFE)" : fw.ok ? "forwarding" : fw.ok === false ? "failing" : "waiting"}</Pill></dd><dt>Sent</dt><dd className="mono">{fw.sent ?? "-"}</dd><dt>Failed</dt><dd className="mono">{fw.failed ?? "-"}</dd><dt>Last error</dt><dd className="mono">{fw.error || "none"}</dd></dl>
        </Card>
      </div>
      <Card title="Logs" aside={<div className="seg">{LOG_SRC.map(([k, l]) => <button key={k} type="button" className={src === k ? "on" : ""} onClick={() => setSrc(k)}>{l}</button>)}</div>}>
        <Term lines={f.logs[src] || []} cls={(l) => (/error|fail|not found/i.test(l) ? "w" : "")} empty="No output yet for this service." />
      </Card>
    </div>
  );
}

/* ------------------------------------------------------------------ project */
function Project({ st }) {
  const g = st.project;
  if (!g) return <div className="skel" />;
  if (!g.available) return <div className="note bad">Git is not available here, or this folder is not a repository.</div>;
  const repo = (g.url || "").replace(/^https:\/\/github\.com\//, "");
  return (
    <div className="stack">
      <div className="stats">
        <Stat small k="Repository" v={repo || "local"} sub={g.url ? <a href={g.url} target="_blank" rel="noopener noreferrer">Open on GitHub</a> : ""} />
        <Stat small k="Branch" v={g.branch} />
        <Stat k="Uncommitted files" v={g.dirty} sub={g.dirty ? "work not saved to git yet" : "all committed"} />
        <Stat k="Not pushed" v={g.unpushed} sub={g.unpushed ? "commits only on this PC" : "GitHub is up to date"} />
      </div>
      <div className="grid2 top">
        <Card title="Recent commits"><div className="commits">{g.commits.map((c, i) => <div key={i}><span className="h">{c.h}</span><span>{c.s}</span><span className="muted">{c.when}</span></div>)}{!g.commits.length && <p className="hint">No commits.</p>}</div></Card>
        <Card title="Run the hub"><p className="hint">This dashboard is served by dashboard/hub.py. Start it from the repository folder (or use the launcher):</p><pre className="raw tiny">python launch.py</pre><p className="hint">It opens this page and starts in SAFE. The output must be armed here before anything reaches the arm.</p></Card>
      </div>
    </div>
  );
}

/* ------------------------------------------------------------------ AI and jobs */
const JOB_EXAMPLE = `{
  "job": "fetch_key",
  "key_slot": 7,
  "deposit": "collection_tray",
  "max_speed_deg_s": 5
}`;
function AIPage() {
  const [text, setText] = useState(JOB_EXAMPLE), [out, setOut] = useState(null);
  const parse = () => { try { return [JSON.parse(text), null]; } catch (e) { return [null, e.message]; } };
  const validate = () => {
    const [j, err] = parse();
    if (err) return setOut({ bad: true, t: "Not valid JSON: " + err });
    if (typeof j !== "object" || j === null || Array.isArray(j)) return setOut({ bad: true, t: "An instruction packet must be a JSON object." });
    const k = Object.keys(j);
    setOut({ t: `Valid JSON with ${k.length} field${k.length === 1 ? "" : "s"}: ${k.join(", ")}. ${"job" in j ? "" : 'No "job" field yet. '}Nothing was sent.` });
  };
  return (
    <div className="grid2 top">
      <div className="stack">
        <Card title="Where this is heading">
          <div className="rowsl">
            {[["AI decision support", "An AI assistant decides what the arm should do and passes its answer to the control software (project brief, section 4). Ask the arm in the Studio already does this for saved sequences.", "started"],
              ["Instruction packets", "The hub sends JSON instruction packets to the microcontroller, which performs the task. Sequences go to the ESP32 through the server today.", "started"],
              ["Car hire key fetcher", "The likely use case: fetch the right key from a rack and drop it where the customer can collect it.", "planned"]].map(([t, d, s]) => (
              <div key={t}><div><b>{t}</b><div className="hint">{d}</div></div><Pill tone={s === "started" ? "ok" : "idle"}>{s}</Pill></div>
            ))}
          </div>
        </Card>
      </div>
      <Card title="Packet composer (dry run)">
        <p className="hint">Check an instruction packet before there is anything to send it to. The example is a draft: its field names are not a fixed format.</p>
        <textarea className="code" spellCheck="false" value={text} onChange={(e) => setText(e.target.value)} aria-label="Instruction packet (JSON)" />
        <div className="row wrap">
          <Btn kind="primary" icon="check" onClick={validate}>Validate</Btn>
          <Btn onClick={() => { const [j, err] = parse(); if (err) setOut({ bad: true, t: "Not valid JSON: " + err }); else setText(JSON.stringify(j, null, 2)); }}>Format</Btn>
          <Btn icon="copy" onClick={async () => { try { await navigator.clipboard.writeText(text); toast("Copied", "ok"); } catch (e) { toast("Copy was blocked by the browser", "bad"); } }}>Copy</Btn>
        </div>
        {out ? <div className={`note ${out.bad ? "bad" : "okk"}`} role="status">{out.t}</div> : null}
      </Card>
    </div>
  );
}

/* ------------------------------------------------------------------ shell */
export default function System({ status, page, setPage }) {
  if (!status) return <div className="sysbody"><div className="skel" /></div>;
  const body = {
    overview: <Overview st={status} go={setPage} />, tuning: <Tuning st={status} />, packets: <Packets go={setPage} />, board: <Board st={status} />,
    firmware: <Firmware st={status} />, services: <Services st={status} />, project: <Project st={status} />, ai: <AIPage />,
  }[page];
  return (
    <div className="system">
      <nav className="sysnav" aria-label="System pages">
        {PAGES.map(([id, label, ic]) => <button key={id} type="button" className={page === id ? "on" : ""} onClick={() => setPage(id)}><Icon n={ic} size={17} /><span>{label}</span></button>)}
      </nav>
      <div className="sysbody" key={page}>{body}</div>
    </div>
  );
}
