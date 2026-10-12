import React, { useCallback, useEffect, useRef, useState } from "react";
import { AnimatePresence, motion } from "framer-motion";
import { api, getJson, getState, refreshSeqs, set, toast, useStore } from "./store.js";
import { Btn, Field, Icon, Num, Pill } from "./ui.jsx";
import { KEY_HELP, pad } from "./input.js";

const spring = { type: "spring", stiffness: 420, damping: 36 };
const sameName = /^[A-Za-z0-9][A-Za-z0-9_-]{0,47}$/;

/* ------------------------------------------------------------------ Teach: the draft being taught + the library */
export function useDraft() {
  const saved = useStore((s) => s.pose?.saved);
  const [draft, setDraft] = useState([]);
  const load = useCallback(async () => {
    const j = await getJson("/api/sequence");
    setDraft(j && j.poses ? j.poses : []);
  }, []);
  useEffect(() => { load(); }, [saved, load]);
  return [draft, load];
}

const packet = (poses) => ({ format: "roobot-arm-sequence", version: 2, name: "", description: "", requires: {}, ends: {}, poses });

export function TeachPanel({ draft, reloadDraft, onPick, picked }) {
  const seqs = useStore((s) => s.seqs);
  const [name, setName] = useState(""), [desc, setDesc] = useState(""), [needs, setNeeds] = useState(""), [leaves, setLeaves] = useState("");
  const [confirm, setConfirm] = useState(null), [rename, setRename] = useState(null);
  const confirmTimer = useRef();

  const record = async () => { const r = await api("/api/sim", { save_here: true }); if (!r.ok) toast(r.error || "Could not record", "bad"); reloadDraft(); };
  const clear = async () => { await api("/api/sim", { clear_saved: true }); reloadDraft(); };
  const push = async (poses) => {
    if (!poses.length) return clear();
    const r = await api("/api/sequence", { load: packet(poses) });
    if (!r.ok) toast(r.error || "Could not update the draft", "bad");
    reloadDraft();
  };
  const setAction = (i, action) => push(draft.map((p, k) => (k === i ? { ...p, action: action || null } : p)));
  const remove = (i) => push(draft.filter((_, k) => k !== i));
  const move = (i, d) => { const j = i + d; if (j < 0 || j >= draft.length) return; const c = draft.slice(); [c[i], c[j]] = [c[j], c[i]]; push(c); };

  const save = async () => {
    const n = name.trim();
    if (!draft.length) return toast("Record at least one pose first", "bad");
    if (n && !sameName.test(n)) return toast("Name: letters, digits, - and _ only", "bad");
    const r = await api("/api/sequence", { name: n, description: desc.trim(), requires: needs.trim() ? { key_in: needs.trim() } : {}, ends: leaves.trim() ? { key_on: leaves.trim() } : {} });
    toast(r.ok ? `Saved ${r.packet.name}` : (r.error || "Could not save"), r.ok ? "ok" : "bad");
    if (r.ok) { setName(""); setDesc(""); setNeeds(""); setLeaves(""); refreshSeqs(); }
  };
  const run = async (n, reverse) => {
    const r = await api("/api/sequence", { name: n, use: true, run: true, reverse });
    toast(r.ok ? `Running ${r.name}${reverse ? " reversed" : ""}` : (r.error || "Could not run"), r.ok ? "ok" : "bad");
  };
  const edit = async (n) => {
    const j = await getJson("/api/sequences/" + encodeURIComponent(n));
    if (!j) return toast("Could not open it", "bad");
    const r = await api("/api/sequence", { load: j });
    if (r.ok) { setName(n); setDesc(j.description || ""); setNeeds(j.requires?.key_in || ""); setLeaves(j.ends?.key_on || ""); reloadDraft(); toast(`Loaded ${n} into the draft`, "ok"); }
  };
  const del = async (n) => {
    if (confirm !== n) { setConfirm(n); clearTimeout(confirmTimer.current); confirmTimer.current = setTimeout(() => setConfirm(null), 3000); return; }
    setConfirm(null);
    const r = await api("/api/sequence", { delete: n });
    toast(r.ok ? `Deleted ${n}` : (r.error || "Could not delete"), r.ok ? "ok" : "bad"); refreshSeqs();
  };
  const doRename = async (from) => {
    const to = rename.to.trim(); setRename(null);
    if (!to || to === from) return;
    const r = await api("/api/sequence", { rename: { from, to } });
    toast(r.ok ? `Renamed to ${to}` : (r.error || "Could not rename"), r.ok ? "ok" : "bad"); refreshSeqs();
  };

  return (
    <div className="stack">
      <section className="card">
        <header className="cardhead"><h3>Draft</h3><span className="muted">{draft.length}/20 poses</span></header>
        <p className="hint">Drive the arm where you want it, then <b>Record pose</b>. Poses are visited in order, and the arm waits to arrive at each. A pose can grab or release the magnet; leave it blank to just go there.</p>
        <div className="row">
          <Btn icon="plus" kind="primary" onClick={record}>Record pose</Btn>
          <Btn icon="trash" onClick={clear} disabled={!draft.length}>Clear</Btn>
        </div>
        <ol className="poses">
          <AnimatePresence initial={false}>
            {draft.map((p, i) => (
              <motion.li key={i + ":" + p.x + p.y + p.z} layout initial={{ opacity: 0, y: -8 }} animate={{ opacity: 1, y: 0 }} exit={{ opacity: 0, height: 0 }} transition={spring}
                className={picked === i ? "pose on" : "pose"} onClick={() => onPick(i)}>
                <span className="idx">{i + 1}</span>
                <span className="xyz"><Num v={p.x} d={0} /> <Num v={p.y} d={0} /> <Num v={p.z} d={0} /><small>{Math.round(p.angle)}°</small></span>
                <select value={p.action || ""} onClick={(e) => e.stopPropagation()} onChange={(e) => setAction(i, e.target.value)} aria-label={`Magnet action at pose ${i + 1}`}>
                  <option value="">go to</option><option value="grab">grab</option><option value="release">release</option>
                </select>
                <span className="mini">
                  <button type="button" aria-label="Move up" onClick={(e) => { e.stopPropagation(); move(i, -1); }}>↑</button>
                  <button type="button" aria-label="Move down" onClick={(e) => { e.stopPropagation(); move(i, 1); }}>↓</button>
                  <button type="button" aria-label="Remove pose" onClick={(e) => { e.stopPropagation(); remove(i); }}><Icon n="x" size={14} /></button>
                </span>
              </motion.li>
            ))}
          </AnimatePresence>
          {!draft.length && <li className="empty">No poses yet. The A button on a controller records one too.</li>}
        </ol>
        <div className="grid2">
          <Field label="Name"><input value={name} onChange={(e) => setName(e.target.value)} placeholder="box2_to_tray" maxLength={48} /></Field>
          <Field label="What it does"><input value={desc} onChange={(e) => setDesc(e.target.value)} placeholder="move the key from box 2 to the tray" maxLength={400} /></Field>
          <Field label="Needs a key in"><input value={needs} onChange={(e) => setNeeds(e.target.value)} placeholder="box2 (optional)" maxLength={20} /></Field>
          <Field label="Leaves a key on"><input value={leaves} onChange={(e) => setLeaves(e.target.value)} placeholder="tray (optional)" maxLength={20} /></Field>
        </div>
        <Btn icon="check" kind="primary" onClick={save} disabled={!draft.length}>Save to library</Btn>
      </section>

      <section className="card">
        <header className="cardhead"><h3>Library</h3><span className="muted">{seqs.length} saved</span></header>
        <ul className="lib">
          <AnimatePresence initial={false}>
            {seqs.map((q) => (
              <motion.li key={q.name} layout initial={{ opacity: 0, scale: 0.98 }} animate={{ opacity: 1, scale: 1 }} exit={{ opacity: 0, x: 24 }} transition={spring} className="seq">
                <div className="seqtop">
                  {rename && rename.from === q.name ? (
                    <input autoFocus value={rename.to} onChange={(e) => setRename({ ...rename, to: e.target.value })} onKeyDown={(e) => { if (e.key === "Enter") doRename(q.name); if (e.key === "Escape") setRename(null); }} onBlur={() => doRename(q.name)} aria-label="New name" />
                  ) : <b>{q.name}</b>}
                  <span className="muted">{q.count} poses</span>
                </div>
                {q.description ? <p className="desc">{q.description}</p> : null}
                <div className="row">
                  <Btn size="sm" kind="primary" icon="play" onClick={() => run(q.name, false)}>Run</Btn>
                  <Btn size="sm" icon="rev" onClick={() => run(q.name, true)}>Reverse</Btn>
                  <Btn size="sm" icon="pen" onClick={() => edit(q.name)}>Edit</Btn>
                  <Btn size="sm" onClick={() => setRename({ from: q.name, to: q.name })}>Rename</Btn>
                  <Btn size="sm" kind={confirm === q.name ? "danger" : ""} icon="trash" onClick={() => del(q.name)}>{confirm === q.name ? "Sure?" : "Delete"}</Btn>
                </div>
              </motion.li>
            ))}
          </AnimatePresence>
          {!seqs.length && <li className="empty">Nothing saved yet.</li>}
        </ul>
      </section>
    </div>
  );
}

/* ------------------------------------------------------------------ Ask */
export function AskPanel({ agent }) {
  const [text, setText] = useState("");
  const seqs = useStore((s) => s.seqs);
  const task = agent?.task || { state: "idle" };
  const busy = task.state === "working" || task.state === "running";
  const go = async (t) => {
    const q = (t ?? text).trim(); if (!q) return;
    const r = await api("/api/ask", { request: q });
    if (!r.ok) toast(r.error || "Could not start", "bad"); else setText("");
  };
  return (
    <div className="stack">
      <section className="card">
        <header className="cardhead"><h3>Ask the arm</h3><Pill tone={agent?.ai === "groq" ? "ok" : "idle"} title={agent?.ai === "groq" ? "A Groq model picks the sequence" : "No GROQ_API_KEY: keyword matching picks the sequence"}>{agent?.ai === "groq" ? "Groq" : "Keywords"}</Pill></header>
        <p className="hint">Say what you want done. It checks the arm is at home grip (and goes there first if not), picks a saved sequence, and runs it. “Put it back” undoes the last move.</p>
        <form className="ask" onSubmit={(e) => { e.preventDefault(); go(); }}>
          <input value={text} onChange={(e) => setText(e.target.value)} placeholder="put the key from box 2 onto the tray" maxLength={300} aria-label="What should the arm do?" disabled={busy} />
          <Btn kind="primary" icon="arrow" type="submit" disabled={busy || !text.trim()}>Do it</Btn>
        </form>
        {!!seqs.length && <div className="chips">{seqs.slice(0, 8).map((q) => <button key={q.name} type="button" className="chip" disabled={busy} onClick={() => go(q.description || q.name.replace(/_/g, " "))}>{q.description || q.name}</button>)}<button type="button" className="chip" disabled={busy} onClick={() => go("put it back")}>put it back</button></div>}
      </section>
      <section className="card">
        <header className="cardhead"><h3>Task</h3><Pill tone={task.state === "done" ? "ok" : task.state === "failed" ? "bad" : busy ? "warn" : "idle"}>{task.state}</Pill></header>
        {task.request ? <p className="quote">“{task.request}”</p> : <p className="hint">Nothing running.</p>}
        <ol className="timeline">
          {(task.log || []).map((l, i) => <motion.li key={i} initial={{ opacity: 0, x: -6 }} animate={{ opacity: 1, x: 0 }} className={/^(Cannot|Could not|Failed|No sequence)/.test(l) ? "bad" : i === task.log.length - 1 && busy ? "now" : "done"}>{l}</motion.li>)}
        </ol>
      </section>
    </div>
  );
}

/* ------------------------------------------------------------------ State */
export function StatePanel({ agent }) {
  const [copied, setCopied] = useState(false);
  if (!agent) return <div className="card"><p className="hint">Waiting for the arm…</p></div>;
  const rows = [["Source", agent.source === "esp32" ? "ESP32 (reporting)" : "Virtual arm"], ["Mode", agent.mode], ["At home", agent.at_home ? "yes" : "no"], ["Magnet", agent.magnet ? "on" : "off"], ["Holding a key", agent.holding ? "yes" : "no"], ["Home grip", agent.home_grip ? "yes" : "no"]];
  const json = JSON.stringify(agent, null, 2);
  return (
    <div className="stack">
      <section className="card">
        <header className="cardhead"><h3>What the AI sees</h3><Btn size="sm" icon="copy" onClick={() => { navigator.clipboard?.writeText(json).then(() => { setCopied(true); setTimeout(() => setCopied(false), 1500); }); }}>{copied ? "Copied" : "Copy"}</Btn></header>
        <dl className="kv">{rows.map(([k, v]) => <React.Fragment key={k}><dt>{k}</dt><dd>{v}</dd></React.Fragment>)}</dl>
        <h4>Keys</h4>
        <div className="keys">{Object.entries(agent.keys || {}).map(([id, where]) => <span key={id} className="keychip"><Icon n="link" size={13} /> key {id} · {where}</span>)}</div>
      </section>
      <section className="card"><header className="cardhead"><h3>Packet</h3><span className="muted">GET /api/state</span></header><pre className="json">{json}</pre></section>
    </div>
  );
}

/* ------------------------------------------------------------------ Pad: on-screen sticks + keyboard help */
function Stick({ label, onMove }) {
  const base = useRef(), [pos, setPos] = useState([0, 0]);
  const drag = useRef(false);
  const upd = (e) => {
    const r = base.current.getBoundingClientRect();
    let x = (e.clientX - (r.left + r.width / 2)) / (r.width / 2), y = (e.clientY - (r.top + r.height / 2)) / (r.height / 2);
    const m = Math.hypot(x, y); if (m > 1) { x /= m; y /= m; }
    setPos([x, y]); onMove(Math.round(x * 100) / 100, Math.round(y * 100) / 100);
  };
  const end = () => { drag.current = false; setPos([0, 0]); onMove(0, 0); };
  return (
    <div className="stickwrap">
      <div ref={base} className="stick" role="slider" aria-label={label} aria-valuetext={`${pos[0].toFixed(1)}, ${pos[1].toFixed(1)}`} tabIndex={0}
        onPointerDown={(e) => { drag.current = true; e.currentTarget.setPointerCapture(e.pointerId); upd(e); }} onPointerMove={(e) => drag.current && upd(e)} onPointerUp={end} onPointerCancel={end}>
        <div className="knob" style={{ transform: `translate(${pos[0] * 34}px, ${pos[1] * 34}px)` }} />
      </div>
      <small>{label}</small>
    </div>
  );
}
export function PadPanel({ status }) {
  const [kb, setKb] = useState(pad.keyboard);
  const [gp, setGp] = useState(null);
  useEffect(() => { const t = setInterval(() => setGp(pad.gamepad), 800); return () => clearInterval(t); }, []);
  const live = status && (status.armed || status.emu_input);
  const press = (k, v) => { pad.virt.buttons[k] = v; };
  const btn = (k, l, cls) => <button type="button" className={`abxy ${cls}`} onPointerDown={() => press(k, true)} onPointerUp={() => press(k, false)} onPointerLeave={() => press(k, false)} onPointerCancel={() => press(k, false)}>{l}</button>;
  return (
    <div className="stack">
      <section className="card">
        <header className="cardhead"><h3>Controller</h3><Pill tone={live ? "ok" : "warn"}>{live ? (status.armed ? "ARMED" : "emulator") : "SAFE"}</Pill></header>
        {!live && <p className="hint warn">Input is blocked while the output is SAFE. Arm it (top right), or start the emulated ESP32 on the ESP32 tab.</p>}
        <div className="sticks">
          <Stick label="Left · move" onMove={(x, y) => { pad.virt.lx = x; pad.virt.ly = y; }} />
          <div className="abxys">{btn("y", "Y", "y")}{btn("x", "X", "x")}{btn("b", "B", "b")}{btn("a", "A", "a")}</div>
          <Stick label="Right · height / angle" onMove={(x, y) => { pad.virt.rx = x; pad.virt.ry = y; }} />
        </div>
        <p className="hint">{gp ? `Gamepad: ${gp.slice(0, 48)}` : "Plug in an Xbox or PlayStation controller and it is used automatically."}</p>
      </section>
      <section className="card">
        <header className="cardhead"><h3>Keyboard</h3>
          <label className="switch"><input type="checkbox" checked={kb} onChange={(e) => { pad.keyboard = e.target.checked; setKb(e.target.checked); }} /><span /> on</label></header>
        <dl className="kv keys2">{KEY_HELP.map(([k, v]) => <React.Fragment key={k}><dt><kbd>{k}</kbd></dt><dd>{v}</dd></React.Fragment>)}</dl>
      </section>
    </div>
  );
}

/* ------------------------------------------------------------------ ESP32 / setup */
export function SetupPanel({ status, agent }) {
  const emu = status?.emu, sim = status?.sim;
  const [fw, setFw] = useState("ours"), [preset, setPreset] = useState("home_wifi");
  const running = emu && (emu.state === "running" || emu.state === "building");
  const start = async () => { const r = await api("/api/emu", { action: "start", firmware: fw, preset }); if (!r.ok) toast(r.error || "Could not start", "bad"); };
  const stop = async () => { await api("/api/emu", { action: "stop" }); };
  const [rate, setRate] = useState(null);
  const rateShown = rate ?? sim?.rate ?? 30;
  return (
    <div className="stack">
      <section className="card">
        <header className="cardhead"><h3>Where the arm runs</h3><Pill tone={agent?.source === "esp32" ? "ok" : "idle"}>{agent?.source === "esp32" ? "ESP32" : "Virtual arm"}</Pill></header>
        <p className="hint">Without an ESP32 reporting, everything drives the virtual arm in the hub. Start the emulated ESP32 to run our real firmware code on this PC over a simulated network, or connect a board.</p>
        <div className="grid2">
          <Field label="Firmware"><select value={fw} onChange={(e) => setFw(e.target.value)} disabled={running}><option value="ours">Ours: stock + patches (magnet, sequences)</option><option value="patched">Stock + damper patch only</option><option value="stock">Stock, as shipped</option><option value="rarm">Rewrite</option></select></Field>
          <Field label="Network"><select value={preset} onChange={(e) => setPreset(e.target.value)} disabled={running}>{Object.keys(emu?.presets || { home_wifi: 1 }).map((p) => <option key={p}>{p}</option>)}</select></Field>
        </div>
        <div className="row">
          {running ? <Btn icon="stop" kind="danger" onClick={stop}>Stop emulator</Btn> : <Btn icon="chip" kind="primary" onClick={start}>Start emulated ESP32</Btn>}
          <Pill tone={emu?.state === "running" ? "ok" : emu?.state === "error" ? "bad" : emu?.state === "building" ? "warn" : "idle"}>{emu?.state || "stopped"}</Pill>
        </div>
        {emu?.error ? <p className="hint warn">{emu.error}</p> : null}
        {emu?.net ? <p className="hint mono">{emu.net.polls} polls · mean {Math.round(emu.net.poll_ms_mean || 0)} ms · {emu.net.poll_fails} failed</p> : null}
      </section>
      <section className="card">
        <header className="cardhead"><h3>Speed</h3><span className="num">{Math.round(rateShown)}°/s</span></header>
        <input type="range" min="5" max="60" step="1" value={rateShown} onChange={(e) => { setRate(+e.target.value); }} onPointerUp={async () => { await api("/api/sim", { rate: rateShown }); setRate(null); }} onKeyUp={async () => { await api("/api/sim", { rate: rateShown }); setRate(null); }} aria-label="Joint speed limit" />
        <p className="hint">Fastest any joint may move. The real firmware accepts 1 to 60. Start slow on hardware.</p>
      </section>
      <section className="card">
        <header className="cardhead"><h3>Phone</h3></header>
        {status?.phone_url ? <p className="hint mono wrap">{status.phone_url}</p> : <p className="hint">Start with the launcher (not --local-only) to get a phone link.</p>}
        <a className="btn" href="/"><Icon n="ext" size={16} /> Open the classic dashboard</a>
      </section>
    </div>
  );
}
