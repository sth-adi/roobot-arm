import React, { Suspense, lazy, useCallback, useEffect, useMemo, useState } from "react";
import { AnimatePresence, motion } from "framer-motion";
import { api, connect, getJson, getState, live, refreshSeqs, set, toast, useStore } from "./store.js";
import { startInput } from "./input.js";
import { Btn, Icon, Num, Pill } from "./ui.jsx";
import { AskPanel, PadPanel, SetupPanel, StatePanel, TeachPanel, useDraft } from "./Panels.jsx";
import Palette from "./Palette.jsx";
import System, { PAGES as SYS_PAGES } from "./System.jsx";

const Scene = lazy(() => import("./Scene.jsx"));

const TABS = [["teach", "Teach", "list"], ["ask", "Ask", "spark"], ["pad", "Control", "pad"], ["state", "State", "pulse"], ["setup", "ESP32", "chip"]];
const VIEWS = [["iso", "3D"], ["top", "Top"], ["side", "Side"], ["front", "Front"]];

function useAgent() {
  const [agent, setAgent] = useState(null);
  useEffect(() => {
    let live = true;
    const tick = async () => { const j = await getJson("/api/state"); if (live && j) setAgent(j); };
    tick();
    const t = setInterval(tick, 700);
    return () => { live = false; clearInterval(t); };
  }, []);
  return agent;
}

export default function App() {
  const online = useStore((s) => s.online);
  const status = useStore((s) => s.status);
  const pose = useStore((s) => s.pose);
  const seqs = useStore((s) => s.seqs);
  const toasts = useStore((s) => s.toasts);
  const [tab, setTab] = useState(() => { try { return localStorage.getItem("studio.tab") || "teach"; } catch (e) { return "teach"; } });
  const [open, setOpen] = useState(true);
  const [rich, setRich] = useState(() => { try { return localStorage.getItem("studio.rich") === "1"; } catch (e) { return false; } });
  useEffect(() => { try { localStorage.setItem("studio.rich", rich ? "1" : "0"); } catch (e) { /* private window */ } }, [rich]);
  const [area, setArea] = useState(() => { try { return localStorage.getItem("studio.area") === "system" ? "system" : "studio"; } catch (e) { return "studio"; } });
  const [page, setPage] = useState(() => { try { return localStorage.getItem("studio.page") || "overview"; } catch (e) { return "overview"; } });
  useEffect(() => { try { localStorage.setItem("studio.area", area); localStorage.setItem("studio.page", page); } catch (e) { /* private window */ } }, [area, page]);
  const openPage = (p) => { setPage(p); setArea("system"); };
  const [view, setView] = useState({ name: "iso", tick: 0 });
  const [palette, setPalette] = useState(false);
  const [armDlg, setArmDlg] = useState(false);
  const [picked, setPicked] = useState(null);
  const agent = useAgent();
  const [draft, reloadDraft] = useDraft();

  useEffect(() => { connect(); startInput(); }, []);
  useEffect(() => { try { localStorage.setItem("studio.tab", tab); } catch (e) { /* private window */ } }, [tab]);
  useEffect(() => {
    const k = (e) => {
      if ((e.metaKey || e.ctrlKey) && e.key.toLowerCase() === "k") { e.preventDefault(); setPalette((v) => !v); }
      else if (e.key === "Escape" && getState().status?.armed) { api("/api/arm", { armed: false }); toast("Output is SAFE", "ok"); }
    };
    window.addEventListener("keydown", k);
    return () => window.removeEventListener("keydown", k);
  }, []);

  const armed = !!status?.armed;
  const toggleArm = async () => {
    if (armed) { const r = await api("/api/arm", { armed: false }); toast(r.ok ? "Output is SAFE" : "Could not reach the hub", r.ok ? "ok" : "bad"); }
    else setArmDlg(true);
  };
  const go = (name) => setView((v) => ({ name, tick: v.tick + 1 }));
  const openTab = (t) => { setTab(t); setOpen(true); };
  const onPickPose = useCallback((i) => { setPicked(i); setTab("teach"); setOpen(true); }, []);

  const items = useMemo(() => [
    { id: "home", label: "Go home", hint: "move the arm to its home pose", icon: "home", run: async () => { const r = await api("/api/sim", { home: true }); if (!r.ok) toast(r.error || "Failed", "bad"); } },
    { id: "record", label: "Record pose", hint: "add the current position to the draft", icon: "plus", run: async () => { const r = await api("/api/sim", { save_here: true }); if (!r.ok) toast(r.error || "Failed", "bad"); reloadDraft(); openTab("teach"); } },
    { id: "clear", label: "Clear draft", hint: "remove all recorded poses", icon: "trash", run: async () => { await api("/api/sim", { clear_saved: true }); reloadDraft(); } },
    { id: "magnet", label: "Toggle magnet", hint: "simulation only", icon: "magnet", run: () => api("/api/world", { magnet: "toggle" }) },
    { id: "reset", label: "Reset keys", hint: "put the keys back in the boxes", icon: "rev", run: () => api("/api/world", { reset: true }) },
    { id: "arm", label: armed ? "Return to SAFE" : "Arm the output", hint: "Esc also returns to SAFE", icon: "shield", run: toggleArm },
    ...VIEWS.map(([n, l]) => ({ id: "v" + n, label: "View: " + l, hint: "camera", icon: "cube", run: () => go(n) })),
    { id: "astudio", label: "Go to Studio", hint: "3D view and teaching", icon: "cube", run: () => setArea("studio") },
    ...SYS_PAGES.map(([p, l, ic]) => ({ id: "p" + p, label: "System: " + l, hint: "page", icon: ic, run: () => openPage(p) })),
    ...TABS.map(([t, l, ic]) => ({ id: "t" + t, label: "Open " + l, hint: "panel", icon: ic, run: () => openTab(t) })),
    { id: "legacy", label: "Open the classic dashboard", hint: "the original pages", icon: "ext", run: () => { location.href = "/?legacy=1"; } },
    ...seqs.flatMap((q) => [
      { id: "run-" + q.name, label: "Run " + q.name, hint: q.description, icon: "play", run: async () => { const r = await api("/api/sequence", { name: q.name, use: true, run: true }); toast(r.ok ? `Running ${q.name}` : (r.error || "Failed"), r.ok ? "ok" : "bad"); } },
      { id: "rev-" + q.name, label: "Run " + q.name + " reversed", hint: "the opposite move", icon: "rev", run: async () => { const r = await api("/api/sequence", { name: q.name, use: true, run: true, reverse: true }); toast(r.ok ? `Running ${q.name} reversed` : (r.error || "Failed"), r.ok ? "ok" : "bad"); } },
    ]),
    { id: "ask", label: "Ask the arm…", hint: "type your request, then Enter", icon: "spark", run: async (q) => { openTab("ask"); if (q) { const r = await api("/api/ask", { request: q }); if (!r.ok) toast(r.error || "Failed", "bad"); } } },
  ], [armed, seqs, reloadDraft]); // eslint-disable-line react-hooks/exhaustive-deps

  const src = agent?.source === "esp32" ? (status?.emu?.running ? "Emulated ESP32" : "ESP32") : "Virtual arm";
  return (
    <div className="shell">
      <header className="top">
        <div className="brand"><span className="logo"><Icon n="cube" size={17} /></span><b>Arm Studio</b></div>
        <div className="pills">
          <Pill tone={online ? "ok" : "bad"}>{online ? "Hub" : "Hub offline"}</Pill>
          <Pill tone={agent?.source === "esp32" ? "ok" : "idle"} title="Where the arm is running">{src}</Pill>
          <Pill tone={agent?.ai === "groq" ? "ok" : "idle"} title="Who picks the sequence for Ask">{agent?.ai === "groq" ? "AI: Groq" : "AI: keywords"}</Pill>
        </div>
        <div className="areas" role="tablist" aria-label="Area">
          {[["studio", "Studio", "cube"], ["system", "System", "sliders"]].map(([k, l, ic]) => <button key={k} type="button" role="tab" aria-selected={area === k} className={area === k ? "on" : ""} onClick={() => setArea(k)}><Icon n={ic} size={15} /><span>{l}</span></button>)}
        </div>
        <div className="grow" />
        <button type="button" className="kbd" onClick={() => setPalette(true)} aria-label="Open the command palette"><Icon n="cmd" size={14} /> <span>Search & commands</span><kbd>Ctrl K</kbd></button>
        <button type="button" className={`arm ${armed ? "on" : ""}`} onClick={toggleArm} aria-pressed={armed}><Icon n="shield" size={16} />{armed ? "ARMED" : "SAFE"}</button>
      </header>

      {area === "system" && <System status={status} page={page} setPage={setPage} />}
      <main className="stage" hidden={area === "system"}>
        <section className="viewport" aria-label="3D view of the arm">
          <Suspense fallback={<div className="loading">Loading the 3D view…</div>}>
            <Scene rich={rich} view={view} draft={draft} onPick={onPickPose} />
          </Suspense>
          <div className="hud tl">
            <Pill tone={pose?.replaying ? "warn" : "idle"}>{pose?.replaying ? "running" : pose?.mode || "idle"}</Pill>
            {pose && <Readout />}
            {pose?.world && <div className={`mag ${pose.world.magnet ? "on" : ""}`}><Icon n="magnet" size={14} /> magnet {pose.world.magnet ? "ON" : "off"}</div>}
          </div>
          <div className="hud tr">
            {VIEWS.map(([n, l]) => <button key={n} type="button" className={view.name === n ? "on" : ""} onClick={() => go(n)}>{l}</button>)}
            <button type="button" className={rich ? "on" : ""} onClick={() => setRich((v) => !v)} title="Reflections, antialiasing and a grid. Needs a graphics card.">{rich ? "Rich" : "Lite"}</button>
          </div>
          <Timeline agent={agent} draft={draft} />
          {!online && <div className="offline">The hub is not reachable. Start it with Start Dashboard.</div>}
          {!open && <button type="button" className="reopen" onClick={() => setOpen(true)}>Panels</button>}
        </section>

        <AnimatePresence initial={false}>
          {open && (
            <motion.aside className="dock" initial={{ x: 40, opacity: 0 }} animate={{ x: 0, opacity: 1 }} exit={{ x: 40, opacity: 0 }} transition={{ type: "spring", stiffness: 380, damping: 34 }} aria-label="Panels">
              <nav className="tabs" role="tablist">
                {TABS.map(([t, l, ic]) => (
                  <button key={t} type="button" role="tab" aria-selected={tab === t} className={tab === t ? "on" : ""} onClick={() => setTab(t)}>
                    {tab === t && <motion.span layoutId="tabpill" className="tabpill" transition={{ type: "spring", stiffness: 500, damping: 40 }} />}
                    <Icon n={ic} size={16} /><span>{l}</span>
                  </button>
                ))}
                <button type="button" className="collapse" aria-label="Hide panels" onClick={() => setOpen(false)}><Icon n="x" size={15} /></button>
              </nav>
              <div className="dockbody">
                <AnimatePresence mode="wait" initial={false}>
                  <motion.div key={tab} initial={{ opacity: 0, y: 8 }} animate={{ opacity: 1, y: 0 }} exit={{ opacity: 0, y: -6 }} transition={{ duration: 0.14 }}>
                    {tab === "teach" && <TeachPanel draft={draft} reloadDraft={reloadDraft} onPick={setPicked} picked={picked} />}
                    {tab === "ask" && <AskPanel agent={agent} />}
                    {tab === "pad" && <PadPanel status={status} />}
                    {tab === "state" && <StatePanel agent={agent} />}
                    {tab === "setup" && <SetupPanel status={status} agent={agent} />}
                  </motion.div>
                </AnimatePresence>
              </div>
            </motion.aside>
          )}
        </AnimatePresence>
      </main>

      <Palette open={palette} onClose={() => setPalette(false)} items={items} />
      <AnimatePresence>
        {armDlg && (
          <motion.div className="scrim" initial={{ opacity: 0 }} animate={{ opacity: 1 }} exit={{ opacity: 0 }} onMouseDown={() => setArmDlg(false)}>
            <motion.div className="dialog" role="alertdialog" aria-labelledby="armt" initial={{ scale: 0.96, y: 10 }} animate={{ scale: 1, y: 0 }} onMouseDown={(e) => e.stopPropagation()}>
              <h2 id="armt">Arm the output?</h2>
              <p>While armed, controller input and sequences are sent to the arm over USB and the network. Keep clear of it. Press Escape at any time to return to SAFE.</p>
              <div className="row end"><Btn onClick={() => setArmDlg(false)}>Cancel</Btn><Btn kind="danger" onClick={async () => { setArmDlg(false); const r = await api("/api/arm", { armed: true }); toast(r.ok ? "Output is ARMED" : "Could not reach the hub", r.ok ? "" : "bad"); }}>Arm output</Btn></div>
            </motion.div>
          </motion.div>
        )}
      </AnimatePresence>
      <div className="toasts" aria-live="polite">
        <AnimatePresence>{toasts.map((t) => <motion.div key={t.id} layout initial={{ opacity: 0, y: 12, scale: 0.96 }} animate={{ opacity: 1, y: 0, scale: 1 }} exit={{ opacity: 0, x: 20 }} className={`toast ${t.kind}`}>{t.msg}</motion.div>)}</AnimatePresence>
      </div>
    </div>
  );
}

// The x/y/z readout follows the live pose on its own, a few times a second, so the rest of the app is not re-rendered for every movement.
function Readout() {
  const [p, setP] = useState(live.pose);
  useEffect(() => {
    let t = null;
    const f = () => { if (t == null) t = setTimeout(() => { t = null; setP(live.pose); }, 100); };
    live.listeners.add(f);
    return () => { live.listeners.delete(f); clearTimeout(t); };
  }, []);
  if (!p) return null;
  return <div className="readout"><span>X <Num v={p.xyz?.[0]} /></span><span>Y <Num v={p.xyz?.[1]} /></span><span>Z <Num v={p.xyz?.[2]} /></span><span>∠ <Num v={p.tool_angle} d={0} />°</span></div>;
}

// Progress of whatever is running: one chip per pose of the sequence the arm is working through.
function Timeline({ agent, draft }) {
  const running = agent && agent.mode === "running" && agent.source !== "esp32";
  const step = agent?.running_step;
  const n = draft.length;
  if (!running || !n) return null;
  return (
    <motion.div className="timeline-strip" initial={{ y: 20, opacity: 0 }} animate={{ y: 0, opacity: 1 }}>
      <span className="muted">running</span>
      {draft.map((p, i) => <span key={i} className={`step ${i < (step ?? 0) ? "done" : i === (step ?? 0) - 1 || (step === 0 && i === 0) ? "now" : ""} ${p.action || ""}`}>{i + 1}</span>)}
    </motion.div>
  );
}
