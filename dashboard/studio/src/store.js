// One small store for everything the hub streams, plus the helpers every component uses.
import { useEffect, useRef, useState, useSyncExternalStore } from "react";

const state = {
  online: false, status: null, pose: null, logs: [], packets: [], toasts: [],
  seqs: [], agent: null, view: "iso", viewTick: 0, hello: false,
};
let version = 0;
const subs = new Set();
const emit = () => { version++; subs.forEach((f) => f()); };
export const getState = () => state;
export const set = (patch) => { Object.assign(state, patch); emit(); };
export function useStore(select = (s) => s) {
  // re-render when the store changes, but only hand back what the component asked for (kept stable when nothing in it changed)
  const last = useRef(undefined);
  return useSyncExternalStore(
    (cb) => { subs.add(cb); return () => subs.delete(cb); },
    () => { const v = select(state); if (last.current === undefined || !shallowEq(last.current, v)) last.current = v; return last.current; },
  );
}
function shallowEq(a, b) {
  if (Object.is(a, b)) return true;
  if (typeof a !== "object" || typeof b !== "object" || !a || !b) return false;
  const ka = Object.keys(a), kb = Object.keys(b);
  return ka.length === kb.length && ka.every((k) => Object.is(a[k], b[k]));
}

export async function api(path, body) {
  try {
    const r = await fetch(path, { method: "POST", headers: { "Content-Type": "application/json", "X-Hub": "1" }, body: JSON.stringify(body || {}) });
    const j = await r.json().catch(() => ({}));
    return { ...j, ok: r.ok && j.ok !== false };
  } catch (e) {
    return { ok: false, error: "The hub is not reachable" };
  }
}
export const getJson = async (path) => { try { return await (await fetch(path)).json(); } catch (e) { return null; } };

let toastId = 1;
export function toast(msg, kind = "") {
  const id = toastId++;
  set({ toasts: [...state.toasts, { id, msg, kind }] });
  setTimeout(() => set({ toasts: state.toasts.filter((t) => t.id !== id) }), kind === "bad" ? 7000 : 3500);
}

export async function refreshSeqs() {
  const j = await getJson("/api/sequences");
  if (j) set({ seqs: j.sequences || [] });
}

// The latest pose is kept outside React state: it arrives ~25 times a second and only the 3D scene needs every frame.
export const live = { pose: null, listeners: new Set() };

// Streams that only the System pages read (packets, board output, per-service logs). Kept outside the store and handed to a
// component through useFeed, which re-renders it at most ~7 times a second.
export const feed = { packets: [], board: [], logs: {}, ppsHist: [], listeners: new Set() };
const pushCap = (a, x, n) => { a.push(x); if (a.length > n) a.shift(); };
const ping = () => feed.listeners.forEach((f) => f());
export function useFeed() {
  const [, setN] = useState(0);
  useEffect(() => {
    let t = null;
    const f = () => { if (t == null) t = setTimeout(() => { t = null; setN((n) => n + 1); }, 150); };
    feed.listeners.add(f);
    return () => { feed.listeners.delete(f); clearTimeout(t); };
  }, []);
  return feed;
}

let started = false;
export function connect() {
  if (started) return;
  started = true;
  const es = new EventSource("/api/events");
  es.addEventListener("hello", (e) => {
    const d = JSON.parse(e.data);
    live.pose = d.status.sim || null;
    feed.packets = d.packets || []; feed.board = d.board || []; feed.logs = d.logs || {};
    set({ online: true, status: d.status, pose: live.pose, hello: true, logs: ((d.logs || {}).hub || []).slice(-60) });
    ping();
  });
  es.addEventListener("status", (e) => {
    const st = JSON.parse(e.data);
    pushCap(feed.ppsHist, st.pps || 0, 120);
    set({ online: true, status: st }); ping();
  });
  es.addEventListener("packet", (e) => { pushCap(feed.packets, JSON.parse(e.data), 500); ping(); });
  es.addEventListener("board", (e) => { pushCap(feed.board, JSON.parse(e.data), 400); ping(); });
  es.addEventListener("pose", (e) => {
    live.pose = JSON.parse(e.data);
    live.listeners.forEach((f) => f(live.pose));
    const p = live.pose, s = state.pose;
    // React only needs to know about the slow-changing parts
    if (!s || s.mode !== p.mode || s.replaying !== p.replaying || s.saved !== p.saved || s.replay_idx !== p.replay_idx || s.emulated !== p.emulated
        || (s.world && p.world && (s.world.magnet !== p.world.magnet || s.world.held !== p.world.held))) {
      state.pose = p; emit();
    }
  });
  es.addEventListener("log", (e) => {
    const l = JSON.parse(e.data);
    pushCap(feed.logs[l.src] = feed.logs[l.src] || [], l, 200); ping();
    if (l.src === "hub" || l.src === "agent" || l.src === "emu") set({ logs: [...state.logs.slice(-59), l] });
  });
  es.onerror = () => set({ online: false });
  refreshSeqs();
}
