import React, { useEffect, useMemo, useRef, useState } from "react";
import { AnimatePresence, motion } from "framer-motion";
import { Icon } from "./ui.jsx";

// Cmd/Ctrl+K: every action in one searchable list.
export default function Palette({ open, onClose, items }) {
  const [q, setQ] = useState(""), [i, setI] = useState(0);
  const input = useRef();
  useEffect(() => { if (open) { setQ(""); setI(0); setTimeout(() => input.current?.focus(), 30); } }, [open]);
  const shown = useMemo(() => {
    const t = q.trim().toLowerCase();
    if (!t) return items;
    const score = (it) => { const s = (it.label + " " + (it.hint || "")).toLowerCase(); let k = 0; for (const w of t.split(/\s+/)) { const p = s.indexOf(w); if (p < 0) return -1; k += 100 - p; } return k; };
    return items.map((it) => [score(it), it]).filter((x) => x[0] >= 0).sort((a, b) => b[0] - a[0]).map((x) => x[1]);
  }, [q, items]);
  useEffect(() => setI(0), [q]);
  const run = (it) => { onClose(); it.run(q.trim()); };
  const onKey = (e) => {
    if (e.key === "Escape") onClose();
    else if (e.key === "ArrowDown") { e.preventDefault(); setI((v) => Math.min(shown.length - 1, v + 1)); }
    else if (e.key === "ArrowUp") { e.preventDefault(); setI((v) => Math.max(0, v - 1)); }
    else if (e.key === "Enter" && shown[i]) run(shown[i]);
  };
  return (
    <AnimatePresence>
      {open && (
        <motion.div className="scrim" initial={{ opacity: 0 }} animate={{ opacity: 1 }} exit={{ opacity: 0 }} onMouseDown={onClose}>
          <motion.div className="palette" role="dialog" aria-label="Command palette" initial={{ y: -14, scale: 0.98, opacity: 0 }} animate={{ y: 0, scale: 1, opacity: 1 }} exit={{ y: -10, opacity: 0 }}
            transition={{ type: "spring", stiffness: 520, damping: 38 }} onMouseDown={(e) => e.stopPropagation()}>
            <div className="pin"><Icon n="cmd" size={17} /><input ref={input} value={q} onChange={(e) => setQ(e.target.value)} onKeyDown={onKey} placeholder="Type a command, a sequence name, or what you want the arm to do…" aria-label="Command" /></div>
            <ul role="listbox">
              {shown.slice(0, 12).map((it, k) => (
                <li key={it.id} role="option" aria-selected={k === i} className={k === i ? "on" : ""} onMouseEnter={() => setI(k)} onClick={() => run(it)}>
                  <Icon n={it.icon || "bolt"} size={16} /><span>{it.label}</span><small>{it.hint}</small>
                </li>
              ))}
              {!shown.length && q.trim() && <li className="none">No match. Press Enter on “Ask the arm” to send it as a request.</li>}
            </ul>
          </motion.div>
        </motion.div>
      )}
    </AnimatePresence>
  );
}
