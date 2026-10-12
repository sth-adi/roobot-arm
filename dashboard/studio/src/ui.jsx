import React from "react";

// Icons: 24px stroke icons, drawn inline so the app needs no font or sprite.
const PATHS = {
  cube: "M12 2 3 7v10l9 5 9-5V7l-9-5Zm0 0v10m0 10V12m9-5-9 5-9-5",
  list: "M8 6h13M8 12h13M8 18h13M3 6h.01M3 12h.01M3 18h.01",
  spark: "M12 3l1.9 5.1L19 10l-5.1 1.9L12 17l-1.9-5.1L5 10l5.1-1.9L12 3Zm7 12 .8 2.2L22 18l-2.2.8L19 21l-.8-2.2L16 18l2.2-.8L19 15Z",
  pad: "M6 11h4M8 9v4m7-1h.01M18 10h.01M17.3 5H6.7a4 4 0 0 0-3.9 3.2L2 15a3 3 0 0 0 5.2 2l1-1.4a2 2 0 0 1 1.6-.8h4.4a2 2 0 0 1 1.6.8l1 1.4A3 3 0 0 0 22 15l-.8-6.8A4 4 0 0 0 17.3 5Z",
  pulse: "M3 12h4l3-8 4 16 3-8h4",
  sliders: "M4 21v-7m0-4V3m8 18v-9m0-4V3m8 18v-5m0-4V3M1 14h6m2-6h6m2 8h6",
  play: "M6 4l14 8-14 8V4Z",
  stop: "M6 6h12v12H6z",
  rev: "M3 12a9 9 0 1 0 3-6.7L3 8m0-5v5h5",
  trash: "M3 6h18M8 6V4h8v2m-9 0 1 14h8l1-14M10 11v6m4-6v6",
  pen: "M4 20h4L19 9l-4-4L4 16v4Zm9-13 4 4",
  plus: "M12 5v14M5 12h14",
  home: "M3 11 12 3l9 8M5 10v10h14V10",
  magnet: "M6 3v8a6 6 0 0 0 12 0V3h-4v8a2 2 0 0 1-4 0V3H6Zm0 4h4m4 0h4",
  shield: "M12 3l8 3v6c0 5-3.5 8-8 9-4.5-1-8-4-8-9V6l8-3Zm-3 9 2 2 4-4",
  bolt: "M13 2 4 14h7l-1 8 9-12h-7l1-8Z",
  cmd: "M9 9V6a3 3 0 1 0-3 3h3Zm0 0h6m-6 0v6m6-6V6a3 3 0 1 1 3 3h-3Zm0 0v6m0 0h3a3 3 0 1 1-3 3v-3Zm0 0H9m0 0H6a3 3 0 1 0 3 3v-3Z",
  x: "M6 6l12 12M18 6 6 18",
  check: "M5 12l5 5 9-10",
  chip: "M9 3v3m6-3v3M9 18v3m6-3v3M3 9h3m12 0h3M3 15h3m12 0h3M7 6h10v12H7V6Zm3 3h4v6h-4V9Z",
  link: "M10 14a4 4 0 0 0 5.7 0l3-3a4 4 0 0 0-5.7-5.7l-1 1M14 10a4 4 0 0 0-5.7 0l-3 3a4 4 0 0 0 5.7 5.7l1-1",
  ext: "M14 4h6v6m0-6L10 14M20 14v5a1 1 0 0 1-1 1H5a1 1 0 0 1-1-1V5a1 1 0 0 1 1-1h5",
  target: "M12 3v4m0 10v4M3 12h4m10 0h4M12 8a4 4 0 1 0 0 8 4 4 0 0 0 0-8Z",
  copy: "M8 8h11v13H8V8Zm-3 8V3h11",
  arrow: "M5 12h14m-6-6 6 6-6 6",
  pause: "M8 5v14m8-14v14",
  hammer: "M14 6l4 4M4 20l9-9m-1-5 3-3 6 6-3 3-6-6Z",
  upload: "M12 16V4m0 0-4 4m4-4 4 4M4 16v4h16v-4",
  server: "M4 4h16v6H4V4Zm0 10h16v6H4v-6Zm3-7h.01M7 17h.01",
  git: "M6 3v12m0 0a3 3 0 1 0 0 6 3 3 0 0 0 0-6Zm12-6a3 3 0 1 0 0-6 3 3 0 0 0 0 6Zm0 0c0 4-6 3-12 6",
};
export const Icon = ({ n, size = 18, ...rest }) => (
  <svg viewBox="0 0 24 24" width={size} height={size} fill="none" stroke="currentColor" strokeWidth="1.7" strokeLinecap="round" strokeLinejoin="round" aria-hidden="true" {...rest}>
    <path d={PATHS[n] || PATHS.cube} />
  </svg>
);

export const Btn = ({ icon, children, kind = "", size = "", ...rest }) => (
  <button type="button" className={`btn ${kind} ${size}`} {...rest}>{icon ? <Icon n={icon} size={size === "sm" ? 15 : 17} /> : null}{children}</button>
);
export const Pill = ({ tone = "idle", children, title, onClick }) => {
  const Tag = onClick ? "button" : "div";
  return <Tag type={onClick ? "button" : undefined} className={`pill ${tone}`} title={title} onClick={onClick}><i />{children}</Tag>;
};
export const Field = ({ label, children }) => <label className="field"><span>{label}</span>{children}</label>;
export const Num = ({ v, d = 1 }) => <span className="num">{v == null || Number.isNaN(v) ? "–" : Number(v).toFixed(d)}</span>;
