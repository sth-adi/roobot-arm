// The 3D view: an AL5D-style arm (last joint and gripper removed, electromagnet at the tip), the work cell and the taught path.
// Units are millimetres, z up, the same frame as the hub's arm model. The hub streams the five joint points; this only draws them.
import React, { useEffect, useMemo, useRef, useState } from "react";
import { Canvas, invalidate, useFrame, useThree } from "@react-three/fiber";
import { Environment, Grid, Html, Lightformer, OrbitControls } from "@react-three/drei";
import * as THREE from "three";
import { live, useStore } from "./store.js";

// Light materials unless "rich": Lambert shading is a fraction of the cost of the physical-style one and looks fine for this arm.
let RICH = false;
const Mat = React.forwardRef(function Mat({ metalness, roughness, ...p }, ref) {
  return RICH ? <Mat ref={ref} metalness={metalness} roughness={roughness} {...p} /> : <meshLambertMaterial ref={ref} {...p} />;
});
const V = (a) => new THREE.Vector3(a[0], a[1], a[2]);
const AL = "#b9c2cf", AL2 = "#8e98a8", SERVO = "#232830", COPPER = "#c0743a";
const _m = new THREE.Matrix4(), _x = new THREE.Vector3(), _y = new THREE.Vector3(), _z = new THREE.Vector3(), _o = new THREE.Vector3();

// Put a unit box/cylinder at `o`, with its local axes along x, y, z (each already scaled to the size wanted).
function place(obj, o, x, y, z) {
  if (!obj) return;
  obj.matrixAutoUpdate = false;
  obj.matrix.makeBasis(x, y, z).setPosition(o);
  obj.matrixWorldNeedsUpdate = true;
}

const PRESETS = {
  iso: { pos: [-520, -620, 420], target: [0, 170, 90] },
  top: { pos: [0, 150, 1300], target: [0, 150, 0] },
  side: { pos: [-1000, 150, 150], target: [0, 150, 120] },
  front: { pos: [0, -900, 260], target: [0, 170, 130] },
};

function Rig({ view }) {
  const controls = useRef();
  const { camera } = useThree();
  const goal = useRef(null);
  useEffect(() => {
    camera.up.set(0, 0, 1);
    const p = PRESETS[view.name] || PRESETS.iso;
    goal.current = { pos: V(p.pos), target: V(p.target) };
    invalidate();
  }, [view.name, view.tick, camera]);
  useFrame((_, dt) => {
    const g = goal.current, c = controls.current;
    if (!g || !c) return;
    invalidate();                           // keep drawing until the camera has arrived
    const k = 1 - Math.exp(-dt * 6);
    camera.position.lerp(g.pos, k);
    c.target.lerp(g.target, k);
    if (camera.position.distanceTo(g.pos) < 2 && c.target.distanceTo(g.target) < 1) goal.current = null;
    c.update();
  });
  return <OrbitControls ref={controls} makeDefault enableDamping dampingFactor={0.09} minDistance={250} maxDistance={2400}
    maxPolarAngle={Math.PI * 0.495} onStart={() => { goal.current = null; }} />;
}

function Arm() {
  const refs = useRef({});
  const r = (n) => (el) => { refs.current[n] = el; };
  const disp = useRef(null), ang = useRef(null);
  const magFace = useRef(), magLight = useRef(), trail = useRef(), trailPts = useRef([]);
  const trailGeo = useMemo(() => { const g = new THREE.BufferGeometry(); g.setAttribute("position", new THREE.BufferAttribute(new Float32Array(160 * 3), 3)); g.setDrawRange(0, 0); return g; }, []);

  useFrame((_, dt) => {
    const p = live.pose;
    if (!p || !p.points) return;
    const target = p.points.map(V);
    const P = target;
    const ref = refs.current;
    const base = ((p.angles[0] - 90) * Math.PI) / 180;
    const d = _x.set(Math.sin(base), Math.cos(base), 0).clone(), l = new THREE.Vector3(Math.cos(base), -Math.sin(base), 0), up = new THREE.Vector3(0, 0, 1);
    const S = (v, s) => v.clone().multiplyScalar(s);
    place(ref.plate, new THREE.Vector3(0, 0, 3), new THREE.Vector3(140, 0, 0), new THREE.Vector3(0, 104, 0), new THREE.Vector3(0, 0, 6));
    place(ref.turret, new THREE.Vector3(0, 0, 22), S(d, 80), S(up, 32), S(l, 80));
    place(ref.marker, new THREE.Vector3(0, 0, 24).addScaledVector(d, 36), S(d, 16), S(l, 14), S(up, 16));
    [-1, 1].forEach((sg, i) => place(ref["bracket" + i], new THREE.Vector3(0, 0, 68).addScaledVector(l, 24 * sg), S(d, 56), S(l, 4), S(up, 60)));
    place(ref.shoulderServo, new THREE.Vector3(0, 0, 72).addScaledVector(l, 36), S(d, 52), S(l, 24), S(up, 48));

    const dirs = [1, 2, 3].map((i) => P[i + 1].clone().sub(P[i]));
    const lens = dirs.map((v) => v.length());
    const u = dirs.map((v) => v.clone().normalize());
    const frame = (uu) => ({ x: uu, y: l, z: new THREE.Vector3().crossVectors(uu, l).normalize() });
    const f1 = frame(u[0]), f2 = frame(u[1]), f3 = frame(u[2]);
    [-1, 1].forEach((sg, i) => {
      place(ref["upper" + i], P[1].clone().add(P[2]).multiplyScalar(0.5).addScaledVector(l, 15 * sg), S(f1.x, lens[0]), S(f1.y, 4), S(f1.z, 16));
      place(ref["fore" + i], P[2].clone().add(P[3]).multiplyScalar(0.5).addScaledVector(l, 12 * sg), S(f2.x, lens[1]), S(f2.y, 4), S(f2.z, 14));
    });
    place(ref.elbowServo, P[2].clone().addScaledVector(u[0], -12).addScaledVector(l, -28), S(f1.x, 48), S(f1.y, 24), S(f1.z, 40));
    place(ref.wristServo, P[3].clone().addScaledVector(u[1], -12).addScaledVector(l, 16), S(f2.x, 34), S(f2.y, 16), S(f2.z, 28));
    const rod = Math.max(2, lens[2] - 22);
    place(ref.rod, P[3].clone().addScaledVector(u[2], rod / 2), S(f3.x, rod), S(f3.y, 8), S(f3.z, 8));
    const mz = new THREE.Vector3().crossVectors(u[2], f3.y).normalize();
    // the electromagnet: a copper coil, its face lit when energised
    place(ref.coil, P[4].clone().addScaledVector(u[2], -11), S(f3.y, 24), S(u[2], 22), S(mz, 24));
    place(ref.face, P[4].clone().addScaledVector(u[2], 0.5), S(f3.y, 25), S(u[2], 3), S(mz, 25));
    const on = !!(p.world && p.world.magnet);
    if (magFace.current) { magFace.current.emissiveIntensity += ((on ? 2.4 : 0) - magFace.current.emissiveIntensity) * 0.3; magFace.current.color.set(on ? "#7fd4ff" : "#4a505b"); }
    if (magLight.current) { magLight.current.intensity += ((on ? 40000 : 0) - magLight.current.intensity) * 0.3; magLight.current.position.copy(P[4]); }

    // path the tip has taken
    const tip = P[4], pts = trailPts.current, last = pts[pts.length - 1];
    if (!last || last.distanceTo(tip) > 3) { pts.push(tip.clone()); if (pts.length > 160) pts.shift(); }
    const attr = trailGeo.getAttribute("position");
    pts.forEach((q, i) => attr.setXYZ(i, q.x, q.y, q.z));
    attr.needsUpdate = true; trailGeo.setDrawRange(0, pts.length);
    if (trail.current) { trail.current.visible = pts.length > 1; }
  });

  const box = (n, color, metal = 0.65, rough = 0.34) => (
    <mesh key={n} ref={r(n)} castShadow receiveShadow matrixAutoUpdate={false}>
      <boxGeometry args={[1, 1, 1]} /><Mat color={color} metalness={metal} roughness={rough} />
    </mesh>
  );
  return (
    <group>
      {box("plate", AL2)}
      <mesh ref={r("turret")} castShadow receiveShadow matrixAutoUpdate={false}><cylinderGeometry args={[0.5, 0.5, 1, 20]} /><Mat color="#1b1f26" metalness={0.5} roughness={0.4} /></mesh>
      {box("marker", AL)}
      {box("bracket0", AL)}{box("bracket1", AL)}
      {box("shoulderServo", SERVO, 0.2, 0.55)}
      {box("upper0", AL)}{box("upper1", AL)}
      {box("fore0", AL)}{box("fore1", AL)}
      {box("elbowServo", SERVO, 0.2, 0.55)}
      {box("wristServo", SERVO, 0.2, 0.55)}
      {box("rod", AL)}
      <mesh ref={r("coil")} castShadow matrixAutoUpdate={false}><cylinderGeometry args={[0.5, 0.5, 1, 18]} /><Mat color={COPPER} metalness={0.85} roughness={0.32} /></mesh>
      <mesh ref={r("face")} matrixAutoUpdate={false}><cylinderGeometry args={[0.5, 0.5, 1, 18]} /><Mat ref={magFace} color="#4a505b" emissive="#5ec8ff" emissiveIntensity={0} metalness={0.4} roughness={0.3} /></mesh>
      {RICH && <pointLight ref={magLight} color="#7fd4ff" intensity={0} distance={260} decay={2} />}
      <line ref={trail} geometry={trailGeo}><lineBasicMaterial color="#6ea8ff" transparent opacity={0.55} /></line>
    </group>
  );
}

function Cell() {
  const w = useStore((s) => s.pose && s.pose.world ? { boxes: s.pose.world.boxes, inner: s.pose.world.box_inner, wall: s.pose.world.box_wall, h: s.pose.world.box_height, fl: s.pose.world.box_floor, tray: s.pose.world.tray, half: s.pose.world.tray_half, top: s.pose.world.tray_top, n: s.pose.world.keys.length } : null);
  const keys = useRef([]);
  useFrame(() => {
    const p = live.pose;
    if (!p || !p.world) return;
    p.world.keys.forEach((k, i) => {
      const g = keys.current[i]; if (!g) return;
      g.position.set(k.pos[0], k.pos[1], k.pos[2]); g.rotation.z = (k.yaw * Math.PI) / 180;
      g.children[0].material.emissiveIntensity = p.world.held === k.id ? 0.8 : 0.1;
    });
  });
  if (!w) return null;
  const CARD = "#a9825b", CARD2 = "#8a6a47";
  return (
    <group>
      {w.boxes.map(([cx, cy], i) => (
        <group key={i} position={[cx, cy, 0]}>
          <mesh position={[0, 0, w.fl / 2]} receiveShadow castShadow><boxGeometry args={[2 * (w.inner + w.wall), 2 * (w.inner + w.wall), w.fl]} /><Mat color={CARD2} roughness={0.9} /></mesh>
          {[-1, 1].map((sg) => (
            <React.Fragment key={sg}>
              <mesh position={[0, sg * (w.inner + w.wall / 2), w.h / 2]} castShadow receiveShadow><boxGeometry args={[2 * (w.inner + w.wall), w.wall, w.h]} /><Mat color={CARD} roughness={0.9} /></mesh>
              <mesh position={[sg * (w.inner + w.wall / 2), 0, w.h / 2]} castShadow receiveShadow><boxGeometry args={[w.wall, 2 * w.inner, w.h]} /><Mat color={CARD} roughness={0.9} /></mesh>
            </React.Fragment>
          ))}
          <Html position={[0, 0, w.h + 30]} center distanceFactor={900} style={{ pointerEvents: "none" }}><div className="tag">Box {i + 1}</div></Html>
        </group>
      ))}
      <group position={[w.tray[0], w.tray[1], 0]}>
        <mesh position={[0, 0, w.top / 2]} receiveShadow castShadow><boxGeometry args={[2 * w.half[0], 2 * w.half[1], w.top]} /><Mat color="#56657a" roughness={0.6} metalness={0.2} /></mesh>
        {[-1, 1].map((sg) => (
          <React.Fragment key={sg}>
            <mesh position={[0, sg * (w.half[1] - 1.5), w.top + 3]} castShadow><boxGeometry args={[2 * w.half[0], 3, 6]} /><Mat color="#6e7f96" roughness={0.6} /></mesh>
            <mesh position={[sg * (w.half[0] - 1.5), 0, w.top + 3]} castShadow><boxGeometry args={[3, 2 * w.half[1] - 6, 6]} /><Mat color="#6e7f96" roughness={0.6} /></mesh>
          </React.Fragment>
        ))}
        <Html position={[0, 0, w.top + 30]} center distanceFactor={900} style={{ pointerEvents: "none" }}><div className="tag">Tray</div></Html>
      </group>
      {Array.from({ length: w.n }).map((_, i) => (
        <group key={i} ref={(el) => { keys.current[i] = el; }}>
          <mesh castShadow position={[6, 0, 0]}><boxGeometry args={[30, 7, 3]} /><Mat color="#d8b04a" metalness={0.7} roughness={0.35} emissive="#ffcf5a" emissiveIntensity={0.1} /></mesh>
          <mesh castShadow position={[13, 0, -1]}><boxGeometry args={[5, 12, 3]} /><Mat color="#d8b04a" metalness={0.7} roughness={0.35} /></mesh>
          <mesh castShadow position={[-14, 0, 0]} rotation={[Math.PI / 2, 0, 0]}><torusGeometry args={[8, 2.6, 8, 16]} /><Mat color="#d8b04a" metalness={0.7} roughness={0.35} /></mesh>
        </group>
      ))}
    </group>
  );
}

function DraftLine({ pts }) {
  const geo = useMemo(() => new THREE.BufferGeometry().setFromPoints(pts.map((p) => new THREE.Vector3(p[0], p[1], p[2]))), [JSON.stringify(pts)]); // eslint-disable-line react-hooks/exhaustive-deps
  return <line geometry={geo}><lineBasicMaterial color="#ffb454" transparent opacity={0.85} /></line>;
}

// The taught poses as numbered markers joined by the path the arm will take.
function Draft({ poses, onPick }) {
  if (!poses || !poses.length) return null;
  const pts = poses.map((p) => [p.x, p.y, p.z]);
  return (
    <group>
      {pts.length > 1 && <DraftLine pts={pts} />}
      {poses.map((p, i) => (
        <group key={i} position={[p.x, p.y, p.z]}>
          <mesh onClick={(e) => { e.stopPropagation(); onPick && onPick(i); }}>
            <sphereGeometry args={[p.action ? 7 : 5, 10, 10]} />
            <Mat color={p.action === "grab" ? "#5ec8ff" : p.action === "release" ? "#ff7a6b" : "#ffb454"} emissive={p.action === "grab" ? "#2d9fd6" : p.action === "release" ? "#d6483a" : "#c98320"} emissiveIntensity={0.9} />
          </mesh>
          <Html center distanceFactor={900} position={[0, 0, 16]} style={{ pointerEvents: "none" }}><div className="pin">{i + 1}</div></Html>
        </group>
      ))}
    </group>
  );
}

// The scene is drawn only while something moves (about 30 times a second) and not at all while it is still, so an idle page costs no GPU.
// "rich" adds shadows, reflections, antialiasing, a grid and a glow light for machines with a graphics card.
function Pulse() {
  useEffect(() => {
    let until = 0;
    const wake = (ms = 500) => { until = performance.now() + ms; };
    const f = () => wake(500);
    live.listeners.add(f); wake(1500);
    const t = setInterval(() => { if (performance.now() < until) invalidate(); }, 33);
    return () => { live.listeners.delete(f); clearInterval(t); };
  }, []);
  return null;
}

export default React.memo(function Scene({ view, draft, onPick, rich }) {
  RICH = !!rich;
  return (
    <Canvas key={rich ? "rich" : "lite"} frameloop="demand" shadows={!!rich} dpr={1} camera={{ position: PRESETS.iso.pos, fov: 38, near: 10, far: 6000, up: [0, 0, 1] }} gl={{ antialias: !!rich, alpha: true, powerPreference: "low-power" }}
      onCreated={({ gl }) => { if (rich) { gl.toneMapping = THREE.ACESFilmicToneMapping; gl.toneMappingExposure = 1.05; } }}>
      <Pulse />
      {rich && <Environment resolution={256} frames={1} background={false}>
        <Lightformer form="rect" intensity={2.6} position={[0, -6, 6]} scale={[14, 3, 1]} rotation-x={Math.PI / 3} color="#cfe3ff" />
        <Lightformer form="rect" intensity={1.4} position={[-8, 4, 3]} scale={[8, 6, 1]} rotation-y={Math.PI / 2} color="#9bb8ff" />
        <Lightformer form="rect" intensity={1.8} position={[8, 3, 4]} scale={[6, 6, 1]} rotation-y={-Math.PI / 2} color="#ffd9b0" />
      </Environment>}
      <hemisphereLight args={["#aebfd8", "#10131a", rich ? 0.55 : 1.5]} />
      <directionalLight position={[350, -450, 900]} intensity={rich ? 2.2 : 1.6} castShadow={!!rich} shadow-mapSize={[1024, 1024]} shadow-bias={-0.0004}
        shadow-camera-left={-650} shadow-camera-right={650} shadow-camera-top={650} shadow-camera-bottom={-650} shadow-camera-near={100} shadow-camera-far={2400} />
      {!rich && <directionalLight position={[-500, 300, 500]} intensity={0.7} color="#9bb8ff" />}
      <mesh position={[0, 150, -0.5]} receiveShadow><circleGeometry args={[1500, 48]} /><Mat color={RICH ? "#0d1015" : "#161b24"} roughness={0.95} metalness={0} /></mesh>
      {rich && <Grid position={[0, 150, 0.2]} rotation={[Math.PI / 2, 0, 0]} args={[2400, 2400]} cellSize={50} cellThickness={0.6} cellColor="#1d242f" sectionSize={250} sectionThickness={1.1} sectionColor="#2b3646" fadeDistance={1700} fadeStrength={2.2} infiniteGrid />}
      <Arm />
      <Cell />
      <Draft poses={draft} onPick={onPick} />
      <Rig view={view} />
    </Canvas>
  );
});
