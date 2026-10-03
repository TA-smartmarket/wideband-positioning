/* ==========================================================================
   UWB positioning — 3D room editor + live tracker
   Three.js scene: room shell, draggable anchors, resizable room, obstacles,
   live tag with EKF trail, and LOS/NLOS link visualisation.
   ========================================================================== */

import * as THREE from 'three';
import { OrbitControls } from '/static/vendor/OrbitControls.js';

const S = {
  scene: { room: { width: 5, depth: 4, height: 2.7 }, anchors: [], obstacles: [] },
  state: { anchors: [], tags: [], links: [], obstacles: [], room: {} },
  nlos: true,
  selected: null,
  mode: 'select',        // select | addAnchor | addObstacle | moveTag
  drag: null,
  dirty: false,
  trail: {},
  showTrail: true,
  autoSpin: false,
};

const $ = (id) => document.getElementById(id);
const COL = {
  room: 0x2a3350, grid: 0x1f2740, anchor: 0x6ee7b7, anchorOff: 0x64748b,
  tag: 0xffd166, los: 0x6ee7b7, nlos: 0xf87171, obstacle: 0x8b5cf6,
  sel: 0x60a5fa, ghost: 0x94a3b8,
};

/* ------------------------------------------------------------------ three */
let renderer, scene3, camera, controls, raycaster;
let roomGroup, anchorGroup, obstacleGroup, tagGroup, linkGroup, trailGroup;
let keyLight = null;
let pickables = [];
const tagMeshes = {};
const anchorObjs = {};     // anchor id -> Group (moved directly while dragging)
const obstacleObjs = {};   // obstacle id -> Group (moved/resized while dragging)
let ghost = null;

function initThree() {
  const host = $('viewport');
  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  host.appendChild(renderer.domElement);

  scene3 = new THREE.Scene();
  scene3.background = new THREE.Color(0x0b0e1a);
  // Fog is ADAPTIVE (see updateFog): a static far plane made the whole room
  // disappear as soon as the user zoomed out past it.
  scene3.fog = new THREE.Fog(0x0b0e1a, 30, 120);

  camera = new THREE.PerspectiveCamera(50, 1, 0.05, 400);
  camera.position.set(6, 7, 8);

  controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.maxPolarAngle = Math.PI * 0.495;   // never below the floor
  controls.minDistance = 0.8;
  controls.maxDistance = 60;
  // OrbitControls' built-in zoom treats raw deltaY as an exponent, so a single
  // mouse notch (deltaY 100) jumped a huge distance while a trackpad (many
  // small events) piled up. Zoom is handled manually below instead.
  controls.enableZoom = false;
  controls.target.set(2.5, 1, 2);

  // lights
  scene3.add(new THREE.HemisphereLight(0xdfe8ff, 0x1a2033, 0.85));
  const key = new THREE.DirectionalLight(0xffffff, 1.5);
  key.position.set(6, 12, 7);
  key.castShadow = true;
  key.shadow.mapSize.set(2048, 2048);
  key.shadow.camera.left = -14; key.shadow.camera.right = 14;
  key.shadow.camera.top = 14; key.shadow.camera.bottom = -14;
  keyLight = key;
  scene3.add(key);

  roomGroup = new THREE.Group();
  anchorGroup = new THREE.Group();
  obstacleGroup = new THREE.Group();
  tagGroup = new THREE.Group();
  linkGroup = new THREE.Group();
  trailGroup = new THREE.Group();
  scene3.add(roomGroup, anchorGroup, obstacleGroup, tagGroup, linkGroup, trailGroup);

  raycaster = new THREE.Raycaster();
  // CRITICAL: the default Line threshold is 1 world unit, so a thin guide line
  // (the obstacle height guide) swallowed every click within a metre — that is
  // why resize handles could never be grabbed. Shrink it to a few centimetres.
  raycaster.params.Line.threshold = 0.02;
  raycaster.params.Points.threshold = 0.02;
  addEventListener('resize', resize);
  bindContextLoss();
  bindTouchZoom(renderer.domElement);
  bindKeyboard();
  if (window.ResizeObserver) new ResizeObserver(resize).observe($('viewport'));
  resize();
  bindPointer();
  animate();
}

function resize() {
  const host = $('viewport');
  const w = Math.max(host.clientWidth, 1), h = Math.max(host.clientHeight, 1);
  renderer.setSize(w, h, false);
  renderer.domElement.style.width = '100%';
  renderer.domElement.style.height = '100%';
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}

// If the GPU context is lost (driver reset, too many contexts) show a clear
// message instead of a silent black viewport, and recover on restore.
function bindContextLoss() {
  const canvas = renderer.domElement;
  canvas.addEventListener('webglcontextlost', (e) => {
    e.preventDefault();
    toast('Graphics context lost — reloading the view…');
    setTimeout(() => location.reload(), 1200);
  });
}

function animate() {
  requestAnimationFrame(animate);
  frameDt = clock.getDelta();
  if (!renderOn) return;          // paused: skip all per-frame work
  applyKeyboardMove();
  if (intro) applyIntro();
  else applyAutoSpin();
  controls.update();
  applyZoom();
  animatePulses();
  updateFog();
  renderer.render(scene3, camera);
}

// Optional slow orbit when the user is idle — pure eye candy, off by default.
let lastInteraction = performance.now();
function applyAutoSpin() {
  if (!S.autoSpin) return;
  if (performance.now() - lastInteraction < 4000) return;
  const a = frameDt * 0.12;
  const t = controls.target;
  const d = camera.position.clone().sub(t);
  const cos = Math.cos(a), sin = Math.sin(a);
  const x = d.x * cos - d.z * sin;
  const z = d.x * sin + d.z * cos;
  camera.position.set(t.x + x, camera.position.y, t.z + z);
  camera.lookAt(t);
}

// ---------------------------------------------------------------------------
// Zoom: one wheel notch moves the camera by a fixed ~8 % of the current
// distance, no matter what the device reports. Trackpads send many small
// events, mice send one big one; both are normalised and clamped, then eased
// over a few frames so the motion feels smooth instead of snappy.
// ---------------------------------------------------------------------------
const ZOOM_STEP = 0.085;      // distance change per "notch"
const ZOOM_UNIT = 120;        // normalised delta that counts as one notch
let zoomAccum = 0;
let zoomFocus = null;

function normalizeWheel(ev) {
  let d = ev.deltaY;
  if (ev.deltaMode === 1) d *= 16;        // deltaMode: lines
  else if (ev.deltaMode === 2) d *= 400;  // deltaMode: pages
  return Math.max(-200, Math.min(200, d));
}

function onWheel(ev) {
  ev.preventDefault();
  zoomAccum += normalizeWheel(ev);
  // remember what is under the cursor so zooming in feels anchored
  const gp = groundPoint(ev);
  if (gp) zoomFocus = gp;
}

function applyZoom() {
  if (!zoomAccum) return;
  // at most one notch per frame -> smooth, never a jump
  const step = Math.max(-1, Math.min(1, zoomAccum / ZOOM_UNIT));
  zoomAccum -= step * ZOOM_UNIT;
  if (Math.abs(zoomAccum) < 1) zoomAccum = 0;

  const dist = camera.position.distanceTo(controls.target);
  if (dist < 1e-4) return;

  const factor = Math.exp(step * ZOOM_STEP);          // >1 = zoom out
  let newDist = dist * factor;
  newDist = Math.max(controls.minDistance, Math.min(controls.maxDistance, newDist));

  // zoom-to-cursor: only when pulling closer, nudge the orbit target toward
  // the point under the pointer so it stays put on screen
  const k = 1 - newDist / dist;
  if (k > 0 && zoomFocus) {
    controls.target.lerp(zoomFocus, Math.min(k * 0.5, 0.25));
  }

  const dir = new THREE.Vector3().subVectors(camera.position, controls.target).normalize();
  camera.position.copy(controls.target).addScaledVector(dir, newDist);
}

/* ---------------------------------------------------------------------------
   Touch: two-finger pinch drives the same zoomAccum as the wheel, so phones
   and tablets get identical, controlled zoom. One-finger rotate and two-finger
   pan are handled by OrbitControls.
   --------------------------------------------------------------------------- */
let pinchPrev = 0;

function touchDistance(a, b) {
  return Math.hypot(a.clientX - b.clientX, a.clientY - b.clientY);
}

function bindTouchZoom(el) {
  el.addEventListener('touchstart', (ev) => {
    pinchPrev = ev.touches.length === 2 ? touchDistance(ev.touches[0], ev.touches[1]) : 0;
  }, { passive: true });

  el.addEventListener('touchmove', (ev) => {
    if (ev.touches.length !== 2 || !pinchPrev) return;
    ev.preventDefault();                       // never zoom the page itself
    const d = touchDistance(ev.touches[0], ev.touches[1]);
    if (d < 1) return;
    // pinch out (d grows) => zoom in => negative step
    const notches = -Math.log(d / pinchPrev) / ZOOM_STEP;
    zoomAccum += notches * ZOOM_UNIT;
    pinchPrev = d;
  }, { passive: false });

  el.addEventListener('touchend', () => { pinchPrev = 0; }, { passive: true });
  el.addEventListener('touchcancel', () => { pinchPrev = 0; }, { passive: true });
}

/* ---------------------------------------------------------------------------
   Keyboard fly-through: WASD / arrow keys pan the camera rig across the floor,
   Q/E (or PageUp/PageDown) change height, Shift = fast, H = reset view.
   Movement is camera-relative, so W always means "away from me".
   --------------------------------------------------------------------------- */
const heldKeys = new Set();
const MOVE_SPEED = 2.2;        // metres per second
const clock = new THREE.Clock();
let frameDt = 0;

function isTyping() {
  const a = document.activeElement;
  return a && (a.tagName === 'INPUT' || a.tagName === 'SELECT' ||
               a.tagName === 'TEXTAREA' || a.isContentEditable);
}

function bindKeyboard() {
  addEventListener('keydown', (e) => {
    lastInteraction = performance.now();
    if (isTyping()) return;
    const k = e.key.toLowerCase();
    if (['w', 'a', 's', 'd', 'q', 'e', 'arrowup', 'arrowdown', 'arrowleft',
         'arrowright', 'pageup', 'pagedown', 'shift'].includes(k)) {
      heldKeys.add(k);
      if (k.startsWith('arrow') || k === 'pageup' || k === 'pagedown') e.preventDefault();
    }
  });
  addEventListener('keyup', (e) => heldKeys.delete(e.key.toLowerCase()));
  addEventListener('blur', () => heldKeys.clear());
}

function applyKeyboardMove() {
  const dt = Math.min(frameDt, 0.1);
  if (!heldKeys.size) return;

  let fwdAmt = 0, rightAmt = 0, upAmt = 0;
  if (heldKeys.has('w') || heldKeys.has('arrowup')) fwdAmt += 1;
  if (heldKeys.has('s') || heldKeys.has('arrowdown')) fwdAmt -= 1;
  if (heldKeys.has('d') || heldKeys.has('arrowright')) rightAmt += 1;
  if (heldKeys.has('a') || heldKeys.has('arrowleft')) rightAmt -= 1;
  if (heldKeys.has('e') || heldKeys.has('pageup')) upAmt += 1;
  if (heldKeys.has('q') || heldKeys.has('pagedown')) upAmt -= 1;
  if (!fwdAmt && !rightAmt && !upAmt) return;

  const speed = MOVE_SPEED * (heldKeys.has('shift') ? 3 : 1) * dt;

  // camera-relative basis on the ground plane
  const fwd = new THREE.Vector3();
  camera.getWorldDirection(fwd);
  fwd.y = 0;
  if (fwd.lengthSq() < 1e-6) fwd.set(0, 0, -1);
  fwd.normalize();
  const right = new THREE.Vector3(-fwd.z, 0, fwd.x);   // fwd rotated -90° about Y

  const delta = new THREE.Vector3()
    .addScaledVector(fwd, fwdAmt * speed)
    .addScaledVector(right, rightAmt * speed);
  delta.y += upAmt * speed;

  camera.position.add(delta);
  controls.target.add(delta);
  zoomAccum = 0;                 // keyboard wins over pending wheel momentum
}

// Keep the fog relative to the room and the camera distance so zooming out
// never swallows the scene, and zooming in never washes it out.
function updateFog() {
  if (!scene3.fog) return;
  const { width: W, depth: D } = S.scene.room;
  const span = Math.max(W, D, 1);
  const dist = camera.position.distanceTo(controls.target);
  const near = Math.max(span * 0.9, dist * 0.9);
  const far = Math.max(near + span * 2.5, dist * 3.2);
  scene3.fog.near = near;
  scene3.fog.far = far;
  // shadow camera follows the room size so big rooms stay lit correctly
  if (keyLight) {
    const half = Math.max(span, dist) * 0.75 + 2;
    keyLight.shadow.camera.left = -half;
    keyLight.shadow.camera.right = half;
    keyLight.shadow.camera.top = half;
    keyLight.shadow.camera.bottom = -half;
    keyLight.shadow.camera.updateProjectionMatrix();
  }
}

/* ------------------------------------------------------------ room shell */
function buildRoom() {
  clearGroup(roomGroup);
  const { width: W, depth: D, height: H } = S.scene.room;

  // floor
  const floor = new THREE.Mesh(
    new THREE.BoxGeometry(W, 0.06, D),
    new THREE.MeshStandardMaterial({ color: COL.room, roughness: 0.95, metalness: 0.05 }));
  floor.position.set(W / 2, -0.03, D / 2);
  floor.receiveShadow = true;
  floor.userData.pick = 'floor';
  roomGroup.add(floor);

  // 1 m grid, aligned with the world origin (0,0) so it doubles as a ruler
  const grid = new THREE.GridHelper(Math.max(W, D) * 1.2, Math.round(Math.max(W, D) * 1.2),
                                    0x33406b, 0x222b45);
  grid.position.set(W / 2, 0.005, D / 2);
  roomGroup.add(grid);

  // ---- coordinate system: X axis (red), Y axis (green), origin marker ----
  const axes = new THREE.Group();
  const axisLen = Math.max(W, D) * 0.35 + 0.6;

  const mkAxis = (from, to, color) => {
    const geo = new THREE.BufferGeometry().setFromPoints([from, to]);
    const line = new THREE.Line(geo, new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.95 }));
    line.userData.pick = 'axis';
    return line;
  };
  const X = new THREE.Vector3(1, 0, 0), Y = new THREE.Vector3(0, 0, 1);
  axes.add(mkAxis(new THREE.Vector3(0, 0.02, 0), X.clone().multiplyScalar(axisLen), 0xff6b6b));
  axes.add(mkAxis(new THREE.Vector3(0, 0.02, 0), Y.clone().multiplyScalar(axisLen), 0x6bff9c));

  // arrow heads
  const head = (dir, color) => {
    const cone = new THREE.Mesh(new THREE.ConeGeometry(0.055, 0.18, 12),
                                new THREE.MeshBasicMaterial({ color }));
    cone.position.copy(dir.clone().multiplyScalar(axisLen));
    cone.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir);
    cone.userData.pick = 'axis';
    return cone;
  };
  axes.add(head(X, 0xff6b6b));
  axes.add(head(Y, 0x6bff9c));

  // origin dot
  const origin = new THREE.Mesh(new THREE.SphereGeometry(0.055, 14, 12),
                                new THREE.MeshBasicMaterial({ color: 0xffffff }));
  origin.position.set(0, 0.02, 0);
  origin.userData.pick = 'axis';
  axes.add(origin);

  axes.add(label('X →', new THREE.Vector3(axisLen * 0.72, 0.14, 0.16), 0xff9a9a, 0.13));
  axes.add(label('Y →', new THREE.Vector3(0.16, 0.14, axisLen * 0.72), 0x9affc0, 0.13));
  axes.add(label('0,0', new THREE.Vector3(-0.02, 0.10, -0.22), 0xd8e0ff, 0.12));
  roomGroup.add(axes);

  // ---- rulers along two edges -------------------------------------------
  roomGroup.add(makeRuler(new THREE.Vector3(0, 0.02, 0), new THREE.Vector3(W, 0, 0), 'x'));
  roomGroup.add(makeRuler(new THREE.Vector3(0, 0.02, 0), new THREE.Vector3(0, 0, D), 'y'));

  // ---- walls (translucent so the room stays readable from any angle) -----
  const wallMat = new THREE.MeshStandardMaterial({
    color: 0x38507f, transparent: true, opacity: 0.16,
    side: THREE.DoubleSide, roughness: 1.0 });
  const mk = (w, h, d, x, y, z) => {
    const m = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), wallMat);
    m.position.set(x, y, z);
    m.userData.pick = 'wall';
    roomGroup.add(m);
  };
  mk(W, H, 0.06, W / 2, H / 2, 0);          // y = 0
  mk(W, H, 0.06, W / 2, H / 2, D);          // y = D
  mk(0.06, H, D, 0, H / 2, D / 2);          // x = 0
  mk(0.06, H, D, W, H / 2, D / 2);          // x = W

  // dimension labels (the rulers already carry per-metre numbers)
  label(`width ${W.toFixed(2)} m`, new THREE.Vector3(W / 2, 0.12, -0.62), 0xa8b8dc);
  label(`depth ${D.toFixed(2)} m`, new THREE.Vector3(-0.62, 0.12, D / 2), 0xa8b8dc);

  controls.target.set(W / 2, 0.8, D / 2);
  updateHud();
}

function label(text, pos, color = 0xffffff, size = 0.16) {
  const canvas = document.createElement('canvas');
  const spr = new THREE.Sprite(new THREE.SpriteMaterial({
    transparent: true, depthTest: false }));
  spr.position.copy(pos);
  spr.userData.labelSize = size;
  spr.userData.labelColor = color;
  spr.userData.pick = 'label';
  // Reuse the same canvas + texture when the text changes (the anchor labels
  // update every poll, so allocating a texture each time leaked GPU memory).
  spr.userData.setText = (t, col) => {
    if (col !== undefined) spr.userData.labelColor = col;
    if (spr.userData.text === t && spr.userData.col === spr.userData.labelColor) return;
    spr.userData.text = t;
    spr.userData.col = spr.userData.labelColor;

    const g = canvas.getContext('2d');
    const font = 'bold 40px system-ui, sans-serif';
    g.font = font;
    const lines = String(t).split('\n');
    const w = Math.ceil(Math.max(...lines.map((l) => g.measureText(l).width))) + 24;
    const lh = 48;
    const h = lh * lines.length;
    if (canvas.width !== w) canvas.width = w;
    if (canvas.height !== h) canvas.height = h;
    g.clearRect(0, 0, w, h);
    g.font = font;
    g.fillStyle = `#${spr.userData.labelColor.toString(16).padStart(6, '0')}`;
    g.textBaseline = 'middle';
    lines.forEach((l, i) => g.fillText(l, 12, lh * i + lh / 2));

    if (!spr.material.map) {
      spr.material.map = new THREE.CanvasTexture(canvas);
      spr.material.map.minFilter = THREE.LinearFilter;
    } else {
      spr.material.map.needsUpdate = true;
    }
    const s = spr.userData.labelSize;
    spr.scale.set(w / 64 * s, h / 64 * s, 1);
  };
  spr.userData.setText(text);
  return spr;
}

/* ------------------------------------------------------------- rulers */
/* --- anchors  + measuring rulers ----------------------------------------- */
// A measuring ruler along one room edge: a baseline with 0.5 m ticks and a
// numeric label every metre, so distances in the 3D view can be read off
// directly instead of guessed.
function makeRuler(from, to) {
  const g = new THREE.Group();
  const len = from.distanceTo(to);
  const dir = new THREE.Vector3().subVectors(to, from).normalize();
  const side = new THREE.Vector3(-dir.z, 0, dir.x);      // outward on the floor

  g.add(new THREE.Line(
    new THREE.BufferGeometry().setFromPoints([from, to]),
    new THREE.LineBasicMaterial({ color: 0x8fa6d8, transparent: true, opacity: 0.9 })));

  const tickMat = new THREE.LineBasicMaterial({ color: 0x8fa6d8, transparent: true, opacity: 0.75 });
  const majorMat = new THREE.LineBasicMaterial({ color: 0xc7d6ff });

  const ticks = Math.round(len * 2);                      // every 0.5 m
  for (let i = 0; i <= ticks; i++) {
    const t = i / 2;                                      // metres
    const p = from.clone().addScaledVector(dir, t);
    const major = Math.abs(t - Math.round(t)) < 1e-6;     // whole metre
    const h = major ? 0.17 : 0.09;
    const a = p.clone().addScaledVector(side, 0.02);
    const b = p.clone().addScaledVector(side, h);
    g.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints([a, b]),
                         major ? majorMat : tickMat));
    if (major && i > 0) {
      const lp = p.clone().addScaledVector(side, h + 0.12);
      lp.y = 0.10;
      g.add(label(`${t} m`, lp, 0xb9c8ee, 0.105));
    }
  }
  return g;
}

function buildAnchors() {
  clearGroup(anchorGroup);
  for (const k of Object.keys(anchorObjs)) delete anchorObjs[k];
  for (const a of S.scene.anchors) {
    const live = S.state.anchors.find((x) => x.id === a.id);
    const online = live ? live.online : false;

    const g = new THREE.Group();
    g.position.set(a.x, a.z, a.y);
    g.userData.pick = 'anchor';
    g.userData.id = a.id;

    // mast
    const mast = new THREE.Mesh(
      new THREE.CylinderGeometry(0.035, 0.05, a.z, 12),
      new THREE.MeshStandardMaterial({ color: online ? COL.anchor : COL.anchorOff,
                                       metalness: 0.5, roughness: 0.35 }));
    mast.position.y = a.z / 2;
    mast.castShadow = true;
    mast.userData.pick = 'anchor';
    mast.userData.id = a.id;
    g.add(mast);

    // head
    const head = new THREE.Mesh(
      new THREE.IcosahedronGeometry(0.14, 1),
      new THREE.MeshStandardMaterial({ color: online ? COL.anchor : COL.anchorOff,
                                       emissive: online ? 0x1f6f4f : 0x000000,
                                       emissiveIntensity: 1.1, metalness: 0.3 }));
    head.position.y = a.z;
    head.castShadow = true;
    head.userData.pick = 'anchor';
    head.userData.id = a.id;
    g.add(head);

    // halo when selected
    if (S.selected && S.selected.type === 'anchor' && S.selected.id === a.id) {
      const halo = new THREE.Mesh(
        new THREE.TorusGeometry(0.26, 0.018, 8, 32),
        new THREE.MeshBasicMaterial({ color: COL.sel }));
      halo.rotation.x = Math.PI / 2;
      halo.position.y = a.z;
      g.add(halo);
    }

    g.add(anchorLabel(a, live));
    anchorObjs[a.id] = g;
    anchorGroup.add(g);
  }
}

// Anchor label: name on the first line, live values underneath
// (position always, plus the current range/RSSI when the device reports them).
function anchorLabel(a, live) {
  const online = live ? live.online : false;
  const spr = label(anchorLabelText(a, live), new THREE.Vector3(0, a.z + 0.42, 0),
                    online ? 0x9ff5cf : 0x93a4c8, 0.125);
  spr.userData.anchorId = a.id;
  return spr;
}

// The text shown above an anchor. Built here (not in the sprite) so a rebuild
// — which happens on every selection change — keeps the full two-line label.
// Previously the sprite was created with the short name only and the live text
// arrived 500 ms later on the next poll, so the label visibly shrank ("kempes")
// for a moment every time an object was selected.
function anchorLabelText(a, live) {
  const online = live ? live.online : false;
  let rng = null;
  for (const t of S.state.tags || []) {
    const l = (t.links || {})[a.id];
    if (l && t.ranges && t.ranges[a.id] !== undefined) { rng = t.ranges[a.id]; break; }
  }
  const lines = [`${a.label || a.id}  (${a.x.toFixed(2)}, ${a.y.toFixed(2)})`];
  if (!online) lines.push('offline');
  else if (rng !== null) lines.push(`${rng.toFixed(2)} m`);
  else lines.push('online');
  return lines.join('\n');
}

// Live text for every anchor label, refreshed from /api/v1/state.
function updateAnchorLabels() {
  for (const a of S.scene.anchors) {
    const g = anchorObjs[a.id];
    if (!g) continue;
    const spr = g.children.find((c) => c.userData && c.userData.setText && c.userData.anchorId);
    if (!spr) continue;

    const live = S.state.anchors.find((x) => x.id === a.id);
    const online = live ? live.online : false;

    // live range to the tag, if any link exists
    let rng = null;
    for (const t of S.state.tags || []) {
      const l = (t.links || {})[a.id];
      if (l && t.ranges && t.ranges[a.id] !== undefined) { rng = t.ranges[a.id]; break; }
    }

    spr.userData.setText(anchorLabelText(a, live), online ? 0x9ff5cf : 0x93a4c8);
  }
}

/* ------------------------------------------------------------- obstacles */
function buildObstacles() {
  clearGroup(obstacleGroup);
  for (const k of Object.keys(obstacleObjs)) delete obstacleObjs[k];
  for (const ob of S.scene.obstacles) {
    const g = new THREE.Group();
    g.position.set(ob.x, ob.z, ob.y);
    g.rotation.y = -THREE.MathUtils.degToRad(ob.rot || 0);
    g.userData.pick = 'obstacle';
    g.userData.id = ob.id;

    const sel = S.selected && S.selected.type === 'obstacle' && S.selected.id === ob.id;
    const box = new THREE.Mesh(
      new THREE.BoxGeometry(ob.sx, ob.sz, ob.sy),
      new THREE.MeshStandardMaterial({
        color: COL.obstacle,
        transparent: true,
        opacity: 0.30 + 0.45 * Math.min(ob.atten ?? 1, 1),
        roughness: 0.8,
        emissive: sel ? 0x2b1f66 : 0x000000 }));
    box.castShadow = true;
    box.userData.pick = 'obstacle';
    box.userData.id = ob.id;
    g.add(box);

    const edges = new THREE.LineSegments(
      new THREE.EdgesGeometry(box.geometry),
      new THREE.LineBasicMaterial({ color: sel ? COL.sel : 0xb794f6 }));
    g.add(edges);

    if (sel) {
      // ---------------------------------------------------------------------
      // Two SEPARATE sets of controls, like a real 3D editor:
      //
      //   ARROWS  (X red / Y green / Z blue)  -> TRANSLATE the obstacle
      //           X = left-right, Y = forward-back, Z = up-down
      //
      //   CUBES at the footprint corners + TOP handle -> RESIZE the body
      //           corners scale width/depth, the top handle changes the height
      //
      // (The arrows used to resize, which read as "the up arrow makes it
      //  bigger instead of lifting it". Resizing now lives on its own handles.)
      // ---------------------------------------------------------------------
      const armLen = 0.62, armR = 0.030, tipR = 0.085, tipH = 0.24;

      const axisArrow = (dir, color, name) => {
        const a = new THREE.Group();
        const shaft = new THREE.Mesh(
          new THREE.CylinderGeometry(armR, armR, armLen, 10),
          new THREE.MeshBasicMaterial({ color }));
        shaft.position.y = armLen / 2;
        const tip = new THREE.Mesh(
          new THREE.ConeGeometry(tipR, tipH, 14),
          new THREE.MeshBasicMaterial({ color }));
        tip.position.y = armLen + tipH / 2;
        a.add(shaft, tip);
        a.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir);
        a.traverse((o) => {
          o.userData.pick = 'gizmoAxis';
          o.userData.axis = name;
          o.userData.id = ob.id;
        });
        return a;
      };

      // The gizmo lives at the obstacle CENTRE so the arrows read as
      // "move this object", and is drawn above the body so the box never
      // swallows the ray.
      const gz = new THREE.Group();
      gz.position.set(0, 0, 0);
      gz.userData.pick = 'gizmo';
      gz.userData.id = ob.id;
      gz.add(axisArrow(new THREE.Vector3(1, 0, 0), 0xff6b6b, 'x'));
      gz.add(axisArrow(new THREE.Vector3(0, 0, 1), 0x6bff9c, 'y'));
      gz.add(axisArrow(new THREE.Vector3(0, 1, 0), 0x6bb5ff, 'z'));
      gz.renderOrder = 20;
      g.add(gz);

      // ---- resize handles -------------------------------------------------
      // footprint corners: scale width (sx) and depth (sy)
      for (const [sx, sz] of [[-1, -1], [1, -1], [1, 1], [-1, 1]]) {
        const h = new THREE.Mesh(
          new THREE.BoxGeometry(0.17, 0.17, 0.17),
          new THREE.MeshBasicMaterial({ color: 0x93c5fd }));
        h.position.set(sx * (ob.sx / 2 + 0.09), 0, sz * (ob.sy / 2 + 0.09));
        h.userData.pick = 'obstacleHandle';
        h.userData.id = ob.id;
        h.userData.axis = 'both';
        h.userData.corner = [sx, sz];
        h.renderOrder = 20;
        g.add(h);
      }
      // top handle: change the height (sz) only
      const top = new THREE.Mesh(
        new THREE.BoxGeometry(0.17, 0.17, 0.17),
        new THREE.MeshBasicMaterial({ color: 0xa7f3d0 }));
      top.position.set(0, ob.sz / 2 + 0.13, 0);
      top.userData.pick = 'sizeZ';
      top.userData.id = ob.id;
      top.renderOrder = 20;
      g.add(top);
    }

    obstacleObjs[ob.id] = g;
    obstacleGroup.add(g);
  }
}

/* ==========================================================================
   Pulse rings: one thin ring per node that expands and fades, like a sonar
   ping. Cheap by design — a handful of meshes share two geometries, nothing is
   allocated per frame, and the whole effect is skipped when rendering is
   paused.
   ========================================================================== */
const PULSE_MAX = 2.6;          // metres: how far the ring travels
const PULSE_PERIOD = 2.4;       // seconds per ping
const PULSE_THICK = 0.012;      // ring thickness in metres (thin!)

const pulseGeo = new THREE.RingGeometry(1.0, 1.0 + PULSE_THICK, 72);
const pulses = [];              // { mesh, group, phase }

function makePulse(color) {
  const mat = new THREE.MeshBasicMaterial({
    color, transparent: true, opacity: 0.5,
    side: THREE.DoubleSide, depthWrite: false });
  const m = new THREE.Mesh(pulseGeo, mat);
  m.rotation.x = -Math.PI / 2;   // lie flat on the floor
  m.renderOrder = 5;
  return m;
}

// Ensure each anchor and tag has a small pool of pulse rings.
function ensurePulses() {
  const want = [];
  for (const a of S.scene.anchors) want.push(['anchor', a.id, a.x, a.y, a.z, 0x6ee7b7]);
  for (const t of S.state.tags || []) want.push(['tag', t.id, t.x, t.y, t.z ?? 0.9, 0xffd166]);

  // drop pulses for things that vanished
  const keys = new Set(want.map((w) => w[0] + ':' + w[1]));
  for (let i = pulses.length - 1; i >= 0; i--) {
    if (!keys.has(pulses[i].key)) {
      pulses[i].group.remove(pulses[i].mesh);
      pulses[i].mesh.material.dispose();
      pulses.splice(i, 1);
    }
  }
  // add missing ones
  for (const [kind, id, x, y, z, color] of want) {
    const key = kind + ':' + id;
    if (pulses.some((p) => p.key === key)) continue;
    const group = kind === 'anchor' ? anchorGroup : tagGroup;
    const mesh = makePulse(color);
    mesh.position.set(x, 0.03, y);
    group.add(mesh);
    pulses.push({ key, mesh, group, phase: Math.random() * PULSE_PERIOD });
  }
}

// Animate: expand + fade, looping. Runs every frame but touches only a few
// floats per ring.
let pulseT = 0;
function animatePulses() {
  if (!pulses.length) return;
  pulseT += frameDt;
  for (const p of pulses) {
    const t = ((pulseT + p.phase) % PULSE_PERIOD) / PULSE_PERIOD;   // 0..1
    const r = 0.12 + t * PULSE_MAX;
    p.mesh.scale.set(r, r, r);
    p.mesh.material.opacity = 0.45 * (1 - t) * (1 - t);
  }
  // keep them under the node even while it moves
  for (const p of pulses) {
    if (p.key.startsWith('tag:')) {
      const id = p.key.slice(4);
      const t = (S.state.tags || []).find((x) => x.id === id);
      if (t) p.mesh.position.set(t.x, 0.03, t.y);
    } else {
      const id = p.key.slice(7);
      const a = S.scene.anchors.find((x) => x.id === id);
      if (a) p.mesh.position.set(a.x, 0.03, a.y);
    }
  }
}

/* ------------------------------------------------------------ tags/links */

/* ------------------------------------------------------------ tags/links */
function buildTags() {
  const seen = new Set();
  for (const t of S.state.tags) {
    seen.add(t.id);
    let mesh = tagMeshes[t.id];
    if (!mesh) {
      mesh = new THREE.Group();
      const body = new THREE.Mesh(
        new THREE.SphereGeometry(0.11, 20, 16),
        new THREE.MeshStandardMaterial({ color: COL.tag, emissive: 0x8a6a10,
                                         emissiveIntensity: 0.9, roughness: 0.35 }));
      body.castShadow = true;
      body.userData.pick = 'tag';
      body.userData.id = t.id;
      mesh.add(body);
      // uncertainty ring, scaled from the EKF sigma
      const ring = new THREE.Mesh(
        new THREE.RingGeometry(0.9, 0.93, 64),
        new THREE.MeshBasicMaterial({ color: COL.tag, transparent: true, opacity: 0.35,
                                      side: THREE.DoubleSide }));
      ring.rotation.x = -Math.PI / 2;
      ring.name = 'sigma';
      mesh.add(ring);
      mesh.add(label(t.id, new THREE.Vector3(0, 0.34, 0), 0xffe6a8, 0.13));
      tagGroup.add(mesh);
      tagMeshes[t.id] = mesh;
    }
    mesh.position.set(t.x, t.z ?? 0.9, t.y);

    const ring = mesh.getObjectByName('sigma');
    if (ring) {
      const r = Math.max(0.25, Math.min(t.sigma || 0.3, 3) * 1.6);
      ring.scale.set(r, r, r);
      ring.material.opacity = t.online ? 0.30 : 0.12;
    }
    mesh.visible = true;

    // trail
    if (S.showTrail) {
      const tr = S.trail[t.id] || (S.trail[t.id] = []);
      const last = tr[tr.length - 1];
      if (!last || Math.hypot(last[0] - t.x, last[1] - t.y) > 0.04) {
        tr.push([t.x, t.y]);
        if (tr.length > 220) tr.shift();
      }
    }
  }
  for (const id of Object.keys(tagMeshes))
    if (!seen.has(id)) tagMeshes[id].visible = false;

  buildTrail();
}

// The trail is updated IN PLACE: rebuilding the Line objects every poll
// (500 ms) allocated two geometries per second per tag for no reason.
const trailLines = {};

function buildTrail() {
  if (!S.showTrail) {
    for (const id of Object.keys(trailLines)) {
      const l = trailLines[id];
      trailGroup.remove(l);
      disposeSubtree(l);
      delete trailLines[id];
    }
    return;
  }

  for (const id of Object.keys(S.trail)) {
    const pts = S.trail[id];
    if (pts.length < 2) continue;

    let line = trailLines[id];
    if (!line) {
      line = new THREE.Line(
        new THREE.BufferGeometry(),
        new THREE.LineBasicMaterial({ color: COL.tag, transparent: true, opacity: 0.55 }));
      line.frustumCulled = false;
      trailLines[id] = line;
      trailGroup.add(line);
    }

    const pos = new Float32Array(pts.length * 3);
    for (let i = 0; i < pts.length; i++) {
      pos[i * 3] = pts[i][0];
      pos[i * 3 + 1] = 0.9;
      pos[i * 3 + 2] = pts[i][1];
    }
    line.geometry.dispose();
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    line.geometry = g;
  }
}

function buildLinks() {
  clearGroup(linkGroup);
  for (const t of S.state.tags) {
    for (const [aid, info] of Object.entries(t.links || {})) {
      const a = S.scene.anchors.find((x) => x.id === aid);
      if (!a) continue;
      const blocked = !!info.blocked;
      const pts = [new THREE.Vector3(a.x, a.z, a.y),
                   new THREE.Vector3(t.x, t.z ?? 0.9, t.y)];
      const line = new THREE.Line(
        new THREE.BufferGeometry().setFromPoints(pts),
        new THREE.LineDashedMaterial({
          color: blocked ? COL.nlos : COL.los,
          dashSize: blocked ? 0.10 : 0.22,
          gapSize: blocked ? 0.10 : 0.12,
          transparent: true,
          opacity: blocked ? 0.95 : 0.6 }));
      line.computeLineDistances();
      linkGroup.add(line);
    }
  }
}

// Move an obstacle's mesh in place while dragging its arrows (no rebuild).
function moveObstacleMesh(o) {
  const g = obstacleObjs[o.id];
  if (!g) return;
  g.position.set(o.x, o.z, o.y);
  g.rotation.y = -THREE.MathUtils.degToRad(o.rot || 0);
}

// Resize an obstacle's mesh in place (used while dragging its handles).
function resizeObstacleMesh(o) {
  const g = obstacleObjs[o.id];
  if (!g) return;
  g.position.set(o.x, o.z, o.y);
  g.rotation.y = -THREE.MathUtils.degToRad(o.rot || 0);
  const box = g.children[0];
  const edges = g.children[1];
  if (box) {
    box.geometry.dispose();
    box.geometry = new THREE.BoxGeometry(o.sx, o.sz, o.sy);
  }
  if (edges) {
    edges.geometry.dispose();
    edges.geometry = new THREE.EdgesGeometry(new THREE.BoxGeometry(o.sx, o.sz, o.sy));
  }
  // reposition the corner cubes and the top (height) cube.
  // The arrows stay at the obstacle centre — they translate, they do not size.
  const ox = o.sx / 2 + 0.09, oy = o.sy / 2 + 0.09;
  for (const c of g.children) {
    const kind = c.userData.pick;
    if (kind === 'obstacleHandle') {
      const [sx, sz] = c.userData.corner || [1, 1];
      c.position.set(sx * ox, 0, sz * oy);
    } else if (kind === 'sizeZ') {
      c.position.set(0, o.sz / 2 + 0.13, 0);
    }
  }
}

// Update the inspector numbers live while dragging (no DOM rebuild).
function liveReadout() {
  if (!S.selected) return;
  if (S.selected.type === 'anchor') {
    const a = S.scene.anchors.find((x) => x.id === S.selected.id);
    if (a && $('i-x') && $('i-y')) {
      $('i-x').value = a.x.toFixed(2);
      $('i-y').value = a.y.toFixed(2);
    }
  } else if (S.selected.type === 'obstacle') {
    const o = S.scene.obstacles.find((x) => x.id === S.selected.id);
    if (!o) return;
    for (const [id, v] of [['o-x', o.x], ['o-y', o.y], ['o-z', o.z ?? 0],
                           ['o-sx', o.sx], ['o-sy', o.sy], ['o-sz', o.sz]]) {
      if ($(id)) $(id).value = v.toFixed(2);
    }
  }
}

/* ------------------------------------------------------- scene → rebuild */

// Free GPU resources before dropping a subtree. Without this, every rebuild
// leaked geometries, materials and CanvasTextures; after a few minutes of
// dragging the WebGL context was lost and the viewport went black.
function disposeSubtree(node) {
  node.traverse((o) => {
    if (o.geometry) o.geometry.dispose();
    const mats = Array.isArray(o.material) ? o.material : (o.material ? [o.material] : []);
    for (const m of mats) {
      if (m.map) m.map.dispose();
      m.dispose();
    }
  });
}

function clearGroup(g) {
  while (g.children.length) {
    const c = g.children[0];
    g.remove(c);
    disposeSubtree(c);
  }
}

function rebuildAll() {
  buildRoom();
  buildAnchors();
  buildObstacles();
  buildTags();
  buildLinks();
  ensurePulses();
  buildPalette();
}

/* --------------------------------------------------------------- picking */
function pointerToNdc(ev) {
  const r = renderer.domElement.getBoundingClientRect();
  return new THREE.Vector2(((ev.clientX - r.left) / r.width) * 2 - 1,
                           -((ev.clientY - r.top) / r.height) * 2 + 1);
}

function pick(ev) {
  raycaster.setFromCamera(pointerToNdc(ev), camera);
  const hits = raycaster.intersectObjects(
    [...anchorGroup.children, ...obstacleGroup.children,
     ...tagGroup.children, ...roomGroup.children], true);

  // Walls are translucent decoration: they must never swallow a click, which
  // is what happened in the Top view (the ray entered through the ceiling).
  const usable = hits.filter((h) => {
    let o = h.object;
    while (o && !o.userData.pick) o = o.parent;
    return o && o.userData.pick && o.userData.pick !== 'wall';
  });

  // Resize handles win over everything else: they are small and often overlap
  // the obstacle body, so a plain "first hit wins" test would make resizing
  // impossible (the body would be grabbed instead).
  for (const h of usable) {
    let o = h.object;
    while (o && !o.userData.pick) o = o.parent;
    if (o.userData.pick === 'obstacleHandle' || o.userData.pick === 'obstacleHeight' ||
        o.userData.pick === 'gizmoAxis' || o.userData.pick === 'gizmo' ||
        o.userData.pick === 'sizeZ')
      return { obj: o, point: h.point, hit: h.object };
  }

  for (const h of usable) {
    let o = h.object;
    while (o && !o.userData.pick) o = o.parent;
    if (o.userData.pick) return { obj: o, point: h.point, hit: h.object };
  }
  return null;
}

function groundPoint(ev) {
  raycaster.setFromCamera(pointerToNdc(ev), camera);
  const plane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
  const p = new THREE.Vector3();
  return raycaster.ray.intersectPlane(plane, p) ? p : null;
}

function bindPointer() {
  const el = renderer.domElement;
  let downAt = null;

  const bump = () => { lastInteraction = performance.now(); };
  el.addEventListener('wheel', (e) => { bump(); onWheel(e); }, { passive: false });
  el.addEventListener('pointerdown', bump);
  el.addEventListener('pointermove', bump);
  el.addEventListener('pointerdown', (ev) => {
    if (ev.button !== 0) return;
    downAt = { x: ev.clientX, y: ev.clientY, t: performance.now() };
    const p = pick(ev);
    if (!p) return;

    const u = p.obj.userData;
    if (u.pick === 'anchor' || u.pick === 'obstacle' ||
        u.pick === 'obstacleHandle' || u.pick === 'obstacleHeight' ||
        u.pick === 'gizmoAxis' || u.pick === 'sizeZ') {
      controls.enabled = false;
      const gp = groundPoint(ev);
      S.drag = {
        type: u.pick, id: u.id, corner: u.corner, axis: u.axis,
        startX: gp ? gp.x : 0, startZ: gp ? gp.z : 0,
        startClientY: ev.clientY, startClientX: ev.clientX,
        orig: JSON.parse(JSON.stringify(
          u.pick === 'anchor'
            ? S.scene.anchors.find((a) => a.id === u.id)
            : S.scene.obstacles.find((o) => o.id === u.id))),
      };
    }
  });

  el.addEventListener('pointermove', (ev) => {
    if (!S.drag) { hover(ev); return; }
    const gp = groundPoint(ev);
    if (!gp) return;
    const dx = gp.x - S.drag.startX, dz = gp.z - S.drag.startZ;

    // Move the existing three.js object directly. A full rebuildAll() on every
    // pointermove recreated every mesh, sprite and texture — that is what made
    // dragging stutter and eventually killed the WebGL context. The scene is
    // rebuilt once on pointerup instead.
    if (S.drag.type === 'anchor') {
      const a = S.scene.anchors.find((x) => x.id === S.drag.id);
      if (a) {
        a.x = clamp(S.drag.orig.x + dx, 0, S.scene.room.width);
        a.y = clamp(S.drag.orig.y + dz, 0, S.scene.room.depth);
        const g = anchorObjs[a.id];
        if (g) g.position.set(a.x, a.z, a.y);
        S.dirty = true;
      }
    } else if (S.drag.type === 'obstacle') {
      const o = S.scene.obstacles.find((x) => x.id === S.drag.id);
      if (o) {
        o.x = S.drag.orig.x + dx;
        o.y = S.drag.orig.y + dz;
        const g = obstacleObjs[o.id];
        if (g) g.position.set(o.x, o.z, o.y);
        S.dirty = true;
      }
    } else if (S.drag.type === 'obstacleHandle') {
      const o = S.scene.obstacles.find((x) => x.id === S.drag.id);
      if (o) {
        const rot = THREE.MathUtils.degToRad(o.rot || 0);
        const lx = dx * Math.cos(-rot) - dz * Math.sin(-rot);
        const lz = dx * Math.sin(-rot) + dz * Math.cos(-rot);
        const [cx, cz] = S.drag.corner;
        const axis = S.drag.axis || 'both';
        // edge handles resize one axis; corner handles resize both
        if (axis === 'both' || axis === 'x')
          o.sx = Math.max(0.2, S.drag.orig.sx + cx * lx * 2);
        if (axis === 'both' || axis === 'y')
          o.sy = Math.max(0.2, S.drag.orig.sy + cz * lz * 2);
        S.dirty = true;
        resizeObstacleMesh(o);
      }
    } else if (S.drag.type === 'gizmoAxis') {
      // Arrows TRANSLATE the obstacle along one axis.
      const o = S.scene.obstacles.find((x) => x.id === S.drag.id);
      if (o) {
        const axis = S.drag.axis;
        if (axis === 'z') {
          // vertical: use the pointer travel and the current zoom for scale
          const dy = (S.drag.startClientY ?? ev.clientY) - ev.clientY;
          const scale = 0.01 * Math.max(camera.position.distanceTo(controls.target) / 6, 0.4);
          o.z = Math.max(0.05, S.drag.orig.z + dy * scale);
        } else {
          // horizontal: convert to the obstacle's local frame so the arrows
          // keep working after the box has been rotated
          const rot = THREE.MathUtils.degToRad(o.rot || 0);
          const lx = dx * Math.cos(-rot) - dz * Math.sin(-rot);
          const lz = dx * Math.sin(-rot) + dz * Math.cos(-rot);
          if (axis === 'x') o.x = S.drag.orig.x + lx;
          else if (axis === 'y') o.y = S.drag.orig.y + lz;
        }
        S.dirty = true;
        moveObstacleMesh(o);
        liveReadout();
      }
    } else if (S.drag.type === 'sizeZ') {
      // top cube RESIZES the height
      const o = S.scene.obstacles.find((x) => x.id === S.drag.id);
      if (o) {
        const dy = (S.drag.startClientY ?? ev.clientY) - ev.clientY;
        const scale = 0.01 * Math.max(camera.position.distanceTo(controls.target) / 6, 0.4);
        o.sz = Math.max(0.2, S.drag.orig.sz + dy * scale);
        S.dirty = true;
        resizeObstacleMesh(o);
      }
    } else if (S.drag.type === 'obstacleHeight') {
      const o = S.scene.obstacles.find((x) => x.id === S.drag.id);
      if (o) {
        // movementY is unreliable (0 for touch and synthetic events), so use
        // the actual pointer travel since the drag started.
        const dy = (S.drag.startClientY ?? ev.clientY) - ev.clientY;
        const scale = 0.01 * Math.max(camera.position.distanceTo(controls.target) / 6, 0.4);
        o.sz = Math.max(0.2, S.drag.orig.sz + dy * scale);
        S.dirty = true;
        resizeObstacleMesh(o);
      }
    }
    liveReadout();
  });

  el.addEventListener('pointerup', (ev) => {
    const quick = downAt && Math.hypot(ev.clientX - downAt.x, ev.clientY - downAt.y) < 4
                  && performance.now() - downAt.t < 400;

    if (S.drag) {
      const dragged = !quick;      // a real move, not just a click
      S.drag = null;
      controls.enabled = true;
      if (dragged) {
        rebuildAll();              // one rebuild after the drag, not per move
        if (S.selected) showSelection();
        downAt = null;
        return;
      }
      // A click on a body selects it (this is what makes the resize handles
      // appear). Previously every pointerdown entered drag mode and pointerup
      // bailed out, so objects could never be selected by clicking.
      const p = pick(ev);
      if (p) {
        const u = p.obj.userData;
        if (u.pick === 'anchor') select('anchor', u.id);
        else if (u.pick === 'obstacle') select('obstacle', u.id);
        else if (u.pick === 'tag') select('tag', u.id);
        else select(null, null);
      } else {
        select(null, null);
      }
      downAt = null;
      return;
    }
    if (!quick) { downAt = null; return; }

    // placing mode wins over picking: the floor is a valid click target
    if (S.mode === 'addAnchor' || S.mode === 'addObstacle') {
      const gp = groundPoint(ev);
      if (gp) {
        const x = clamp(gp.x, 0, S.scene.room.width);
        const y = clamp(gp.z, 0, S.scene.room.depth);
        if (S.mode === 'addAnchor') addAnchor(x, y);
        else addObstacle(x, y);
      }
      downAt = null;
      return;
    }

    const p = pick(ev);
    if (p) {
      const u = p.obj.userData;
      if (u.pick === 'anchor') select('anchor', u.id);
      else if (u.pick === 'obstacle') select('obstacle', u.id);
      else if (u.pick === 'tag') select('tag', u.id);
      else select(null, null);
    } else {
      select(null, null);
    }
    downAt = null;
  });

  el.addEventListener('pointerleave', () => { S.drag = null; controls.enabled = true; });
}

function hover(ev) {
  const p = pick(ev);
  const el = renderer.domElement;
  if (!p) { el.style.cursor = S.mode === 'select' ? 'grab' : 'crosshair'; return; }
  const k = p.obj.userData.pick;
  el.style.cursor = (k === 'anchor' || k === 'obstacle') ? 'move'
                  : (k === 'obstacleHandle' || k === 'obstacleHeight') ? 'nwse-resize'
                  : 'pointer';
}

const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

/* ------------------------------------------------------------- mutations */
function addAnchor(x, y) {
  let n = 1;
  while (S.scene.anchors.some((a) => a.id === `anchor-${n}`)) n++;
  const id = `anchor-${n}`;
  S.scene.anchors.push({ id, label: `A${n}`, x, y, z: 2.2 });
  S.dirty = true;
  S.mode = 'select';
  rebuildAll();
  select('anchor', id);
  toast(`Anchor ${id} added — drag it to the corner, then Save.`);
}

function addObstacle(x, y) {
  let n = 1;
  while (S.scene.obstacles.some((o) => o.id === `obstacle-${n}`)) n++;
  const id = `obstacle-${n}`;
  S.scene.obstacles.push({
    id, label: `Obstacle ${n}`, x, y, z: S.scene.room.height / 2,
    sx: 1.0, sy: 0.4, sz: S.scene.room.height, rot: 0, atten: 1.0 });
  S.dirty = true;
  S.mode = 'select';
  rebuildAll();
  select('obstacle', id);
  toast('Obstacle added — drag the body to move, corners to resize, cone for height.');
}

function removeSelected() {
  if (!S.selected) return;
  const { type, id } = S.selected;
  if (type === 'anchor') S.scene.anchors = S.scene.anchors.filter((a) => a.id !== id);
  else if (type === 'obstacle') S.scene.obstacles = S.scene.obstacles.filter((o) => o.id !== id);
  else return;
  S.dirty = true;
  select(null, null);
  rebuildAll();
}

function select(type, id) {
  S.selected = type ? { type, id } : null;
  // Only the anchor/obstacle groups render selection visuals, so avoid a full
  // rebuild (which would also recreate the tag meshes and the trail).
  buildAnchors();
  buildObstacles();
  showSelection();
}

function showSelection() {
  const host = $('inspector');
  if (!S.selected) { host.innerHTML = '<p class="muted">Nothing selected.<br>Click an anchor, obstacle or the tag.</p>'; return; }
  const { type, id } = S.selected;

  if (type === 'anchor') {
    const a = S.scene.anchors.find((x) => x.id === id);
    if (!a) { host.innerHTML = ''; return; }
    host.innerHTML = `
      <h3>Anchor <span class="pill">${a.id}</span></h3>
      <label>Label<input id="i-label" value="${esc(a.label || a.id)}"></label>
      <div class="row">
        <label>X (m)<input id="i-x" type="number" step="0.05" value="${a.x.toFixed(2)}"></label>
        <label>Y (m)<input id="i-y" type="number" step="0.05" value="${a.y.toFixed(2)}"></label>
      </div>
      <label>Height Z (m)<input id="i-z" type="number" step="0.05" value="${a.z.toFixed(2)}"></label>
      <div class="row btns"><button id="i-del" class="danger">Delete</button></div>`;
    $('i-label').oninput = (e) => { a.label = e.target.value; S.dirty = true; rebuildAll(); };
    $('i-x').oninput = (e) => { a.x = +e.target.value; S.dirty = true; rebuildAll(); };
    $('i-y').oninput = (e) => { a.y = +e.target.value; S.dirty = true; rebuildAll(); };
    $('i-z').oninput = (e) => { a.z = +e.target.value; S.dirty = true; rebuildAll(); };
    $('i-del').onclick = removeSelected;
  } else if (type === 'obstacle') {
    const o = S.scene.obstacles.find((x) => x.id === id);
    if (!o) { host.innerHTML = ''; return; }
    host.innerHTML = `
      <h3>Obstacle <span class="pill">${o.id}</span></h3>
      <label>Label<input id="o-label" value="${esc(o.label || o.id)}"></label>
      <div class="row">
        <label>X (m)<input id="o-x" type="number" step="0.05" value="${o.x.toFixed(2)}"></label>
        <label>Y (m)<input id="o-y" type="number" step="0.05" value="${o.y.toFixed(2)}"></label>
      </div>
      <div class="row">
        <label>Size X (m)<input id="o-sx" type="number" step="0.05" min="0.2" value="${o.sx.toFixed(2)}"></label>
        <label>Size Y (m)<input id="o-sy" type="number" step="0.05" min="0.2" value="${o.sy.toFixed(2)}"></label>
      </div>
      <label>Height Z (m)<input id="o-sz" type="number" step="0.05" min="0.2" value="${o.sz.toFixed(2)}"></label>
      <label>Centre height Z (m) <span class="muted">(drag the blue arrow)</span>
        <input id="o-z" type="number" step="0.05" value="${(o.z ?? 0).toFixed(2)}"></label>
      <label>Rotation (°)<input id="o-rot" type="number" step="5" value="${(o.rot || 0).toFixed(0)}"></label>
      <label>Attenuation (0 = transparent, 1 = solid)
        <input id="o-atten" type="range" min="0" max="1" step="0.05" value="${o.atten ?? 1}">
        <span id="o-atten-v">${(o.atten ?? 1).toFixed(2)}</span></label>
      <div class="row btns"><button id="o-del" class="danger">Delete</button></div>`;
    const bind = (key, el, fn) => {
      $(el).oninput = (e) => { fn(+e.target.value); S.dirty = true; rebuildAll(); };
    };
    $('o-label').oninput = (e) => { o.label = e.target.value; S.dirty = true; };
    bind('x', 'o-x', (v) => o.x = v);
    bind('y', 'o-y', (v) => o.y = v);
    bind('sx', 'o-sx', (v) => o.sx = Math.max(0.2, v));
    bind('sy', 'o-sy', (v) => o.sy = Math.max(0.2, v));
    bind('sz', 'o-sz', (v) => o.sz = Math.max(0.2, v));
    bind('rot', 'o-rot', (v) => o.rot = v);
    bind('z', 'o-z', (v) => { o.z = v; moveObstacleMesh(o); });
    $('o-atten').oninput = (e) => {
      o.atten = +e.target.value;
      $('o-atten-v').textContent = o.atten.toFixed(2);
      S.dirty = true; rebuildAll();
    };
    $('o-del').onclick = removeSelected;
  } else if (type === 'tag') {
    const t = S.state.tags.find((x) => x.id === id) || { id };
    host.innerHTML = `
      <h3>Tag <span class="pill">${t.id}</span></h3>
      <table class="kv">
        <tr><td>Position</td><td>${fmt(t.x)}, ${fmt(t.y)} m</td></tr>
        <tr><td>Height</td><td>${fmt(t.z ?? 0.9)} m</td></tr>
        <tr><td>Speed</td><td>${Math.hypot(t.vx || 0, t.vy || 0).toFixed(2)} m/s</td></tr>
        <tr><td>1σ</td><td>${fmt(t.sigma)} m</td></tr>
        <tr><td>Confidence</td><td>${((t.confidence ?? 0) * 100).toFixed(0)}%</td></tr>
        <tr><td>Status</td><td>${t.online ? 'live' : 'offline'}</td></tr>
      </table>
      <h4>Links</h4>${linksTable(t)}
      ${t.geometry_ok === false ? `<p class="bad" style="font-size:12px">⚠ Range measurements are geometrically inconsistent by ${(t.geometry_slack || 0).toFixed(2)} m — one or more anchors are likely measuring a reflection (NLOS), not the direct path.</p>` : ''}`;
  }
}

function linksTable(t) {
  const rows = Object.entries(t.links || {}).map(([aid, i]) => `
    <tr><td>${aid}</td><td>${fmt(t.ranges?.[aid])} m</td>
        <td>${i.blocked ? '<span class="bad">NLOS</span>' : '<span class="good">LOS</span>'}</td>
        <td>σ ${fmt(i.sigma)}</td></tr>`).join('');
  return rows ? `<table class="kv"><tr><th>Anchor</th><th>Range</th><th>Path</th><th>Noise</th></tr>${rows}</table>`
              : '<p class="muted">No links yet.</p>';
}

const esc = (s) => String(s).replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
const fmt = (v) => (v === undefined || v === null) ? '—' : Number(v).toFixed(2);

/* ---------------------------------------------------------------- palette */
function buildPalette() {
  const host = $('palette');
  const items = [
    { t: 'anchor', n: 'Anchor', c: '#6ee7b7' },
    { t: 'obstacle', n: 'Obstacle', c: '#8b5cf6' },
  ];
  host.innerHTML = items.map((i) =>
    `<button class="pal" data-add="${i.t}"><span style="background:${i.c}"></span>${i.n}</button>`).join('');
  host.querySelectorAll('.pal').forEach((b) => {
    b.onclick = () => {
      S.mode = b.dataset.add === 'anchor' ? 'addAnchor' : 'addObstacle';
      toast(`Click on the floor to place the ${b.dataset.add}.`);
    };
  });
}

/* -------------------------------------------------------------------- io */
async function loadScene() {
  const r = await fetch('/api/v1/scene');
  const d = await r.json();
  S.scene = d.scene;
  S.nlos = d.nlos_enabled !== false;
  $('nlos').checked = S.nlos;
  syncRoomInputs();
  rebuildAll();
}

async function saveScene() {
  const r = await fetch('/api/v1/scene', {
    method: 'PUT', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ scene: S.scene, nlos_enabled: S.nlos }) });
  const d = await r.json();
  S.dirty = false;
  toast(d.ok ? `Saved — pushed to ${d.pushed?.length || 0} device(s).` : 'Save failed.');
  setSaved();
}

async function poll() {
  try {
    const r = await fetch('/api/v1/state');
    S.state = await r.json();
    if (!S.drag) { buildTags(); buildLinks(); updateAnchorLabels(); ensurePulses(); }
    updateHud();
    renderTagList();
    cueTransitions();
    if (S.selected?.type === 'tag') showSelection();
    updateFog();
  } catch (e) { /* server restarting */ }
}

function updateHud() {
  const live = S.state.tags?.filter((t) => t.online).length || 0;
  const anc = S.state.anchors?.filter((a) => a.online).length || 0;
  const tag = S.state.tags?.[0];
  const coord = tag && tag.online
    ? `<span class="sep"></span><span class="mono">tag (${tag.x.toFixed(2)}, ${tag.y.toFixed(2)}) m</span>`
    : '';
  // Geometry warning: ranges that violate the triangle inequality mean at
  // least one anchor is measuring a reflection (NLOS), not the direct path.
  const geoWarn = (tag && tag.online && tag.geometry_ok === false)
    ? `<span class="sep"></span><span class="warn" title="Range measurements are geometrically inconsistent — likely NLOS reflections. Slack: ${(tag.geometry_slack || 0).toFixed(2)} m">⚠ NLOS ${(tag.geometry_slack || 0).toFixed(2)} m</span>`
    : '';
  $('hud').innerHTML =
    `<span class="dot ${anc ? 'on' : 'off'}"></span>${anc} anchor${anc === 1 ? '' : 's'} online` +
    `<span class="sep"></span><span class="dot ${live ? 'on' : 'off'}"></span>` +
    `${live} tag${live === 1 ? '' : 's'} live` + coord + geoWarn +
    (S.state.nlos_enabled === false ? '<span class="sep"></span><span class="warn">NLOS off</span>' : '');
}

function syncRoomInputs() {
  $('r-w').value = S.scene.room.width.toFixed(2);
  $('r-d').value = S.scene.room.depth.toFixed(2);
  $('r-h').value = S.scene.room.height.toFixed(2);
}

function setDirty() { S.dirty = true; $('save').classList.add('dirty'); }
function setSaved() { $('save').classList.remove('dirty'); }

let toastTimer = null;
function toast(msg) {
  const el = $('toast');
  el.textContent = msg;
  el.classList.add('show');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => el.classList.remove('show'), 3800);
}

/* ------------------------------------------------------------------- init */
/* ==========================================================================
   Sound: tiny synthesised cues (no assets to ship). A soft blip when a device
   comes online, a two-tone chime for an update push, a low tone for a lost
   link. All optional and off until the user enables it.
   ========================================================================== */
const Audio2 = (() => {
  let ctx = null;
  let enabled = true;

  const ensure = () => {
    if (!ctx) {
      const AC = window.AudioContext || window.webkitAudioContext;
      if (!AC) return null;
      ctx = new AC();
    }
    if (ctx.state === 'suspended') ctx.resume();
    return ctx;
  };

  // one shaped tone
  const tone = (freq, dur, type = 'sine', gain = 0.06, slideTo = null) => {
    if (!enabled) return;
    const c = ensure();
    if (!c) return;
    const o = c.createOscillator();
    const g = c.createGain();
    o.type = type;
    o.frequency.setValueAtTime(freq, c.currentTime);
    if (slideTo) o.frequency.exponentialRampToValueAtTime(slideTo, c.currentTime + dur);
    g.gain.setValueAtTime(0.0001, c.currentTime);
    g.gain.exponentialRampToValueAtTime(gain, c.currentTime + 0.012);
    g.gain.exponentialRampToValueAtTime(0.0001, c.currentTime + dur);
    o.connect(g).connect(c.destination);
    o.start();
    o.stop(c.currentTime + dur + 0.02);
  };

  return {
    get enabled() { return enabled; },
    set enabled(v) { enabled = v; if (v) ensure(); },
    online()  { tone(660, 0.10, 'triangle', 0.05); setTimeout(() => tone(990, 0.12, 'triangle', 0.04), 90); },
    offline() { tone(220, 0.22, 'sawtooth', 0.035, 140); },
    update()  { tone(440, 0.09, 'square', 0.035); setTimeout(() => tone(880, 0.14, 'square', 0.03), 100); },
    nlos()    { tone(300, 0.18, 'sine', 0.03, 200); },
    boot()    { tone(523, 0.10, 'sine', 0.04); setTimeout(() => tone(784, 0.14, 'sine', 0.035), 110);
                setTimeout(() => tone(1046, 0.20, 'sine', 0.03), 230); },
  };
})();

/* ==========================================================================
   Tabs
   ========================================================================== */
function setTab(name) {
  document.querySelectorAll('#tabs button').forEach((b) =>
    b.classList.toggle('active', b.dataset.tab === name));
  document.querySelectorAll('#panel section[data-panel]').forEach((s) =>
    (s.hidden = s.dataset.panel !== name));
  if (name === 'devices') renderDevices();
  if (name === 'api') renderApi();
  if (name === 'setup') loadOta();
}

/* ==========================================================================
   Devices panel — IP, signal, firmware, per-device OTA key
   ========================================================================== */
async function renderDevices() {
  const host = $('devicelist');
  const devs = S.state.devices || [];
  if (!devs.length) { host.innerHTML = '<p class="muted">No device has reported yet.</p>'; return; }

  let ota = { devices: [] };
  try { ota = await (await fetch('/api/v1/ota')).json(); } catch (e) { /* offline */ }
  const keyOf = (id) => (ota.devices || []).find((d) => d.id === id) || {};

  host.innerHTML = devs.map((d) => {
    const o = keyOf(d.id);
    const rssi = (d.rssi === null || d.rssi === undefined) ? '—' : `${d.rssi} dBm`;
    return `<div class="dev">
      <span class="dot ${d.online ? 'on' : 'off'}"></span>
      <div>
        <div class="name">${esc(d.id)} <span class="pill">${esc(d.role || '')}</span></div>
        <div class="ip">${esc(d.ip || 'no ip yet')}</div>
        <div class="meta">${rssi} · fw ${esc(d.fw || '?')}</div>
      </div>
      <div class="spacer"></div>
      <button data-ota="${esc(d.id)}" title="Push firmware to this device">OTA</button>
      <button data-key="${esc(d.id)}" title="Show / rotate the OTA key">🔑</button>
    </div>`;
  }).join('');

  host.querySelectorAll('[data-ota]').forEach((b) => {
    b.onclick = () => pushOta(b.dataset.ota);
  });
  host.querySelectorAll('[data-key]').forEach((b) => {
    b.onclick = async () => {
      const id = b.dataset.key;
      const o = keyOf(id);
      const rotate = confirm(`Key for ${id}:\n\n${o.key || '(unknown)'}\n\nRotate it? The device picks up the new key on its next config sync.`);
      if (!rotate) return;
      try {
        const r = await (await fetch('/api/v1/ota/key', {
          method: 'POST', headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ device_id: id }) })).json();
        toast(`New key for ${id}: ${r.key}`);
        renderDevices();
      } catch (e) { toast('Key rotation failed'); }
    };
  });
}

/* ==========================================================================
   OTA — list server-side images and push them
   ========================================================================== */
let otaInfo = { firmware: [], devices: [] };

async function loadOta() {
  try {
    otaInfo = await (await fetch('/api/v1/ota')).json();
  } catch (e) {
    $('ota-hint').textContent = 'Server OTA endpoint unreachable.';
    return;
  }
  const sel = $('ota-file');
  const fw = otaInfo.firmware || [];
  sel.innerHTML = fw.length
    ? fw.map((f, i) => {
        const when = f.mtime ? new Date(f.mtime * 1000).toLocaleString() : '';
        return `<option value="${esc(f.name)}">${i === 0 ? '★ ' : ''}${esc(f.name)} — ` +
               `${(f.size / 1024).toFixed(0)} kB${when ? ' · ' + esc(when) : ''}</option>`;
      }).join('')
    : '<option value="">(no .bin in server/firmware/)</option>';
  // the server sorts newest first, so index 0 is already the latest build
  if (fw.length) sel.selectedIndex = 0;

  const online = (otaInfo.devices || []).filter((d) => d.online).length;
  $('ota-hint').innerHTML =
    `Server folder <code>server/firmware/</code> · port <b>${otaInfo.port || 3232}</b> · ` +
    `${online} device${online === 1 ? '' : 's'} online.<br>` +
    `Each device carries its own key; updates are refused without it.`;
}

async function pushOta(deviceId) {
  const name = $('ota-file')?.value;
  if (!name) { toast('No firmware image on the server.'); return; }
  const who = deviceId === 'all' ? 'ALL online devices' : deviceId;
  if (!confirm(`Push ${name} to ${who}?\n\nThe device reboots when the update finishes.`)) return;

  $('ota-result').textContent = `Uploading ${name} to ${who}…`;
  Audio2.update();
  try {
    const r = await (await fetch('/api/v1/ota/push', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ device_id: deviceId, firmware: name }) })).json();
    const lines = Object.entries(r.results || {}).map(([k, v]) => `${k}: ${v}`);
    $('ota-result').innerHTML = lines.map(esc).join('<br>');
    toast(r.ok ? 'Update pushed.' : 'Update failed — see the result panel.');
  } catch (e) {
    $('ota-result').textContent = 'Push failed: ' + e;
    toast('Update failed.');
  }
}

/* ==========================================================================
   API tab — endpoints come from the server, so they never go stale
   ========================================================================== */
let apiInfo = null;

async function renderApi() {
  const host = $('apilist');
  try {
    apiInfo = await (await fetch('/api/v1/meta')).json();
  } catch (e) { host.innerHTML = '<p class="muted">Server unreachable.</p>'; return; }

  host.innerHTML = apiInfo.endpoints.map((e) => `
    <div class="ep">
      ${e.methods.map((m) => `<span class="m ${m}">${m}</span>`).join('')}
      <span class="p">${esc(e.path)}</span>
      ${e.doc ? `<div class="d">${esc(e.doc)}</div>` : ''}
    </div>`).join('');

  const base = (S.scene && S.state && S.state.site) || 'uwb/home';
  $('mqtttopics').innerHTML = `
    <tr><th>Topic</th><th>Dir</th></tr>
    <tr><td><code>uwb/home/telemetry</code></td><td>device → server</td></tr>
    <tr><td><code>uwb/home/range</code></td><td>device → server</td></tr>
    <tr><td><code>uwb/home/status/&lt;id&gt;</code></td><td>device → server</td></tr>
    <tr><td><code>uwb/home/config/&lt;id&gt;</code></td><td>server → device</td></tr>
    <tr><td><code>uwb/home/cmd/&lt;id&gt;</code></td><td>server → device</td></tr>
    <tr><td><code>uwb/home/state</code></td><td>server → all</td></tr>`;
}

/* ==========================================================================
   Theme
   ========================================================================== */
function applyTheme(theme) {
  document.documentElement.dataset.theme = theme;
  try { localStorage.setItem('uwb-theme', theme); } catch (e) {}
  $('theme-toggle').textContent = theme === 'light' ? '🌙' : '☀';
  if (scene3) {
    const light = theme === 'light';
    scene3.background = new THREE.Color(light ? 0xeef2fb : 0x0b0e1a);
    if (scene3.fog) scene3.fog.color = scene3.background;
    if (roomGroup) buildRoom();
  }
}

/* ==========================================================================
   Render toggle — pausing the loop saves a lot of CPU/GPU on a laptop when
   the view is only being watched. Default: rendering ON.
   ========================================================================== */
let renderOn = true;

function toggleRender() {
  renderOn = !renderOn;
  const b = $('render-toggle');
  b.textContent = renderOn ? '⏸ Pause' : '▶ Resume';
  b.classList.toggle('on', !renderOn);
  toast(renderOn ? 'Rendering resumed' : 'Rendering paused — CPU saved (Space to resume)');
}

/* ==========================================================================
   Tags panel
   ========================================================================== */
function renderTagList() {
  const host = $('taglist');
  const tags = S.state.tags || [];
  if (!tags.length) { host.innerHTML = '<p class="muted">No tag reporting yet.</p>'; return; }
  host.innerHTML = tags.map((t) => `
    <div class="dev">
      <span class="dot ${t.online ? 'on' : 'off'}"></span>
      <div>
        <div class="name">${esc(t.id)}</div>
        <div class="meta mono">(${t.x.toFixed(2)}, ${t.y.toFixed(2)}) m · σ ${(t.sigma || 0).toFixed(2)}</div>
        <div class="meta">${esc(t.ip || 'no ip yet')} · conf ${((t.confidence || 0) * 100).toFixed(0)}%</div>
      </div>
    </div>`).join('');
}

/* ==========================================================================
   Sound cues: track online/offline transitions so a beep only fires on change
   ========================================================================== */
const wasOnline = {};

function cueTransitions() {
  for (const d of S.state.devices || []) {
    const prev = wasOnline[d.id];
    if (prev === undefined) { wasOnline[d.id] = d.online; continue; }
    if (prev !== d.online) {
      wasOnline[d.id] = d.online;
      if (d.online) Audio2.online(); else Audio2.offline();
    }
  }
  // NLOS warning tone, only on the rising edge
  const tag = S.state.tags?.[0];
  const bad = tag && tag.online && tag.geometry_ok === false;
  if (bad && !cueTransitions._nlos) { Audio2.nlos(); }
  cueTransitions._nlos = !!bad;
}

/* ==========================================================================
   Intro: fade the boot overlay, chime, and sweep the camera in with a spin.
   Runs once per page load.
   ========================================================================== */
function playIntro() {
  const boot = $('boot');
  setTimeout(() => {
    boot.classList.add('gone');
    Audio2.boot();
  }, 900);

  // camera sweep: start wide and high, settle into the home view
  const home = {
    pos: camera.position.clone(),
    target: controls.target.clone(),
  };
  const start = home.pos.clone().multiplyScalar(2.1);
  start.y = Math.max(home.pos.y * 2.4, 12);
  const t0 = performance.now();
  const dur = 2100;
  intro = { home, start, t0, dur };
}

let intro = null;

function applyIntro() {
  if (!intro) return;
  const p = Math.min((performance.now() - intro.t0) / intro.dur, 1);
  const e = 1 - Math.pow(1 - p, 3);            // ease-out cubic
  // position: lerp from the wide start to home, plus one full rotation
  const pos = intro.start.clone().lerp(intro.home.pos, e);
  const ang = (1 - e) * Math.PI * 1.35;        // spin ~240 degrees
  const t = intro.home.target;
  const dx = pos.x - t.x, dz = pos.z - t.z;
  const c = Math.cos(ang), s = Math.sin(ang);
  camera.position.set(t.x + dx * c - dz * s, pos.y, t.z + dx * s + dz * c);
  camera.lookAt(t);
  if (p >= 1) { intro = null; rememberHome(); }
}

function initUi() {
  // tabs
  document.querySelectorAll('#tabs button').forEach((b) => {
    b.onclick = () => setTab(b.dataset.tab);
  });

  ['r-w', 'r-d', 'r-h'].forEach((id) => {
    $(id).onchange = () => {
      S.scene.room.width = Math.max(0.5, +$('r-w').value);
      S.scene.room.depth = Math.max(0.5, +$('r-d').value);
      S.scene.room.height = Math.max(0.5, +$('r-h').value);
      setDirty();
      rebuildAll();
      syncRoomInputs();
    };
  });
  $('save').onclick = saveScene;
  $('reload').onclick = loadScene;
  $('nlos').onchange = (e) => { S.nlos = e.target.checked; setDirty(); };
  $('trail').onchange = (e) => { S.showTrail = e.target.checked; buildTrail(); };
  $('spin').onchange = (e) => { S.autoSpin = e.target.checked; };
  $('view-top').onclick = () => setView('top');
  $('view-iso').onclick = () => setView('iso');
  $('view-home').onclick = resetView;
  $('zoom-in').onclick = () => zoomBy(-1);
  $('zoom-out').onclick = () => zoomBy(1);
  $('trail-clear').onclick = () => { S.trail = {}; buildTrail(); };
  $('render-toggle').onclick = toggleRender;
  $('sound-toggle').onclick = () => {
    Audio2.enabled = !Audio2.enabled;
    $('sound-toggle').textContent = Audio2.enabled ? '🔊' : '🔇';
    $('sound-toggle').classList.toggle('on', !Audio2.enabled);
    if (Audio2.enabled) Audio2.online();
  };
  $('theme-toggle').onclick = () =>
    applyTheme(document.documentElement.dataset.theme === 'light' ? 'dark' : 'light');
  $('ota-push-all').onclick = () => pushOta('all');

  addEventListener('keydown', (e) => {
    if (e.key === 'Delete' || e.key === 'Backspace') {
      if (document.activeElement.tagName !== 'INPUT') { removeSelected(); e.preventDefault(); }
    }
    if (e.key === 'Escape') select(null, null);
    if (document.activeElement.tagName === 'INPUT') return;
    if (e.key === '+' || e.key === '=') zoomBy(-1);
    if (e.key === '-' || e.key === '_') zoomBy(1);
    if (e.key === 'h' || e.key === 'H') resetView();
    if (e.key === ' ') { toggleRender(); e.preventDefault(); }
  });
  addEventListener('beforeunload', (e) => {
    if (S.dirty) { e.preventDefault(); e.returnValue = ''; }
  });
}

function setView(kind) {
  const { width: W, depth: D } = S.scene.room;
  if (kind === 'top') {
    camera.position.set(W / 2, Math.max(W, D) * 1.25, D / 2 + 0.01);
    controls.target.set(W / 2, 0, D / 2);
  } else {
    camera.position.set(W * 1.25, Math.max(W, D) * 1.05, D * 1.45);
    controls.target.set(W / 2, 0.8, D / 2);
  }
  zoomAccum = 0;            // drop any pending wheel momentum
  zoomFocus = null;
  controls.update();
  rememberHome();
}

// The default viewport, remembered so "Reset view" always returns to it
// (also used as the starting point on load).
let homeView = null;

function rememberHome() {
  homeView = {
    pos: camera.position.clone(),
    target: controls.target.clone(),
  };
}

function resetView() {
  if (!homeView) { setView('iso'); return; }
  camera.position.copy(homeView.pos);
  controls.target.copy(homeView.target);
  zoomAccum = 0;
  zoomFocus = null;
  select(null, null);
  controls.update();
}

// One controlled notch, reusing the smooth wheel path.
function zoomBy(notches) {
  zoomAccum += notches * ZOOM_UNIT;
}

initThree();
initUi();
loadScene().then(() => { setView('iso'); playIntro(); });
applyTheme(localStorage.getItem('uwb-theme') || 'dark');
poll();
setInterval(poll, 500);

// Small read-only hook used by the automated UI tests (camera distance,
// pending zoom momentum, held keys). Harmless in normal use.

// Screen positions of the selected obstacle's handles (test/debug helper).
window.__uwbHandles = (id) => {
  const g = obstacleObjs[id];
  if (!g) return [];
  const r = renderer.domElement.getBoundingClientRect();
  const out = [];
  for (const c of g.children) {
    if (c.userData.pick !== 'obstacleHandle' && c.userData.pick !== 'sizeZ') continue;
    const p = new THREE.Vector3();
    c.getWorldPosition(p);
    p.project(camera);
    out.push({
      kind: c.userData.pick, axis: c.userData.axis || null,
      x: r.left + (p.x + 1) / 2 * r.width,
      y: r.top + (1 - p.y) / 2 * r.height,
    });
  }
  return out;
};

// What would pick() return at these client coordinates? (test/debug helper)
window.__uwbPickAt = (clientX, clientY) => {
  const p = pick({ clientX, clientY });
  if (!p) return null;
  return { pick: p.obj.userData.pick, id: p.obj.userData.id || null,
           axis: p.obj.userData.axis || null };
};

// Label sprite state (test/debug helper).
window.__uwbLabels = () => {
  const out = [];
  const walk = (root, where) => {
    root.traverse((o) => {
      if (o.isSprite && o.userData && o.userData.setText) {
        const img = o.material.map && o.material.map.image;
        out.push({
          where,
          text: (o.userData.text || '').replace(/\n/g, '|'),
          scale: [+o.scale.x.toFixed(4), +o.scale.y.toFixed(4)],
          aspect: +(o.scale.x / o.scale.y).toFixed(3),
          canvas: img ? [img.width, img.height] : null,
          texAspect: img && img.height ? +(img.width / img.height).toFixed(3) : null,
        });
      }
    });
  };
  walk(anchorGroup, 'anchor');
  walk(tagGroup, 'tag');
  return out;
};

// Pulse ring state (test/debug helper).
window.__uwbPulse = () => pulses.map((p) => ({
  key: p.key,
  scale: +p.mesh.scale.x.toFixed(3),
  opacity: +p.mesh.material.opacity.toFixed(3),
  pos: [+p.mesh.position.x.toFixed(2), +p.mesh.position.z.toFixed(2)],
  visible: p.mesh.visible,
}));

window.__uwbProbe = () => ({
  dist: camera.position.distanceTo(controls.target),
  zoomAccum,
  keys: [...heldKeys],
  target: [controls.target.x, controls.target.y, controls.target.z],
  pos: [camera.position.x, camera.position.y, camera.position.z],
  selected: S.selected,
  mode: S.mode,
  scene: S.scene,
  anchors: S.scene.anchors.map((a) => [a.id, a.x, a.y]),
  obstacles: S.scene.obstacles.map((o) => [o.id, o.x, o.y, o.sx, o.sy, o.sz, o.z ?? 0]),
  handles: Object.keys(obstacleObjs).map((id) => {
    const g = obstacleObjs[id];
    return [id, g.children.map((c) => c.userData.pick || 'mesh')];
  }),
});
