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

/* 3D palette. The scene has to sit on two very different backdrops, so the
   colours are declared per theme and swapped in setPalette() below. Before
   this, only the background followed the theme and the room/obstacles stayed
   dark blue on a paper-white canvas. */
const COL_DARK = {
  room: 0x1a1a1f, wall: 0x24242b, grid: 0x2a2a31, anchor: 0x34d399, anchorOff: 0x55555c,
  tag: 0xffb020, los: 0x34d399, nlos: 0xe8243b, obstacle: 0x3a3a44,
  sel: 0xe01f34, ghost: 0x8e8e92,
  ruler: 0xffffff, rulerMajor: 0xffffff, rulerText: 0xffffff,
  dimText: 0xa8b8dc, originText: 0xd8e0ff,
  axisX: 0xff6b6b, axisY: 0x6bff9c, axisZ: 0x6bb5ff,
  labelOutline: 0x0a0a0c,
  labelOn: 0xcffbe6, labelOff: 0xc3c9d6, labelTag: 0xffe6a8,
  plateBg: 0x111114, plateLine: 0x3a3a44,
  gridMajor: 0x2e2e38, wallGlass: 0x3a3a46,
  anchorBody: 0x1c3b30, anchorBodyOff: 0x24242b, tagBody: 0x4a3208,
};
const COL_LIGHT = {
  room: 0xdedbd5, wall: 0xcfccc5, grid: 0xc9c6c0, anchor: 0x0f8a5f, anchorOff: 0x9a9aa4,
  tag: 0xd97706, los: 0x0f8a5f, nlos: 0xe8243b, obstacle: 0xc9c6c0,
  sel: 0xe01f34, ghost: 0x5c5c66,
  // Darker than the paper background: the old pale blues were unreadable here.
  ruler: 0xe01f34, rulerMajor: 0xc4162a, rulerText: 0xe01f34,
  dimText: 0x2e2e38, originText: 0x2a2a33,
  axisX: 0xc81e2b, axisY: 0x0f8a5f, axisZ: 0x1d5fa8,
  labelOutline: 0xeceae6,
  // On paper, a pale green label over a pale wall disappeared; these are dark
  // enough to hold against #eceae6 while still reading as "online".
  labelOn: 0x0a5c3f, labelOff: 0x4a4a55, labelTag: 0x7a4a00,
  plateBg: 0xf6f5f2, plateLine: 0xc9c6c0,
  gridMajor: 0xb8b4ac, wallGlass: 0xa9a49b,
  anchorBody: 0xcfccc5, anchorBodyOff: 0xc9c6c0, tagBody: 0xf0d9a8,
};
const COL = { ...COL_DARK };

function setPalette(theme) {
  Object.assign(COL, theme === 'light' ? COL_LIGHT : COL_DARK);
}

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
  // Orbit and pan happen inside OrbitControls, so they are marked here too —
  // otherwise dragging the view would still be overridden by an auto re-fit.
  controls.addEventListener('start', markCameraMoved);
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
  // Publish the header height so the sticky tab row sits exactly below it,
  // even if the title wraps on a narrow panel.
  const hdr = document.querySelector('#panel header');
  if (hdr) document.documentElement.style.setProperty('--header-h', hdr.offsetHeight + 'px');

  // Re-fit ONLY when the camera is still on the default framing. Calling
  // setView() unconditionally here made the view jump: choosing an object
  // rewrites the inspector, that changes the panel size, the ResizeObserver
  // fires, and the camera snapped back to the default pose — the "unselect
  // makes the viewport offset itself" bug. A user who has moved the camera
  // keeps their view; only the projection is updated.
  if (S.scene && !S.drag && homeView && !S.userMovedCamera) setView(S.viewKind || 'iso');
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

// Labels are three.js sprites, so they shrink as the camera pulls back and
// become unreadable at exactly the zoom where the whole room matters. Rescale
// every frame so a label keeps a constant size on screen, and hide it when the
// node is too small to matter.
const LABEL_PX = 30;          // on-screen height of a label at reference size
const LABEL_REF = 0.20;       // `size` of the anchor label = the reference
function updateLabelScale() {
  const h = renderer.domElement.clientHeight || 1;
  for (const g of [anchorGroup, tagGroup, roomGroup]) {
    if (!g) continue;
    for (const node of g.children) {
      for (const c of node.children) {
        if (!c.isSprite || !c.userData.labelSize) continue;
        const dist = camera.position.distanceTo(c.getWorldPosition(_labelTmp));
        // World units covered by one screen pixel at this depth.
        const worldPerPx = 2 * Math.tan(THREE.MathUtils.degToRad(camera.fov) / 2) * dist / h;
        // `size` scales the label, so ruler numbers stay smaller than anchors.
        const weight = (c.userData.labelSize || LABEL_REF) / LABEL_REF;
        const target = LABEL_PX * weight * worldPerPx;
        c.scale.set(target * (c.userData.aspect || 1), target, 1);
        c.visible = dist < 26;                        // drop the far ones
      }
    }
  }
}
const _labelTmp = new THREE.Vector3();

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
  animateSelRig();
  updateFog();
  updateLabelScale();
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
  markCameraMoved();
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
  markCameraMoved();

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
    new THREE.MeshStandardMaterial({ color: COL.wall, roughness: 0.95, metalness: 0.05 }));
  floor.position.set(W / 2, -0.03, D / 2);
  floor.receiveShadow = true;
  floor.userData.pick = 'floor';
  roomGroup.add(floor);

  // 1 m grid, aligned with the world origin (0,0) so it doubles as a ruler
  // Grid colours come from the palette; the fixed blues did not follow the
  // theme and read as a stray element on the paper background.
  const grid = new THREE.GridHelper(Math.max(W, D) * 1.4, Math.round(Math.max(W, D) * 1.4),
                                    COL.gridMajor, COL.grid);
  grid.position.set(W / 2, 0.005, D / 2);
  roomGroup.add(grid);

  // ---- coordinate system: X axis (red), Y axis (green), origin marker ----
  const axes = new THREE.Group();
  const axisLen = 0.45;        // a corner marker, not a line across the room

  const mkAxis = (from, to, color) => {
    const geo = new THREE.BufferGeometry().setFromPoints([from, to]);
    const line = new THREE.Line(geo, new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.95 }));
    line.userData.pick = 'axis';
    return line;
  };
  const X = new THREE.Vector3(1, 0, 0), Y = new THREE.Vector3(0, 0, 1);
  axes.add(mkAxis(new THREE.Vector3(0, 0.02, 0), X.clone().multiplyScalar(axisLen), COL.axisX));
  axes.add(mkAxis(new THREE.Vector3(0, 0.02, 0), Y.clone().multiplyScalar(axisLen), COL.axisY));

  // arrow heads
  const head = (dir, color) => {
    const cone = new THREE.Mesh(new THREE.ConeGeometry(0.035, 0.11, 12),
                                new THREE.MeshBasicMaterial({ color }));
    cone.position.copy(dir.clone().multiplyScalar(axisLen));
    cone.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir);
    cone.userData.pick = 'axis';
    return cone;
  };
  axes.add(head(X, COL.axisX));
  axes.add(head(Y, COL.axisY));

  // origin dot
  const origin = new THREE.Mesh(new THREE.SphereGeometry(0.04, 14, 12),
                                new THREE.MeshBasicMaterial({ color: COL.originText }));
  origin.position.set(0, 0.02, 0);
  origin.userData.pick = 'axis';
  axes.add(origin);

  axes.add(label('X →', new THREE.Vector3(axisLen * 1.35, 0.10, 0.10), COL.axisX, 0.11));
  axes.add(label('Y →', new THREE.Vector3(0.10, 0.10, axisLen * 1.35), COL.axisY, 0.11));
  axes.add(label('0,0', new THREE.Vector3(-0.14, 0.10, -0.34), COL.originText, 0.10));
  roomGroup.add(axes);

  // ---- rulers along two edges -------------------------------------------
  const centre = new THREE.Vector3(W / 2, 0, D / 2);
  // Numbers read along their own axis, so the Z rulers get rotated text.
  roomGroup.add(makeRuler(new THREE.Vector3(0, 0.02, 0), new THREE.Vector3(W, 0, 0), centre, 0));
  roomGroup.add(makeRuler(new THREE.Vector3(0, 0.02, 0), new THREE.Vector3(0, 0, D), centre, -Math.PI / 2));
  roomGroup.add(makeRuler(new THREE.Vector3(0, 0.02, D), new THREE.Vector3(W, 0, D), centre, 0));
  roomGroup.add(makeRuler(new THREE.Vector3(W, 0.02, 0), new THREE.Vector3(W, 0, D), centre, -Math.PI / 2));

  // ---- walls (translucent so the room stays readable from any angle) -----
  const wallMat = new THREE.MeshStandardMaterial({
    color: COL.wallGlass, transparent: true, opacity: 0.15,
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
  // Outside the footprint, clear of the rulers' own numbers.
  label(`width ${W.toFixed(2)} m`, new THREE.Vector3(W / 2, 0.12, -1.05), COL.rulerText, 0.145, true);
  label(`depth ${D.toFixed(2)} m`, new THREE.Vector3(-1.05, 0.12, D / 2), COL.rulerText, 0.145, true);

  controls.target.set(W / 2, 0.8, D / 2);
  updateHud();
}

function label(text, pos, color = 0xffffff, size = 0.16, plate = false) {
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
    const font = 'bold 40px "JetBrains Mono", ui-monospace, monospace';
    g.font = font;
    const lines = String(t).split('\n');
    const pad = plate ? 20 : 12;
    const w = Math.ceil(Math.max(...lines.map((l) => g.measureText(l).width))) + pad * 2;
    const lh = 48;
    const h = lh * lines.length + (plate ? 14 : 0);
    if (canvas.width !== w) canvas.width = w;
    if (canvas.height !== h) canvas.height = h;
    g.clearRect(0, 0, w, h);
    g.font = font;
    g.textBaseline = 'middle';
    // A plate behind the glyphs: the labels sit over a grid, a floor and walls,
    // and an outlined hairline was still hard to read at sprite scale. A solid
    // chip is also what the rest of this interface uses for a label.
    if (plate) {
      const bg = `#${(COL.plateBg ?? 0x111114).toString(16).padStart(6, '0')}`;
      const br = `#${(COL.plateLine ?? 0x2a2a30).toString(16).padStart(6, '0')}`;
      g.fillStyle = bg;
      g.fillRect(0, 0, w, h);
      g.strokeStyle = br;
      g.lineWidth = 3;
      g.strokeRect(1.5, 1.5, w - 3, h - 3);
      g.fillStyle = br;
      g.fillRect(0, 0, 5, h);                 // accent edge, like the panel rows
      g.fillStyle = bg;
    }
    // Outline in the background colour first: the labels sit over a grid, a
    // floor plane and walls, and a hairline glyph on a same-tone surface was
    // unreadable — worst on the paper-white theme.
    g.lineWidth = 9;
    g.lineJoin = 'round';
    g.strokeStyle = `#${(COL.labelOutline ?? 0x000000).toString(16).padStart(6, '0')}`;
    g.fillStyle = `#${spr.userData.labelColor.toString(16).padStart(6, '0')}`;
    lines.forEach((l, i) => {
      const ty = (plate ? 7 : 0) + lh * i + lh / 2;
      if (!plate) g.strokeText(l, pad, ty);   // plate already separates the text
      g.fillText(l, pad, ty);
    });

    if (!spr.material.map) {
      spr.material.map = new THREE.CanvasTexture(canvas);
      spr.material.map.minFilter = THREE.LinearFilter;
    } else {
      spr.material.map.needsUpdate = true;
    }
    const s = spr.userData.labelSize;
    spr.userData.aspect = (w / 64 * s) / (h / 64 * s);   // w/h, for uniform rescale
    spr.scale.set(w / 64 * s, h / 64 * s, 1);
  };
  spr.userData.setText(text);
  return spr;
}

/* --------------------------------------------------------- selection rig ---
   A selection marker that reads at a glance: four corner brackets orbiting the
   object, a pulsing ring, and a ping that expands outward. Built once and
   re-parented on each selection, so switching objects costs nothing and the
   animation never restarts. Works for anchors, obstacles and tags alike. */
let selRig = null;

function buildSelRig() {
  const rig = new THREE.Group();
  rig.name = 'selRig';

  // four L brackets, as one LineSegments in a unit square
  const L = 0.30;
  const pts = [];
  for (const [sx, sz] of [[-1, -1], [1, -1], [1, 1], [-1, 1]]) {
    pts.push(new THREE.Vector3(sx, 0, sz), new THREE.Vector3(sx - sx * L, 0, sz));
    pts.push(new THREE.Vector3(sx, 0, sz), new THREE.Vector3(sx, 0, sz - sz * L));
  }
  const brackets = new THREE.LineSegments(
    new THREE.BufferGeometry().setFromPoints(pts),
    new THREE.LineBasicMaterial({ color: COL.sel, transparent: true, opacity: 0.95 }));
  brackets.name = 'brackets';
  rig.add(brackets);

  const ring = new THREE.Mesh(
    new THREE.RingGeometry(0.98, 1.0, 64),
    new THREE.MeshBasicMaterial({ color: COL.sel, transparent: true, opacity: 0.7,
                                  side: THREE.DoubleSide, depthWrite: false }));
  ring.rotation.x = -Math.PI / 2;
  ring.name = 'ring';
  rig.add(ring);

  const ping = new THREE.Mesh(
    new THREE.RingGeometry(0.98, 1.0, 64),
    new THREE.MeshBasicMaterial({ color: COL.sel, transparent: true, opacity: 0.0,
                                  side: THREE.DoubleSide, depthWrite: false }));
  ping.rotation.x = -Math.PI / 2;
  ping.name = 'ping';
  rig.add(ping);

  return rig;
}

// Attach the rig to whatever is selected, sized to the object.
function updateSelRig() {
  const sel = S.selected;
  if (!selRig) selRig = buildSelRig();
  if (!sel) { selRig.visible = false; return; }

  let parent = null, radius = 0.5, y = 0;
  if (sel.type === 'anchor') {
    parent = anchorObjs[sel.id];
    radius = 0.55;
  } else if (sel.type === 'obstacle') {
    parent = obstacleObjs[sel.id];
    const o = S.scene.obstacles.find((x) => x.id === sel.id);
    radius = o ? Math.max(o.sx, o.sy) * 0.78 : 0.5;
    y = o ? o.sz / 2 : 0;
  } else if (sel.type === 'tag') {
    parent = tagMeshes[sel.id];
    radius = 0.42;
  }

  if (!parent) { selRig.visible = false; return; }
  if (selRig.parent !== parent) parent.add(selRig);
  selRig.visible = true;
  selRig.scale.set(radius, radius, radius);
  selRig.position.set(0, y, 0);
  selRig.traverse((c) => { if (c.material && c.material.color) c.material.color.setHex(COL.sel); });
}

// Called every frame: spin the brackets, pulse the ring, expand the ping.
let selT = 0;
function animateSelRig() {
  if (!selRig || !selRig.visible) return;
  selT += frameDt;
  const b = selRig.getObjectByName('brackets');
  const ring = selRig.getObjectByName('ring');
  const ping = selRig.getObjectByName('ping');
  if (b) {
    b.rotation.y = selT * 0.7;                       // slow orbit
    b.material.opacity = 0.75 + 0.25 * Math.sin(selT * 3.1);
  }
  if (ring) {
    const k = 1 + 0.05 * Math.sin(selT * 2.6);
    ring.scale.set(k, k, 1);
    ring.material.opacity = 0.55 + 0.35 * Math.sin(selT * 2.6);
  }
  if (ping) {
    const t = (selT % 1.6) / 1.6;                    // 0..1, repeating
    const k = 1 + t * 0.55;
    ping.scale.set(k, k, 1);
    ping.material.opacity = 0.45 * (1 - t) * (1 - t);
  }
}

/* ------------------------------------------------------------- rulers */
/* --- anchors  + measuring rulers ----------------------------------------- */
// A measuring ruler along one room edge: a baseline with 0.5 m ticks and a
// numeric label every metre, so distances in the 3D view can be read off
// directly instead of guessed.
// `outward` is the room centre; the tick side is chosen to point AWAY from it.
// The old fixed normal put the X ruler's numbers inside the room (the Y one
// happened to land outside), so half the scale sat over the floor it measures.

/* A text label painted flat on the floor. Unlike the billboard sprites (which
   always face the camera and are rescaled per frame), this is a real plane in
   the scene: it lies in the XZ plane, keeps a fixed world size, and therefore
   reads as part of the floor rather than as an overlay. */
function floorLabel(text, pos, color, size = 0.17, rotY = 0) {
  const canvas = document.createElement('canvas');
  const g = canvas.getContext('2d');
  const font = 'bold 40px "JetBrains Mono", ui-monospace, monospace';
  g.font = font;
  const pad = 10;
  const w = Math.ceil(g.measureText(String(text)).width) + pad * 2;
  const h = 48;
  canvas.width = w; canvas.height = h;
  const g2 = canvas.getContext('2d');
  g2.font = font;
  g2.textBaseline = 'middle';
  g2.fillStyle = `#${color.toString(16).padStart(6, '0')}`;
  g2.fillText(String(text), pad, h / 2);

  const tex = new THREE.CanvasTexture(canvas);
  tex.minFilter = THREE.LinearFilter;
  const mesh = new THREE.Mesh(
    new THREE.PlaneGeometry((w / 64) * size, (h / 64) * size),
    new THREE.MeshBasicMaterial({ map: tex, transparent: true, depthWrite: false,
                                  side: THREE.DoubleSide }));
  mesh.rotation.x = -Math.PI / 2;      // lie flat, face up
  mesh.rotation.z = rotY;
  mesh.position.copy(pos);
  mesh.userData.pick = 'label';
  return mesh;
}

function makeRuler(from, to, outward, rotY = 0) {
  const g = new THREE.Group();
  const len = from.distanceTo(to);
  const dir = new THREE.Vector3().subVectors(to, from).normalize();
  const side = new THREE.Vector3(-dir.z, 0, dir.x);
  if (outward) {
    const mid = from.clone().addScaledVector(dir, len / 2);
    const toCentre = outward.clone().sub(mid);
    if (side.dot(toCentre) > 0) side.negate();           // face away from the room
  }
  // Lift the whole ruler clear of the floor edge. Sitting exactly on the edge,
  // the near side's numbers visually crossed the floor they measure.
  const OFFSET = 0.30;
  from = from.clone().addScaledVector(side, OFFSET);
  to = to.clone().addScaledVector(side, OFFSET);

  g.add(new THREE.Line(
    new THREE.BufferGeometry().setFromPoints([from, to]),
    new THREE.LineBasicMaterial({ color: COL.ruler, transparent: true, opacity: 0.9 })));

  const tickMat = new THREE.LineBasicMaterial({ color: COL.ruler, transparent: true, opacity: 0.75 });
  const majorMat = new THREE.LineBasicMaterial({ color: COL.rulerMajor });

  const ticks = Math.round(len * 2);                      // every 0.5 m
  for (let i = 0; i <= ticks; i++) {
    const t = i / 2;                                      // metres
    const p = from.clone().addScaledVector(dir, t);
    const major = Math.abs(t - Math.round(t)) < 1e-6;     // whole metre
    const h = major ? 0.22 : 0.11;
    const a = p.clone().addScaledVector(side, 0.02);
    const b = p.clone().addScaledVector(side, h);
    g.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints([a, b]),
                         major ? majorMat : tickMat));
    if (major && i > 0) {
      // Slightly further out than the tick so the number never sits on the line.
      const lp = p.clone().addScaledVector(side, h + 0.16);
      lp.y = 0.012;                      // just above the floor, no z-fighting
      g.add(floorLabel(`${t} m`, lp, COL.rulerText, 0.19, rotY));
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

    // The group is already translated to y = a.z, so every child is positioned
    // RELATIVE to that. Adding a.z again here put the whole marker at twice the
    // anchor height — the post, plate and label floated above the room and the
    // label sat off-screen at y = 5.1 m in a 2.7 m room.
    const bodyMat = new THREE.MeshStandardMaterial({
      color: online ? COL.anchor : COL.anchorOff,
      metalness: 0.35, roughness: 0.5,
    });
    const post = new THREE.Mesh(new THREE.BoxGeometry(0.11, a.z, 0.11), bodyMat);
    post.position.y = -a.z / 2;              // spans -a.z .. 0, i.e. floor to head
    post.castShadow = true;
    post.userData.pick = 'anchor';
    post.userData.id = a.id;
    g.add(post);

    const plate = new THREE.Mesh(new THREE.BoxGeometry(0.34, 0.06, 0.34), bodyMat);
    plate.position.y = 0;                    // at the anchor's own height
    plate.rotation.y = Math.PI / 4;
    plate.castShadow = true;
    plate.userData.pick = 'anchor';
    plate.userData.id = a.id;
    g.add(plate);

    // A short fin on top so the plate reads as a direction, not a floating tile.
    const fin = new THREE.Mesh(new THREE.BoxGeometry(0.05, 0.22, 0.05), bodyMat);
    fin.position.y = 0.14;
    fin.userData.pick = 'anchor';
    fin.userData.id = a.id;
    g.add(fin);

    g.add(anchorLabel(a, live));
    anchorObjs[a.id] = g;
    anchorGroup.add(g);
  }
  updateSelRig();          // the groups were rebuilt; re-attach the marker
}

// Anchor label: name on the first line, live values underneath
// (position always, plus the current range/RSSI when the device reports them).
function anchorLabel(a, live) {
  const online = live ? live.online : false;
  const spr = label(anchorLabelText(a, live), new THREE.Vector3(0, 0.55, 0),
                    online ? COL.labelOn : COL.labelOff, 0.20, true);
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

    spr.userData.setText(anchorLabelText(a, live), online ? COL.labelOn : COL.labelOff);
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
      gz.add(axisArrow(new THREE.Vector3(1, 0, 0), COL.axisX, 'x'));
      gz.add(axisArrow(new THREE.Vector3(0, 0, 1), COL.axisY, 'y'));
      gz.add(axisArrow(new THREE.Vector3(0, 1, 0), COL.axisZ, 'z'));
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
  updateSelRig();          // groups rebuilt
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
const tagFlashers = {};   // tag id -> value flasher for the coordinate readout
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
      // An octagonal puck rather than a sphere: same flat language as the
      // anchor plate, and it still reads as a distinct shape from the anchors.
      const body = new THREE.Mesh(
        new THREE.CylinderGeometry(0.135, 0.135, 0.07, 8),
        new THREE.MeshStandardMaterial({ color: COL.tag, emissive: COL.tagBody,
                                         emissiveIntensity: 0.9, roughness: 0.35 }));
      body.castShadow = true;
      body.userData.pick = 'tag';
      body.userData.id = t.id;
      mesh.add(body);

      // crosshair bars on top, so the tag is identifiable from above
      const barMat = new THREE.MeshBasicMaterial({ color: COL.tag });
      for (const rot of [0, Math.PI / 2]) {
        const bar = new THREE.Mesh(new THREE.BoxGeometry(0.30, 0.012, 0.022), barMat);
        bar.position.y = 0.035;
        bar.rotation.y = rot;
        bar.userData.pick = 'tag';
        bar.userData.id = t.id;
        mesh.add(bar);
      }
      // uncertainty ring, scaled from the EKF sigma
      const ring = new THREE.Mesh(
        new THREE.RingGeometry(0.9, 0.93, 64),
        new THREE.MeshBasicMaterial({ color: COL.tag, transparent: true, opacity: 0.35,
                                      side: THREE.DoubleSide }));
      ring.rotation.x = -Math.PI / 2;
      ring.name = 'sigma';
      mesh.add(ring);
      mesh.add(label(t.id, new THREE.Vector3(0, 0.46, 0), COL.labelTag, 0.19, true));
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
  updateSelRig();          // meshes rebuilt
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
        if (S.mode === 'addAnchor') { addAnchor(x, y); Audio2.place(); }
        else { addObstacle(x, y); Audio2.place(); }
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
  updateSelRig();
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
    { t: 'anchor', n: 'Anchor' },
    { t: 'obstacle', n: 'Obstacle' },
  ];
  // Swatches read the live 3D palette, so the legend cannot drift out of sync
  // with the scene when the theme changes.
  host.innerHTML = items.map((i) =>
    `<button class="pal" data-add="${i.t}"><span style="background:#${
      COL[i.t].toString(16).padStart(6, '0')}"></span>${i.n}</button>`).join('');
  host.querySelectorAll('.pal').forEach((b) => {
    b.onclick = () => {
      S.mode = b.dataset.add === 'anchor' ? 'addAnchor' : 'addObstacle';
      toast(`Click on the floor to place the ${b.dataset.add}.`);
    };
  });
}

/* -------------------------------------------------------------------- io */
async function loadScene() {
  try {
  const r = await fetch('/api/v1/scene');
  const d = await r.json();
  S.scene = d.scene;
  S.nlos = d.nlos_enabled !== false;
  $('nlos').checked = S.nlos;
  syncRoomInputs();
  rebuildAll();
  Audio2.success();
  } catch (e) { Audio2.error(); }
}

async function saveScene() {
  const r = await fetch('/api/v1/scene', {
    method: 'PUT', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ scene: S.scene, nlos_enabled: S.nlos }) });
  const d = await r.json();
  S.dirty = false;
  toast(d.ok ? `Saved — pushed to ${d.pushed?.length || 0} device(s).` : 'Save failed.');
  setSaved();
  if (d.ok) Audio2.success(); else Audio2.error();
}

async function poll() {
  try {
    const r = await fetch('/api/v1/state');
    S.state = await r.json();
    if (!S.drag) { buildTags(); buildLinks(); updateAnchorLabels(); ensurePulses(); }
    updateHud();
    // Rebuilding the list on every poll (5 Hz) would restart its entry
    // animation constantly and thrash the DOM. Only the numbers are refreshed;
    // the structure is rebuilt when the set of tags actually changes.
    const ids = (S.state.tags || []).map((t) => t.id + (t.online ? '+' : '-')).join(',');
    if (ids !== poll._tagIds) { poll._tagIds = ids; renderTagList(); }
    else updateTagValues();
    cueTransitions();
    if (S.selected?.type === 'tag') showSelection();
    updateFog();
  } catch (e) { /* server restarting */ }
}

function updateHud() {
  const live = S.state.tags?.filter((t) => t.online).length || 0;
  const anc = S.state.anchors?.filter((a) => a.online).length || 0;
  const tag = S.state.tags?.[0];
  const coordTxt = tag && tag.online
    ? `tag (${tag.x.toFixed(2)}, ${tag.y.toFixed(2)}) m` : null;
  const coord = coordTxt
    ? `<span class="sep"></span><span class="mono" data-coord>${coordTxt}</span>` : '';
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

  // Mark the coordinate when it changes: the HUD is rebuilt every poll, so a
  // changed value needs an explicit cue or the eye cannot follow it.
  const el = $('hud').querySelector('[data-coord]');
  if (el && coordTxt) {
    if (updateHud._last !== null && coordTxt !== updateHud._last) {
      el.classList.add('flash');
    }
    updateHud._last = coordTxt;
  } else if (!coordTxt) {
    updateHud._last = null;
  }
}
updateHud._last = null;

function syncRoomInputs() {
  $('r-w').value = S.scene.room.width.toFixed(2);
  $('r-d').value = S.scene.room.depth.toFixed(2);
  $('r-h').value = S.scene.room.height.toFixed(2);
}

/* ------------------------------------------------------------------ motion
   Small helpers so the animation rules stay declarative in CSS. */

// Stagger children of a freshly revealed section: the eye reads the block
// top-to-bottom instead of every row appearing on the same frame.
function stagger(host, sel = ':scope > *') {
  const kids = host ? [...host.querySelectorAll(sel)] : [];
  kids.forEach((k, i) => k.style.setProperty('--i', i));
  return kids;
}

// Refresh just the coordinate readouts of the existing rows.
function updateTagValues() {
  for (const t of S.state.tags || []) {
    const f = tagFlashers[t.id];
    if (f) f(t);
  }
}

// Flash a value that just changed. Returns a function to feed the next value.
function flasher(el, fmt = (v) => v) {
  let last = null;
  return (v) => {
    if (!el) return;
    const txt = fmt(v);
    if (last !== null && txt !== last && el.textContent !== txt) {
      el.classList.remove('flash');
      void el.offsetWidth;              // restart the animation
      el.classList.add('flash');
    }
    last = txt;
    el.textContent = txt;
  };
}

// Mark a button busy while it waits on the network, so a slow reply looks
// like work in progress rather than a dead control.
function busy(btn, on) {
  if (!btn) return;
  btn.classList.toggle('busy', !!on);
  btn.setAttribute('aria-busy', on ? 'true' : 'false');
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
  let master = null;

  const ensure = () => {
    if (!ctx) {
      const AC = window.AudioContext || window.webkitAudioContext;
      if (!AC) return null;
      ctx = new AC();
      master = ctx.createGain();
      master.gain.value = 0.5;
      master.connect(ctx.destination);
    }
    if (ctx.state === 'suspended') ctx.resume();
    return ctx;
  };

  // One shaped tone. `slideTo` sweeps the pitch, which is what makes a click
  // sound like a mechanical detent rather than a beep.
  const tone = (freq, dur, type = 'sine', gain = 0.05, slideTo = null, delay = 0) => {
    if (!enabled) return;
    const c = ensure();
    if (!c) return;
    const t0 = c.currentTime + delay;
    const o = c.createOscillator();
    const g = c.createGain();
    o.type = type;
    o.frequency.setValueAtTime(freq, t0);
    if (slideTo) o.frequency.exponentialRampToValueAtTime(slideTo, t0 + dur);
    g.gain.setValueAtTime(0.0001, t0);
    g.gain.exponentialRampToValueAtTime(gain, t0 + 0.010);
    g.gain.exponentialRampToValueAtTime(0.0001, t0 + dur);
    o.connect(g).connect(master);
    o.start(t0);
    o.stop(t0 + dur + 0.02);
  };

  // Short filtered noise: used for clicks and sweeps, which a plain oscillator
  // cannot make without sounding like a tone.
  const noise = (dur, gain = 0.05, freq = 1800, q = 1.2, delay = 0) => {
    if (!enabled) return;
    const c = ensure();
    if (!c) return;
    const n = Math.max(1, Math.floor(c.sampleRate * dur));
    const buf = c.createBuffer(1, n, c.sampleRate);
    const d = buf.getChannelData(0);
    for (let i = 0; i < n; i++) d[i] = (Math.random() * 2 - 1) * (1 - i / n);
    const src = c.createBufferSource();
    src.buffer = buf;
    const bp = c.createBiquadFilter();
    bp.type = 'bandpass'; bp.frequency.value = freq; bp.Q.value = q;
    const g = c.createGain();
    g.gain.value = gain;
    src.connect(bp).connect(g).connect(master);
    src.start(c.currentTime + delay);
  };

  return {
    get enabled() { return enabled; },
    set enabled(v) { enabled = v; if (v) ensure(); },
    get volume() { return master ? master.gain.value * 2 : 0.5; },
    set volume(v) { if (master) master.gain.value = Math.max(0, Math.min(v, 1)) * 2; },

    /* --- interaction cues ------------------------------------------------ */
    hover()   { tone(1560, 0.030, 'sine', 0.012); },                      // tick under the cursor
    click()   { noise(0.035, 0.05, 2400, 0.9);
                tone(880, 0.045, 'square', 0.022, 620); },                // mechanical press
    toggleOn(){ tone(520, 0.05, 'triangle', 0.03, 880); },
    toggleOff(){ tone(880, 0.05, 'triangle', 0.03, 520); },
    tab()     { noise(0.05, 0.035, 1400, 1.1);
                tone(300, 0.07, 'sine', 0.022, 480); },                   // panel slide
    error()   { tone(180, 0.30, 'sawtooth', 0.045, 110); },
    success() { tone(660, 0.09, 'sine', 0.035);
                tone(990, 0.16, 'sine', 0.030, null, 0.08); },

    /* --- system cues ----------------------------------------------------- */
    online()  { tone(660, 0.10, 'triangle', 0.05);
                tone(990, 0.12, 'triangle', 0.04, null, 0.09); },
    offline() { tone(220, 0.22, 'sawtooth', 0.035, 140); },
    update()  { tone(440, 0.09, 'square', 0.035);
                tone(880, 0.14, 'square', 0.03, null, 0.10); },
    nlos()    { tone(300, 0.18, 'sine', 0.03, 200); },
    place()   { noise(0.05, 0.045, 900, 0.8);
                tone(420, 0.10, 'triangle', 0.03, 700); },                // object dropped
    remove()  { noise(0.06, 0.05, 700, 0.8);
                tone(520, 0.12, 'sawtooth', 0.03, 180); },
    boot()    { tone(523, 0.10, 'sine', 0.04);
                tone(784, 0.14, 'sine', 0.035, null, 0.11);
                tone(1046, 0.20, 'sine', 0.03, null, 0.23); },
  };
})();

/* Wire a sound to every control automatically, so a new button cannot be
   added without feedback. Delegated at the document level: the panel rebuilds
   its rows on every poll, and per-element listeners would be lost. */
function bindSound() {
  let lastHover = 0;
  document.addEventListener('pointerover', (e) => {
    const el = e.target.closest('button, .pal, .dev, .ep, input, select');
    if (!el) return;
    const now = performance.now();
    if (now - lastHover < 70) return;      // do not machine-gun on fast sweeps
    lastHover = now;
    if (el.matches('input[type=checkbox], input[type=range], select')) Audio2.hover();
    else if (el.tagName === 'BUTTON' || el.classList.contains('pal')) Audio2.hover();
  }, { passive: true });

  document.addEventListener('click', (e) => {
    const btn = e.target.closest('button');
    if (!btn || btn.disabled) return;
    // These own their cue; playing one here as well would double it.
    if (btn.id === 'sound-toggle') return;
    if (btn.id === 'save' || btn.id === 'reload') { Audio2.click(); return; }
    if (btn.dataset.tab) { Audio2.tab(); return; }
    if (btn.classList.contains('pal')) { Audio2.click(); return; }
    if (btn.classList.contains('danger') || btn.id === 'i-del' || btn.id === 'o-del') { Audio2.remove(); return; }
    if (btn.classList.contains('on')) { Audio2.toggleOff(); return; }
    if (btn.id === 'render-toggle' || btn.id === 'view-iso' || btn.id === 'view-top') {
      Audio2.toggleOn(); return;
    }
    Audio2.click();
  }, true);
}

/* ==========================================================================
   Tabs
   ========================================================================== */
function setTab(name) {
  document.querySelectorAll('#tabs button').forEach((b) =>
    b.classList.toggle('active', b.dataset.tab === name));
  document.querySelectorAll('#panel section[data-panel]').forEach((s) => {
    const on = s.dataset.panel === name;
    s.hidden = !on;
    // Reveal with a staggered rise. The class is re-added each time so the
    // animation replays on every switch rather than only the first.
    if (on) {
      s.classList.remove('rise');
      void s.offsetWidth;
      s.classList.add('rise');
      stagger(s);
    }
  });
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

  host.innerHTML = devs.map((d, i) => {
    const o = keyOf(d.id);
    const rssi = (d.rssi === null || d.rssi === undefined) ? '—' : `${d.rssi} dBm`;
    return `<div class="dev" style="--i:${i}">
      <span class="dot ${d.online ? 'on' : 'off'}"></span>
      <div>
        <div class="name">${esc(d.id)} <span class="pill">${esc(d.role || '')}</span></div>
        <div class="ip">${esc(d.ip || 'no ip yet')}</div>
        <div class="meta">${rssi} · fw ${esc(d.fw || '?')}</div>
      </div>
      <div class="spacer"></div>
      <button data-ota="${esc(d.id)}" ${o.pending ? 'disabled' : ''} title="${o.pending ? 'Already waiting for ' + esc(o.pending) : 'Queue a firmware update for this device'}">${o.pending ? '⏳ Queued' : '⬆ Update'}</button>
      ${o.pending ? `<button data-cancel="${esc(d.id)}" title="Cancel the queued update">✕</button>` : ''}
      <button data-key="${esc(d.id)}" title="Show / rotate the OTA key">🔑</button>
    </div>`;
  }).join('');

  host.querySelectorAll('[data-ota]').forEach((b) => {
    b.onclick = () => pushOta(b.dataset.ota);
  });
  host.querySelectorAll('[data-cancel]').forEach((b) => {
    b.onclick = () => cancelOta(b.dataset.cancel);
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

  // Fool protection: one request in flight at a time, and the button reports
  // the result immediately. Repeated clicking cannot build a queue (the server
  // is idempotent per device+image too).
  if (otaRequestInFlight) return;
  otaRequestInFlight = true;

  const who = deviceId === 'all' ? 'all devices' : deviceId;
  if (!confirm(`Queue ${name} for ${who}?\n\nThe device installs it on its next check (within ~1 min) and reboots.\nNothing is downloaded until you confirm.`)) {
    otaRequestInFlight = false;
    return;
  }

  setOtaBusy(true, `Queuing ${name} for ${who}…`);
  try {
    const r = await (await fetch('/api/v1/ota/request', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ device_id: deviceId, firmware: name }) })).json();

    if (!r.ok) {
      $('ota-result').innerHTML = `<span class="bad">Failed: ${esc(r.error || 'unknown')}</span>`;
      toast('Queue failed.');
    } else {
      const q = (r.queued || []), a = (r.already || []), sk = (r.skipped || []);
      const parts = [];
      if (q.length) parts.push(`<span class="good">queued: ${q.join(', ')}</span>`);
      if (a.length) parts.push(`already waiting: ${a.join(', ')}`);
      if (sk.length) parts.push(`<span class="warn">not newer: ${sk.join(', ')}</span>`);
      $('ota-result').innerHTML = parts.join('<br>') ||
        'Nothing to do — the device already runs this image.';
      toast(q.length ? `Queued on ${q.length} device(s) — installs within a minute.`
                     : 'Already queued / nothing to do.');
      Audio2.update();
    }
  } catch (e) {
    $('ota-result').textContent = 'Request failed: ' + e;
    toast('Request failed.');
  } finally {
    otaRequestInFlight = false;
    setOtaBusy(false);
    renderDevices();          // reflect the new pending state right away
    loadOta();
  }
}

// Show progress on the buttons themselves so a click always has feedback.
function setOtaBusy(busy, label) {
  for (const id of ['ota-push-all']) {
    const b = $(id);
    if (!b) continue;
    b.disabled = busy;
    b.textContent = busy ? '⏳ Working…' : '⬆ Queue update for all';
  }
  document.querySelectorAll('[data-ota]').forEach((b) => { b.disabled = busy; });
  if (busy && label) $('ota-result').textContent = label;
}

let otaRequestInFlight = false;

// Cancel a queued update.
async function cancelOta(deviceId) {
  try {
    await fetch('/api/v1/ota/request', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ device_id: deviceId, cancel: true }) });
    toast(`Cancelled the queued update for ${deviceId}`);
    renderDevices();
  } catch (e) { toast('Cancel failed'); }
}

/* ==========================================================================
   Device screen (OLED) power policy — pushed to every device as a normal
   config update, so the node dims/blanks its panel while staying online.
   ========================================================================== */
async function applyScreenPolicy() {
  const mode = $('scr-mode').value;
  const timeout_s = Math.max(0, parseInt($('scr-timeout').value || '0', 10));
  const ids = (S.state.devices || []).map((d) => d.id);
  if (!ids.length) { toast('No device known yet.'); return; }

  $('scr-result').textContent = `Sending to ${ids.length} device(s)…`;
  let ok = 0;
  for (const id of ids) {
    const [role, num] = id.split('-');
    try {
      const r = await fetch('/api/v1/config', {
        method: 'PUT', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ role, id: parseInt(num, 10),
                               display: { mode, timeout_s } }),
      });
      if (r.ok) ok++;
    } catch (e) { /* keep going */ }
  }
  $('scr-result').textContent =
    `Applied to ${ok}/${ids.length} device(s): screen ${mode}, timeout ${timeout_s}s.`;
  toast(`Screen policy sent (${mode}, ${timeout_s}s)`);
  Audio2.update();
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

  host.innerHTML = apiInfo.endpoints.map((e, i) => `
    <div class="ep" style="--i:${i}">
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
  $('theme-toggle').textContent = theme === 'light' ? 'DARK' : 'LIGHT';
  if (scene3) {
    const light = theme === 'light';
    setPalette(theme);
    scene3.background = new THREE.Color(light ? 0xeceae6 : 0x0a0a0c);
    if (scene3.fog) scene3.fog.color = scene3.background;
    // Rebuild every group that carries a themed colour. Before this only the
    // room was rebuilt, so anchors, obstacles, tags and links kept the old
    // palette after a theme switch.
    if (roomGroup) buildRoom();
    if (anchorGroup) buildAnchors();
    if (obstacleGroup) buildObstacles();
    if (tagGroup) buildTags();
    if (linkGroup) buildLinks();
    updateAnchorLabels();
    ensurePulses();
    buildPalette();          // legend swatches follow the 3D palette
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
  host.innerHTML = tags.map((t, i) => `
    <div class="dev" style="--i:${i}">
      <span class="dot ${t.online ? 'on' : 'off'}"></span>
      <div>
        <div class="name">${esc(t.id)}</div>
        <div class="meta mono"><span data-coord>(${t.x.toFixed(2)}, ${t.y.toFixed(2)}) m</span> · σ ${(t.sigma || 0).toFixed(2)}</div>
        <div class="meta">${esc(t.ip || 'no ip yet')} · conf ${((t.confidence || 0) * 100).toFixed(0)}%</div>
      </div>
    </div>`).join('');
  // Coordinates are the value that actually moves; flash them on change.
  tags.forEach((t) => {
    const el = host.querySelector(`.dev:nth-child(${tags.indexOf(t) + 1}) [data-coord]`);
    const f = flasher(el, (v) => `(${v.x.toFixed(2)}, ${v.y.toFixed(2)}) m`);
    f(t);
    tagFlashers[t.id] = f;
  });
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
  bindSound();          // every control gets a cue, present and future
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
  $('save').onclick = async () => { busy($('save'), true); try { await saveScene(); } finally { busy($('save'), false); } };
  $('reload').onclick = async () => { busy($('reload'), true); try { await loadScene(); } finally { busy($('reload'), false); } };
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
  $('theme-toggle').onclick = () => {
    Audio2.toggleOn();
    applyTheme(document.documentElement.dataset.theme === 'light' ? 'dark' : 'light');
  };
  $('ota-push-all').onclick = () => pushOta('all');
  $('scr-apply').onclick = applyScreenPolicy;

  addEventListener('keydown', (e) => {
    if (e.key === 'Delete' || e.key === 'Backspace') {
      if (document.activeElement.tagName !== 'INPUT') {
        if (S.selected) Audio2.remove();          // keyboard path has no click event
        removeSelected(); e.preventDefault();
      }
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
  const { width: W, depth: D, height: H } = S.scene.room;
  // Frame the room by its bounding sphere, not by a fixed multiple of its
  // size. A fixed factor zoomed far too close in a long narrow room, so part
  // of the floor fell outside the viewport. This keeps the whole room visible
  // at any aspect ratio.
  const fit = (dir, pad = 1.28) => {
    const radius = Math.hypot(W, D, H) / 2;
    const vFov = THREE.MathUtils.degToRad(camera.fov);
    const hFov = 2 * Math.atan(Math.tan(vFov / 2) * camera.aspect);
    const dist = radius / Math.sin(Math.min(vFov, hFov) / 2) * pad;
    const center = new THREE.Vector3(W / 2, H / 2, D / 2);
    camera.position.copy(center).addScaledVector(dir.clone().normalize(), dist);
    controls.target.copy(kind === 'top' ? new THREE.Vector3(W / 2, 0, D / 2) : center);
  };

  if (kind === 'top') {
    fit(new THREE.Vector3(0, 1, 0.0001), 1.12);
  } else {
    fit(new THREE.Vector3(1, 0.95, 1.15), 1.28);
  }

  zoomAccum = 0;            // drop any pending wheel momentum
  zoomFocus = null;
  controls.update();
  rememberHome();
  S.viewKind = kind;        // remembered so a resize can re-fit the framing
  S.userMovedCamera = false; // an explicit view change re-arms the auto-fit
  // The view buttons are a radio group, so the current mode has to be visible.
  $('view-iso').classList.toggle('on', kind !== 'top');
  $('view-top').classList.toggle('on', kind === 'top');
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

// Set as soon as the user orbits, pans, zooms or flies the camera. resize()
// uses it to decide whether the framing may still be recomputed: once the user
// has taken control, a panel resize must not snap the view back.
function markCameraMoved() { S.userMovedCamera = true; }

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
  markCameraMoved();
  zoomAccum += notches * ZOOM_UNIT;
}

initThree();
initUi();
loadScene().then(() => { setView('iso'); playIntro(); });
applyTheme(localStorage.getItem('uwb-theme') || 'dark');
setPalette(document.documentElement.dataset.theme);   // 3D colours before the first build
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
