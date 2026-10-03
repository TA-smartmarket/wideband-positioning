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
let pickables = [];
const tagMeshes = {};
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
  scene3.fog = new THREE.Fog(0x0b0e1a, 14, 40);

  camera = new THREE.PerspectiveCamera(50, 1, 0.05, 200);
  camera.position.set(6, 7, 8);

  controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.maxPolarAngle = Math.PI * 0.495;   // never below the floor
  controls.minDistance = 1.5;
  controls.maxDistance = 40;
  controls.target.set(2.5, 1, 2);

  // lights
  scene3.add(new THREE.HemisphereLight(0xdfe8ff, 0x1a2033, 0.85));
  const key = new THREE.DirectionalLight(0xffffff, 1.5);
  key.position.set(6, 12, 7);
  key.castShadow = true;
  key.shadow.mapSize.set(2048, 2048);
  key.shadow.camera.left = -14; key.shadow.camera.right = 14;
  key.shadow.camera.top = 14; key.shadow.camera.bottom = -14;
  scene3.add(key);

  roomGroup = new THREE.Group();
  anchorGroup = new THREE.Group();
  obstacleGroup = new THREE.Group();
  tagGroup = new THREE.Group();
  linkGroup = new THREE.Group();
  trailGroup = new THREE.Group();
  scene3.add(roomGroup, anchorGroup, obstacleGroup, tagGroup, linkGroup, trailGroup);

  raycaster = new THREE.Raycaster();
  addEventListener('resize', resize);
  resize();
  bindPointer();
  animate();
}

function resize() {
  const host = $('viewport');
  const w = host.clientWidth, h = host.clientHeight;
  renderer.setSize(w, h, false);
  camera.aspect = w / Math.max(h, 1);
  camera.updateProjectionMatrix();
}

function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene3, camera);
}

/* ------------------------------------------------------------ room shell */
function buildRoom() {
  while (roomGroup.children.length) roomGroup.remove(roomGroup.children[0]);
  const { width: W, depth: D, height: H } = S.scene.room;

  // floor
  const floor = new THREE.Mesh(
    new THREE.BoxGeometry(W, 0.06, D),
    new THREE.MeshStandardMaterial({ color: COL.room, roughness: 0.95, metalness: 0.05 }));
  floor.position.set(W / 2, -0.03, D / 2);
  floor.receiveShadow = true;
  floor.userData.pick = 'floor';
  roomGroup.add(floor);

  // grid on the floor
  const grid = new THREE.GridHelper(Math.max(W, D) * 1.2, Math.round(Math.max(W, D) * 2),
                                    0x33406b, 0x222b45);
  grid.position.set(W / 2, 0.005, D / 2);
  roomGroup.add(grid);

  // walls (translucent so the room stays readable from any angle)
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

  // dimension labels
  label(`W ${W.toFixed(2)} m`, new THREE.Vector3(W / 2, 0.12, -0.45), 0x93a4c8);
  label(`D ${D.toFixed(2)} m`, new THREE.Vector3(-0.45, 0.12, D / 2), 0x93a4c8);

  controls.target.set(W / 2, 0.8, D / 2);
  updateHud();
}

function label(text, pos, color = 0xffffff, size = 0.16) {
  const c = document.createElement('canvas');
  const ctx = c.getContext('2d');
  const font = `bold 48px system-ui, sans-serif`;
  ctx.font = font;
  c.width = Math.ceil(ctx.measureText(text).width) + 24;
  c.height = 64;
  const g = c.getContext('2d');
  g.font = font;
  g.fillStyle = `#${color.toString(16).padStart(6, '0')}`;
  g.textBaseline = 'middle';
  g.fillText(text, 12, 32);
  const tex = new THREE.CanvasTexture(c);
  tex.minFilter = THREE.LinearFilter;
  const spr = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, depthTest: false,
                                                          transparent: true }));
  spr.scale.set(c.width / 64 * size, c.height / 64 * size, 1);
  spr.position.copy(pos);
  spr.userData.pick = 'label';
  return spr;
}

/* --------------------------------------------------------------- anchors */
function buildAnchors() {
  while (anchorGroup.children.length) anchorGroup.remove(anchorGroup.children[0]);
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

    g.add(label(a.label || a.id, new THREE.Vector3(0, a.z + 0.32, 0),
                online ? 0x9ff5cf : 0x93a4c8, 0.13));
    anchorGroup.add(g);
  }
}

/* ------------------------------------------------------------- obstacles */
function buildObstacles() {
  while (obstacleGroup.children.length) obstacleGroup.remove(obstacleGroup.children[0]);
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
      // resize handles on the horizontal footprint corners
      for (const [dx, dz] of [[-1, -1], [1, -1], [1, 1], [-1, 1]]) {
        const h = new THREE.Mesh(
          new THREE.SphereGeometry(0.07, 12, 12),
          new THREE.MeshBasicMaterial({ color: COL.sel }));
        h.position.set(dx * ob.sx / 2, 0, dz * ob.sy / 2);
        h.userData.pick = 'obstacleHandle';
        h.userData.id = ob.id;
        h.userData.corner = [dx, dz];
        g.add(h);
      }
      const up = new THREE.Mesh(
        new THREE.ConeGeometry(0.08, 0.2, 12),
        new THREE.MeshBasicMaterial({ color: COL.sel }));
      up.position.set(0, ob.sz / 2 + 0.14, 0);
      up.userData.pick = 'obstacleHeight';
      up.userData.id = ob.id;
      g.add(up);
    }

    obstacleGroup.add(g);
  }
}

/* ------------------------------------------------------------ tags/links */
function buildTags() {
  const seen = new Set();
  for (const t of S.state.tags) {
    seen.add(t.id);
    let mesh = tagMeshes[t.id];
    if (!mesh) {
      mesh = new THREE.Group();
      const body = new THREE.Mesh(
        new THREE.SphereGeometry(0.16, 20, 16),
        new THREE.MeshStandardMaterial({ color: COL.tag, emissive: 0x8a6a10,
                                         emissiveIntensity: 0.9, roughness: 0.35 }));
      body.castShadow = true;
      body.userData.pick = 'tag';
      body.userData.id = t.id;
      mesh.add(body);
      // uncertainty ring, scaled from the EKF sigma
      const ring = new THREE.Mesh(
        new THREE.RingGeometry(0.9, 1.0, 48),
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
      ring.material.opacity = t.online ? 0.4 : 0.15;
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

function buildTrail() {
  while (trailGroup.children.length) trailGroup.remove(trailGroup.children[0]);
  if (!S.showTrail) return;
  for (const id of Object.keys(S.trail)) {
    const pts = S.trail[id];
    if (pts.length < 2) continue;
    const geo = new THREE.BufferGeometry().setFromPoints(
      pts.map(([x, y]) => new THREE.Vector3(x, 0.9, y)));
    trailGroup.add(new THREE.Line(geo, new THREE.LineBasicMaterial({
      color: COL.tag, transparent: true, opacity: 0.55 })));
  }
}

function buildLinks() {
  while (linkGroup.children.length) linkGroup.remove(linkGroup.children[0]);
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

/* ------------------------------------------------------- scene → rebuild */
function rebuildAll() {
  buildRoom();
  buildAnchors();
  buildObstacles();
  buildTags();
  buildLinks();
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
  for (const h of hits) {
    let o = h.object;
    while (o && !o.userData.pick) o = o.parent;
    if (o && o.userData.pick) return { obj: o, point: h.point, hit: h.object };
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

  el.addEventListener('pointerdown', (ev) => {
    if (ev.button !== 0) return;
    downAt = { x: ev.clientX, y: ev.clientY, t: performance.now() };
    const p = pick(ev);
    if (!p) return;

    const u = p.obj.userData;
    if (u.pick === 'anchor' || u.pick === 'obstacle' ||
        u.pick === 'obstacleHandle' || u.pick === 'obstacleHeight') {
      controls.enabled = false;
      const gp = groundPoint(ev);
      S.drag = {
        type: u.pick, id: u.id, corner: u.corner,
        startX: gp ? gp.x : 0, startZ: gp ? gp.z : 0,
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

    if (S.drag.type === 'anchor') {
      const a = S.scene.anchors.find((x) => x.id === S.drag.id);
      if (a) {
        a.x = clamp(S.drag.orig.x + dx, 0, S.scene.room.width);
        a.y = clamp(S.drag.orig.y + dz, 0, S.scene.room.depth);
        S.dirty = true;
        rebuildAll();
        if (S.selected) showSelection();
      }
    } else if (S.drag.type === 'obstacle') {
      const o = S.scene.obstacles.find((x) => x.id === S.drag.id);
      if (o) {
        o.x = S.drag.orig.x + dx;
        o.y = S.drag.orig.y + dz;
        S.dirty = true;
        rebuildAll();
        if (S.selected) showSelection();
      }
    } else if (S.drag.type === 'obstacleHandle') {
      const o = S.scene.obstacles.find((x) => x.id === S.drag.id);
      if (o) {
        const rot = THREE.MathUtils.degToRad(o.rot || 0);
        const lx = dx * Math.cos(-rot) - dz * Math.sin(-rot);
        const lz = dx * Math.sin(-rot) + dz * Math.cos(-rot);
        const [cx, cz] = S.drag.corner;
        o.sx = Math.max(0.2, S.drag.orig.sx + cx * lx * 2);
        o.sy = Math.max(0.2, S.drag.orig.sy + cz * lz * 2);
        S.dirty = true;
        rebuildAll();
        showSelection();
      }
    } else if (S.drag.type === 'obstacleHeight') {
      const o = S.scene.obstacles.find((x) => x.id === S.drag.id);
      if (o) {
        o.sz = Math.max(0.2, S.drag.orig.sz - (ev.movementY || 0) * 0.01);
        S.dirty = true;
        rebuildAll();
        showSelection();
      }
    }
  });

  el.addEventListener('pointerup', (ev) => {
    const quick = downAt && Math.hypot(ev.clientX - downAt.x, ev.clientY - downAt.y) < 4
                  && performance.now() - downAt.t < 400;
    if (S.drag) { S.drag = null; controls.enabled = true; return; }
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
  rebuildAll();
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
      <h4>Links</h4>${linksTable(t)}`;
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
    if (!S.drag) { buildTags(); buildLinks(); }
    updateHud();
    if (S.selected?.type === 'tag') showSelection();
  } catch (e) { /* server restarting */ }
}

function updateHud() {
  const live = S.state.tags?.filter((t) => t.online).length || 0;
  const anc = S.state.anchors?.filter((a) => a.online).length || 0;
  $('hud').innerHTML =
    `<span class="dot ${anc ? 'on' : 'off'}"></span>${anc} anchor${anc === 1 ? '' : 's'} online` +
    `<span class="sep"></span><span class="dot ${live ? 'on' : 'off'}"></span>` +
    `${live} tag${live === 1 ? '' : 's'} live` +
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
function initUi() {
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
  $('view-top').onclick = () => setView('top');
  $('view-iso').onclick = () => setView('iso');
  $('view-reset').onclick = () => { S.trail = {}; buildTrail(); };
  addEventListener('keydown', (e) => {
    if (e.key === 'Delete' || e.key === 'Backspace') {
      if (document.activeElement.tagName !== 'INPUT') { removeSelected(); e.preventDefault(); }
    }
    if (e.key === 'Escape') select(null, null);
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
  controls.update();
}

initThree();
initUi();
loadScene().then(() => setView('iso'));
poll();
setInterval(poll, 500);
