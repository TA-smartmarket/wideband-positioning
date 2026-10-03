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
  controls.minDistance = 0.6;
  controls.maxDistance = 120;                 // generous: the room can be large
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
  addEventListener('resize', resize);
  bindContextLoss();
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
  controls.update();
  updateFog();
  renderer.render(scene3, camera);
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

    g.add(label(a.label || a.id, new THREE.Vector3(0, a.z + 0.32, 0),
                online ? 0x9ff5cf : 0x93a4c8, 0.13));
    anchorObjs[a.id] = g;
    anchorGroup.add(g);
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

    obstacleObjs[ob.id] = g;
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
  // reposition the corner handles / height cone
  const handles = g.children.slice(2);
  let hi = 0;
  for (const c of handles) {
    if (c.userData.pick === 'obstacleHandle') {
      const [dx, dz] = [[-1, -1], [1, -1], [1, 1], [-1, 1]][hi++] || [0, 0];
      c.position.set(dx * o.sx / 2, 0, dz * o.sy / 2);
    } else if (c.userData.pick === 'obstacleHeight') {
      c.position.set(0, o.sz / 2 + 0.14, 0);
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
    for (const [id, v] of [['o-x', o.x], ['o-y', o.y], ['o-sx', o.sx],
                           ['o-sy', o.sy], ['o-sz', o.sz]]) {
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
        o.sx = Math.max(0.2, S.drag.orig.sx + cx * lx * 2);
        o.sy = Math.max(0.2, S.drag.orig.sy + cz * lz * 2);
        S.dirty = true;
        resizeObstacleMesh(o);
      }
    } else if (S.drag.type === 'obstacleHeight') {
      const o = S.scene.obstacles.find((x) => x.id === S.drag.id);
      if (o) {
        o.sz = Math.max(0.2, S.drag.orig.sz - (ev.movementY || 0) * 0.01);
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
      S.drag = null;
      controls.enabled = true;
      rebuildAll();          // one rebuild after the drag, not per pointermove
      if (S.selected) showSelection();
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
    updateFog();
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
