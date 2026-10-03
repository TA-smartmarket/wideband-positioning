#!/usr/bin/env python3
"""ESP32 UWB positioning server — REST + MQTT ingest, solver, web UI.

Run:
    python3 app.py                    # http://127.0.0.1:8080
    python3 app.py --port 9000 --token secret
    python3 app.py --mqtt 0           # disable MQTT listener

Endpoints (see docs/API.md):
    POST /api/v1/telemetry              ingest telemetry from devices
    GET  /api/v1/config/device?role=&id=  config for one device (merged)
    PUT  /api/v1/config                 save partial config for a device
    GET  /api/v1/state                  current world state
    GET  /api/v1/scene                  room + anchors + obstacles (3D editor)
    PUT  /api/v1/scene                  save the 3D scene (pushes anchor config)
    GET  /api/v1/devices                known devices + last seen
    POST /api/v1/position               solve now (debug)
    GET  /  -> web UI

State is kept in memory and broadcast on MQTT `<base>/state` (retained).
"""

import argparse
import copy
import json
import math
import os
import threading
import time

from flask import Flask, jsonify, request, send_from_directory

import scene as scene_mod

# ---------------------------------------------------------------------------
# config model
# ---------------------------------------------------------------------------

BASE_CFG = {
    "device_id": "", "role": "anchor", "id": 1, "site": "home",
    "wifi": {"ssid": "", "password": ""},
    "server": {"url": "", "token": ""},
    "mqtt": {"enabled": False, "host": "", "port": 1883,
             "user": "", "password": "", "base_topic": "uwb/home"},
    "position": {"x": 0.0, "y": 0.0, "z": 2.2},
    "room": {"width": 5.0, "height": 4.0},
    "uwb": {"mode": "longdata_range_lowpower",
            "range_filter": True, "update_ms": 200},
}

DEVICES = {}          # device_id -> config dict (merged)
LOCK = threading.RLock()
STALE_MS = 5000       # device offline threshold

RANGES = {}           # (anchor_id, tag_id) -> {"range","rx","fp","q","ts"}
TAG_POS = {}          # tag_id -> {"x","y","conf","amb","ts","src"}
STATUS = {}           # device_id -> status dict
SCENE = scene_mod.default_scene()   # room + anchors + obstacles (3D editor)
NLOS = {}             # (anchor_id, tag_id) -> {"blocked","atten","sigma","bias"}
NLOS_ENABLED = True   # inflate sigma / bias for blocked paths

APP = Flask(__name__)


def now_ms():
    return int(time.time() * 1000)


def device_id_of(cfg):
    return f"{cfg['role']}-{cfg['id']}"


def touch_device(cfg):
    dev = DEVICES.setdefault(device_id_of(cfg), copy.deepcopy(BASE_CFG))
    dev.update(cfg)
    dev["last_seen"] = now_ms()
    return dev


def merged_config(role, did):
    """Config for a device: its stored values over the defaults, plus the
    anchor map.

    For an *unknown* device only the identity + anchor map are returned — the
    network/placement fields are omitted so the device keeps whatever it has
    in NVS instead of being wiped by the server defaults.
    """
    with LOCK:
        key = f"{role}-{did}"
        entry = DEVICES.get(key)
        # "_auto" devices were only seen in telemetry — their defaults must not
        # be pushed back, or the device would lose its locally set WiFi/server.
        known = entry is not None and not entry.get("_auto")
        cfg = copy.deepcopy(BASE_CFG)
        if known:
            cfg.update(entry)
        cfg["device_id"] = key
        cfg["role"] = role
        cfg["id"] = did
        # anchors[] array so a tag can solve locally too
        anchors = []
        for dk, dc in DEVICES.items():
            if dc.get("role") == "anchor" and not dc.get("_auto"):
                anchors.append({"id": dc.get("id"), "x": dc["position"]["x"],
                                "y": dc["position"]["y"], "z": dc["position"]["z"],
                                "device_id": dk})
        cfg["anchors"] = anchors
        if not known:
            # strip everything the device already knows locally
            return {"device_id": key, "role": role, "id": did, "anchors": anchors}
        # never push empty credentials back — that would wipe the device
        if not cfg["wifi"]["ssid"]:
            cfg.pop("wifi", None)
        if not cfg["server"]["url"]:
            cfg.pop("server", None)
        if not cfg["mqtt"]["host"]:
            cfg.pop("mqtt", None)
        return cfg


# ---------------------------------------------------------------------------
# solver (mirror of src/solver.h)
# ---------------------------------------------------------------------------

def solve_2d(fixes, room_w, room_h):
    """fixes: list of (x, y, range). Returns dict or None."""
    if len(fixes) < 2:
        return None

    use_room = room_w > 0 and room_h > 0

    if len(fixes) == 2:
        (x1, y1, r1), (x2, y2, r2) = fixes
        dx, dy = x2 - x1, y2 - y1
        d = math.hypot(dx, dy)
        if d < 1e-4:
            return None
        if d > r1 + r2 or d < abs(r1 - r2):
            return {"x": (x1 + x2) / 2, "y": (y1 + y2) / 2,
                    "confidence": 0.1, "ambiguous": True}
        aa = (r1 * r1 - r2 * r2 + d * d) / (2.0 * d)
        hh = math.sqrt(max(r1 * r1 - aa * aa, 0.0))
        mx, my = x1 + aa * dx / d, y1 + aa * dy / d
        c = [(mx + hh * dy / d, my - hh * dx / d),
             (mx - hh * dy / d, my + hh * dx / d)]
        if use_room:
            inside = [(i, pt) for i, pt in enumerate(c)
                      if 0 <= pt[0] <= room_w and 0 <= pt[1] <= room_h]
            if len(inside) == 1:
                i, pt = inside[0]
                return {"x": pt[0], "y": pt[1], "confidence": 0.5,
                        "ambiguous": False}
        # tie / no room: nearest to centre
        cx, cy = room_w / 2, room_h / 2
        pt = min(c, key=lambda p: (p[0] - cx) ** 2 + (p[1] - cy) ** 2)
        return {"x": pt[0], "y": pt[1], "confidence": 0.5, "ambiguous": True}

    # 3+ anchors: linearised least squares + Gauss-Newton refinement
    (x0, y0, r0) = fixes[0]
    A0 = A1 = B0 = B1 = C = 0.0
    for (xi, yi, ri) in fixes[1:]:
        ax, ay = 2.0 * (xi - x0), 2.0 * (yi - y0)
        b = (ri * ri - r0 * r0) - xi * xi + x0 * x0 - yi * yi + y0 * y0
        A0 += ax * ax; A1 += ax * ay; B0 += ax * b; B1 += ay * b; C += ay * ay
    det = A0 * C - A1 * A1
    if abs(det) < 1e-9:
        return None
    px = (B0 * C - B1 * A1) / det
    py = (A0 * B1 - A1 * B0) / det

    for _ in range(2):  # Gauss-Newton
        j0 = j1 = j2 = e0 = e1 = 0.0
        for (xi, yi, ri) in fixes:
            ddx, ddy = px - xi, py - yi
            dist = math.hypot(ddx, ddy) + 1e-9
            res = dist - ri
            j0 += ddx * ddx / (dist * dist)
            j1 += ddx * ddy / (dist * dist)
            j2 += ddy * ddy / (dist * dist)
            e0 += ddx * res / dist
            e1 += ddy * res / dist
        dj = j0 * j2 - j1 * j1
        if abs(dj) < 1e-12:
            break
        px -= (j2 * e0 - j1 * e1) / dj
        py -= (j0 * e1 - j1 * e0) / dj

    ss = sum((math.hypot(px - xi, py - yi) - ri) ** 2 for xi, yi, ri in fixes)
    rms = math.sqrt(ss / len(fixes))
    conf = max(0.0, min(1.0, 1.0 - rms))
    return {"x": px, "y": py, "confidence": conf, "ambiguous": False}


# ---------------------------------------------------------------------------
# Extended Kalman Filter (mirror of src/ekf.h)
#
#   state x = [px, py, vx, vy], constant-velocity motion, range measurement
#   h(x) = ||p - a_i|| is non-linear -> linearised with the Jacobian
#   H = [(px-ax)/d, (py-ay)/d, 0, 0]. Innovation gating rejects NLOS outliers.
# ---------------------------------------------------------------------------
EKF_N = 4


class TagEKF:
    def __init__(self, px, py, sigma_a=1.0, sigma_r=0.15):
        self.x = [px, py, 0.0, 0.0]
        self.P = [[1.0, 0, 0, 0],
                  [0, 1.0, 0, 0],
                  [0, 0, 4.0, 0],
                  [0, 0, 0, 4.0]]
        self.sigma_a = sigma_a
        self.sigma_r = sigma_r
        self.last_ms = now_ms()
        self.updates = 0

    # x = F x , P = F P F^T + Q   (constant velocity)
    def predict(self, dt):
        if dt <= 0:
            return
        dt = min(dt, 2.0)
        self.x[0] += self.x[2] * dt
        self.x[1] += self.x[3] * dt

        P = self.P
        FP = [[P[0][j] + dt * P[2][j] for j in range(4)],
              [P[1][j] + dt * P[3][j] for j in range(4)],
              [P[2][j] for j in range(4)],
              [P[3][j] for j in range(4)]]
        FPFt = [[FP[i][0] + dt * FP[i][2], FP[i][1] + dt * FP[i][3],
                 FP[i][2], FP[i][3]] for i in range(4)]

        dt2, dt3, dt4 = dt * dt, dt ** 3, dt ** 4
        q = self.sigma_a ** 2
        Q = [[q * dt4 / 4, 0, q * dt3 / 2, 0],
             [0, q * dt4 / 4, 0, q * dt3 / 2],
             [q * dt3 / 2, 0, q * dt2, 0],
             [0, q * dt3 / 2, 0, q * dt2]]
        self.P = [[FPFt[i][j] + Q[i][j] for j in range(4)] for i in range(4)]

    # one scalar range measurement to anchor (ax, ay); returns True if accepted
    # sigma_r may be overridden per measurement (NLOS paths are noisier).
    def update_range(self, ax, ay, rng, gate_sigma=3.0, sigma_r=None):
        if rng <= 0.01:
            return False
        sr = self.sigma_r if sigma_r is None else sigma_r
        dx, dy = self.x[0] - ax, self.x[1] - ay
        d = math.hypot(dx, dy) or 1e-3

        H = [dx / d, dy / d, 0.0, 0.0]                    # Jacobian
        innov = rng - d                                    # y = z - h(x)

        PHt = [sum(self.P[i][k] * H[k] for k in range(4)) for i in range(4)]
        S = sum(H[i] * PHt[i] for i in range(4)) + sr ** 2
        if S < 1e-9:
            return False

        if abs(innov) > gate_sigma * math.sqrt(S):         # innovation gate
            return False

        K = [PHt[i] / S for i in range(4)]
        for i in range(4):
            self.x[i] += K[i] * innov

        Pn = [[self.P[i][j] - K[i] * sum(H[k] * self.P[k][j] for k in range(4))
               for j in range(4)] for i in range(4)]
        self.P = Pn
        self.updates += 1
        return True

    def position(self):
        return self.x[0], self.x[1], self.x[2], self.x[3]

    def sigma(self):
        return math.sqrt(max(self.P[0][0], 0.0) + max(self.P[1][1], 0.0))


EKF = {}      # tag_id -> TagEKF


def anchor_xyz(device_id, fallback_id=None):
    """Anchor position: the 3D scene is the source of truth, the device config
    is the fallback (so a device that was configured before the scene existed
    still works)."""
    with LOCK:
        for a in SCENE.get("anchors", []):
            if a["id"] == device_id:
                return a["x"], a["y"], a.get("z", 2.2)
        dev = DEVICES.get(device_id)
        if dev and dev.get("role") == "anchor":
            p = dev.get("position", {})
            return p.get("x", 0.0), p.get("y", 0.0), p.get("z", 2.2)
    return None


def recompute_positions():
    """Recompute all tag positions: geometric solver for the first fix, then
    the EKF tracks each tag over time (see class TagEKF).

    Obstacles in the scene make this NLOS-aware: a blocked anchor→tag path gets
    an inflated measurement sigma (so the EKF trusts it less, or rejects it via
    the innovation gate) plus a small positive bias (UWB NLOS reads long)."""
    global NLOS
    now = now_ms()
    out = {}
    tags = {dk for dk, dc in DEVICES.items() if dc.get("role") == "tag"}
    for (a, t) in RANGES.keys():
        tags.add(t)

    with LOCK:
        room = dict(SCENE["room"])
        obstacles = list(SCENE.get("obstacles", []))

    for tag in tags:
        fixes, anchors_used = [], []
        links = []
        for (a, t), r in RANGES.items():
            if t != tag:
                continue
            if now - r.get("recv_ts", r["ts"]) > STALE_MS:
                continue
            pos = anchor_xyz(a)
            if not pos:
                continue
            ax, ay, az = pos
            dz = az - scene_mod.DEFAULT_TAG_Z
            rng2d = scene_mod.horizontal_range(r["range"], dz)
            if rng2d <= 0.01:
                continue
            fixes.append((ax, ay, rng2d))
            anchors_used.append(a)
            links.append((a, ax, ay, az, rng2d))

        if len(fixes) < 2:
            continue

        # --- bootstrap: closed-form fix for the first estimate --------------
        if tag not in EKF:
            sol = solve_2d(fixes, room["width"], room["depth"])
            if not sol:
                continue
            EKF[tag] = TagEKF(sol["x"], sol["y"])
        else:
            EKF[tag].predict((now - EKF[tag].last_ms) / 1000.0)
            EKF[tag].last_ms = now

        # --- track: one range update per anchor, NLOS-aware ------------------
        used = 0
        link_info = {}
        for (a, ax, ay, az, rng2d) in links:
            blockers = scene_mod.los_blockers(
                (ax, ay, az), (EKF[tag].x[0], EKF[tag].x[1], scene_mod.DEFAULT_TAG_Z),
                obstacles) if NLOS_ENABLED else []
            sigma = scene_mod.measurement_sigma(blockers, EKF[tag].sigma_r)
            bias = scene_mod.nlos_bias(blockers)
            link_info[a] = {
                "blocked": bool(blockers),
                "atten": max([ob.get("atten", 1.0) for ob in blockers], default=0.0),
                "sigma": sigma,
                "bias": bias,
                "obstacles": [ob["id"] for ob in blockers],
            }
            if EKF[tag].update_range(ax, ay, rng2d + bias, sigma_r=sigma):
                used += 1

        NLOS.update({(a, tag): info for a, info in link_info.items()})

        px, py, vx, vy = EKF[tag].position()
        sigma = EKF[tag].sigma()

        out[tag] = {
            "ts": now, "src": "server", "id": tag,
            "x": px, "y": py, "z": scene_mod.DEFAULT_TAG_Z,
            "vx": vx, "vy": vy,
            "confidence": 1.0 / (1.0 + sigma),
            "sigma": sigma, "ambiguous": False,
            "updates": EKF[tag].updates, "used": used,
            "ranges": {a: RANGES[(a, tag)]["range"] for a in anchors_used},
            "links": link_info,
        }
    return out


# ---------------------------------------------------------------------------
# ingest
# ---------------------------------------------------------------------------

def handle_status(device_id, st):
    with LOCK:
        STATUS[device_id] = dict(st, ts=now_ms())


def ingest_telemetry(payload, source):
    """payload: dict (telemetry object or single-range object)."""
    with LOCK:
        did = payload.get("device_id")
        cfg = None
        if did:
            cfg = DEVICES.get(did)
            if cfg is None:
                # auto-register unknown device by parsing "role-id"
                role_guess = did.split("-")[0] if did.count("-") == 1 else None
                if role_guess in ("tag", "anchor"):
                    cfg = copy.deepcopy(BASE_CFG)
                    cfg["role"] = role_guess
                    try:
                        cfg["id"] = int(did.split("-")[1])
                    except ValueError:
                        cfg["id"] = 0
                    cfg["device_id"] = did
                    cfg["last_seen"] = now_ms()
                    cfg["_auto"] = True        # seen, but not configured yet
                    DEVICES[did] = cfg
        if payload.get("status"):
            handle_status(payload["device_id"], payload["status"])

        for r in payload.get("ranges", []):
            src, dst = r.get("src"), r.get("dst")
            if not src or not dst:
                continue
            tag = dst if src.startswith("anchor") else src
            anc = src if src.startswith("anchor") else dst
            if payload.get("device_id") != did or not cfg:
                continue
            # Device clocks are millis()-based (uptime), so they cannot be
            # compared with the server epoch. Keep both: `ts` for display and
            # `recv_ts` for staleness, which is what the solver uses.
            RANGES[(anc, tag)] = {
                "range": r.get("range", 0.0), "rx": r.get("rx_power", 0.0),
                "fp": r.get("fp_power", 0.0), "q": r.get("quality", 0.0),
                "ts": r.get("ts", 0), "recv_ts": now_ms(),
            }
            if cfg.get("role") == "anchor":
                cfg.setdefault("position", BASE_CFG["position"])

        for p in payload.get("positions", []):
            # The server EKF is authoritative; a device estimate is only used
            # as a fallback when the server has no track yet (otherwise it
            # would wipe sigma/vx/vy every cycle).
            if p["id"] in EKF:
                continue
            TAG_POS[p["id"]] = {"x": p["x"], "y": p["y"],
                                "vx": p.get("vx", 0.0), "vy": p.get("vy", 0.0),
                                "sigma": p.get("sigma", 0.0),
                                "conf": p.get("confidence", 0.5),
                                "amb": p.get("ambiguous", False),
                                "ts": now_ms(), "src": "device"}

        if did and did in DEVICES and payload.get("device_id"):
            DEVICES[did]["last_seen"] = now_ms()

    positions = recompute_positions()
    with LOCK:
        for tag, sol in positions.items():
            TAG_POS[tag] = {"x": sol["x"], "y": sol["y"],
                            "vx": sol.get("vx", 0.0), "vy": sol.get("vy", 0.0),
                            "sigma": sol.get("sigma", 0.0),
                            "conf": sol["confidence"], "amb": sol["ambiguous"],
                            "ts": sol["ts"], "src": "server"}
    broadcast_state()
    return positions


def build_state():
    now = now_ms()
    with LOCK:
        room = dict(SCENE["room"])
        scene_anchors = {a["id"]: a for a in SCENE.get("anchors", [])}
        obstacles = list(SCENE.get("obstacles", []))

    anchors = []
    for dk, dc in DEVICES.items():
        if dc.get("role") != "anchor":
            continue
        sa = scene_anchors.get(dk)
        pos = sa or dc.get("position", {})
        anchors.append({
            "id": dk,
            "label": sa.get("label", dk) if sa else dk,
            "x": pos.get("x", 0.0), "y": pos.get("y", 0.0), "z": pos.get("z", 2.2),
            "online": now - dc.get("last_seen", 0) < 5000,
            "last_seen": dc.get("last_seen", 0)})

    # anchors that exist in the scene but have not reported yet
    for aid, sa in scene_anchors.items():
        if aid not in DEVICES:
            anchors.append({"id": aid, "label": sa.get("label", aid),
                            "x": sa["x"], "y": sa["y"], "z": sa.get("z", 2.2),
                            "online": False, "last_seen": 0})

    tags = []
    for tag, p in TAG_POS.items():
        tags.append({"id": tag, "x": p["x"], "y": p["y"],
                     "z": p.get("z", scene_mod.DEFAULT_TAG_Z),
                     "vx": p.get("vx", 0.0), "vy": p.get("vy", 0.0),
                     "sigma": p.get("sigma", 0.0),
                     "confidence": p["conf"], "ambiguous": p["amb"],
                     "online": now - p["ts"] < STALE_MS, "last_seen": p["ts"],
                     "source": p.get("src", "server"),
                     "ranges": {a: r["range"] for (a, t), r in RANGES.items()
                                if t == tag},
                     "links": p.get("links", {})})

    links = []
    for (a, t), r in RANGES.items():
        info = NLOS.get((a, t), {})
        links.append({"src": a, "dst": t, "range": r["range"], "rx_power": r["rx"],
                      "ts": r["ts"],
                      "blocked": bool(info.get("blocked", False)),
                      "obstacles": info.get("obstacles", [])})

    return {"site": BASE_CFG["site"], "ts": now, "room": room,
            "obstacles": obstacles,
            "nlos_enabled": NLOS_ENABLED,
            "anchors": anchors, "tags": tags, "links": links}


STATE = {"site": "home", "ts": 0, "room": dict(scene_mod.DEFAULT_ROOM),
         "obstacles": [], "nlos_enabled": True,
         "anchors": [], "tags": [], "links": []}


def broadcast_state():
    global STATE
    STATE = build_state()
    mqtt_publish(cfg['mqtt']['base_topic'] + "/state", json.dumps(STATE), retained=True)


# ---------------------------------------------------------------------------
# MQTT (paho)
# ---------------------------------------------------------------------------

mqtt_client = None


def mqtt_publish(topic, payload, retained=False):
    if mqtt_client and mqtt_client.is_connected():
        try:
            mqtt_client.publish(topic, payload, retain=retained)
        except Exception as e:  # pragma: no cover
            print(f"[mqtt] publish failed: {e}")


def mqtt_on_message(client, userdata, msg):
    topic = msg.topic
    base = cfg['mqtt']['base_topic']
    try:
        payload = json.loads(msg.payload)
    except Exception:
        return
    if topic == f"{base}/telemetry" or topic == f"{base}/range":
        ingest_telemetry(payload, "mqtt")
    elif topic.startswith(f"{base}/status/"):
        dev = topic[len(f"{base}/status/"):]
        if isinstance(payload, dict):
            handle_status(dev, payload)


def mqtt_start():
    global mqtt_client
    import paho.mqtt.client as mqtt

    mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    if cfg['mqtt']['user']:
        mqtt_client.username_pw_set(cfg['mqtt']['user'], cfg['mqtt']['password'])
    mqtt_client.on_message = mqtt_on_message
    base = cfg['mqtt']['base_topic']
    try:
        mqtt_client.connect(cfg['mqtt']['host'], cfg['mqtt']['port'], keepalive=30)
        mqtt_client.subscribe([(f"{base}/range", 0), (f"{base}/telemetry", 0),
                               (f"{base}/status/+", 1)])
        mqtt_client.loop_start()
        print(f"[mqtt] listening {cfg['mqtt']['host']}:{cfg['mqtt']['port']} base '{base}'")
    except Exception as e:
        print(f"[mqtt] failed to start: {e}")


# ---------------------------------------------------------------------------
# REST API
# ---------------------------------------------------------------------------

def require_token():
    if not cfg.get("token"):
        return True
    auth = request.headers.get("Authorization", "")
    return auth == f"Bearer {cfg['token']}"


APP.config["JSON_AS_ASCII"] = False


@APP.before_request
def auth():
    if request.path.startswith("/api/") and not require_token():
        return jsonify({"ok": False, "error": "unauthorized"}), 401


@APP.route("/api/v1/telemetry", methods=["POST"])
def api_telemetry():
    payload = request.get_json(silent=True)
    if not payload:
        return jsonify({"ok": False, "error": "bad json"}), 400
    positions = ingest_telemetry(payload, "rest")
    return jsonify({"ok": True, "positions": positions})


@APP.route("/api/v1/config/device", methods=["GET"])
def api_config_device():
    role = request.args.get("role", "anchor")
    try:
        did = int(request.args.get("id", 1))
    except ValueError:
        return jsonify({"ok": False, "error": "bad id"}), 400
    return jsonify(merged_config(role, did))


@APP.route("/api/v1/config", methods=["PUT"])
def api_config_put():
    """Save a device config (legacy/API path). Anchor positions and the room
    are mirrored into the 3D scene so the editor and the API never diverge."""
    global SCENE
    body = request.get_json(silent=True)
    if not body:
        return jsonify({"ok": False, "error": "bad json"}), 400
    with LOCK:
        role = body.get("role", "anchor")
        did = body.get("id", 1)
        key = f"{role}-{did}"
        dev = DEVICES.setdefault(key, copy.deepcopy(BASE_CFG))
        deep_merge(dev, body)
        dev["role"] = role
        dev["id"] = did
        dev.pop("_auto", None)        # explicitly configured now
        dev["last_seen"] = now_ms()

        # keep the 3D scene in sync
        pos = dev.get("position") or {}
        if role == "anchor" and ("x" in pos or "y" in pos):
            a = next((x for x in SCENE["anchors"] if x["id"] == key), None)
            if not a:
                a = {"id": key, "label": key, "x": 0.0, "y": 0.0, "z": 2.2}
                SCENE["anchors"].append(a)
            a["x"] = float(pos.get("x", a["x"]))
            a["y"] = float(pos.get("y", a["y"]))
            a["z"] = float(pos.get("z", a.get("z", 2.2)))
        room = body.get("room") or {}
        if room.get("width"):
            SCENE["room"]["width"] = float(room["width"])
        if room.get("height"):
            SCENE["room"]["depth"] = float(room["height"])
        SCENE = scene_mod.normalise_scene(SCENE)
        save_scene()

        devcfg = merged_config(role, did)
    # push to device over MQTT (retained -> node picks it up even if offline)
    try:
        mqtt_publish(cfg["mqtt"]["base_topic"] + f"/config/{key}",
                     json.dumps(devcfg), retained=True)
    except Exception:
        pass
    EKF.clear()          # geometry changed -> restart the filters
    broadcast_state()
    return jsonify({"ok": True, "config": devcfg})


def deep_merge(base, patch):
    for k, v in patch.items():
        if k in ("role", "id", "device_id"):
            continue
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            deep_merge(base[k], v)
        else:
            base[k] = v


@APP.route("/api/v1/state", methods=["GET"])
def api_state():
    return jsonify(STATE)


@APP.route("/api/v1/scene", methods=["GET"])
def api_scene_get():
    with LOCK:
        return jsonify({"ok": True, "scene": SCENE,
                        "nlos_enabled": NLOS_ENABLED,
                        "device_positions": {
                            dk: dc.get("position", {})
                            for dk, dc in DEVICES.items()
                            if dc.get("role") == "anchor"}})


@APP.route("/api/v1/scene", methods=["PUT"])
def api_scene_put():
    """Save the 3D scene and push each anchor's position to its device."""
    global SCENE
    body = request.get_json(silent=True)
    if not body:
        return jsonify({"ok": False, "error": "bad json"}), 400

    with LOCK:
        SCENE = scene_mod.normalise_scene(body.get("scene", body))
        if "nlos_enabled" in body:
            global NLOS_ENABLED
            NLOS_ENABLED = bool(body["nlos_enabled"])

        pushed = []
        for a in SCENE["anchors"]:
            did = a["id"]
            dev = DEVICES.setdefault(did, copy.deepcopy(BASE_CFG))
            dev.setdefault("position", {})
            dev["position"] = {"x": a["x"], "y": a["y"], "z": a.get("z", 2.2)}
            dev["role"] = "anchor"
            dev.pop("_auto", None)
            try:
                dev["id"] = int(did.split("-")[1])
            except (IndexError, ValueError):
                pass
            # room is kept on the device too (used by its standalone solver)
            dev["room"] = {"width": SCENE["room"]["width"],
                           "height": SCENE["room"]["depth"]}
            pushed.append(did)

        save_scene()

    # push the new anchor position to each device over MQTT (retained)
    for did in pushed:
        role, _, num = did.partition("-")
        devcfg = merged_config(role or "anchor", int(num) if num.isdigit() else 1)
        mqtt_publish(cfg["mqtt"]["base_topic"] + f"/config/{did}",
                     json.dumps(devcfg), retained=True)

    # anchors moved -> existing EKF states are stale, restart the filters
    EKF.clear()
    recompute_positions()
    broadcast_state()
    return jsonify({"ok": True, "scene": SCENE, "pushed": pushed})


@APP.route("/static/<path:filename>")
def static_files(filename):
    return send_from_directory(os.path.join(os.path.dirname(__file__), "static"),
                               filename)


def save_scene(path=None):
    """Persist the scene next to the server so restarts keep the room setup."""
    path = path or os.path.join(os.path.dirname(__file__), "scene.json")
    try:
        with open(path, "w", encoding="utf-8") as fh:
            json.dump({"scene": SCENE, "nlos_enabled": NLOS_ENABLED}, fh, indent=2)
    except OSError as e:
        print(f"[scene] save failed: {e}")


def load_scene(path=None):
    global SCENE, NLOS_ENABLED
    path = path or os.path.join(os.path.dirname(__file__), "scene.json")
    try:
        with open(path, encoding="utf-8") as fh:
            data = json.load(fh)
        SCENE = scene_mod.normalise_scene(data.get("scene", data))
        NLOS_ENABLED = bool(data.get("nlos_enabled", True))
        print(f"[scene] loaded {path} "
              f"({len(SCENE['anchors'])} anchors, {len(SCENE['obstacles'])} obstacles)")
    except (OSError, ValueError):
        pass          # no saved scene yet -> defaults


@APP.route("/api/v1/devices", methods=["GET"])
def api_devices():
    out = [{"device_id": dk,
            **{k: d[k] for k in ("role", "id") if k in d},
            "configured": not d.get("_auto", False),
            "last_seen": d.get("last_seen", 0),
            "online": now_ms() - d.get("last_seen", 0) < 5000}
           for dk, d in DEVICES.items()
           if dk != "_room" and d.get("role") in ("tag", "anchor")]
    return jsonify({"ok": True, "devices": out})


@APP.route("/api/v1/position", methods=["POST"])
def api_position():
    positions = recompute_positions()
    return jsonify({"ok": True, "positions": positions})


# ---------------------------------------------------------------------------
# web UI
# ---------------------------------------------------------------------------

@APP.route("/")
def web():
    return send_from_directory(os.path.join(os.path.dirname(__file__), "static"),
                               "index.html")


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

cfg = {"mqtt": {"base_topic": "uwb/home", "host": "", "port": 1883,
                "user": "", "password": ""},
       "token": "", "port": 8080}


def main():
    ap = argparse.ArgumentParser(description="ESP32 UWB positioning server")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--token", default="", help="optional bearer token")
    ap.add_argument("--mqtt-host", default="127.0.0.1")
    ap.add_argument("--mqtt-port", type=int, default=1883)
    ap.add_argument("--mqtt-base", default="uwb/home")
    ap.add_argument("--mqtt", type=int, default=1, help="0 = disable mqtt")
    args = ap.parse_args()

    cfg["port"] = args.port
    cfg["token"] = args.token
    cfg["mqtt"]["host"] = args.mqtt_host
    cfg["mqtt"]["port"] = args.mqtt_port
    cfg["mqtt"]["base_topic"] = args.mqtt_base

    print(f"[server] http://0.0.0.0:{args.port}" + ("  (token auth)" if args.token else ""))
    load_scene()
    if args.mqtt:
        mqtt_start()

    APP.run(host="0.0.0.0", port=args.port, threaded=True)


if __name__ == "__main__":
    main()