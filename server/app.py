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
    GET  /api/v1/devices                known devices + last seen
    POST /api/v1/position               solve now (debug)
    GET  /  -> web UI

State is kept in memory and broadcast on MQTT `<base>/state` (retained).
"""

import argparse
import copy
import json
import math
import threading
import time

from flask import Flask, jsonify, request

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
    def update_range(self, ax, ay, rng, gate_sigma=3.0):
        if rng <= 0.01:
            return False
        dx, dy = self.x[0] - ax, self.x[1] - ay
        d = math.hypot(dx, dy) or 1e-3

        H = [dx / d, dy / d, 0.0, 0.0]                    # Jacobian
        innov = rng - d                                    # y = z - h(x)

        PHt = [sum(self.P[i][k] * H[k] for k in range(4)) for i in range(4)]
        S = sum(H[i] * PHt[i] for i in range(4)) + self.sigma_r ** 2
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


def recompute_positions():
    """Recompute all tag positions: geometric solver for the first fix, then
    the EKF tracks each tag over time (see class TagEKF)."""
    now = now_ms()
    out = {}
    tags = {dk for dk, dc in DEVICES.items() if dc.get("role") == "tag"}
    for (a, t) in RANGES.keys():
        tags.add(t)
    for tag in tags:
        fixes, anchors_used = [], []
        for (a, t), r in RANGES.items():
            if t != tag:
                continue
            if now - r["ts"] > STALE_MS:
                continue
            acfg = DEVICES.get(a)
            if not acfg:
                continue
            fixes.append((acfg["position"]["x"], acfg["position"]["y"], r["range"]))
            anchors_used.append(a)
        if len(fixes) < 2:
            continue

        room = DEVICES.get("_room", BASE_CFG["room"])

        # --- bootstrap: closed-form fix for the first estimate --------------
        if tag not in EKF:
            sol = solve_2d(fixes, room["width"], room["height"])
            if not sol:
                continue
            EKF[tag] = TagEKF(sol["x"], sol["y"])
        else:
            EKF[tag].predict((now - EKF[tag].last_ms) / 1000.0)
            EKF[tag].last_ms = now

        # --- track: one range update per anchor ----------------------------
        used = sum(1 for (ax, ay, rng) in fixes if EKF[tag].update_range(ax, ay, rng))
        px, py, vx, vy = EKF[tag].position()
        sigma = EKF[tag].sigma()

        out[tag] = {
            "ts": now, "src": "server", "id": tag,
            "x": px, "y": py, "vx": vx, "vy": vy,
            "confidence": 1.0 / (1.0 + sigma),
            "sigma": sigma, "ambiguous": False,
            "updates": EKF[tag].updates, "used": used,
            "ranges": {a: RANGES[(a, tag)]["range"] for a in anchors_used},
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
            RANGES[(anc, tag)] = {
                "range": r.get("range", 0.0), "rx": r.get("rx_power", 0.0),
                "fp": r.get("fp_power", 0.0), "q": r.get("quality", 0.0),
                "ts": r.get("ts", now_ms()),
            }
            if cfg.get("role") == "anchor":
                cfg.setdefault("position", BASE_CFG["position"])

        for p in payload.get("positions", []):
            TAG_POS[p["id"]] = {"x": p["x"], "y": p["y"],
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
    room = DEVICES.get("_room", BASE_CFG["room"])
    anchors = []
    for dk, dc in DEVICES.items():
        if dc.get("role") == "anchor":
            anchors.append({
                "id": dk, "x": dc["position"]["x"], "y": dc["position"]["y"],
                "z": dc["position"]["z"], "online": now - dc.get("last_seen", 0) < 5000,
                "last_seen": dc.get("last_seen", 0)})
    tags = []
    for tag, p in TAG_POS.items():
        tags.append({"id": tag, "x": p["x"], "y": p["y"], "z": 0.9,
                     "vx": p.get("vx", 0.0), "vy": p.get("vy", 0.0),
                     "sigma": p.get("sigma", 0.0),
                     "confidence": p["conf"], "ambiguous": p["amb"],
                     "online": now - p["ts"] < STALE_MS, "last_seen": p["ts"],
                     "source": p.get("src", "server"),
                     "ranges": {a: r["range"] for (a, t), r in RANGES.items()
                                if t == tag}})
    links = [{"src": a, "dst": t, "range": r["range"], "rx_power": r["rx"],
              "ts": r["ts"]} for (a, t), r in RANGES.items()]
    return {"site": BASE_CFG["site"], "ts": now, "room": room,
            "anchors": anchors, "tags": tags, "links": links}


STATE = {"site": "home", "ts": 0, "room": BASE_CFG["room"],
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
    body = request.get_json(silent=True)
    if not body:
        return jsonify({"ok": False, "error": "bad json"}), 400
    with LOCK:
        if body.get("room"):
            DEVICES["_room"] = body["room"]
        role = body.get("role", "anchor")
        did = body.get("id", 1)
        key = f"{role}-{did}"
        dev = DEVICES.setdefault(key, copy.deepcopy(BASE_CFG))
        deep_merge(dev, body)
        dev["role"] = role
        dev["id"] = did
        dev.pop("_auto", None)        # explicitly configured now
        dev["last_seen"] = now_ms()
        devcfg = merged_config(role, did)
    # push to device over MQTT (retained -> node picks it up even if offline)
    try:
        mqtt_publish(cfg["mqtt"]["base_topic"] + f"/config/{key}",
                     json.dumps(devcfg), retained=True)
    except Exception:
        pass
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

UI = """<!doctype html>
<html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>UWB positioning — __SITE__</title>
<style>
 body{font:14px system-ui;margin:0;background:#0f1220;color:#e8e8f0}
 .wrap{max-width:1100px;margin:0 auto;padding:16px}
 h1,h2{font-weight:600}
 .grid{display:grid;grid-template-columns:2fr 1fr;gap:16px}
 @media(max-width:800px){.grid{grid-template-columns:1fr}}
 .card{background:#1a1e33;border:1px solid #2a2f4a;border-radius:10px;padding:14px;margin-bottom:16px}
 svg{background:#11152a;border:1px solid #22264a;border-radius:8px;width:100%}
 table{border-collapse:collapse;width:100%}
 td,th{padding:5px 8px;text-align:left;border-bottom:1px solid #272b48}
 .tag{color:#ffd166}.anc{color:#6ee7b7}.stale{opacity:.35}
 input,select{padding:6px;margin:3px 0 8px;border-radius:6px;border:1px solid #333;background:#12152a;color:#eee}
 button{padding:8px 16px;border:0;border-radius:6px;background:#3b82f6;color:#fff;cursor:pointer}
 .ok{color:#6ee7b7}.off{color:#f87171}
</style></head><body><div class="wrap">
<h1>📡 UWB positioning <span style="font-weight:400;color:#8b8fb0">· __SITE__</span></h1>
<div class="grid">
 <div>
  <div class="card"><h2>Live map</h2><div id="map"></div></div>
  <div class="card"><h2>Tags</h2><table id="tags"><tr><th>id</th><th>x</th><th>y</th><th>conf</th><th>status</th></tr></table></div>
 </div>
 <div>
  <div class="card"><h2>Anchors / devices</h2><table id="anc"><tr><th>id</th><th>pos</th><th>state</th></tr></table></div>
  <div class="card"><h2>Setup</h2>
   <label>Role</label><select id="setR"><option>anchor</option><option>tag</option></select>
   <label>ID (1-10)</label><input id="setI" type="number" min="1" max="10" value="1">
   <label>Position X (m)</label><input id="setX" type="number" step="0.01" value="0">
   <label>Position Y (m)</label><input id="setY" type="number" step="0.01" value="0">
   <label>Room width / height (m)</label><input id="setW" type="number" step="0.01" value="5">
   <input id="setH" type="number" step="0.01" value="4">
   <label>WiFi SSID</label><input id="setS" placeholder="MyWiFi">
   <label>WiFi password</label><input id="setP" type="password">
   <label>Server URL (on device)</label><input id="setU" placeholder="http://192.168.1.10:8080">
   <button onclick="saveCfg()">💾 Save to device</button>
   <p style="color:#8b8fb0;font-size:12px">Save pushes config over MQTT (retained). The device applies it and reboots.</p>
  </div>
 </div>
</div>
<script>
let state={};
async function poll(){try{const r=await fetch('/api/v1/state');state=await r.json();render();}catch(e){}}
function render(){
 const W=400,H=320,r=state.room||{width:5,height:4};
 const sx=W/r.width, sy=H/r.height;
 let svg=`<svg viewBox="0 0 ${W} ${H}">`;
 svg+=`<rect x="0" y="0" width="${W}" height="${H}" fill="none" stroke="#2a2f4a"/>`;
 (state.anchors||[]).forEach(a=>{svg+=`<g class="anc"><circle cx="${a.x*sx}" cy="${a.y*sy}" r="8" fill="#6ee7b7"/><text x="${a.x*sx+10}" y="${a.y*sy+4}" font-size="11">${a.id}</text></g>`;});
 (state.links||[]).forEach(l=>{const a=(state.anchors||[]).find(x=>x.id===l.src);if(!a)return;const t=(state.tags||[]).find(x=>x.id===l.dst);if(!t)return;svg+=`<line x1="${a.x*sx}" y1="${a.y*sy}" x2="${t.x*sx}" y2="${t.y*sy}" stroke="#ffd166" stroke-width="1.5" stroke-dasharray="4,3"/>`;});
 (state.tags||[]).forEach(t=>{svg+=`<g class="tag"><circle cx="${t.x*sx}" cy="${t.y*sy}" r="10" fill="#ffd166"/><text x="${t.x*sx+12}" y="${t.y*sy+4}" font-size="11">${t.id}</text></g>`;});
 svg+='</svg>';
 document.getElementById('map').innerHTML=svg;
 const tc=document.getElementById('tags');tc.innerHTML='<tr><th>id</th><th>x</th><th>y</th><th>conf</th><th>status</th></tr>';
 (state.tags||[]).forEach(t=>{tc.innerHTML+=`<tr><td>${t.id}</td><td>${t.x.toFixed(2)}</td><td>${t.y.toFixed(2)}</td><td>${(t.confidence*100).toFixed(0)}%</td><td class="${t.online?'ok':'off'}">${t.online?'live':(t.ambiguous?'ambig':'off')}</td></tr>`;});
 const ac=document.getElementById('anc');ac.innerHTML='<tr><th>id</th><th>pos</th><th>state</th></tr>';
 (state.anchors||[]).forEach(a=>{ac.innerHTML+=`<tr><td>${a.id}</td><td>${a.x.toFixed(1)},${a.y.toFixed(1)}</td><td class="${a.online?'ok':'off'}">${a.online?'online':'off'}</td></tr>`;});
}
function saveCfg(){const body={role:document.getElementById('setR').value,id:+document.getElementById('setI').value,
 position:{x:+document.getElementById('setX').value,y:+document.getElementById('setY').value},
 room:{width:+document.getElementById('setW').value,height:+document.getElementById('setH').value},
 wifi:{ssid:document.getElementById('setS').value,password:document.getElementById('setP').value},
 server:{url:document.getElementById('setU').value}};
 fetch('/api/v1/config',{method:'PUT',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)}).then(async r=>{alert('Saved: '+JSON.stringify((await r.json()).config||{}).slice(0,120));});
}
poll();setInterval(poll,700);
</script></div></body></html>"""


@APP.route("/")
def web():
    room = DEVICES.get("_room", BASE_CFG["room"])
    return UI.replace("__SITE__", BASE_CFG["site"])


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
    if args.mqtt:
        mqtt_start()

    APP.run(host="0.0.0.0", port=args.port, threaded=True)


if __name__ == "__main__":
    main()