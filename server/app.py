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
import secrets
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
    # OLED power saving on the node (the device stays online)
    "display": {"mode": "dim", "timeout_s": 60},
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

# --- OTA -------------------------------------------------------------------
# Each device gets its own key. It is generated here, stored with the device
# config and pushed to the node through the normal config channel; the device
# refuses any firmware upload that does not carry it.
OTA_TOKENS = {}       # device_id -> token
OTA_PORT = 3232       # must match the firmware default
OTA_DIR = os.path.join(os.path.dirname(__file__), "firmware")


OTA_TOKENS = {}       # device_id -> token (persisted, see ota_tokens_path)
# Manual updates: an update is queued per device and only handed out once the
# operator requests it. Without this the device would pull the newest image on
# every check (observed: 103 downloads of the same file), which wears the flash
# for no reason.
OTA_PENDING = {}      # device_id -> {"firmware": name, "requested": ts}
OTA_TOKENS_PATH = os.path.join(os.path.dirname(__file__), "ota_tokens.json")


def _load_ota_tokens():
    """Tokens must survive a server restart: the device keeps the key it was
    given, so a new random key after every restart would make every push fail
    with 401/rejected even though nothing changed on the device."""
    global OTA_TOKENS
    try:
        with open(OTA_TOKENS_PATH, encoding="utf-8") as fh:
            data = json.load(fh)
        if isinstance(data, dict):
            OTA_TOKENS = {str(k): str(v) for k, v in data.items()}
            print(f"[ota] loaded {len(OTA_TOKENS)} device key(s)")
    except (OSError, ValueError):
        pass          # first run


def _save_ota_tokens():
    try:
        with open(OTA_TOKENS_PATH, "w", encoding="utf-8") as fh:
            json.dump(OTA_TOKENS, fh, indent=2)
    except OSError as e:
        print(f"[ota] could not save keys: {e}")


def version_tuple(v):
    """'1.0.10' -> (1, 0, 10). Non-numeric parts are ignored."""
    out = []
    for part in str(v or "").split("."):
        digits = "".join(ch for ch in part if ch.isdigit())
        out.append(int(digits) if digits else 0)
    return tuple(out) or (0,)


def version_newer(candidate, current):
    """True only when `candidate` is strictly newer than `current`."""
    return version_tuple(candidate) > version_tuple(current)


def ota_token(device_id, regenerate=False):
    with LOCK:
        if regenerate or device_id not in OTA_TOKENS:
            OTA_TOKENS[device_id] = secrets.token_urlsafe(18)
            _save_ota_tokens()
        return OTA_TOKENS[device_id]


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


def auto_place_anchor(device_id, height=2.2):
    """Put a newly seen anchor in the next free room corner.

    Corners are tried in a fixed order; the first one that is not taken (by an
    existing anchor) wins. Nothing is pushed to the device here — this only
    makes the node visible in the 3D editor so the operator can drag it to the
    true position and Save. Dynamic: adding an anchor never needs a code change
    or a server restart.
    """
    global SCENE
    with LOCK:
        room = SCENE["room"]
        w, d = room["width"], room["depth"]
        corners = [(0.0, 0.0), (w, 0.0), (w, d), (0.0, d),
                   (w / 2, 0.0), (w, d / 2), (w / 2, d), (0.0, d / 2)]
        taken = [(a["x"], a["y"]) for a in SCENE["anchors"] if a["id"] != device_id]
        pos = None
        for c in corners:
            if all(abs(c[0] - t[0]) > 0.3 or abs(c[1] - t[1]) > 0.3 for t in taken):
                pos = c
                break
        if pos is None:
            # more anchors than corners: spread them along the ceiling edge
            n = len(taken)
            pos = ((n % 6) * (w / 6) + w / 12, d if n % 2 else 0.0)

        entry = next((a for a in SCENE["anchors"] if a["id"] == device_id), None)
        if entry is None:
            entry = {"id": device_id, "label": device_id, "x": pos[0], "y": pos[1], "z": height}
            SCENE["anchors"].append(entry)
            print(f"[scene] new anchor {device_id} auto-placed at "
                  f"({pos[0]:.1f}, {pos[1]:.1f}) — drag it to the real corner in the UI")
        SCENE = scene_mod.normalise_scene(SCENE)


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
        # OTA credentials: the device needs its key to accept an update
        cfg["ota"] = {"enabled": True, "port": OTA_PORT, "token": ota_token(key)}
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
            return {"device_id": key, "role": role, "id": did, "anchors": anchors,
                    "ota": {"enabled": True, "port": OTA_PORT, "token": ota_token(key)}}
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

        # Bound the covariance: when measurements are rejected for a long time
        # (heavy NLOS) P grows without limit and sigma became meaningless
        # (observed 11 m). Cap the position/velocity variance so confidence
        # stays interpretable and the filter cannot "give up" on the room.
        for i, cap in ((0, 25.0), (1, 25.0), (2, 9.0), (3, 9.0)):
            if self.P[i][i] > cap:
                self.P[i][i] = cap

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


EKF = {}        # tag_id -> TagEKF
EKF_REJECTS = {}  # tag_id -> consecutive rejected cycles


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


def geometry_check(fixes):
    """Detect range pairs that are geometrically impossible.

    For two anchors the triangle inequality must hold:
        |r1 - r2| <= d <= r1 + r2
    Violations mean at least one range is wrong (usually NLOS: the signal
    bounced, so the measured distance is longer than the straight line).
    Returns (ok, slack) where slack is how far outside the bound we are.
    """
    if len(fixes) < 2:
        return True, 0.0
    worst = 0.0
    for i in range(len(fixes)):
        for j in range(i + 1, len(fixes)):
            (x1, y1, r1), (x2, y2, r2) = fixes[i], fixes[j]
            d = math.hypot(x2 - x1, y2 - y1)
            if d < 1e-6:
                continue
            if r1 + r2 < d:
                worst = max(worst, d - (r1 + r2))
            elif abs(r1 - r2) > d:
                worst = max(worst, abs(r1 - r2) - d)
    return worst <= 0.0, worst


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

        # Self-healing: if every measurement keeps being rejected (heavy NLOS
        # reflections), the filter has lost the track and its covariance is at
        # the cap. Restart it from the current geometry instead of coasting on
        # a stale estimate forever.
        if used == 0:
            EKF_REJECTS[tag] = EKF_REJECTS.get(tag, 0) + 1
            if EKF_REJECTS[tag] >= 8:
                sol = solve_2d(fixes, room["width"], room["depth"])
                if sol:
                    EKF[tag] = TagEKF(sol["x"], sol["y"])
                    EKF_REJECTS[tag] = 0
                    used = sum(1 for (_a, ax, ay, _az, rng2d) in links
                               if EKF[tag].update_range(ax, ay, rng2d))
        else:
            EKF_REJECTS[tag] = 0

        px, py, vx, vy = EKF[tag].position()

        # Never report a position outside the room: with only two anchors an
        # inconsistent range pair can push the estimate far away (observed
        # x = -38 m). Clamp to the room rectangle and, if we had to clamp,
        # drop the velocity so the filter does not keep flying outward.
        margin = 0.5
        cx = min(max(px, -margin), room["width"] + margin)
        cy = min(max(py, -margin), room["depth"] + margin)
        if abs(cx - px) > 1e-6 or abs(cy - py) > 1e-6:
            EKF[tag].x[0], EKF[tag].x[1] = cx, cy
            EKF[tag].x[2] *= 0.2
            EKF[tag].x[3] *= 0.2
            EKF[tag].P[0][0] = max(EKF[tag].P[0][0], 0.5)
            EKF[tag].P[1][1] = max(EKF[tag].P[1][1], 0.5)
            px, py = cx, cy

        sigma = EKF[tag].sigma()

        ok_geo, slack = geometry_check(fixes)

        out[tag] = {
            "ts": now, "src": "server", "id": tag,
            "x": px, "y": py, "z": scene_mod.DEFAULT_TAG_Z,
            "vx": vx, "vy": vy,
            "confidence": 1.0 / (1.0 + sigma),
            "sigma": sigma, "ambiguous": False,
            "geometry_ok": ok_geo, "geometry_slack": slack,
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
                    # A brand-new anchor is placed automatically in the next
                    # free room corner, so it shows up in the 3D view at once.
                    # The operator can then drag it to the real position.
                    if role_guess == "anchor":
                        auto_place_anchor(did)
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
                            "geometry_ok": sol.get("geometry_ok", True),
                            "geometry_slack": sol.get("geometry_slack", 0.0),
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
            "last_seen": dc.get("last_seen", 0),
            **{k: STATUS.get(dk, {}).get(k) for k in ("ip", "rssi", "fw", "uptime_s")
               if STATUS.get(dk, {}).get(k) is not None}})

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
                     "geometry_ok": p.get("geometry_ok", True),
                     "geometry_slack": p.get("geometry_slack", 0.0),
                     "confidence": p["conf"], "ambiguous": p["amb"],
                     "online": now - p["ts"] < STALE_MS, "last_seen": p["ts"],
                     "source": p.get("src", "server"),
                     "ip": STATUS.get(tag, {}).get("ip"),
                     "rssi": STATUS.get(tag, {}).get("rssi"),
                     "fw": STATUS.get(tag, {}).get("fw"),
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

    devices = []
    for dk, dc in DEVICES.items():
        if dk == "_room" or dc.get("role") not in ("tag", "anchor"):
            continue
        st = STATUS.get(dk, {})
        devices.append({
            "id": dk, "role": dc.get("role"),
            "online": now - dc.get("last_seen", 0) < 5000,
            "last_seen": dc.get("last_seen", 0),
            "ip": st.get("ip"), "rssi": st.get("rssi"), "fw": st.get("fw"),
            "uptime_s": st.get("uptime_s"),
        })

    return {"site": BASE_CFG["site"], "ts": now, "room": room,
            "obstacles": obstacles,
            "nlos_enabled": NLOS_ENABLED,
            "anchors": anchors, "tags": tags, "links": links,
            "devices": devices}


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
    # geometry changed -> discard ranges measured against the old geometry
    with LOCK:
        RANGES.clear()
        NLOS.clear()
    EKF.clear()
    EKF_REJECTS.clear()
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

    # Anchors moved -> everything measured against the OLD geometry is invalid.
    # Keeping those ranges (they live up to STALE_MS) mixed old distances with
    # new anchor positions, which produced a bogus first fix and then the EKF
    # tracked that wrong point: the "tag drifts when I edit an anchor" report.
    with LOCK:
        RANGES.clear()
        NLOS.clear()
    EKF.clear()
    EKF_REJECTS.clear()
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



@APP.route("/api/v1/ota/check", methods=["GET"])
def api_ota_check():
    """The device asks whether a newer image is available.

    Query: device=<role-id>&key=<token>
    Reply: {"ok":true,"update":bool,"url":"...","version":"..."}

    Pull is the primary update path: the ESP32 web server cannot accept a ~1 MB
    multipart body (it buffers the request in RAM and drops it before the
    handler runs), so the device downloads the image instead.
    """
    dev = request.args.get("device", "")
    key = request.args.get("key", "")
    if not dev:
        return jsonify({"ok": False, "error": "device required"}), 400
    if not key or key != ota_token(dev):
        return jsonify({"ok": False, "error": "bad key"}), 401

    with LOCK:
        st = STATUS.get(dev, {})
    cur = st.get("fw") or ""

    fw = []
    if os.path.isdir(OTA_DIR):
        for f in os.listdir(OTA_DIR):
            if f.endswith(".bin"):
                p = os.path.join(OTA_DIR, f)
                fw.append({"name": f, "size": os.path.getsize(p),
                           "mtime": int(os.path.getmtime(p))})
        fw.sort(key=lambda x: x["mtime"], reverse=True)

    if not fw:
        return jsonify({"ok": True, "update": False, "reason": "no image on server"})

    # MANUAL MODE: an update is served only when one has been requested for
    # this device (POST /api/v1/ota/request). Checking is cheap and happens
    # every minute, but downloading must never happen on its own.
    with LOCK:
        pending = OTA_PENDING.get(dev)
    if not pending:
        return jsonify({"ok": True, "update": False, "reason": "no update requested",
                        "current": cur})

    name = pending["firmware"]
    path = os.path.join(OTA_DIR, name)
    if not os.path.isfile(path):
        with LOCK:
            OTA_PENDING.pop(dev, None)
        return jsonify({"ok": True, "update": False, "reason": "image missing"})

    ver = name.rsplit("-", 1)[-1].replace(".bin", "")
    # Only ever move FORWARD. A queued request for an older image (e.g. queued
    # while the device still ran that version, then the device was flashed by
    # hand) must not downgrade the node: 1.0.8 would have been replaced by
    # 1.0.7 here. Clear the stale request instead.
    if cur and not version_newer(ver, cur):
        with LOCK:
            OTA_PENDING.pop(dev, None)
        return jsonify({"ok": True, "update": False, "version": ver,
                        "reason": "already installed" if cur == ver else "not newer",
                        "current": cur})

    base = request.host_url.rstrip("/")
    return jsonify({"ok": True, "update": True, "version": ver,
                    "size": os.path.getsize(path),
                    "url": f"{base}/api/v1/ota/firmware/{name}",
                    "current": cur})


@APP.route("/api/v1/ota/request", methods=["POST"])
def api_ota_request():
    """Queue an update for one device (or all). The device installs it on its
    next check — nothing is downloaded until this is called.

    body: {"device_id": "anchor-2", "firmware": "uwb-node-1.0.7.bin"}
          {"device_id": "all",      "firmware": "..."}
          {"device_id": "anchor-2", "cancel": true}
    """
    body = request.get_json(silent=True) or {}
    target = body.get("device_id", "all")
    cancel = bool(body.get("cancel"))

    with LOCK:
        targets = [dk for dk, dc in DEVICES.items()
                   if dk != "_room" and dc.get("role") in ("tag", "anchor")
                   and (target == "all" or dk == target)]
        if not targets:
            return jsonify({"ok": False, "error": "no matching device"}), 404
        if cancel:
            for dk in targets:
                OTA_PENDING.pop(dk, None)
            return jsonify({"ok": True, "cancelled": targets})

        name = body.get("firmware")
        if not name or "/" in name or "\\" in name:
            return jsonify({"ok": False, "error": "bad firmware name"}), 400
        if not os.path.isfile(os.path.join(OTA_DIR, name)):
            return jsonify({"ok": False, "error": f"firmware not found: {name}"}), 404
        ver = name.rsplit("-", 1)[-1].replace(".bin", "")
        skipped, queued, already = [], [], []
        for dk in targets:
            cur = (STATUS.get(dk, {}) or {}).get("fw") or ""
            if cur and not version_newer(ver, cur):
                skipped.append(f"{dk} (runs {cur})")
                continue
            # Idempotent: pressing the button repeatedly must not build a queue.
            # A device already waiting for this exact image is left alone.
            pend = OTA_PENDING.get(dk)
            if pend and pend.get("firmware") == name:
                already.append(dk)
                continue
            OTA_PENDING[dk] = {"firmware": name, "requested": now_ms()}
            queued.append(dk)

        if skipped and not queued and not already:
            return jsonify({"ok": False, "firmware": name,
                            "error": f"not newer than the device firmware: {', '.join(skipped)}"}), 409

    print(f"[ota] queued {queued}, already pending {already}, skipped {skipped} ({name})")
    return jsonify({"ok": True, "queued": queued, "already": already,
                    "skipped": skipped, "firmware": name,
                    "pending": {dk: (OTA_PENDING.get(dk) or {}).get("firmware")
                                for dk in targets}})


@APP.route("/api/v1/ota/firmware/<path:name>", methods=["GET"])
def api_ota_firmware(name):
    """Serve a firmware image for the device to pull."""
    if "/" in name or "\\" in name or not name.endswith(".bin"):
        return jsonify({"ok": False, "error": "bad name"}), 400
    path = os.path.join(OTA_DIR, name)
    if not os.path.isfile(path):
        return jsonify({"ok": False, "error": "not found"}), 404
    return send_from_directory(OTA_DIR, name, mimetype="application/octet-stream")


@APP.route("/api/v1/ota/ack", methods=["POST"])
def api_ota_ack():
    """Device reports the outcome of a pull (the success case reboots, so this
    usually only arrives for failures)."""
    body = request.get_json(silent=True) or {}
    dev, key = body.get("device"), body.get("key")
    if not dev or not key or key != ota_token(dev):
        return jsonify({"ok": False, "error": "bad key"}), 401
    ok = bool(body.get("ok"))
    with LOCK:
        OTA_PENDING.pop(dev, None)     # one shot: never serve it twice
    print(f"[ota] {dev} reported {'success' if ok else 'failure'} "
          f"{body.get('version','')} {body.get('error','')}")
    return jsonify({"ok": True})


@APP.route("/api/v1/ota", methods=["GET"])
def api_ota_list():
    """Devices eligible for an update, their IP, key and the firmware available
    on the server. The key is shown so an operator can also curl the device
    directly; keep this endpoint behind --token on untrusted networks."""
    firmwares = []
    if os.path.isdir(OTA_DIR):
        for f in os.listdir(OTA_DIR):
            if not f.endswith(".bin"):
                continue
            p = os.path.join(OTA_DIR, f)
            firmwares.append({"name": f, "size": os.path.getsize(p),
                              "mtime": int(os.path.getmtime(p))})
        # newest build first, so the UI dropdown defaults to the latest image
        firmwares.sort(key=lambda x: x["mtime"], reverse=True)
    with LOCK:
        devs = []
        for dk, dc in DEVICES.items():
            if dk == "_room" or dc.get("role") not in ("tag", "anchor"):
                continue
            st = STATUS.get(dk, {})
            devs.append({"id": dk, "role": dc.get("role"),
                         "online": now_ms() - dc.get("last_seen", 0) < 5000,
                         "ip": st.get("ip"), "fw": st.get("fw"),
                         "key": ota_token(dk), "port": OTA_PORT,
                         "pending": (OTA_PENDING.get(dk) or {}).get("firmware")})
    return jsonify({"ok": True, "devices": devs, "firmware": firmwares,
                    "port": OTA_PORT})


@APP.route("/api/v1/ota/push", methods=["POST"])
def api_ota_push():
    """Push a firmware image to one device (or all online devices).

    body: {"device_id": "anchor-2", "firmware": "firmware.bin"}
          {"device_id": "all",      "firmware": "firmware.bin"}
    """
    body = request.get_json(silent=True) or {}
    name = body.get("firmware")
    target = body.get("device_id", "all")
    if not name or "/" in name or "\\" in name:
        return jsonify({"ok": False, "error": "bad firmware name"}), 400
    path = os.path.join(OTA_DIR, name)
    if not os.path.isfile(path):
        return jsonify({"ok": False, "error": f"firmware not found: {name}"}), 404

    with LOCK:
        targets = [dk for dk, dc in DEVICES.items()
                   if dk != "_room" and dc.get("role") in ("tag", "anchor")
                   and (target == "all" or dk == target)]
    if not targets:
        return jsonify({"ok": False, "error": "no matching device"}), 404

    results = {}
    for dk in targets:
        st = STATUS.get(dk, {})
        ip = st.get("ip")
        if not ip:
            results[dk] = "no ip (device has not reported its status yet)"
            continue
        key = ota_token(dk)
        url = f"http://{ip}:{OTA_PORT}/update?key={key}"
        try:
            import urllib.error
            import urllib.request
            with open(path, "rb") as fh:
                data = fh.read()
            # multipart/form-data so the firmware sees a normal file upload
            boundary = "----uwbota" + secrets.token_hex(8)
            body_bytes = (
                f"--{boundary}\r\n"
                f'Content-Disposition: form-data; name="firmware"; filename="{name}"\r\n'
                f"Content-Type: application/octet-stream\r\n\r\n"
            ).encode() + data + f"\r\n--{boundary}--\r\n".encode()
            req = urllib.request.Request(url, data=body_bytes, method="POST")
            req.add_header("Content-Type", f"multipart/form-data; boundary={boundary}")
            # No Expect: 100-continue — the ESP32 web server does not answer it
            # and the connection would be dropped mid-upload.
            req.add_header("Expect", "")
            req.add_header("Connection", "close")
            try:
                with urllib.request.urlopen(req, timeout=120) as resp:
                    results[dk] = f"{resp.status} {resp.read().decode(errors='replace')[:80]}"
            except urllib.error.HTTPError as he:
                results[dk] = f"http {he.code}: {he.read().decode(errors='replace')[:80]}"
            except Exception as e2:
                # A reset right at the end is normal: the node reboots as soon
                # as the image is written. Report it as "sent" so the operator
                # is not misled, and let the version check confirm it.
                if "10054" in str(e2) or "forcibly closed" in str(e2).lower() or "reset" in str(e2).lower():
                    results[dk] = "sent (device rebooted; verify with /api/v1/ota)"
                else:
                    results[dk] = f"failed: {e2}"
        except Exception as e:
            results[dk] = f"failed: {e}"

    ok = any(str(v).startswith("200") or str(v).startswith("sent") for v in results.values())
    return jsonify({"ok": ok, "results": results, "firmware": name})


@APP.route("/api/v1/ota/key", methods=["POST"])
def api_ota_key():
    """Rotate a device's key (the device picks the new one up on next config)."""
    body = request.get_json(silent=True) or {}
    did = body.get("device_id")
    if not did:
        return jsonify({"ok": False, "error": "device_id required"}), 400
    tok = ota_token(did, regenerate=True)
    # push the new config straight away
    role, _, num = did.partition("-")
    try:
        devcfg = merged_config(role or "anchor", int(num) if num.isdigit() else 1)
        mqtt_publish(cfg["mqtt"]["base_topic"] + f"/config/{did}",
                     json.dumps(devcfg), retained=True)
    except Exception:
        pass
    return jsonify({"ok": True, "device_id": did, "key": tok})


@APP.route("/api/v1/meta", methods=["GET"])
def api_meta():
    """Self-describing API list, so the web UI never hardcodes endpoints."""
    eps = []
    for rule in sorted(APP.url_map.iter_rules(), key=lambda r: str(r)):
        if not str(rule).startswith("/api/"):
            continue
        eps.append({
            "path": str(rule),
            "methods": sorted(m for m in rule.methods if m not in ("HEAD", "OPTIONS")),
            "doc": (APP.view_functions[rule.endpoint].__doc__ or "").strip().split("\n")[0],
        })
    return jsonify({"ok": True, "endpoints": eps, "fw_version": "1.0.0",
                    "ota_port": OTA_PORT, "sites": [BASE_CFG["site"]]})


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
    _load_ota_tokens()
    if args.mqtt:
        mqtt_start()

    APP.run(host="0.0.0.0", port=args.port, threaded=True)


if __name__ == "__main__":
    main()