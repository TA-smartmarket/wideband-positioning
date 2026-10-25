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
    GET/PUT /api/v1/tracking            the tracker (fixed) and its smoothing switch
    POST /api/v1/position               solve now (debug)
    GET  /api/v1/navigation/position    trolley position (navigation contract)
    GET  /api/v1/navigation/map         map metadata (dims from the scene)
    GET  /api/v1/navigation/scene       static obstacles (navigation contract)
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

import calibration as calib
import recorder
import scene as scene_mod
import tracker

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
    # OLED on the node: always on by default; the Setup checkbox switches to
    # "off after 60 s idle" to save power (the device stays online either way)
    "display": {"mode": "always", "timeout_s": 0},
}

DEVICES = {}          # device_id -> config dict (merged)
LOCK = threading.RLock()
STALE_MS = 5000       # device offline threshold
NAV_STALE_MS = 1500   # a navigation position older than this is not valid
# Experimental (docs/riset/RISET_LANJUTAN.md): --robust-round 0/1 pushes the
# tag's robust ranging round as uwb.robust_round. None = not pushed at all, so
# every node keeps its own setting (default off).
ROBUST_ROUND = None

RANGES = {}           # (anchor_id, tag_id) -> {"range","rx","fp","q","ts"}
TAG_POS = {}          # tag_id -> {"x","y","conf","amb","ts","src"}
STATUS = {}           # device_id -> status dict
SCENE = scene_mod.default_scene()   # room + anchors + obstacles (3D editor)
NLOS = {}             # (anchor_id, tag_id) -> {"blocked","atten","sigma","bias"}
NLOS_ENABLED = True   # inflate sigma / bias for blocked paths

# --- navigation integration (wideband-navigation contract) ------------------
# These constants mirror navigation_core/docs/data_contract.md + integration.md.
# The navigation core consumes processed positions (x_m, y_m, quality, valid)
# and a static scene envelope (scene.room + scene.obstacles) — never raw ranges.
NAV_SCHEMA_VERSION = 1
NAV_TROLLEY_ID = "TROLLEY_01"
NAV_FRAME_ID = "smart_market_map"
NAV_MAP_ID = "SMART_MARKET_MAIN"
NAV_MAP_VERSION = 1

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
# What happened to each requested update, for the progress shown in the UI:
# waiting (the node checks once a minute) -> downloading (it was told to) ->
# restarting (it went quiet or reported success) -> done (it reports the new
# version) | failed (it reported an error) | no-response (OTA_TIMEOUT_MS).
OTA_PROGRESS = {}     # device_id -> {"firmware", "version", "stage", "since", "requested", "error"}
OTA_TIMEOUT_MS = 4 * 60 * 1000


def ota_stage(dev, stage, **extra):
    p = OTA_PROGRESS.get(dev)
    if p is not None and p["stage"] != stage:
        p.update(stage=stage, since=now_ms(), **extra)


def ota_progress(dev):
    """The progress of dev's update, with the time-out applied (None: none)."""
    p = OTA_PROGRESS.get(dev)
    if not p:
        return None
    p = dict(p)
    if p["stage"] == "downloading" and now_ms() - (DEVICES.get(dev) or {}).get("last_seen", 0) > STALE_MS:
        p["stage"] = "restarting"            # quiet: writing the image / rebooting
    if p["stage"] in ("waiting", "downloading", "restarting") and now_ms() - p["requested"] > OTA_TIMEOUT_MS:
        p["error"] = f"no answer while {p['stage']}"
        p["stage"] = "no-response"
    return p


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


def device_fw_version(ip, timeout=5):
    """Read the version a device reports about itself (ground truth).

    The server's own STATUS entry is whatever the node last published, which
    goes stale the moment it reboots into a new image, so it cannot be used to
    confirm an update. The node's web page always shows the running version.
    """
    import re
    import urllib.request
    try:
        with urllib.request.urlopen(f"http://{ip}:{OTA_PORT}/", timeout=timeout) as r:
            html = r.read().decode(errors="replace")
    except Exception:
        return None
    m = re.search(r"firmware\s+(1\.0\.\d+)", html)
    return m.group(1) if m else None


def ota_token(device_id, regenerate=False):
    with LOCK:
        if regenerate or device_id not in OTA_TOKENS:
            OTA_TOKENS[device_id] = secrets.token_urlsafe(18)
            _save_ota_tokens()
        return OTA_TOKENS[device_id]


APP = Flask(__name__)


def now_ms():
    return int(time.time() * 1000)


def monotonic_ms():
    """Monotonic milliseconds for the navigation position contract.

    navigation's `timestamp_ms` is used only for staleness detection, so it
    must be a monotonic (uptime) clock rather than wall time — a clock change
    would otherwise age a fresh sample out instantly or keep a stale one alive.
    """
    return time.monotonic_ns() // 1_000_000


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


def _anchor_num(device_id):
    """'anchor-3' -> 3. The firmware keys its anchor map by this number, so the
    config has to carry it as an integer; 0 means 'no usable number'."""
    try:
        return int(str(device_id).split("-")[1])
    except (IndexError, ValueError):
        return 0


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
        # Server bookkeeping must never reach the device. `last_seen` changes
        # on every packet, so the config looked new on every 15-s poll and the
        # node rewrote its whole NVS each time. A flash write disables the
        # cache while the DW1000 IRQ handler is not in IRAM: anchors, which
        # take radio interrupts constantly, crashed every 1.5-3 minutes.
        for k in ("last_seen", "_auto"):
            cfg.pop(k, None)
        cfg["device_id"] = key
        cfg["role"] = role
        cfg["id"] = did
        # OTA credentials: the device needs its key to accept an update
        cfg["ota"] = {"enabled": True, "port": OTA_PORT, "token": ota_token(key)}
        # Height of the tag above the floor. The device projects its 3D ranges
        # against this when it solves locally, so it has to travel with the
        # config or the tag and the server would disagree about the same data.
        cfg["tag_z"] = float(SCENE.get("tag_z", scene_mod.DEFAULT_TAG_Z))
        # anchors[] array so a tag can solve locally too. Built from the SCENE,
        # which is the source of truth for where anchors are (the 3D editor
        # writes it), falling back to the device's own report. Building it from
        # DEVICES alone skipped every anchor that had been auto-placed but not
        # explicitly saved, so the map arrived empty and a tag could not solve.
        anchors = []
        seen_ids = set()
        for a in SCENE.get("anchors", []):
            aid = a.get("id")
            if not aid or aid in seen_ids:
                continue
            seen_ids.add(aid)
            anchors.append({"id": a.get("id_int", 0) or _anchor_num(aid),
                            "x": a["x"], "y": a["y"], "z": a.get("z", 2.2),
                            "device_id": aid})
        for dk, dc in DEVICES.items():
            if dc.get("role") != "anchor" or dk in seen_ids:
                continue
            pos = dc.get("position") or {}
            anchors.append({"id": dc.get("id"), "x": pos.get("x", 0.0),
                            "y": pos.get("y", 0.0), "z": pos.get("z", 2.2),
                            "device_id": dk})
        cfg["anchors"] = anchors
        if not known:
            # strip everything the device already knows locally — but tag_z is
            # not a credential, it is measurement data the device cannot derive,
            # so it must be sent even here. Without it a device seen only
            # through telemetry kept its NVS default and solved with the wrong
            # height.
            out = {"device_id": key, "role": role, "id": did, "anchors": anchors,
                   "tag_z": float(SCENE.get("tag_z", scene_mod.DEFAULT_TAG_Z)),
                   "ota": {"enabled": True, "port": OTA_PORT, "token": ota_token(key)}}
            if ROBUST_ROUND is not None:
                out["uwb"] = {"robust_round": ROBUST_ROUND}
            return out
        if ROBUST_ROUND is not None:
            cfg.setdefault("uwb", {})["robust_round"] = ROBUST_ROUND
        # never push empty credentials back — that would wipe the device
        if not cfg["wifi"]["ssid"]:
            cfg.pop("wifi", None)
        if not cfg["server"]["url"]:
            cfg.pop("server", None)
        if not cfg["mqtt"]["host"]:
            cfg.pop("mqtt", None)
        return cfg


# ---------------------------------------------------------------------------
# tracking — see server/tracker.py for the model and why it is built this way
#
# Every telemetry packet goes to ENGINE.ingest(), which uses each range sample
# exactly once, at the time it was measured, and runs the EKF on slant ranges
# with an online range-offset state. The old recompute-everything-on-every-
# packet loop (which re-used stale ranges as new evidence and zig-zagged) is
# gone; tools/simulate.py reproduces both for comparison.
# ---------------------------------------------------------------------------
ENGINE = tracker.Engine()
RECORDER = recorder.Recorder(os.path.join(os.path.dirname(__file__), "logs"))
CAL_JOBS = []          # active calibration/static-test collectors


def configure_engine(reset=False):
    """Load the current scene (anchors, room, tag height, calibration, tuning)
    into the tracker. reset=True when the geometry changed: every track built
    against the old geometry is wrong."""
    with LOCK:
        extra = {}
        for dk, dc in DEVICES.items():
            if dc.get("role") == "anchor" and not dc.get("_auto"):
                p = dc.get("position") or {}
                extra[dk] = (p.get("x", 0.0), p.get("y", 0.0), p.get("z", 2.2))
        ENGINE.configure(SCENE, NLOS_ENABLED, extra_anchors=extra, nlos_mod=scene_mod)
        if reset:
            ENGINE.reset()
            TAG_POS.clear()


def _on_sample(tag, s):
    """Engine hook: one range sample consumed (feeds calibration jobs)."""
    anchor, raw = s[1], s[2]
    for job in CAL_JOBS:
        if job["tag"] == tag:
            job["ranges"].setdefault(anchor, []).append(raw)


ENGINE.on_sample = _on_sample


def anchor_xyz(device_id):
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


def publish_estimate(tag, est, now):
    """Store one tracker estimate as the tag's current position."""
    TAG_POS[tag] = {"x": est["x"], "y": est["y"], "z": est["z"],
                    "vx": est["vx"], "vy": est["vy"],
                    "sigma": est["sigma"],
                    "range_bias": est["range_bias"],
                    "geometry_ok": est["geometry_ok"],
                    "geometry_slack": est["geometry_slack"],
                    "rate_hz": est["rate_hz"], "used": est["used"],
                    "conf": est["confidence"], "amb": est["ambiguous"],
                    "valid": est.get("valid", True), "held": est.get("held", False),
                    "still": est.get("still", False),
                    "anchors_fresh": est.get("anchors_fresh"),
                    "anchor_offsets": est.get("anchor_offsets", {}),
                    "offset_warning": est.get("offset_warning", []),
                    "links": est["links"],
                    "ts": now, "src": "server"}
    for a, info in est["links"].items():
        NLOS[(a, tag)] = {"blocked": info.get("blocked", False),
                          "atten": info.get("atten", 0.0),
                          "sigma": info.get("sigma"), "bias": info.get("bias"),
                          "obstacles": info.get("obstacles", [])}
    for job in CAL_JOBS:
        if job["tag"] == tag and not est.get("held", False):
            job["est"].append((est["x"], est["y"]))
    if RECORDER.active:
        RECORDER.write({"k": "est", "t": now, "tag": tag,
                        "x": round(est["x"], 4), "y": round(est["y"], 4),
                        "vx": round(est["vx"], 3), "vy": round(est["vy"], 3),
                        "sigma": round(est["sigma"], 4),
                        "b": round(est["range_bias"], 4), "used": est["used"],
                        "v": int(bool(est.get("valid", True))),
                        "h": int(bool(est.get("held", False))),
                        "st": int(bool(est.get("still", False)))})


def recompute_positions():
    """Current estimate of every tracked tag (debug endpoint)."""
    out = {}
    with LOCK:
        for tag in list(ENGINE.tracks):
            est = ENGINE.estimate(tag)
            if est:
                out[tag] = est
    return out


# ---------------------------------------------------------------------------
# ingest
# ---------------------------------------------------------------------------

def handle_status(device_id, st):
    with LOCK:
        STATUS[device_id] = dict(st, ts=now_ms())
        p = OTA_PROGRESS.get(device_id)
        if p and p["stage"] not in ("done", "failed") and st.get("fw") == p["version"]:
            ota_stage(device_id, "done")


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

        if not did or not cfg:
            # e.g. a bare per-range MQTT message: no reporter, no clock to map
            # its timestamp with, so it cannot be used for tracking
            return {}

        now = now_ms()
        if RECORDER.active:
            RECORDER.write({"k": "rx", "t": now, "dev": did, "p": payload})

        for r in payload.get("ranges", []):
            src, dst = r.get("src"), r.get("dst")
            if not src or not dst:
                continue
            tag = dst if src.startswith("anchor") else src
            anc = src if src.startswith("anchor") else dst
            # latest value per link, for the UI (the tracker keeps its own)
            RANGES[(anc, tag)] = {
                "range": r.get("raw", r.get("range", 0.0)),
                "rx": r.get("rx_power", 0.0),
                "fp": r.get("fp_power", 0.0), "q": r.get("quality", 0.0),
                "ts": r.get("ts", 0), "recv_ts": now,
            }
            if cfg.get("role") == "anchor":
                cfg.setdefault("position", BASE_CFG["position"])

        if any(a not in ENGINE.anchors for a in
               {r.get("src") for r in payload.get("ranges", [])
                if str(r.get("src", "")).startswith("anchor")}):
            configure_engine()            # a new anchor was just auto-placed

        touched = ENGINE.ingest(did, payload, now)
        positions = {}
        for tag in touched:
            est = ENGINE.estimate(tag, now)
            if est:
                positions[tag] = est
                publish_estimate(tag, est, now)

        for p in payload.get("positions", []):
            # The server track is authoritative; a device estimate is only
            # shown while the server has no track of that tag yet.
            tr = ENGINE.tracks.get(p.get("id"))
            if tr is not None and tr.f is not None:
                continue
            TAG_POS[p["id"]] = {"x": p["x"], "y": p["y"],
                                "vx": p.get("vx", 0.0), "vy": p.get("vy", 0.0),
                                "sigma": p.get("sigma", 0.0),
                                "conf": p.get("confidence", 0.5),
                                "amb": p.get("ambiguous", False),
                                "ts": now, "src": "device"}

        DEVICES[did]["last_seen"] = now

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
                     "z": p.get("z", scene_mod.DEFAULT_TAG_Z),   # solved value, already tag_z
                     "vx": p.get("vx", 0.0), "vy": p.get("vy", 0.0),
                     "sigma": p.get("sigma", 0.0),
                     "range_bias": p.get("range_bias"),
                     "rate_hz": p.get("rate_hz"), "used": p.get("used"),
                     "geometry_ok": p.get("geometry_ok", True),
                     "geometry_slack": p.get("geometry_slack", 0.0),
                     "confidence": p["conf"], "ambiguous": p["amb"],
                     "valid": p.get("valid", True) and now - p["ts"] < NAV_STALE_MS,
                     "held": p.get("held", False), "still": p.get("still", False),
                     "anchors_fresh": p.get("anchors_fresh"),
                     "anchor_offsets": p.get("anchor_offsets", {}),
                     "offset_warning": p.get("offset_warning", []),
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
            # firmware >= 1.0.32: why the node last restarted, and its lowest
            # free heap since boot (crash / leak diagnostics)
            "reset": st.get("reset"), "heap_min": st.get("heap_min"),
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
# Navigation position over MQTT (push transport)
#
# The trolley can consume positions two ways: HTTP pull (GET
# /api/v1/navigation/position) or MQTT push. This thread publishes the same
# navigation position-contract document to `<base>/navigation/position` at
# NAV_POS_PUSH_HZ. It never publishes `valid:false` samples: on a drop-out the
# navigation core treats a missing sample as stale and enters POSITION_LOST,
# which is safer than pushing a repeated stale value.
# ---------------------------------------------------------------------------
NAV_POS_PUSH_HZ = 10.0
NAV_POS_PUSH_INTERVAL_S = 1.0 / NAV_POS_PUSH_HZ
NAV_POS_TOPIC_SUFFIX = "/navigation/position"

_nav_pos_thread = None
_nav_pos_stop = threading.Event()


def _nav_position_push_loop():
    while not _nav_pos_stop.is_set():
        doc = _nav_position_doc()
        if doc is not None and doc["valid"]:
            mqtt_publish(cfg['mqtt']['base_topic'] + NAV_POS_TOPIC_SUFFIX,
                         json.dumps(doc))
        _nav_pos_stop.wait(NAV_POS_PUSH_INTERVAL_S)


def start_nav_position_push():
    global _nav_pos_thread
    if _nav_pos_thread is not None:
        return
    _nav_pos_thread = threading.Thread(target=_nav_position_push_loop,
                                       name="nav-position-push", daemon=True)
    _nav_pos_thread.start()
    print(f"[nav] publishing position to MQTT "
          f"'{cfg['mqtt']['base_topic']}{NAV_POS_TOPIC_SUFFIX}' @ {NAV_POS_PUSH_HZ} Hz")


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
    configure_engine(reset=True)
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
        incoming = body.get("scene", body)
        # The 3D editor sends room/anchors/obstacles only. Calibration and
        # tracking settings live in the same scene file but are edited through
        # their own endpoints, so an editor save must not wipe them.
        if isinstance(incoming, dict):
            incoming = dict(incoming)
            for k in ("calibration", "tracking"):
                if k not in incoming and k in SCENE:
                    incoming[k] = SCENE[k]
        SCENE = scene_mod.normalise_scene(incoming)
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
    configure_engine(reset=True)
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
        # tuning saved by older versions: only the smoothing setting remains
        SCENE["tracking"] = {"smooth": int(tracker.tracking_params(SCENE)["smooth"])}
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
    with LOCK:
        ota_stage(dev, "downloading")
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
                OTA_PROGRESS.pop(dk, None)
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
            OTA_PROGRESS[dk] = {"firmware": name, "version": ver, "stage": "waiting",
                                "since": now_ms(), "requested": now_ms(), "error": None}
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
        if ok:
            ota_stage(dev, "restarting")
        else:
            ota_stage(dev, "failed", error=str(body.get("error") or "the device reported a failure"))
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
                         "pending": (OTA_PENDING.get(dk) or {}).get("firmware"),
                         "progress": ota_progress(dk)})
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
                # The socket drops both when the node reboots after a good
                # write AND when the write aborts, so the reset alone proves
                # nothing. Confirm against the version the device reports
                # about itself once it is back up.
                results[dk] = f"connection dropped ({e2}); verifying..."
        except Exception as e:
            results[dk] = f"failed: {e}"

    # Verification pass: the node needs a few seconds to reboot, then its own
    # page is the only trustworthy source for the running version.
    wanted = version_tuple(name.split("-")[-1].replace(".bin", ""))
    for dk in targets:
        if not str(results.get(dk, "")).endswith("verifying..."):
            continue
        st = STATUS.get(dk, {})
        ip = st.get("ip")
        got = None
        if ip:
            for _ in range(12):
                time.sleep(5)
                got = device_fw_version(ip)
                if got and version_tuple(got) >= wanted:
                    break
        if got and version_tuple(got) >= wanted:
            results[dk] = f"installed {got}"
        elif got:
            results[dk] = (f"NOT installed: device is still {got} "
                           f"(write aborted; see the serial log)")
        else:
            results[dk] = "unreachable after the push — could not verify"

    ok = all(str(v).startswith("200") or str(v).startswith("installed")
             for v in results.values())
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
# calibration + static accuracy test
#
# Put the tag on a spot whose (x, y) you measured with the laser meter, keep it
# still, and call /calibration/point. The server collects every raw range for
# `seconds` and compares it with the true slant distance to each anchor
# (anchor x, y, z from the scene; tag height from the scene). That gives
#   * the range error per anchor  -> calibration (antenna delay, cable, etc.)
#   * the position error           -> a static accuracy test with real ground truth
# ---------------------------------------------------------------------------

def _median(v):
    s = sorted(v)
    n = len(s)
    return 0.0 if n == 0 else (s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2]))


def _quantile(v, q):
    s = sorted(v)
    return s[min(int(round(q * (len(s) - 1))), len(s) - 1)] if s else 0.0


def position_stats(est, x, y):
    """Static accuracy of estimates est=[(x, y)] around the truth (x, y)."""
    if not est:
        return None
    n = len(est)
    mx = sum(p[0] for p in est) / n
    my = sum(p[1] for p in est) / n
    err = [math.hypot(px - x, py - y) for px, py in est]
    spread = [math.hypot(px - mx, py - my) for px, py in est]
    return {"n": n, "mean_x": mx, "mean_y": my,
            "mean_error": math.hypot(mx - x, my - y),          # accuracy (bias)
            "std": math.sqrt(sum(d * d for d in spread) / n),   # precision
            "rmse": math.sqrt(sum(e * e for e in err) / n),
            "cep50": _quantile(err, 0.50), "cep95": _quantile(err, 0.95),
            "max": max(err)}


def _default_tag():
    with LOCK:
        live = [t for t, p in TAG_POS.items() if now_ms() - p.get("ts", 0) < STALE_MS]
        return (live or list(ENGINE.tracks) or ["tag-1"])[0]


def _scene_anchors():
    """{anchor id: (x, y, z)} from the floor plan."""
    return {a["id"]: (a["x"], a["y"], a.get("z", 2.2)) for a in SCENE.get("anchors", [])}


@APP.route("/api/v1/calibration", methods=["GET"])
def api_calibration_get():
    """Stored calibration points, the per-anchor correction in use, and
    "review": what Apply would do now (server/calibration.py fit, dry run)."""
    with LOCK:
        cal = SCENE.get("calibration", {})
        pts = cal.get("points") or []
        review = calib.fit(pts, _scene_anchors()) if pts else None
        return jsonify({"ok": True, "calibration": cal, "review": review,
                        "jobs": [{"tag": j["tag"], "until": j["until"]} for j in CAL_JOBS]})


@APP.route("/api/v1/calibration/point", methods=["POST"])
def api_calibration_point():
    """Collect at a measured spot: {"x":..,"y":..,"seconds":15,"store":true}.

    Blocks for `seconds`. Returns the range error per anchor and the position
    error (mean error, std, RMSE, CEP50/95). store=true keeps the point for
    /calibration/apply; store=false is a pure static accuracy test.
    """
    body = request.get_json(silent=True) or {}
    try:
        x, y = float(body["x"]), float(body["y"])
    except (KeyError, TypeError, ValueError):
        return jsonify({"ok": False, "error": "x and y (metres) are required"}), 400
    seconds = min(max(float(body.get("seconds", 15)), 3.0), 120.0)
    tag = body.get("tag") or _default_tag()
    label = str(body.get("label") or f"({x:.2f}, {y:.2f})")
    store = bool(body.get("store", True))

    job = {"tag": tag, "ranges": {}, "est": [], "until": now_ms() + int(seconds * 1000)}
    with LOCK:
        CAL_JOBS.append(job)
        if RECORDER.active:
            RECORDER.write({"k": "mark", "t": now_ms(), "label": label, "x": x, "y": y,
                            "phase": "static-start"})
    time.sleep(seconds)
    with LOCK:
        CAL_JOBS.remove(job)
        if RECORDER.active:
            RECORDER.write({"k": "mark", "t": now_ms(), "label": label, "x": x, "y": y,
                            "phase": "static-end"})
        tag_z = float(SCENE.get("tag_z", scene_mod.DEFAULT_TAG_Z))

    per_anchor = {}
    for aid, vals in sorted(job["ranges"].items()):
        pos = anchor_xyz(aid)
        if not pos or len(vals) < 3:
            continue
        ax, ay, az = pos
        true = math.sqrt((x - ax) ** 2 + (y - ay) ** 2 + (tag_z - az) ** 2)
        mean = sum(vals) / len(vals)
        med = _median(vals)
        per_anchor[aid] = {"n": len(vals), "true": round(true, 4),
                           "median": round(med, 4), "mean": round(mean, 4),
                           "std": round(math.sqrt(sum((v - mean) ** 2 for v in vals) / len(vals)), 4),
                           "error": round(med - true, 4)}
    if not per_anchor:
        return jsonify({"ok": False, "error": f"no ranges from {tag} in {seconds:.0f} s"}), 409

    point = {"label": label, "x": x, "y": y, "tag": tag, "tag_z": tag_z,
             "t": now_ms(), "seconds": seconds, "anchors": per_anchor}
    check = None
    if store:
        with LOCK:
            pts = SCENE.setdefault("calibration", {}).setdefault("points", [])
            pts.append(point)
            save_scene()
            # how it agrees with the points before it: shown at once in Setup
            check = calib.check_point(pts, len(pts) - 1, _scene_anchors())
    return jsonify({"ok": True, "stored": store, "point": point, "check": check,
                    "position": position_stats(job["est"], x, y)})


@APP.route("/api/v1/calibration/apply", methods=["POST"])
def api_calibration_apply():
    """Fit the per-anchor range correction from the stored points, safely
    (server/calibration.py): noisy and inconsistent ranges are left out, a
    scale only with 3+ consistent points, and the result is refused (409,
    nothing changes) when it would not locate the points themselves well."""
    with LOCK:
        cal = SCENE.setdefault("calibration", {})
        res = calib.fit(cal.get("points") or [], _scene_anchors())
        if res["refused"]:
            return jsonify({"ok": False, **res}), 409
        cal["anchors"] = res["anchors"]
        save_scene()
    configure_engine(reset=True)
    return jsonify({"ok": True, **res})


@APP.route("/api/v1/calibration/remove", methods=["POST"])
def api_calibration_remove():
    """Remove one stored point: {"point": n} (1 = the first one measured).
    The correction in use does not change until Apply."""
    body = request.get_json(silent=True) or {}
    with LOCK:
        pts = SCENE.setdefault("calibration", {}).setdefault("points", [])
        try:
            n = int(body["point"])
        except (KeyError, TypeError, ValueError):
            n = 0
        if not 1 <= n <= len(pts):
            return jsonify({"ok": False, "error": f"point must be 1..{len(pts)}"}), 400
        pts.pop(n - 1)
        save_scene()
        return jsonify({"ok": True, "points": len(pts)})


@APP.route("/api/v1/calibration/clear", methods=["POST"])
def api_calibration_clear():
    """Forget calibration: {"points":true,"anchors":true} (both by default)."""
    body = request.get_json(silent=True) or {}
    with LOCK:
        cal = SCENE.setdefault("calibration", {})
        if body.get("points", True):
            cal["points"] = []
        if body.get("anchors", True):
            cal["anchors"] = {}
        save_scene()
    configure_engine(reset=True)
    return jsonify({"ok": True, "calibration": SCENE["calibration"]})


@APP.route("/api/v1/tracking", methods=["GET", "PUT"])
def api_tracking():
    """The tracker: one fixed algorithm (server/tracker.py). The only setting is
    PUT {"smooth": true|false}, the smoothed path; "calibrated" tells whether
    the range offsets come from the static calibration or are learned."""
    if request.method == "PUT":
        body = request.get_json(silent=True) or {}
        other = sorted(k for k in body if k != "smooth")
        if other or "smooth" not in body:
            return jsonify({"ok": False, "error": "only 'smooth' can be set; the algorithm "
                            f"is fixed (got {', '.join(other) or 'nothing'})"}), 400
        with LOCK:
            SCENE["tracking"] = {"smooth": 1 if body["smooth"] else 0}
            save_scene()
        configure_engine()
    with LOCK:
        params = tracker.tracking_params(SCENE)
        return jsonify({"ok": True, "smooth": params["smooth"] >= 0.5,
                        "calibrated": ENGINE.calibrated, "params": params})


# ---------------------------------------------------------------------------
# session recording (server/logs/*.jsonl, analysed by tools/evaluate.py)
# ---------------------------------------------------------------------------

@APP.route("/api/v1/record", methods=["GET"])
def api_record_status():
    """Recorder status and the recorded sessions."""
    return jsonify({"ok": True, **RECORDER.status(), "files": RECORDER.list()})


@APP.route("/api/v1/record/start", methods=["POST"])
def api_record_start():
    """Start recording: {"name":"garis-AB","note":"jalan lurus A->B"}."""
    body = request.get_json(silent=True) or {}
    with LOCK:
        path = RECORDER.start(body.get("name", "sesi"), body.get("note", ""),
                              copy.deepcopy(SCENE),
                              {"nlos_enabled": NLOS_ENABLED,
                               "tracking": tracker.tracking_params(SCENE)},
                              t=now_ms())
    print(f"[record] -> {path}")
    return jsonify({"ok": True, **RECORDER.status()})


@APP.route("/api/v1/record/stop", methods=["POST"])
def api_record_stop():
    """Stop recording."""
    with LOCK:
        path = RECORDER.stop()
    return jsonify({"ok": True, "file": os.path.basename(path) if path else None})


@APP.route("/api/v1/record/mark", methods=["POST"])
def api_record_mark():
    """Marker in the recording: {"label":"di titik A","x":0.5,"y":1.0}."""
    body = request.get_json(silent=True) or {}
    if not RECORDER.active:
        return jsonify({"ok": False, "error": "not recording"}), 409
    mark = {"k": "mark", "t": now_ms(), "label": str(body.get("label", "mark"))}
    for k in ("x", "y"):
        if k in body:
            try:
                mark[k] = float(body[k])
            except (TypeError, ValueError):
                pass
    with LOCK:
        RECORDER.write(mark)
    return jsonify({"ok": True, "mark": mark})


@APP.route("/api/v1/record/file/<path:name>", methods=["GET"])
def api_record_file(name):
    """Download one recorded session."""
    if "/" in name or "\\" in name or not name.endswith(".jsonl"):
        return jsonify({"ok": False, "error": "bad name"}), 400
    return send_from_directory(RECORDER.dir, name, mimetype="application/x-ndjson",
                               as_attachment=True)


# ---------------------------------------------------------------------------
# navigation integration (wideband-navigation contract)
#
# The trolley (wideband-navigation) consumes the *processed* position and the
# *static* scene, never raw ranges. Position is polled periodically (~10 Hz
# target); the scene is fetched once per navigation session / map change.
# ---------------------------------------------------------------------------

def _nav_select_tag(requested):
    """Pick the tag that becomes `TROLLEY_01`. An explicit `?tag=` wins; with
    several online tags the most recent one is used, otherwise the first."""
    with LOCK:
        now = now_ms()
        entries = list(TAG_POS.items())

    if requested:
        for tag, p in entries:
            if tag == requested:
                return tag, p
        return None, None

    # Prefer the tag with the freshest position that is still considered online.
    fresh = [(tag, p) for tag, p in entries if now - p.get("ts", 0) < STALE_MS]
    if fresh:
        return max(fresh, key=lambda item: item[1].get("ts", 0))
    if entries:
        return entries[0]
    return None, None


def _nav_quality(p):
    """EKF confidence clamped to [0, 1] and made JSON-finite."""
    try:
        quality = float(p.get("conf", 0.0))
    except (TypeError, ValueError):
        quality = 0.0
    if not math.isfinite(quality):
        quality = 0.0
    return min(max(quality, 0.0), 1.0)


def _nav_position_doc(requested=None, timestamp_ms=None):
    """Build the navigation position-contract dict (or None when no tag exists).

    Shared by the REST endpoint and the periodic MQTT push so both transports
    emit byte-identical payloads. `timestamp_ms` defaults to a fresh monotonic
    stamp when not supplied (the MQTT publisher batches one stamp per publish).
    """
    tag, p = _nav_select_tag(requested)
    if tag is None:
        return None

    # valid only while the tracker could fix the position (3+ anchors, small
    # sigma) and it is recent: a held or stale position must not steer
    online = now_ms() - p.get("ts", 0) < NAV_STALE_MS and p.get("valid", True)
    return {
        "schema_version": NAV_SCHEMA_VERSION,
        "trolley_id": NAV_TROLLEY_ID,
        "frame_id": NAV_FRAME_ID,
        "timestamp_ms": timestamp_ms if timestamp_ms is not None else monotonic_ms(),
        "position": {"x_m": p["x"], "y_m": p["y"]},
        "quality": _nav_quality(p),
        "valid": online,
    }


def _nav_map_doc():
    """Map metadata in the navigation /map contract, sourced from the scene."""
    with LOCK:
        room = dict(SCENE["room"])
    return {
        "map_id": NAV_MAP_ID,
        "map_version": NAV_MAP_VERSION,
        "frame_id": NAV_FRAME_ID,
        "width_m": room["width"],
        "height_m": room["depth"],
        "origin_x_m": 0.0,
        "origin_y_m": 0.0,
    }


def _nav_scene_doc():
    """Static obstacle scene in the navigation scene contract (rot in radians)."""
    with LOCK:
        room = dict(SCENE["room"])
        obstacles = list(SCENE.get("obstacles", []))

    out_obstacles = []
    for ob in obstacles:
        # Positioning stores `rot` in degrees (scene.py). Navigation's obstacle
        # parser reads `rot` directly as radians, so convert here at the
        # integration boundary. This keeps positioning's stored unit and the
        # UI (which labels the field "Rotation (°)") unchanged.
        try:
            rot_rad = math.radians(float(ob.get("rot", 0.0)))
        except (TypeError, ValueError):
            rot_rad = 0.0
        out_obstacles.append({
            "id": ob.get("id"),
            "label": ob.get("label", ""),
            "x": ob.get("x", 0.0),
            "y": ob.get("y", 0.0),
            "z": ob.get("z", 0.0),
            "sx": ob.get("sx", 0.0),
            "sy": ob.get("sy", 0.0),
            "sz": ob.get("sz", 0.0),
            "rot": rot_rad,
            "atten": ob.get("atten", 1.0),
        })

    return {
        "schema_version": NAV_SCHEMA_VERSION,
        "map_id": NAV_MAP_ID,
        "map_version": NAV_MAP_VERSION,
        "frame_id": NAV_FRAME_ID,
        "scene": {
            "room": {"width": room["width"], "depth": room["depth"]},
            "obstacles": out_obstacles,
        },
        "nlos_enabled": NLOS_ENABLED,
    }


@APP.route("/api/v1/navigation/position", methods=["GET"])
def api_navigation_position():
    """Current trolley position in the navigation position contract.

    Response (single JSON object, no envelope):
        {"schema_version":1,"trolley_id":"TROLLEY_01","frame_id":"smart_market_map",
         "timestamp_ms":<monotonic ms>,"position":{"x_m":..,"y_m":..},
         "quality":..,"valid":bool}
    `valid` is false when no tag is online (position is stale/missing), so the
    navigation core rejects it instead of driving on a dead estimate.
    """
    doc = _nav_position_doc(request.args.get("tag"))
    if doc is None:
        return jsonify({"ok": False,
                        "error": "no tag position available"}), 404
    return jsonify(doc)


@APP.route("/api/v1/navigation/map", methods=["GET"])
def api_navigation_map():
    """Map metadata (dimensions/version/frame) sourced from the 3D scene.

    navigation loads this once at startup so its map rectangle always matches
    the positioning room instead of being hard-coded in config/map.json.
    """
    return jsonify(_nav_map_doc())


@APP.route("/api/v1/navigation/scene", methods=["GET"])
def api_navigation_scene():
    """Static obstacle scene in the navigation scene contract.

    Mirrors the GET /api/v1/scene envelope but drops the positioning-only
    fields (anchors, tag_z, device_positions) and adds `map_id` /
    `map_version` / `frame_id` so navigation can version-check the map before
    enabling motion. `rot` is converted to radians here (positioning stores
    degrees; navigation's parser reads radians).
    """
    return jsonify(_nav_scene_doc())


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
    ap.add_argument("--robust-round", type=int, choices=(0, 1), default=None,
                    help="experimental: push the tag's robust ranging round (1 on, 0 off); "
                         "not given = not pushed, nodes keep their own setting")
    args = ap.parse_args()

    global ROBUST_ROUND
    ROBUST_ROUND = None if args.robust_round is None else bool(args.robust_round)
    cfg["port"] = args.port
    cfg["token"] = args.token
    cfg["mqtt"]["host"] = args.mqtt_host
    cfg["mqtt"]["port"] = args.mqtt_port
    cfg["mqtt"]["base_topic"] = args.mqtt_base

    print(f"[server] http://0.0.0.0:{args.port}" + ("  (token auth)" if args.token else ""))
    load_scene()
    configure_engine(reset=True)
    _load_ota_tokens()
    if args.mqtt:
        mqtt_start()
        start_nav_position_push()

    APP.run(host="0.0.0.0", port=args.port, threaded=True)


if __name__ == "__main__":
    main()