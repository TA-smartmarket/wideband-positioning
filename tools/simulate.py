#!/usr/bin/env python3
"""Before/after simulation of the positioning pipeline on a known trajectory.

A synthetic walk (rectangle + diagonal, 0.8 m/s, pauses at the corners) is
measured by a model of the real system — DW1000 two-way ranging at the
library's poll rate, per-anchor range bias (uncalibrated antenna delay), noise,
NLOS outliers, body shadowing, packet loss, device clocks and REST delivery —
and fed to:

  A  legacy server  (the exact code of commit 1aff68c, loaded from git)
     + firmware 1.0.31 (1 Hz telemetry, filtered latest-range per link)
  B  new server (server/tracker.py) + firmware 1.0.31   <- server update only
  C  new server + firmware 1.0.32 (every raw sample, 5 Hz uplink)
  D  C + per-anchor range calibration (residual error 3 cm)

Because the truth is known here, the errors are real errors — this is what the
hardware test cannot give without a motion-capture system.

    python tools/simulate.py                # 20 Monte-Carlo runs + plot
    python tools/simulate.py --runs 5 --no-plot
"""
import argparse
import importlib.util
import json
import math
import random
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "server"))
import scene as scene_mod   # noqa: E402
import tracker              # noqa: E402

LEGACY_COMMIT = "1aff68c"

DEFAULT_SCENE = {
    "room": {"width": 4.5, "depth": 5.4, "height": 2.7},
    "anchors": [
        {"id": "anchor-4", "x": 0.0, "y": 0.0, "z": 2.2},
        {"id": "anchor-3", "x": 4.5, "y": 0.0, "z": 2.2},
        {"id": "anchor-5", "x": 4.5, "y": 5.4, "z": 2.2},
        {"id": "anchor-2", "x": 0.0, "y": 5.4, "z": 2.2},
    ],
    "obstacles": [], "tag_z": 1.0,
}

WAYPOINTS = [(0.9, 0.9), (0.9, 4.5), (3.6, 4.5), (3.6, 0.9), (0.9, 0.9), (3.6, 4.5)]
SPEED = 0.8          # m/s
PAUSE_S = 2.0        # at every waypoint
START_PAUSE_S = 4.0

# --- radio / error model -----------------------------------------------------
POLL_PERIOD_MS = 80 + 4 * 3 * 7      # DW1000Ranging timerDelay with 4 anchors
NOISE_M = 0.06                       # LOS TWR noise (1 sigma)
BIAS_COMMON = 0.20                   # uncalibrated antenna delay (m)
BIAS_SPREAD = 0.08                   # per-anchor variation (1 sigma)
OUTLIER_P = 0.04                     # multipath / NLOS outlier probability
OUTLIER_RANGE = (0.3, 1.5)           # outlier size (m, always positive)
BODY_BIAS = 0.15                     # carrier's body between tag and anchor
BODY_NOISE = 0.05
LOSS_P = 0.08                        # whole exchange lost
REPORT_LOSS_P = 0.03                 # only the RANGE_REPORT to the tag lost
CAL_RESIDUAL = 0.03                  # bias error left after calibration (1 sigma)


# ---------------------------------------------------------------------------
# truth
# ---------------------------------------------------------------------------

def build_truth():
    """Piecewise-linear truth: list of (t0, t1, p0, p1) segments in seconds."""
    segs = []
    t = 0.0
    p = WAYPOINTS[0]
    segs.append((t, t + START_PAUSE_S, p, p))
    t += START_PAUSE_S
    for q in WAYPOINTS[1:]:
        dur = math.dist(p, q) / SPEED
        segs.append((t, t + dur, p, q))
        t += dur
        segs.append((t, t + PAUSE_S, q, q))
        t += PAUSE_S
        p = q
    return segs


def truth_at(segs, t):
    for t0, t1, p0, p1 in segs:
        if t0 <= t <= t1:
            a = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
            return (p0[0] + a * (p1[0] - p0[0]), p0[1] + a * (p1[1] - p0[1]))
    return segs[-1][3]


def heading_at(segs, t):
    """Direction of travel (the carrier keeps facing it while paused)."""
    last = (0.0, 1.0)
    for t0, t1, p0, p1 in segs:
        if p0 != p1:
            d = math.dist(p0, p1)
            last = ((p1[0] - p0[0]) / d, (p1[1] - p0[1]) / d)
        if t <= t1:
            return last
    return last


def cross_track(segs, x, y):
    """Distance from (x, y) to the true path (polyline of moving segments)."""
    best = float("inf")
    for _t0, _t1, p0, p1 in segs:
        vx, vy = p1[0] - p0[0], p1[1] - p0[1]
        L2 = vx * vx + vy * vy
        a = 0.0 if L2 == 0 else max(0.0, min(1.0, ((x - p0[0]) * vx + (y - p0[1]) * vy) / L2))
        best = min(best, math.hypot(x - (p0[0] + a * vx), y - (p0[1] + a * vy)))
    return best


# ---------------------------------------------------------------------------
# firmware models
# ---------------------------------------------------------------------------

class FwRangeFilter:
    """Exact port of rfUpdate() in src/main.cpp (gate + median-of-3 + EMA)."""

    def __init__(self):
        self.hist, self.out, self.seeded = [], 0.0, False

    def update(self, raw):
        if self.seeded and abs(raw - self.out) > 5.0:
            return self.out
        s = sorted(self.hist + [raw])
        med = s[len(s) // 2]
        self.hist = (self.hist + [raw])[-3:]
        self.out = 0.35 * med + 0.65 * self.out if self.seeded else med
        self.seeded = True
        return self.out


def simulate_radio(scene, segs, rng, duration_s):
    """Every two-way-ranging exchange: list of dicts with true time (ms),
    anchor id, range value, and which side received it."""
    anchors = scene["anchors"]
    tag_z = scene["tag_z"]
    bias = {a["id"]: BIAS_COMMON + rng.gauss(0, BIAS_SPREAD) for a in anchors}
    out = []
    t = rng.uniform(0, POLL_PERIOD_MS)
    while t < duration_s * 1000:
        for i, a in enumerate(anchors):
            if rng.random() < LOSS_P:
                continue
            tm = t + 7 * (2 * i + 1) + 3
            px, py = truth_at(segs, tm / 1000)
            hx, hy = heading_at(segs, tm / 1000)
            d = math.sqrt((px - a["x"]) ** 2 + (py - a["y"]) ** 2 + (tag_z - a["z"]) ** 2)
            r = d + bias[a["id"]] + rng.gauss(0, NOISE_M)
            # body shadowing: the anchor is behind the person carrying the tag
            ux, uy = a["x"] - px, a["y"] - py
            un = math.hypot(ux, uy) or 1.0
            if (ux * hx + uy * hy) / un < -0.35:
                r += BODY_BIAS + abs(rng.gauss(0, BODY_NOISE))
            if rng.random() < OUTLIER_P:
                r += rng.uniform(*OUTLIER_RANGE)
            out.append({"t": tm, "anchor": a["id"], "r": r,
                        "tag_rx": rng.random() >= REPORT_LOSS_P})
        t += POLL_PERIOD_MS + rng.uniform(0, 10)
    return out, bias


def telemetry_stream(scene, exchanges, rng, duration_s, fw):
    """Telemetry payloads as the server receives them: list of
    (recv_ms, reporter, payload). fw = "1.0.31" or "1.0.32"."""
    tag = "tag-1"
    devices = [tag] + [a["id"] for a in scene["anchors"]]
    boot = {d: rng.uniform(0, 50_000) for d in devices}      # millis() offsets
    filt = {}                                               # (device, anchor) -> filter
    latest = {d: {} for d in devices}                       # device -> anchor -> rec
    fresh = {d: [] for d in devices}                        # unsent samples
    period = {tag: 200 if fw == "1.0.32" else 1000}
    for a in scene["anchors"]:
        period[a["id"]] = 1000
    next_send = {d: rng.uniform(0, period[d]) for d in devices}
    out = []
    ex_i = 0
    t = 0.0
    seq = {d: 0 for d in devices}
    while t < duration_s * 1000:
        t += 1.0
        while ex_i < len(exchanges) and exchanges[ex_i]["t"] <= t:
            ex = exchanges[ex_i]
            ex_i += 1
            for dev in (ex["anchor"], tag):
                if dev == tag and not ex["tag_rx"]:
                    continue
                f = filt.setdefault((dev, ex["anchor"]), FwRangeFilter())
                seq[dev] += 1
                rec = {"ts": int(ex["t"] + (4 if dev == tag else 0) + boot[dev]),
                       "src": ex["anchor"], "dst": tag,
                       "range": round(f.update(ex["r"]), 4), "raw": round(ex["r"], 4),
                       "rx_power": -60.0, "fp_power": -62.0, "seq": seq[dev]}
                latest[dev][ex["anchor"]] = rec
                fresh[dev].append(rec)
        for dev in devices:
            if t < next_send[dev]:
                continue
            next_send[dev] = t + period[dev] + 1
            if fw == "1.0.32":
                ranges, fresh[dev] = fresh[dev], []
            else:   # latest per link, raw value not sent
                ranges = [{k: v for k, v in r.items() if k not in ("raw", "seq")}
                          for r in latest[dev].values()]
            if not ranges:
                continue
            payload = {"device_id": dev, "ts": int(t + boot[dev]), "ranges": ranges}
            out.append((t + rng.uniform(20, 170), dev, payload))
    out.sort(key=lambda e: e[0])
    return out


# ---------------------------------------------------------------------------
# pipelines
# ---------------------------------------------------------------------------

def load_legacy(commit=LEGACY_COMMIT):
    """Import server/app.py + scene.py exactly as they were at `commit`."""
    tmp = Path(tempfile.mkdtemp(prefix="uwb_legacy_"))
    for name in ("app.py", "scene.py"):
        blob = subprocess.check_output(["git", "show", f"{commit}:server/{name}"], cwd=ROOT)
        (tmp / name).write_bytes(blob)
    saved = sys.modules.pop("scene", None)
    sys.path.insert(0, str(tmp))
    try:
        spec = importlib.util.spec_from_file_location("legacy_app", tmp / "app.py")
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
    finally:
        sys.path.remove(str(tmp))
        sys.modules.pop("scene", None)
        if saved is not None:
            sys.modules["scene"] = saved
    return mod


def run_legacy(legacy, scene, stream):
    clock = [0.0]
    legacy.now_ms = lambda: int(clock[0])
    for d in (legacy.DEVICES, legacy.RANGES, legacy.TAG_POS, legacy.EKF,
              legacy.EKF_REJECTS, legacy.NLOS, legacy.STATUS):
        d.clear()
    legacy.SCENE = legacy.scene_mod.normalise_scene(json.loads(json.dumps(scene)))
    track = []
    for recv, _dev, payload in stream:
        clock[0] = recv
        legacy.ingest_telemetry(json.loads(json.dumps(payload)), "rest")
        p = legacy.TAG_POS.get("tag-1")
        if p and p.get("src") == "server":
            track.append((recv, p["x"], p["y"]))
    return track


def run_new(scene, stream, calibration=None):
    eng = tracker.Engine()
    sc = json.loads(json.dumps(scene))
    if calibration:
        sc["calibration"] = {"anchors": calibration}
    eng.configure(sc, True, nlos_mod=scene_mod)
    track = []
    for recv, dev, payload in stream:
        tags = eng.ingest(dev, payload, recv)
        for tag in tags:
            est = eng.estimate(tag, recv)
            if est:
                track.append((recv, est["x"], est["y"]))
    return track, eng


# ---------------------------------------------------------------------------
# metrics
# ---------------------------------------------------------------------------

def metrics(segs, track, duration_s, warmup_s=2.0):
    """Zero-order-hold the displayed position, sample at 10 Hz, compare."""
    if not track:
        return None
    err, xt = [], []
    j = 0
    t = track[0][0] + warmup_s * 1000
    while t < duration_s * 1000:
        while j + 1 < len(track) and track[j + 1][0] <= t:
            j += 1
        _, x, y = track[j]
        tx, ty = truth_at(segs, t / 1000)
        err.append(math.hypot(x - tx, y - ty))
        xt.append(cross_track(segs, x, y))
        t += 100
    xs = sorted(xt)
    n_upd = len(track)
    span = (track[-1][0] - track[0][0]) / 1000 or 1
    return {
        "rmse": math.sqrt(sum(e * e for e in err) / len(err)),
        "xt_rmse": math.sqrt(sum(e * e for e in xt) / len(xt)),
        "xt_p95": xs[int(0.95 * (len(xs) - 1))],
        "xt_max": xs[-1],
        "rate": n_upd / span,
    }


def jumps(track, thresh=0.5):
    """Count displayed jumps larger than `thresh` metres between updates."""
    return sum(1 for a, b in zip(track, track[1:])
               if math.hypot(b[1] - a[1], b[2] - a[2]) > thresh)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", type=int, default=20)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--no-plot", action="store_true")
    ap.add_argument("--out", default=str(ROOT / "docs" / "img" / "sim_before_after.png"))
    args = ap.parse_args()

    scene = DEFAULT_SCENE
    segs = build_truth()
    duration = segs[-1][1] + 1.0

    legacy = load_legacy()
    names = {"A": "Lama: server 1aff68c + FW 1.0.31",
             "B": "Server baru + FW 1.0.31 (tanpa OTA)",
             "C": "Server baru + FW 1.0.32 (raw 5 Hz)",
             "D": "C + kalibrasi bias per anchor"}
    agg = {k: [] for k in names}
    jumps_agg = {k: 0 for k in names}
    first = None
    for run in range(args.runs):
        rng = random.Random(args.seed + run)
        ex, bias = simulate_radio(scene, segs, rng, duration)
        s_old = telemetry_stream(scene, ex, random.Random(rng.random()), duration, "1.0.31")
        s_new = telemetry_stream(scene, ex, random.Random(rng.random()), duration, "1.0.32")
        cal = {a: {"bias": b + rng.gauss(0, CAL_RESIDUAL), "scale": 1.0} for a, b in bias.items()}
        tracks = {
            "A": run_legacy(legacy, scene, s_old),
            "B": run_new(scene, s_old)[0],
            "C": run_new(scene, s_new)[0],
            "D": run_new(scene, s_new, cal)[0],
        }
        for k, tr in tracks.items():
            m = metrics(segs, tr, duration)
            if m:
                agg[k].append(m)
            jumps_agg[k] += jumps(tr)
        if first is None:
            first = tracks

    print(f"\n{args.runs} Monte-Carlo runs, walk {sum(math.dist(a, b) for a, b in zip(WAYPOINTS, WAYPOINTS[1:])):.1f} m "
          f"at {SPEED} m/s, room {scene['room']['width']} x {scene['room']['depth']} m, "
          f"{len(scene['anchors'])} anchors\n")
    hdr = f"{'pipeline':40s} {'RMSE':>7s} {'XT-RMSE':>8s} {'XT-P95':>7s} {'XT-max':>7s} {'Hz':>5s} {'jump>0.5m':>9s}"
    print(hdr)
    print("-" * len(hdr))
    summary = {}
    for k, name in names.items():
        ms = agg[k]
        avg = {f: sum(m[f] for m in ms) / len(ms) for f in ms[0]}
        summary[k] = avg
        print(f"{name:40s} {avg['rmse']:7.3f} {avg['xt_rmse']:8.3f} {avg['xt_p95']:7.3f} "
              f"{avg['xt_max']:7.3f} {avg['rate']:5.1f} {jumps_agg[k] / args.runs:9.1f}")
    print("\nRMSE    = error vs truth at the same instant (includes latency)")
    print("XT-*    = cross-track: distance to the true path (straightness)")
    print("jump    = displayed position moved > 0.5 m between two updates (per run)")

    if not args.no_plot:
        plot(scene, segs, first, names, summary, args.out)


def plot(scene, segs, tracks, names, summary, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 4, figsize=(18, 5.6), sharey=True)
    w, d = scene["room"]["width"], scene["room"]["depth"]
    for ax, (k, name) in zip(axes, names.items()):
        ax.add_patch(plt.Rectangle((0, 0), w, d, fill=False, lw=1, color="#888"))
        for a in scene["anchors"]:
            ax.plot(a["x"], a["y"], "s", color="#2a9d8f", ms=8)
        tx = [p[0] for p in WAYPOINTS]
        ty = [p[1] for p in WAYPOINTS]
        ax.plot(tx, ty, "-", color="#222", lw=2.2, label="lintasan asli", zorder=1)
        tr = tracks[k]
        ax.plot([p[1] for p in tr], [p[2] for p in tr], "-", color="#e76f51",
                lw=1.0, alpha=0.9, label="estimasi", zorder=2)
        ax.plot([p[1] for p in tr], [p[2] for p in tr], ".", color="#e76f51", ms=2.5, zorder=3)
        m = summary[k]
        ax.set_title(f"{name}\nXT-RMSE {m['xt_rmse'] * 100:.1f} cm · RMSE {m['rmse'] * 100:.1f} cm · "
                     f"{m['rate']:.1f} Hz", fontsize=9)
        ax.set_aspect("equal")
        ax.set_xlim(-0.4, w + 0.4)
        ax.set_ylim(-0.4, d + 0.4)
        ax.set_xlabel("x (m)")
        ax.grid(alpha=0.25)
    axes[0].set_ylabel("y (m)")
    axes[0].legend(loc="center", fontsize=8)
    fig.suptitle("Simulasi lintasan persegi + diagonal (0,8 m/s) — lintasan sama, error radio sama", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    Path(out).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=130)
    print(f"\nplot: {out}")


if __name__ == "__main__":
    main()
