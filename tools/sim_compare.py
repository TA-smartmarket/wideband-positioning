"""Simulate the measured setup and compare it with the hardware recordings.

    python tools/sim_compare.py [--runs 10]

Uses the radio/error model of tools/simulate.py with the scene of the 9.8 x
6.82 m recordings (same anchors, tag height and the ~82 cm common range bias
seen on the hardware). In the simulation the true position is known, so it
gives the accuracy the hardware cannot measure without ground truth, plus the
precision to check that the model behaves like the real system:

  * static: the tag held for 15 s at each point where the hardware was still
  * moving: a walk along the route of recording V2

Writes docs/laporan-akurasi/simulasi.json and the plots used by
tools/report_excel.py (sheet "Simulasi").
"""
import argparse
import json
import math
import random
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
import simulate as sim  # noqa: E402
import report_excel as rep  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "docs" / "laporan-akurasi"
LOGS = sorted((ROOT / "server" / "logs").glob("*ROOM9x6.8-V*.jsonl"))
STATIC_S = 15.0
WARMUP_S = 3.0
WALK = [(8.27, 0.60), (8.35, 5.40), (6.8, 6.05), (0.5, 6.15), (0.2, 3.9), (0.4, 0.8),
        (2.3, 0.9), (4.1, 3.3)]
WALK_SPEED = 0.6


def scene_from_log(path):
    meta, _, _ = rep.load(path)
    sc = json.loads(json.dumps(meta["scene"]))
    sc["obstacles"] = []
    return sc


def run(scene, waypoints, duration_s, seed, pause_s=2.0):
    sim.WAYPOINTS = waypoints
    sim.START_PAUSE_S = duration_s if len(waypoints) == 1 else 4.0
    sim.PAUSE_S = pause_s
    segs = sim.build_truth()
    end = segs[-1][1]
    rng = random.Random(seed)
    ex, _ = sim.simulate_radio(scene, segs, rng, end)
    stream = sim.telemetry_stream(scene, ex, rng, end, "1.0.32")
    track, _ = sim.run_new(scene, stream)
    return segs, track, end


def static_point(scene, x, y, runs):
    errs, prec95, prec50, means = [], [], [], []
    for k in range(runs):
        _, track, _ = run(scene, [(x, y)], STATIC_S, 1000 + k)
        pts = [(px, py) for t, px, py in track if t / 1000 >= WARMUP_S]
        if not pts:
            continue
        mx = sum(p[0] for p in pts) / len(pts)
        my = sum(p[1] for p in pts) / len(pts)
        d = [math.hypot(px - mx, py - my) for px, py in pts]
        errs += [math.hypot(px - x, py - y) for px, py in pts]
        prec50.append(rep.pct(d, 0.5))
        prec95.append(rep.pct(d, 0.95))
        means.append((mx, my))
    return {
        "mean_err": sum(errs) / len(errs),
        "rmse": math.sqrt(sum(e * e for e in errs) / len(errs)),
        "cep50": rep.pct(errs, 0.5), "cep95": rep.pct(errs, 0.95), "max": max(errs),
        "prec50": sum(prec50) / len(prec50), "prec95": sum(prec95) / len(prec95),
        "mean_x": sum(m[0] for m in means) / len(means),
        "mean_y": sum(m[1] for m in means) / len(means),
    }


def walk(scene, runs):
    res, example = [], None
    sim.SPEED = WALK_SPEED
    for k in range(runs):
        segs, track, end = run(scene, WALK, 0, 2000 + k)
        m = sim.metrics(segs, track, end)
        res.append(m)
        if example is None:
            example = (segs, track)
    avg = {key: sum(r[key] for r in res) / len(res) for key in res[0]}
    return avg, example


def plot_static(scene, points, out):
    fig, ax = plt.subplots(figsize=(8, 6))
    room = scene["room"]
    ax.add_patch(plt.Rectangle((0, 0), room["width"], room["depth"], fill=False, ec="#777"))
    for a in scene["anchors"]:
        ax.plot(a["x"], a["y"], "^", ms=11, color="#c62828")
        ax.annotate(a.get("label") or a["id"], (a["x"], a["y"]), xytext=(6, 6),
                    textcoords="offset points", fontsize=9, color="#c62828")
    for i, p in enumerate(points):
        ax.plot(p["x"], p["y"], "o", ms=9, mfc="none", mec="black",
                label="posisi benar (simulasi)" if i == 0 else None)
        ax.plot(p["sim"]["mean_x"], p["sim"]["mean_y"], "x", ms=8, color="#1565c0",
                label="rata-rata estimasi simulasi" if i == 0 else None)
        ax.annotate(f'{p["name"]}\nsim {p["sim"]["mean_err"] * 100:.1f} cm', (p["x"], p["y"]),
                    xytext=(8, -22), textcoords="offset points", fontsize=8)
    ax.set_aspect("equal")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.grid(alpha=0.3)
    ax.set_title("Simulasi titik statis pada posisi yang sama dengan pengukuran")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.1), ncol=2, fontsize=9)
    fig.tight_layout()
    fig.savefig(out, dpi=130, bbox_inches="tight")
    plt.close(fig)


def plot_walk(scene, segs, track, measured, out):
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(13, 5.4))
    room = scene["room"]
    truth = [segs[0][2]] + [s[3] for s in segs]
    for ax, title in ((a1, "Simulasi: lintasan benar vs estimasi"),
                      (a2, "Pengukuran V2 (hardware) vs lintasan simulasi")):
        ax.add_patch(plt.Rectangle((0, 0), room["width"], room["depth"], fill=False, ec="#777"))
        for a in scene["anchors"]:
            ax.plot(a["x"], a["y"], "^", ms=10, color="#c62828")
        ax.plot([p[0] for p in truth], [p[1] for p in truth], "-", color="black", lw=2,
                label="lintasan benar (simulasi)")
        ax.set_aspect("equal")
        ax.grid(alpha=0.3)
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_title(title, fontsize=11)
    a1.plot([p[1] for p in track], [p[2] for p in track], "-", color="#1565c0", lw=1,
            label="estimasi EKF (simulasi)")
    a2.plot([e["x"] for e in measured], [e["y"] for e in measured], "-", color="#e69500",
            lw=1.2, label="estimasi EKF (hardware V2)")
    for ax in (a1, a2):
        ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.12), ncol=2, fontsize=9)
    fig.tight_layout()
    fig.savefig(out, dpi=130, bbox_inches="tight")
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--runs", type=int, default=10)
    a = ap.parse_args()

    scene = scene_from_log(LOGS[0])
    sim.BIAS_COMMON = 0.82          # common bias measured on the hardware
    # The hardware server had been running, so its tracks already knew the
    # common bias; a cold start with 82 cm unexplained bias can wait a long
    # time for a consistent first fix. Start every simulated track the same way.
    init = sim.tracker.TagTrack.__init__

    def warm_init(self, *args, **kw):
        init(self, *args, **kw)
        self.bias = sim.BIAS_COMMON
    sim.tracker.TagTrack.__init__ = warm_init

    # points where the hardware was still, with its measured precision
    points = []
    for path in LOGS:
        meta, est, _ = rep.load(path)
        short = (meta.get("name") or path.stem).replace("SESSION-TEST-01-", "")
        for i, st in enumerate(rep.find_stops(est), 1):
            if len(st) < 3:
                continue
            ss = rep.stop_stats(st)
            points.append({"name": f"{short} S{i}", "x": ss["mx"], "y": ss["my"],
                           "hw_prec50": ss["cep50"], "hw_prec95": ss["cep95"],
                           "hw_sigma": ss["sigma"]})
    dash = json.loads((OUT / "uji_statis_dashboard.json").read_text(encoding="utf-8"))
    for t in dash["tests"]:
        points.append({"name": f"Dashboard {t['name']}", "x": t["x"], "y": t["y"],
                       "hw_prec50": None, "hw_prec95": None, "hw_sigma": t["sigma_cm"] / 100})

    for p in points:
        p["sim"] = static_point(scene, p["x"], p["y"], a.runs)
        print(f'{p["name"]:22s} sim err {p["sim"]["mean_err"] * 100:5.1f} cm  '
              f'prec95 sim {p["sim"]["prec95"] * 100:5.1f}  hw '
              f'{(p["hw_prec95"] or float("nan")) * 100:5.1f}')

    wm, (segs, track) = walk(scene, a.runs)
    print("walk", {k: round(v * 100, 1) if k != "rate" else round(v, 2) for k, v in wm.items()})

    _, measured, _ = rep.load(LOGS[0])
    plot_static(scene, points, OUT / "SIM_titik_statis.png")
    plot_walk(scene, segs, track, measured, OUT / "SIM_lintasan.png")

    (OUT / "simulasi.json").write_text(json.dumps({
        "runs": a.runs, "bias_common": sim.BIAS_COMMON, "noise": sim.NOISE_M,
        "outlier_p": sim.OUTLIER_P, "loss_p": sim.LOSS_P, "static_s": STATIC_S,
        "walk_speed": WALK_SPEED, "points": points, "walk": wm,
        "images": ["SIM_titik_statis.png", "SIM_lintasan.png"],
    }, indent=1), encoding="utf-8")
    print("wrote", OUT / "simulasi.json")


if __name__ == "__main__":
    main()
