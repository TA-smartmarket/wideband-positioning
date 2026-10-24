"""Windowing study: every pre/post window combination on the same data.

    python tools/smoothing_study.py                 # full grid, 8 runs
    python tools/smoothing_study.py --runs 3 --quick

Stage 1 ("pre") filters each anchor's ranges before the EKF, stage 2 ("post")
filters the reported position after it (server/tracker.py, DEFAULT_TRACKING).
Every combination is run on identical simulated measurements — same walk,
same noise, same outliers — so the differences come from the filters alone.

Scenarios (radio/error model of tools/simulate.py):
  lab      4.5 x 5.4 m room, rectangle + diagonal at 0.8 m/s, 2 s stops
  market   9.5 x 5.9 m room with the anchors of the 23-10 recordings, tag at
           1.0 m, rectangle + diagonal at 0.6 m/s, 6 s stops (shelves), and the
           hardware's ranging rate: ~2.5 samples per anchor per second
  dropout  market, but from 20 s to 26 s only anchor-5 is heard (the
           "hallucination" case seen in recording test-4)
  market-los  market without body shadowing and outliers (line of sight)

The pre x post grid runs on the current tracker (DEFAULT_TRACKING); the
named algorithms A-G (NAMED) are what docs/hasil-simulasi compares.

Metrics, from the displayed position sampled at 10 Hz against the truth:
  MAE / RMSE     mean absolute / root-mean-square error (moving and still apart)
  XT             cross-track RMSE: distance to the true path (straightness)
  jitter         RMS second difference of consecutive outputs (line stipple)
  avail          share of the time a valid position is available
  drop-max       dropout: largest error among outputs flagged valid

Writes docs/img/smoothing_study.png and docs/smoothing_study.json.
"""
import argparse
import json
import math
import random
import sys
from multiprocessing import Pool
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import simulate as sim  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent

MARKET = {
    "room": {"width": 9.5, "depth": 5.9, "height": 2.0},
    "anchors": [
        {"id": "anchor-2", "x": 0.0, "y": 0.0, "z": 1.74},
        {"id": "anchor-3", "x": 0.0, "y": 5.9, "z": 1.74},
        {"id": "anchor-4", "x": 9.5, "y": 5.9, "z": 1.74},
        {"id": "anchor-5", "x": 9.5, "y": 0.0, "z": 2.2},
    ],
    "obstacles": [], "tag_z": 1.0,
}
MARKET_WALK = [(1.0, 1.0), (8.5, 1.0), (8.5, 4.9), (1.0, 4.9), (1.0, 1.0), (4.75, 2.95)]
DROPOUT = (20.0, 26.0, "anchor-5")
# One ranging round over all anchors every 400 ms: 2.5 samples per anchor per
# second, as measured on the 16-10 and 23-10 recordings (2.0-2.8 Hz).
HW_POLL_MS = 400

SCENARIOS = {
    "lab": dict(scene=sim.DEFAULT_SCENE, walk=sim.WAYPOINTS, speed=0.8, pause=2.0, start=4.0),
    "market": dict(scene=MARKET, walk=MARKET_WALK, speed=0.6, pause=6.0, start=6.0,
                   poll_ms=HW_POLL_MS),
    "dropout": dict(scene=MARKET, walk=MARKET_WALK, speed=0.6, pause=6.0, start=6.0,
                    poll_ms=HW_POLL_MS, dropout=DROPOUT),
    # line of sight everywhere: no body shadowing, no multipath outliers —
    # what is left is ranging noise and (uncalibrated) antenna delay
    "market-los": dict(scene=MARKET, walk=MARKET_WALK, speed=0.6, pause=6.0, start=6.0,
                       poll_ms=HW_POLL_MS, los=True),
}

PRE = {"none": ("none", 1), "median3": ("median", 3), "median5": ("median", 5),
       "mean5": ("mean", 5), "linfit5": ("linfit", 5), "median20": ("median", 20),
       "mean20": ("mean", 20), "linfit20": ("linfit", 20)}
POST = {"none": ("none", 1), "mean5": ("mean", 5), "median5": ("median", 5),
        "linfit5": ("linfit", 5), "linfit10": ("linfit", 10), "mean20": ("mean", 20),
        "linfit20": ("linfit", 20),
        "still20": ("still", 20)}
QUICK_PRE = ("none", "median3", "mean5", "linfit5", "mean20")
QUICK_POST = ("none", "mean5", "linfit10", "mean20", "still20")


def build_segs(walk, speed, pause, start):
    segs, t, p = [], 0.0, walk[0]
    segs.append((t, t + start, p, p))
    t += start
    for q in walk[1:]:
        dur = math.dist(p, q) / speed
        segs.append((t, t + dur, p, q))
        t += dur
        segs.append((t, t + pause, q, q))
        t += pause
        p = q
    return segs


def moving_at(segs, t):
    for t0, t1, p0, p1 in segs:
        if t0 <= t <= t1:
            return p0 != p1, t - t0
    return False, 99.0


# Tracker settings layered on top of a variant's windows.
EXTRA = {
    # the tracker as it was on main before this work: no validity gate, no
    # per-anchor offsets, symmetric Huber
    "old": {"min_anchors": 1, "valid_sigma": 99.0, "fresh_ms": 1e9,
            "anchor_offsets": 0, "huber_k_long": 0.0},
    # step 1 of this work: windows + validity gate, still the 5-state EKF
    "v1": {"anchor_offsets": 0, "huber_k_long": 0.0},
    # predict the output to "now" (packet arrival), at most 300 ms
    "x300": {"extrapolate_ms": 300},
}

# The algorithms compared in docs/hasil-simulasi (key -> label). "fable-MA500"
# is computed from the "old" track; every other key is pre|post|extra.
NAMED = [
    ("none|none|old", "A. EKF saja (main sebelum)"),
    ("fable-MA500", "B. A + rata-rata 500 ms (Fable)"),
    ("mean20|mean20|old", "C. A + jendela 20 x 20 (catatan bimbingan)"),
    ("median3|none|old", "D. A + median 3 pada jarak"),
    ("none|linfit10|old", "E. A + garis regresi 10 pada posisi"),
    ("none|still20|v1", "F. A + jendela adaptif + gerbang validitas"),
    ("none|still20", "G. F + offset per anchor + bobot jarak panjang (final)"),
]
# variants beyond the pre x post grid (which runs on the final tracker)
EXTRA_VARIANTS = [("mean20", "mean20", "old"), ("median3", "none", "old"),
                  ("none", "linfit10", "old"), ("none", "still20", "v1"),
                  ("none", "still20", "x300")]


def tracking(pre, post, extra=None):
    pf, pw = PRE[pre]
    qf, qw = POST[post]
    t = {"pre_filter": pf, "pre_window": pw, "post_filter": qf, "post_window": qw}
    t.update(EXTRA.get(extra, {}))
    return t


def score(segs, track, duration, dropout=None, warmup=3.0):
    """Metrics of one displayed track (list of (ms, x, y, valid))."""
    if not track:
        return None
    mov, still, xt, sync, valid_n, n = [], [], [], [], 0, 0
    stops = {}                                 # stop start -> [(x, y)]
    still_flag = []
    drop_max = 0.0
    j = 0
    t = track[0][0] + warmup * 1000
    while t < duration * 1000:
        while j + 1 < len(track) and track[j + 1][0] <= t:
            j += 1
        _ms, x, y, ok, t_f = track[j][:5]
        sx, sy = sim.truth_at(segs, t_f / 1000)
        age = t - track[j][0]
        ok = ok and age <= 1500
        tx, ty = sim.truth_at(segs, t / 1000)
        e = math.hypot(x - tx, y - ty)
        moving, since = moving_at(segs, t / 1000)
        n += 1
        valid_n += ok
        if moving:
            mov.append(e)
            sync.append(math.hypot(x - sx, y - sy))
            xt.append(sim.cross_track(segs, x, y))
        elif since > 1.0:                     # settled at a stop
            still.append(e)
            stops.setdefault(round(t / 1000 - since, 1), []).append((x, y))
            still_flag.append(bool(track[j][5]) if len(track[j]) > 5 else False)
        if dropout and ok and dropout[0] <= t / 1000 <= dropout[1] + 3.0:
            drop_max = max(drop_max, e)
        t += 100
    allerr = mov + still
    dev = []                                   # spread around each stop's own mean
    for pts in stops.values():
        mx = sum(q[0] for q in pts) / len(pts)
        my = sum(q[1] for q in pts) / len(pts)
        dev += [math.hypot(q[0] - mx, q[1] - my) for q in pts]
    jit = sim.jitter(track)
    rms = (lambda v: math.sqrt(sum(e * e for e in v) / len(v)) if v else float("nan"))
    avg = (lambda v: sum(v) / len(v) if v else float("nan"))
    p95 = (lambda v: sorted(v)[int(0.95 * (len(v) - 1))] if v else float("nan"))
    return {"mae": avg(allerr), "rmse": rms(allerr), "mae_move": avg(mov),
            "sync_move": avg(sync), "mae_still": avg(still),
            "p95_still": p95(still), "prec50": sorted(dev)[len(dev) // 2] if dev else float("nan"),
            "prec95": p95(dev), "still_det": avg(still_flag) if still_flag else float("nan"),
            "xt": rms(xt), "jitter": jit, "avail": valid_n / n,
            "drop_max": drop_max}


def fable_smooth(track, win_ms=500):
    """The output smoothing Fable added on 22-10 (uncommitted, replaced):
    mean of the filter positions of the last 500 ms, shifted along the EKF
    velocity by the window's mean age. Applied to an ungated track."""
    out, hist = [], []
    for row in track:
        recv, x, y, ok, t, st, vx, vy = row[:8]
        if not hist or t > hist[-1][0]:
            hist.append((t, x, y))
        while t - hist[0][0] > win_ms:
            del hist[0]
        n = len(hist)
        age = (t - sum(h[0] for h in hist) / n) / 1000.0
        out.append((recv, sum(h[1] for h in hist) / n + vx * age,
                    sum(h[2] for h in hist) / n + vy * age, ok, t, st, vx, vy))
    return out


def one_run(job):
    """All variants on one simulated data set (scenario, seed, calibrated)."""
    name, seed, variants, calibrated = job
    sc = SCENARIOS[name]
    scene = sc["scene"]
    sim.BODY_BIAS, sim.OUTLIER_P = (0.0, 0.0) if sc.get("los") else (0.15, 0.04)
    sim.POLL_PERIOD_MS = sc.get("poll_ms", 80 + 4 * 3 * 7)
    segs = build_segs(sc["walk"], sc["speed"], sc["pause"], sc["start"])
    duration = segs[-1][1] + 1.0
    rng = random.Random(seed)
    ex, bias = sim.simulate_radio(scene, segs, rng, duration)
    if sc.get("dropout"):
        t0, t1, keep = sc["dropout"]
        ex = [e for e in ex if not (t0 * 1000 <= e["t"] <= t1 * 1000 and e["anchor"] != keep)]
    stream = sim.telemetry_stream(scene, ex, random.Random(rng.random()), duration, "1.0.32")
    cal = None
    if calibrated:
        cal = {a: {"bias": b + rng.gauss(0, sim.CAL_RESIDUAL), "scale": 1.0}
               for a, b in bias.items()}
    out = {}
    variants = list(variants) + [("none", "none", "old")]
    for v in variants:
        track, _eng = sim.run_new(scene, stream, cal, tracking(*v))
        out["|".join(v)] = score(segs, track, duration, sc.get("dropout"))
        if v == ("none", "none", "old"):
            out["fable-MA500"] = score(segs, fable_smooth(track), duration, sc.get("dropout"))
    return name, calibrated, out


def aggregate(results):
    agg = {}
    for name, calibrated, out in results:
        for key, m in out.items():
            if m is None:
                continue
            d = agg.setdefault((name, calibrated, key), {})
            for f, v in m.items():
                if not (isinstance(v, float) and math.isnan(v)):
                    d.setdefault(f, []).append(v)
    return {k: {f: sum(v) / len(v) for f, v in d.items()} for k, d in agg.items()}


# variants shown in docs/img/smoothing_study.png and the docs' table
SHOW = NAMED


def plot_summary(rows, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    def get(scn, key, cal=False):
        for r in rows:
            if r["scenario"] == scn and r["variant"] == key and r["calibrated"] == cal:
                return r
        return None

    panels = [("market", "mae_move", "Galat saat bergerak (MAE, cm)"),
              ("market", "mae_still", "Galat saat diam (MAE, cm)"),
              ("market", "prec50", "Sebaran saat diam (CEP50, cm)"),
              ("market", "jitter", "Kegoyahan garis (cm)"),
              ("dropout", "drop_max", "Galat maks. saat anchor hilang (cm)")]
    fig, axes = plt.subplots(1, len(panels), figsize=(4.4 * len(panels), 5.0), sharey=True)
    labels = [lab for _k, lab in SHOW]
    colors = ["#888888", "#e9c46a", "#e76f51", "#f4a261", "#8ab17d", "#6fa8dc", "#1f4e79"]
    for ax, (scn, f, title) in zip(axes, panels):
        vals = []
        for key, _lab in SHOW:
            r = get(scn, key)
            vals.append((r or {}).get(f) or 0.0)
        y = range(len(SHOW))
        ax.barh(list(y), [v * 100 for v in vals], color=colors)
        for yi, v in zip(y, vals):
            ax.text(min(v * 100, 60) + 0.8, yi, f"{v * 100:.1f}", va="center", fontsize=8)
        ax.set_xlim(0, max(min(max(vals) * 100 * 1.25, 70), 10))
        ax.set_title(title, fontsize=10)
        ax.grid(axis="x", alpha=0.3)
    axes[0].set_yticks(list(range(len(SHOW))))
    axes[0].set_yticklabels(labels, fontsize=9)
    axes[0].invert_yaxis()
    fig.suptitle("Studi windowing — simulasi ruangan 9,5 x 5,9 m, laju anchor seperti hardware "
                 "(nilai > 70 cm dipotong)", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    fig.savefig(out, dpi=130)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--runs", type=int, default=8)
    ap.add_argument("--seed", type=int, default=100)
    ap.add_argument("--quick", action="store_true", help="smaller grid")
    ap.add_argument("--workers", type=int, default=8)
    a = ap.parse_args()

    pres = QUICK_PRE if a.quick else tuple(PRE)
    posts = QUICK_POST if a.quick else tuple(POST)
    grid = [(p, q) for p in pres for q in posts] + EXTRA_VARIANTS
    jobs = [(name, a.seed + k, grid, False) for name in SCENARIOS for k in range(a.runs)]
    # calibrated: the baseline and the spec's 20x20 cascade, for the accuracy floor
    cal_grid = [("none", "still20"), ("mean20", "mean20", "old"), ("median3", "none", "old"),
                ("none", "linfit10", "old"), ("none", "still20", "v1")]
    jobs += [(name, a.seed + k, cal_grid, True) for name in ("lab", "market", "market-los")
             for k in range(a.runs)]
    with Pool(a.workers) as pool:
        results = pool.map(one_run, jobs)
    agg = aggregate(results)

    cols = ("mae", "rmse", "mae_move", "sync_move", "mae_still", "p95_still", "prec50",
            "prec95", "still_det", "xt", "jitter", "avail", "drop_max")
    rows = []
    for (name, calibrated, key), m in sorted(agg.items()):
        rows.append({"scenario": name, "calibrated": calibrated, "variant": key,
                     **{c: m.get(c) for c in cols}})
    out = ROOT / "docs" / "smoothing_study.json"
    out.write_text(json.dumps({"runs": a.runs, "rows": rows}, indent=1), encoding="utf-8")
    plot_summary(rows, ROOT / "docs" / "img" / "smoothing_study.png")

    for name in SCENARIOS:
        for calibrated in (False, True):
            sel = [r for r in rows if r["scenario"] == name and r["calibrated"] == calibrated]
            if not sel:
                continue
            sel.sort(key=lambda r: r["mae"])
            print(f"\n== {name}{' (calibrated)' if calibrated else ''}  — cm, sorted by MAE")
            print(f"{'pre|post':22s} {'MAE':>6s} {'RMSE':>6s} {'move':>6s} {'sync':>6s} {'still':>6s} "
                  f"{'P95st':>6s} {'pr50':>5s} {'pr95':>5s} {'stDet':>5s} {'XT':>6s} {'jit':>6s} "
                  f"{'avail':>6s} {'dropMx':>7s}")
            for r in sel:
                print(f"{r['variant']:22s} {r['mae'] * 100:6.1f} {r['rmse'] * 100:6.1f} "
                      f"{r['mae_move'] * 100:6.1f} {r['sync_move'] * 100:6.1f} {r['mae_still'] * 100:6.1f} "
                      f"{r['p95_still'] * 100:6.1f} {r['prec50'] * 100:5.1f} "
                      f"{r['prec95'] * 100:5.1f} {r['still_det'] * 100:4.0f}% {r['xt'] * 100:6.1f} "
                      f"{r['jitter'] * 100:6.1f} {r['avail'] * 100:5.0f}% "
                      f"{r['drop_max'] * 100:7.1f}")
    print("\nwrote", out)


if __name__ == "__main__":
    main()
