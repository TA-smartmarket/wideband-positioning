#!/usr/bin/env python3
"""Evaluate a recorded session (server/logs/*.jsonl) against the path you walked.

Without a motion-capture system the exact position at every instant is
unknown, but the *path* is known when you walk along something you measured:
a taped straight line between two marked points, a rectangle, or a spot where
the tag stands still. The error that can be measured honestly is then

  * cross-track error  — distance from each estimate to the walked path
                         (how straight / on-path the track is), and
  * static error       — distance to a measured point while the tag is still
                         (accuracy = mean error, precision = spread, CEP50/95).

For a straight walk the best-fit line through the estimates also gives a
truth-free "straightness" number (RMS distance to that line).

    # straight line from A(0.8, 1.0) to B(3.6, 4.4)
    python tools/evaluate.py server/logs/2026..._garis.jsonl --path 0.8,1.0 3.6,4.4

    # closed rectangle, and replay the raw packets through the current tracker
    python tools/evaluate.py LOG --path 0.9,0.9 0.9,4.5 3.6,4.5 3.6,0.9 0.9,0.9 --replay

    # static test at one measured point (or use the marks written by
    # /api/v1/calibration/point, which are picked up automatically)
    python tools/evaluate.py LOG --point 1.5,2.0

    # try other tuning on the same data
    python tools/evaluate.py LOG --path ... --replay --set sigma_a=1.2 --set sigma_r=0.10
"""
import argparse
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "server"))
import scene as scene_mod   # noqa: E402
import tracker              # noqa: E402


def load(path):
    meta, rx, est, marks = None, [], [], []
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            o = json.loads(line)
            k = o.get("k")
            if k == "meta":
                meta = o
            elif k == "rx":
                rx.append(o)
            elif k == "est":
                est.append(o)
            elif k == "mark":
                marks.append(o)
    if meta is None:
        sys.exit("not a session log (no meta line)")
    return meta, rx, est, marks


def replay(meta, rx, overrides, tag=None):
    """Feed every recorded packet through server/tracker.py again."""
    scene = json.loads(json.dumps(meta["scene"]))
    if overrides:
        scene.setdefault("tracking", {}).update(overrides)
    eng = tracker.Engine()
    eng.configure(scene, meta.get("nlos_enabled", True), nlos_mod=scene_mod)
    out = []
    for o in rx:
        for t in eng.ingest(o["dev"], o["p"], o["t"]):
            if tag and t != tag:
                continue
            e = eng.estimate(t, o["t"])
            if e:
                out.append({"t": o["t"], "tag": t, "x": e["x"], "y": e["y"],
                            "b": e["range_bias"]})
    return out


# ---------------------------------------------------------------------------
# metrics
# ---------------------------------------------------------------------------

def seg_dist(px, py, a, b):
    vx, vy = b[0] - a[0], b[1] - a[1]
    L2 = vx * vx + vy * vy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - a[0]) * vx + (py - a[1]) * vy) / L2))
    return math.hypot(px - (a[0] + t * vx), py - (a[1] + t * vy))


def path_dist(px, py, path):
    if len(path) == 1:
        return math.hypot(px - path[0][0], py - path[0][1])
    return min(seg_dist(px, py, a, b) for a, b in zip(path, path[1:]))


def q(v, p):
    s = sorted(v)
    return s[min(int(round(p * (len(s) - 1))), len(s) - 1)]


def best_fit_line(pts):
    """Total-least-squares line: returns (rms distance, direction angle deg)."""
    n = len(pts)
    mx = sum(p[0] for p in pts) / n
    my = sum(p[1] for p in pts) / n
    sxx = sum((p[0] - mx) ** 2 for p in pts)
    syy = sum((p[1] - my) ** 2 for p in pts)
    sxy = sum((p[0] - mx) * (p[1] - my) for p in pts)
    ang = 0.5 * math.atan2(2 * sxy, sxx - syy)
    nx, ny = -math.sin(ang), math.cos(ang)
    d = [(p[0] - mx) * nx + (p[1] - my) * ny for p in pts]
    return math.sqrt(sum(v * v for v in d) / n), math.degrees(ang)


def path_metrics(pts, path):
    d = [path_dist(x, y, path) for x, y in pts]
    out = {"n": len(d), "xt_mean": sum(d) / len(d),
           "xt_rmse": math.sqrt(sum(v * v for v in d) / len(d)),
           "xt_p95": q(d, 0.95), "xt_max": max(d)}
    if len(path) == 2 and len(pts) >= 5:
        rms, ang = best_fit_line(pts)
        truth = math.degrees(math.atan2(path[1][1] - path[0][1], path[1][0] - path[0][0]))
        dang = (ang - truth + 90) % 180 - 90
        out.update({"fit_rms": rms, "fit_angle_err_deg": dang})
    return out


def static_metrics(pts, x, y):
    n = len(pts)
    mx = sum(p[0] for p in pts) / n
    my = sum(p[1] for p in pts) / n
    err = [math.hypot(px - x, py - y) for px, py in pts]
    spread = [math.hypot(px - mx, py - my) for px, py in pts]
    return {"n": n, "mean_x": mx, "mean_y": my,
            "mean_error": math.hypot(mx - x, my - y),
            "std": math.sqrt(sum(v * v for v in spread) / n),
            "rmse": math.sqrt(sum(v * v for v in err) / n),
            "cep50": q(err, 0.5), "cep95": q(err, 0.95), "max": max(err)}


def fmt(m):
    return "  ".join(f"{k}={v * 100:.1f}cm" if isinstance(v, float) and k not in
                     ("fit_angle_err_deg", "mean_x", "mean_y") else
                     (f"{k}={v:.2f}" if isinstance(v, float) else f"{k}={v}")
                     for k, v in m.items())


def parse_pts(items):
    out = []
    for it in items:
        x, y = it.split(",")
        out.append((float(x), float(y)))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("log")
    ap.add_argument("--path", nargs="+", metavar="X,Y", help="walked path (2+ points)")
    ap.add_argument("--point", metavar="X,Y", help="static truth point")
    ap.add_argument("--tag", default=None)
    ap.add_argument("--t0", type=float, default=None, help="start, s after recording start")
    ap.add_argument("--t1", type=float, default=None, help="end, s after recording start")
    ap.add_argument("--min-speed", type=float, default=0.0,
                    help="path test: ignore estimates slower than this (m/s), e.g. 0.1 to skip standing")
    ap.add_argument("--replay", action="store_true", help="also re-run the raw packets")
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VAL",
                    help="tracker parameter override for --replay")
    ap.add_argument("--out", default=None, help="plot file (default: next to the log)")
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args()

    meta, rx, est, marks = load(args.log)
    t_start = meta["t"]
    tag = args.tag or (est[0]["tag"] if est else None)

    def window(lst):
        out = []
        for o in lst:
            if tag and o.get("tag") != tag:
                continue
            rel = (o["t"] - t_start) / 1000
            if args.t0 is not None and rel < args.t0:
                continue
            if args.t1 is not None and rel > args.t1:
                continue
            out.append(o)
        return out

    tracks = {"terekam": window(est)}
    if args.replay:
        overrides = {}
        for kv in args.set:
            k, v = kv.split("=")
            overrides[k] = float(v)
        label = "replay" + (" " + " ".join(args.set) if args.set else "")
        tracks[label] = window(replay(meta, rx, overrides, tag))

    def moving(lst):
        if args.min_speed <= 0:
            return lst
        keep = []
        for a, b in zip(lst, lst[1:]):
            dt = (b["t"] - a["t"]) / 1000
            if dt > 0 and math.hypot(b["x"] - a["x"], b["y"] - a["y"]) / dt >= args.min_speed:
                keep.append(b)
        return keep

    print(f"\n{Path(args.log).name}: {meta.get('name')}  {meta.get('note') or ''}")
    print(f"  {len(rx)} packets, {len(est)} estimates, {len(marks)} marks, tag {tag}")
    cal = (meta["scene"].get("calibration") or {}).get("anchors") or {}
    cal_txt = ", ".join(f"{a} {c.get('bias', 0) * 100:+.1f}cm" for a, c in cal.items())
    print(f"  calibration: {cal_txt or 'none'}")

    results = {}
    path = parse_pts(args.path) if args.path else None
    for name, lst in tracks.items():
        if not lst:
            print(f"  [{name}] no estimates in the window")
            continue
        pts = [(o["x"], o["y"]) for o in moving(lst)] if path else [(o["x"], o["y"]) for o in lst]
        if path and pts:
            results[name] = path_metrics(pts, path)
            print(f"  [{name}] path   {fmt(results[name])}")
        if args.point:
            (px, py), = parse_pts([args.point])
            m = static_metrics([(o["x"], o["y"]) for o in lst], px, py)
            print(f"  [{name}] static {fmt(m)}")
        if "b" in lst[-1]:
            print(f"  [{name}] estimated common range offset b = {lst[-1]['b'] * 100:+.1f} cm")

    # static segments marked by /api/v1/calibration/point
    starts = [m for m in marks if m.get("phase") == "static-start"]
    for s in starts:
        e = next((m for m in marks if m.get("phase") == "static-end" and m["t"] > s["t"]), None)
        if not e:
            continue
        for name, lst in tracks.items():
            seg = [(o["x"], o["y"]) for o in lst if s["t"] <= o["t"] <= e["t"]]
            if seg:
                print(f"  [{name}] static @ {s['label']}: {fmt(static_metrics(seg, s['x'], s['y']))}")

    if not args.no_plot:
        plot(meta, tracks, path, args, marks)


def plot(meta, tracks, path, args, marks):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    sc = meta["scene"]
    w, d = sc["room"]["width"], sc["room"]["depth"]
    fig, ax = plt.subplots(figsize=(7, 7 * d / w if w else 7))
    ax.add_patch(plt.Rectangle((0, 0), w, d, fill=False, color="#888"))
    for a in sc.get("anchors", []):
        ax.plot(a["x"], a["y"], "s", color="#2a9d8f", ms=9)
        ax.annotate(a.get("label") or a["id"], (a["x"], a["y"]), textcoords="offset points",
                    xytext=(6, 6), fontsize=8)
    if path:
        ax.plot([p[0] for p in path], [p[1] for p in path], "-", color="#222", lw=2.5,
                label="lintasan yang dijalani")
    if args.point:
        (px, py), = parse_pts([args.point])
        ax.plot(px, py, "x", color="#222", ms=14, mew=3, label="titik acuan")
    for m in marks:
        if "x" in m and "y" in m:
            ax.plot(m["x"], m["y"], "+", color="#6d597a", ms=12, mew=2)
    colors = ["#e76f51", "#457b9d", "#8ab17d", "#b5838d"]
    for (name, lst), c in zip(tracks.items(), colors):
        ax.plot([o["x"] for o in lst], [o["y"] for o in lst], ".-", color=c, ms=3, lw=0.8,
                alpha=0.85, label=f"estimasi ({name})")
    ax.set_aspect("equal")
    ax.set_xlim(-0.4, w + 0.4)
    ax.set_ylim(-0.4, d + 0.4)
    ax.grid(alpha=0.25)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(f"{meta.get('name')} — {Path(args.log).name}", fontsize=9)
    ax.legend(fontsize=8, loc="best")
    out = args.out or str(Path(args.log).with_suffix(".png"))
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    print(f"  plot: {out}")


if __name__ == "__main__":
    main()
