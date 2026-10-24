"""Walking-test video of the algorithm the dashboard's "Recommended (smooth)"
button switches on, run by the server's own tracker (simulation only).

    python tools/web_walk_video.py

Same simulated world as tools/window_study.py (self-driving trolley among
shoppers). Compared on the same data:
  G                      the server defaults
  web: G+Z+L + mean 3    what the button sets without a static calibration
  web + calibration      the same after a static calibration (K): offsets
                         taken from it, not learned
Writes docs/hasil-simulasi/web_uji_jalan_{ramai,kosong}.gif/.mp4 and
web_uji_jalan_lintasan.png.
"""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import window_report as wr  # noqa: E402
import window_study as ws  # noqa: E402

lab, sim, tracker = ws.lab, ws.sim, ws.tracker
OUT = ws.OUT
# (name, None = the server's own tracker with smoothing | lab settings, calibrated, colour)
VARIANTS = [
    ("G: server default", {}, False, "#8c8c8c"),
    ("Web: G+Z+L + rata 3", None, False, "#e76f51"),
    ("Web + kalibrasi (K)", None, True, "#1f4e79"),
]


def server_rows(scene, stream, cal, settings, calibrated):
    """Rows (ms, x, y, valid, still): settings None = server/tracker.py as the
    dashboard runs it (smoothing on), else tools/tracker_lab.py with them."""
    tracker.TagFilter, tracker.TagTrack = lab.ORIG_FILTER, lab.ORIG_TRACK
    sc = json.loads(json.dumps(scene))
    sc["tracking"] = {"smooth": 1} if settings is None else dict(settings)
    if calibrated:
        sc["calibration"] = {"anchors": {a: {"bias": v, "scale": 1.0} for a, v in cal.items()}}
    eng = tracker.Engine() if settings is None else lab.tracker_lab.LabEngine()
    eng.configure(sc, True, nlos_mod=sim.scene_mod)
    rows = []
    for recv, dev, payload in stream:
        for tag in eng.ingest(dev, payload, recv):
            e = eng.estimate(tag, recv)
            if e:
                rows.append((recv, e["x"], e["y"], e["valid"], e["still"]))
    return rows


def main():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    demos = {}
    for world in ("ramai", "kosong"):
        scene, robot, people, stream, cal = ws.make_case(world, wr.DEMO_SEED)
        tracks = {name: server_rows(scene, stream, cal, s, k) for name, s, k, _c in VARIANTS}
        demos[world] = (scene, robot, people, tracks)
        for name in tracks:
            m = ws.measure(robot, tracks[name])
            print(f"{world:7s} {name:24s} rak {m['shelf'] * 100:5.1f} cm  bergerak {m['move'] * 100:5.1f} cm  "
                  f"goyah {m['jitter'] * 100:5.1f} cm")
    colors = {name: c for name, _s, _k, c in VARIANTS}

    # the usual trajectory figure: black = where the trolley really drove
    fig, axes = plt.subplots(2, 3, figsize=(16, 8.4))
    for r, (world, (scene, robot, _people, tracks)) in enumerate(demos.items()):
        truth = [robot.segs[0][2]] + [s_[3] for s_ in robot.segs]
        W, D = scene["room"]["width"], scene["room"]["depth"]
        for c, (name, _s, _k, col) in enumerate(VARIANTS):
            ax = axes[r][c]
            ax.add_patch(plt.Rectangle((0, 0), W, D, fill=False, ec="#777", lw=1))
            for a in scene["anchors"]:
                ax.plot(a["x"], a["y"], "^", color="#c62828", ms=8)
            ax.plot([p[0] for p in truth], [p[1] for p in truth], "-", color="#111", lw=2.2)
            tr = tracks[name]
            ax.plot([p[1] for p in tr], [p[2] for p in tr], "-", color=col, lw=1.1)
            m = ws.measure(robot, tr)
            ax.set_xlim(-0.6, W + 0.6)
            ax.set_ylim(-0.6, D + 0.6)
            ax.set_aspect("equal")
            ax.grid(alpha=0.25)
            ax.set_title(f"{name} — {wr.WORLD_LABEL[world]}\nrak {m['shelf'] * 100:.1f} cm · bergerak "
                         f"{m['move'] * 100:.1f} cm · goyah {m['jitter'] * 100:.1f} cm (rute ini)", fontsize=9)
    fig.suptitle("Algoritma di web (tombol 'Recommended (smooth)') — garis hitam: gerak troli sebenarnya",
                 fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(OUT / "web_uji_jalan_lintasan.png", dpi=115)
    plt.close(fig)

    for world, (scene, robot, people, tracks) in demos.items():
        wr.animate(scene, robot, people, tracks, colors, OUT / f"web_uji_jalan_{world}.gif",
                   OUT / f"web_uji_jalan_{world}.mp4")
        print("wrote", f"web_uji_jalan_{world}.gif / .mp4")
    print("wrote web_uji_jalan_lintasan.png")


if __name__ == "__main__":
    main()
