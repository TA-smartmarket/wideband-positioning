"""Feed a NEW simulated walk into a running server, live, and compare what the
web shows with where the trolley really was.

    python tools/sim_feed.py --world ramai --seed 1500 --server http://127.0.0.1:8090

The world is the one of tools/window_study.py (a self-driving trolley among
shoppers, the room and anchors of server/scene.json). Its packets go through
the server's own HTTP ingest (/api/v1/telemetry) exactly like the devices',
at their real pace, so the dashboard shows the trolley moving with whatever
tracker settings are active. Every 0.2 s the position the dashboard shows is
read from /api/v1/state; at the end it is compared with the true route:
docs/hasil-simulasi/web_sim_live_<world>.png / .json.
"""
import argparse
import json
import math
import sys
import threading
import time
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import window_study as ws  # noqa: E402

OUT = ws.OUT


def get(url):
    with urllib.request.urlopen(url, timeout=3) as r:
        return json.loads(r.read())


def post(url, body):
    req = urllib.request.Request(url, data=json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=3) as r:
        r.read()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--world", default="ramai", choices=list(ws.WORLDS))
    ap.add_argument("--seed", type=int, default=1500)
    ap.add_argument("--server", default="http://127.0.0.1:8090")
    ap.add_argument("--tag", default="tag-1")
    a = ap.parse_args()

    scene, robot, people, stream, _cal = ws.make_case(a.world, a.seed)
    settings = get(f"{a.server}/api/v1/tracking")["params"]
    print(f"route {robot.end:.0f} s, {len(people)} shoppers, {len(stream)} packets -> {a.server}")
    sim0 = stream[0][0]
    wall0 = time.time()
    shown, done = [], threading.Event()

    def poll():                                     # what the dashboard shows
        while not done.is_set():
            try:
                st = get(f"{a.server}/api/v1/state")
                for tg in st.get("tags", []):
                    if tg["id"] == a.tag:
                        t_sim = sim0 + (time.time() - wall0) * 1000
                        shown.append((t_sim, tg["x"], tg["y"], bool(tg.get("valid", True)), False))
            except Exception:  # noqa: BLE001 - keep polling
                pass
            time.sleep(0.2)

    th = threading.Thread(target=poll, daemon=True)
    th.start()
    failed = 0
    for recv, _dev, payload in stream:
        wait = (recv - sim0) / 1000 - (time.time() - wall0)
        if wait > 0:
            time.sleep(wait)
        try:
            post(f"{a.server}/api/v1/telemetry", payload)
        except Exception:  # noqa: BLE001
            failed += 1
    time.sleep(1.0)
    done.set()
    th.join()

    m = ws.measure(robot, shown)
    print(f"failed posts {failed}; web: shelf {m['shelf'] * 100:.1f} cm, stop {m['stop'] * 100:.1f} cm, "
          f"moving {m['move'] * 100:.1f} cm, jitter {m['jitter'] * 100:.1f} cm, valid {m['avail'] * 100:.0f}%")
    calibrated = get(f"{a.server}/api/v1/tracking").get("calibrated")
    active = (f"algoritma final, haluskan jalur {'ya' if settings.get('smooth', 0) >= 0.5 else 'tidak'}, "
              f"offset {'dari kalibrasi' if calibrated else 'dipelajari sendiri'}")
    (OUT / f"web_sim_live_{a.world}.json").write_text(json.dumps(
        {"world": a.world, "seed": a.seed, "settings": settings, "metrics": m, "shown": shown}),
        encoding="utf-8")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(16, 5.6), gridspec_kw={"width_ratios": [1.15, 1]})
    W, D = scene["room"]["width"], scene["room"]["depth"]
    ax.add_patch(plt.Rectangle((0, 0), W, D, fill=False, ec="#777", lw=1))
    for an in scene["anchors"]:
        ax.plot(an["x"], an["y"], "^", color="#c62828", ms=8)
    truth = [robot.segs[0][2]] + [s_[3] for s_ in robot.segs]
    ax.plot([p[0] for p in truth], [p[1] for p in truth], "-", color="#111", lw=2.2,
            label="gerak troli sebenarnya")
    ax.plot([r[1] for r in shown], [r[2] for r in shown], "-", color="#1f4e79", lw=1.1,
            label="posisi yang ditampilkan web")
    ax.set_xlim(-0.6, W + 0.6)
    ax.set_ylim(-0.6, D + 0.6)
    ax.set_aspect("equal")
    ax.grid(alpha=0.25)
    ax.legend(fontsize=8, loc="center right")
    ax.set_title(f"Web vs ground truth — {len(people)} pembeli, simulasi baru (seed {a.seed})\n"
                 f"rak {m['shelf'] * 100:.1f} cm · bergerak {m['move'] * 100:.1f} cm · goyah "
                 f"{m['jitter'] * 100:.1f} cm · valid {m['avail'] * 100:.0f}%", fontsize=10)
    for t0, t1, p0, p1 in robot.segs:
        if p0 == p1:
            ax2.axvspan(t0, t1, color="#eeeeee", lw=0)
    ts, err, j = [], [], 0
    t = shown[0][0] / 1000 if shown else 0
    while shown and t < robot.end:
        while j + 1 < len(shown) and shown[j + 1][0] <= t * 1000:
            j += 1
        tx, ty = robot.pos(t)
        ts.append(t)
        err.append(math.hypot(shown[j][1] - tx, shown[j][2] - ty) * 100)
        t += 0.2
    ax2.plot(ts, err, color="#1f4e79", lw=1)
    ax2.axhspan(2, 4, color="#6a994e", alpha=0.15, lw=0)
    ax2.set_ylim(0, 60)
    ax2.set_xlabel("waktu (s) — abu-abu: troli berhenti; hijau: target 2–4 cm")
    ax2.set_ylabel("galat posisi web (cm)")
    ax2.grid(alpha=0.3)
    ax2.set_title("Galat posisi yang ditampilkan web terhadap ground truth", fontsize=10)
    fig.suptitle(f"Simulasi baru dialirkan ke server web (seperti perangkat asli) — pengaturan aktif: {active}",
                 fontsize=10.5)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(OUT / f"web_sim_live_{a.world}.png", dpi=115)
    plt.close(fig)
    print("wrote", OUT / f"web_sim_live_{a.world}.png")


if __name__ == "__main__":
    main()
