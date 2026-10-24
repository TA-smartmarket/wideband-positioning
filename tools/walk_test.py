"""Walking test of BZ-EKF on shopping routes it was NOT tuned on (simulation only).

    python tools/walk_test.py              # 20 random routes per world + animation

BZ-EKF (tools/accuracy_lab.py) was tuned on one rectangle walked at a
constant 0.6 m/s with 6 s stops. Here every run walks a different route along
the aisles of the same 9.5 x 5.9 m room, the way a shopper would:
  - trapezoidal speed: speeds up / brakes at 0.5 m/s^2, never jumps
  - every leg its own cruising speed, 0.5-1.0 m/s
  - stops of 2-12 s at random shelves, short slow-downs at aisle corners
Nothing in the server changes. Writes docs/hasil-simulasi/uji_jalan.json,
uji_jalan_ringkasan.png and the animation uji_jalan_animasi.gif / .mp4.
"""
import argparse
import json
import math
import random
import sys
from multiprocessing import Pool
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import accuracy_lab as lab  # noqa: E402

sim, st = lab.sim, lab.st
ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "docs" / "hasil-simulasi"
AISLES_Y = (0.8, 2.95, 5.1)              # walkways along the room
CROSS_X = (0.8, 4.75, 8.7)               # walkways across
ACCEL = 0.5                              # m/s^2, speeding up and braking
DT = 0.1                                 # s, resolution of the true path
MODES = [("G", "G: EKF sekarang"), (lab.FINAL, "BZ-EKF"), (lab.FINAL_IMU, "BZ-EKF + IMU")]
COLORS = {"G": "#8c8c8c", lab.FINAL: "#1f4e79", lab.FINAL_IMU: "#6a994e"}
DEMO_SEED = 9001


# ---------------------------------------------------------------------------
# routes
# ---------------------------------------------------------------------------

def leg(p, q, v, t0, accel=ACCEL):
    """Segments (t0, t1, p0, p1) of DT for one straight leg p -> q with a
    trapezoidal speed profile (cruise v, accel up and down, 0 at both ends)."""
    d = math.dist(p, q)
    if d < 1e-6:
        return [], t0
    if d < v * v / accel:                # never reaches v: triangle
        v = math.sqrt(accel * d)
    ta = v / accel
    T = d / v + ta

    def s(t):
        if t < ta:
            return 0.5 * accel * t * t
        if t < T - ta:
            return 0.5 * accel * ta * ta + v * (t - ta)
        r = T - t
        return d - 0.5 * accel * r * r

    ux, uy = (q[0] - p[0]) / d, (q[1] - p[1]) / d
    pt = lambda dist: (p[0] + ux * dist, p[1] + uy * dist)  # noqa: E731
    segs, t = [], 0.0
    while t < T - 1e-9:
        t1 = min(t + DT, T)
        segs.append((t0 + t, t0 + t1, pt(s(t)), pt(min(s(t1), d))))
        t = t1
    return segs, t0 + T


def shop_route(rng, n_stops=8, speed=(0.5, 1.0), pause=(2.0, 12.0), start=None, wait=5.0):
    """Waypoints [(x, y, pause_s, speed)] of a shopping trip along the aisles,
    from the entrance (or start) where it first stands for wait seconds."""
    cur = start or (CROSS_X[0], AISLES_Y[0])
    wps = [(cur[0], cur[1], wait, 0.0)]
    for _ in range(n_stops):
        ty = rng.choice([y for y in AISLES_Y if y != cur[1]] if rng.random() < 0.7 else AISLES_Y)
        tx = round(rng.uniform(1.3, 8.2), 2)
        pts = []
        if ty != cur[1]:
            cx = min(CROSS_X, key=lambda c: abs(c - cur[0]) + abs(tx - c))
            pts += [(cx, cur[1]), (cx, ty)]
        pts.append((tx, ty))
        v = rng.uniform(*speed)
        for i, q in enumerate(pts):
            if math.dist(q, cur) < 0.05:
                continue
            last = i == len(pts) - 1
            wps.append((q[0], q[1], rng.uniform(*pause) if last else 0.0, v))
            cur = q
    return wps


def route_segs(wps, accel=ACCEL):
    segs, t = [], 0.0
    p = (wps[0][0], wps[0][1])
    segs.append((0.0, wps[0][2], p, p))
    t = wps[0][2]
    for x, y, pause, v in wps[1:]:
        q = (x, y)
        part, t = leg(p, q, v, t, accel)
        segs += part
        if pause > 0:
            segs.append((t, t + pause, q, q))
            t += pause
        p = q
    return segs


# ---------------------------------------------------------------------------
# runs
# ---------------------------------------------------------------------------

def make_case(world_name, seed, n_anchors=4):
    rng = random.Random(seed)
    w = lab.world(world_name, rng)
    segs = route_segs(shop_route(random.Random(seed * 13 + 5)))
    scene = lab.scene_for(n_anchors)
    dur = segs[-1][1] + 1.0
    ex, bias = lab.radio(scene, segs, rng, dur, w)
    stream = sim.telemetry_stream(scene, ex, random.Random(rng.random()), dur, "1.0.32")
    crng = random.Random(seed * 7 + 1)
    cal = {aid: v + crng.gauss(0, lab.CAL_SIGMA) for aid, v in sorted(bias.items())}
    return scene, segs, dur, stream, cal, w


def speed_at(segs, t):
    for t0, t1, p0, p1 in segs:
        if t0 <= t <= t1:
            return math.dist(p0, p1) / (t1 - t0) if t1 > t0 else 0.0
    return 0.0


def measure(segs, track, warmup=3.0):
    """Every 100 ms: error at stops (from 1 s after stopping), the shelf
    reading (mean of the last second of each stop), error while moving and
    its part across the walkway (are we shown in the right aisle?)."""
    stop, settled, move, cross, shelf = [], [], [], [], []
    by_stop = {}
    j = 0
    t = track[0][0] + warmup * 1000
    end = segs[-1][1]
    while t < end * 1000:
        while j + 1 < len(track) and track[j + 1][0] <= t:
            j += 1
        x, y = track[j][1], track[j][2]
        tx, ty = sim.truth_at(segs, t / 1000)
        e = math.hypot(x - tx, y - ty)
        mv, since = st.moving_at(segs, t / 1000)
        if mv:
            move.append(e)
            cross.append(sim.cross_track(segs, x, y))
        elif since > 1.0:
            stop.append(e)
            if since > 2.0:
                settled.append(e)
            by_stop.setdefault(round(t / 1000 - since, 1), []).append((since, e))
        t += 100
    for v in by_stop.values():
        last = max(s for s, _ in v)
        tail = [e for s, e in v if s >= last - 1.0]
        shelf.append(sum(tail) / len(tail))
    return {"stop": stop, "settled": settled, "move": move, "cross": cross, "shelf": shelf}


def one(job):
    world_name, seed = job
    scene, segs, dur, stream, cal, _w = make_case(world_name, seed)
    out = {}
    for mode, _ in MODES:
        tr = lab.run(scene, stream, mode, cal=cal, imu=lab.imu_model(segs, seed))
        out[mode] = measure(segs, tr)
    out["route"] = {"length_m": sum(math.dist(p0, p1) for _t0, _t1, p0, p1 in segs),
                    "duration_s": segs[-1][1],
                    "stops": sum(1 for _t0, _t1, p0, p1 in segs if p0 == p1)}
    return world_name, out


def pct(v, q):
    v = sorted(v)
    return v[min(int(q * (len(v) - 1) + 0.5), len(v) - 1)] if v else float("nan")


def mean(v):
    return sum(v) / len(v) if v else float("nan")


def summarize(res, world_name, mode):
    runs = [o[mode] for n, o in res if n == world_name]
    pool = lambda k: [e for r in runs for e in r[k]]  # noqa: E731
    stop, shelf, move = pool("stop"), pool("shelf"), pool("move")
    return {
        "stop_mean": mean(stop), "stop_med": pct(stop, 0.5), "stop_p95": pct(stop, 0.95),
        "settled_mean": mean(pool("settled")),
        "shelf_mean": mean(shelf), "shelf_med": pct(shelf, 0.5), "shelf_p95": pct(shelf, 0.95),
        "shelf_within4": sum(e <= 0.04 for e in shelf) / len(shelf) if shelf else float("nan"),
        "n_shelf": len(shelf),
        "move_mean": mean(move), "move_p95": pct(move, 0.95),
        "cross_mean": mean(pool("cross")), "cross_p95": pct(pool("cross"), 0.95),
    }


# ---------------------------------------------------------------------------
# figures
# ---------------------------------------------------------------------------

def demo_data():
    """The demo route (market world) with both algorithms, plus what BZ-EKF
    believes at every estimate: beta and the shade of each anchor."""
    scene, segs, dur, stream, cal, w = make_case("market", DEMO_SEED)
    probe_rows = []

    def probe(eng, tag, est, recv, x, y):
        f = eng.tracks[tag].f
        if getattr(f, "ib", None) is not None:
            probe_rows.append((recv, f.x[f.ib], [f.shade(a["x"], a["y"]) for a in scene["anchors"]]))

    imu = lab.imu_model(segs, DEMO_SEED)
    tracks = {"G": lab.run(scene, stream, "G", cal=cal),
              lab.FINAL: lab.run(scene, stream, lab.FINAL, cal=cal, probe=probe),
              lab.FINAL_IMU: lab.run(scene, stream, lab.FINAL_IMU, cal=cal, imu=imu)}
    return scene, segs, w, tracks, probe_rows


def latest(track, t_ms, k=0):
    """Index of the newest row of track with time <= t_ms, searching from k."""
    while k + 1 < len(track) and track[k + 1][0] <= t_ms:
        k += 1
    return k


def fig_summary(S, demo, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    scene, segs, _w, tracks, _probe = demo
    fig = plt.figure(figsize=(15, 10))
    gs = fig.add_gridspec(2, 6, height_ratios=[1, 1])
    for k, (mode, label) in enumerate(MODES):
        ax = fig.add_subplot(gs[0, 2 * k:2 * k + 2])
        room(ax, scene)
        ax.plot([p[0] for _t0, _t1, p, _q in segs], [p[1] for _t0, _t1, p, _q in segs],
                color="black", lw=1, ls="--", alpha=0.5, label="rute sebenarnya")
        tr = tracks[mode]
        ax.plot([r[1] for r in tr], [r[2] for r in tr], color=COLORS[mode], lw=1.1, alpha=0.85,
                label=f"posisi {label}")
        for t0, t1, p0, p1 in segs:
            if p0 == p1 and t1 - t0 > 1.5:
                ax.plot(*p0, "s", color="#e76f51", ms=5)
        ax.set_title(f"{label} — rute demo (Market)", fontsize=10.5)
        ax.legend(fontsize=7.5, loc="center right")

    def bars(ax, groups, key_of, title):
        w = 0.8 / len(MODES)
        for g, (glabel, wn, key) in enumerate(groups):
            for k, (mode, label) in enumerate(MODES):
                v = S[wn][mode][key] * 100
                x = g + (k - (len(MODES) - 1) / 2) * w
                ax.bar(x, v, w * 0.9, color=COLORS[mode], label=label if g == 0 else None)
                ax.text(x, v + 0.4, f"{v:.0f}" if v >= 10 else f"{v:.1f}", ha="center", fontsize=8)
        ax.axhspan(2, 4, color="#6a994e", alpha=0.13, lw=0)
        ax.set_xticks(range(len(groups)))
        ax.set_xticklabels([g[0] for g in groups], fontsize=9)
        ax.set_ylabel("cm")
        ax.grid(axis="y", alpha=0.3)
        ax.legend(fontsize=8, loc="upper left")
        ax.set_title(title, fontsize=10.5)

    bars(fig.add_subplot(gs[1, :3]),
         [("Galat saat berhenti\n(rata-rata)", "market", "stop_mean"),
          ("Bacaan di rak\n(detik terakhir berhenti)", "market", "shelf_mean"),
          ("Galat saat bergerak", "market", "move_mean"),
          ("Melenceng dari lorong\nsaat bergerak", "market", "cross_mean")],
         None, "Market: 20 rute belanja acak")
    bars(fig.add_subplot(gs[1, 3:]),
         [("Market", "market", "shelf_mean"), ("Tanpa halangan", "market-los", "shelf_mean"),
          ("Robust", "robust", "shelf_mean")],
         None, "Bacaan di rak (rata-rata) di tiga dunia; hijau = target 2–4 cm")
    fig.suptitle("Uji gerak jalan: troli belanja dengan percepatan, kecepatan & lama berhenti berbeda-beda",
                 fontsize=13)
    fig.tight_layout()
    fig.savefig(path, dpi=125)
    plt.close(fig)


def room(ax, scene):
    W, D = scene["room"]["width"], scene["room"]["depth"]
    ax.add_patch(__import__("matplotlib").patches.Rectangle((0, 0), W, D, fill=False, lw=1.5))
    for x in range(0, int(W) + 1):
        ax.axvline(x, color="#e6e6e6", lw=0.6, zorder=0)
    for y in range(0, int(D) + 1):
        ax.axhline(y, color="#e6e6e6", lw=0.6, zorder=0)
    for a in scene["anchors"]:
        ax.plot(a["x"], a["y"], "^", color="#2a9d8f", ms=10, mec="black", zorder=4)
        ax.text(a["x"], a["y"] + (0.22 if a["y"] < 1 else -0.35), a["id"].replace("anchor-", "A"),
                ha="center", fontsize=8)
    ax.set_xlim(-0.4, W + 0.4)
    ax.set_ylim(-0.4, D + 0.4)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m) — garis kisi tiap 1 m")
    ax.set_ylabel("y (m)")


def animate(demo, path_gif, path_mp4, step=0.3, fps=10):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FFMpegWriter, FuncAnimation, PillowWriter
    scene, segs, w, tracks, probe_rows = demo
    anchors = scene["anchors"]
    end = segs[-1][1]
    times = [3.0 + k * step for k in range(int((end - 3.0) / step))]

    # error of every algorithm at every frame, for the strip below the room
    err = {m: [] for m, _ in MODES}
    for mode, _ in MODES:
        tr, k = tracks[mode], 0
        for t in times:
            k = latest(tr, t * 1000, k)
            tx, ty = sim.truth_at(segs, t)
            err[mode].append(math.hypot(tr[k][1] - tx, tr[k][2] - ty) * 100)

    fig = plt.figure(figsize=(10, 9.2))
    gs = fig.add_gridspec(2, 1, height_ratios=[3, 1.05], top=0.80, bottom=0.07, hspace=0.25)
    ax = fig.add_subplot(gs[0])
    room(ax, scene)
    ax.plot([p[0] for _t0, _t1, p, _q in segs], [p[1] for _t0, _t1, p, _q in segs],
            color="black", lw=0.8, ls="--", alpha=0.25)
    marks = [ax.plot(a["x"], a["y"], "^", ms=17, mfc="none", mec=COLORS[lab.FINAL], mew=0, zorder=5)[0]
             for a in anchors]
    body = [ax.plot(a["x"], a["y"], "^", ms=10, color="#e76f51", mec="black", zorder=4, alpha=0)[0]
            for a in anchors]
    trails, dots = {}, {}
    for z, (mode, _label) in enumerate(MODES):
        trails[mode], = ax.plot([], [], color=COLORS[mode], lw=1.3, alpha=0.75)
        dots[mode], = ax.plot([], [], "o", color=COLORS[mode], ms=8, mec="white" if z else "black",
                              zorder=8 + z)
    pusher, = ax.plot([], [], "o", color="#f4a261", ms=11, mec="black", zorder=6)
    truth, = ax.plot([], [], "s", color="white", mec="black", mew=2, ms=11, zorder=7)
    info = fig.text(0.02, 0.985, "", va="top", fontsize=9, family="monospace",
                    bbox=dict(boxstyle="round", fc="white", ec="#cccccc"))
    body[0].set_alpha(1)
    ax.legend(handles=[truth, pusher] + [dots[m] for m, _ in MODES] + [body[0], marks[0]],
              labels=["troli (posisi sebenarnya)", "pendorong (badan, model simulasi)"]
              + [label for _m, label in MODES]
              + ["anchor terhalang badan (sebenarnya)", "anchor yang dianggap terhalang (BZ-EKF)"],
              loc="lower right", bbox_to_anchor=(1.0, 1.01), ncol=2, fontsize=7.5, frameon=False)

    ax2 = fig.add_subplot(gs[1])
    for t0, t1, p0, p1 in segs:
        if p0 == p1:
            ax2.axvspan(t0, t1, color="#eeeeee", lw=0)
    ax2.axhspan(2, 4, color="#6a994e", alpha=0.15, lw=0)
    for mode, label in MODES:
        ax2.plot(times, err[mode], color=COLORS[mode], lw=1.1, label=label)
    cursor = ax2.axvline(times[0], color="#e76f51", lw=1.5)
    ax2.set_ylim(0, 60)
    ax2.set_xlim(times[0], times[-1])
    ax2.set_ylabel("galat (cm)")
    ax2.set_xlabel("waktu (s) — abu-abu: troli berhenti, hijau: target 2–4 cm")
    ax2.legend(fontsize=8, loc="upper right", ncol=3)
    ax2.grid(alpha=0.3)

    idx = {m: 0 for m, _ in MODES}
    state = {"ip": 0}

    def frame(n):
        t = times[n]
        tms = t * 1000
        while state["ip"] + 1 < len(probe_rows) and probe_rows[state["ip"] + 1][0] <= tms:
            state["ip"] += 1
        tx, ty = sim.truth_at(segs, t)
        hx, hy = sim.heading_at(segs, t)
        mv, since = st.moving_at(segs, t)
        truth.set_data([tx], [ty])
        pusher.set_data([tx - 0.55 * hx], [ty - 0.55 * hy])
        for mode, _label in MODES:
            tr = tracks[mode]
            idx[mode] = k = latest(tr, tms, idx[mode])
            dots[mode].set_data([tr[k][1]], [tr[k][2]])
            rec = [r for r in tr[max(0, k - 40):k + 1] if tms - r[0] <= 6000]
            trails[mode].set_data([r[1] for r in rec], [r[2] for r in rec])
        beta, shade = probe_rows[state["ip"]][1:] if probe_rows else (0.0, [0.0] * len(anchors))
        for i, a in enumerate(anchors):
            ux, uy = a["x"] - tx, a["y"] - ty
            un = math.hypot(ux, uy) or 1.0
            blocked = w["body"] > 0 and (ux * hx + uy * hy) / un < w["cos"]
            body[i].set_alpha(1.0 if blocked else 0.0)
            marks[i].set_markeredgewidth(2.5 * shade[i])
        status = f"berjalan {speed_at(segs, t):.2f} m/s" if mv else f"berhenti {since:4.1f} s"
        lines = [f"t = {t:5.1f} s   {status}"]
        lines += [f"galat {label:13s}: {err[mode][n]:5.1f} cm" for mode, label in MODES]
        lines.append(f"\u03b2 dipelajari (BZ) : {beta * 100:4.1f} cm")
        info.set_text("\n".join(lines))
        cursor.set_xdata([t, t])
        return []

    anim = FuncAnimation(fig, frame, frames=len(times), blit=False)
    anim.save(str(path_mp4), writer=FFMpegWriter(fps=fps, bitrate=1800), dpi=100)
    anim.save(str(path_gif), writer=PillowWriter(fps=fps), dpi=62)
    plt.close(fig)
    return len(times) * step / (len(times) / fps)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--runs", type=int, default=20)
    ap.add_argument("--seed", type=int, default=900)
    ap.add_argument("--worlds", default="market,market-los,robust")
    ap.add_argument("--no-anim", action="store_true")
    a = ap.parse_args()
    worlds = a.worlds.split(",")
    with Pool(8) as pool:
        res = pool.map(one, [(wn, a.seed + k) for wn in worlds for k in range(a.runs)])
    S = {wn: {m: summarize(res, wn, m) for m, _ in MODES} for wn in worlds}
    routes = [o["route"] for _n, o in res]
    info = {"runs": a.runs, "seed": a.seed,
            "route_length_m": mean([r["length_m"] for r in routes]),
            "route_duration_s": mean([r["duration_s"] for r in routes]),
            "route_stops": mean([r["stops"] for r in routes])}
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "uji_jalan.json").write_text(json.dumps({**info, "summary": S}, indent=1), encoding="utf-8")
    print(f"rute: rata-rata {info['route_length_m']:.0f} m, {info['route_duration_s']:.0f} s, "
          f"{info['route_stops']:.1f} titik berhenti")
    for wn in worlds:
        print(f"\n== {wn}  (cm)")
        print(f"{'algoritma':12s} {'diam':>6s} {'median':>6s} {'>=2s':>6s} {'rak':>6s} {'rak<=4':>7s} "
              f"{'P95rak':>6s} {'gerak':>6s} {'lorong':>6s}")
        for m, _ in MODES:
            r = S[wn][m]
            print(f"{m:12s} {r['stop_mean'] * 100:6.1f} {r['stop_med'] * 100:6.1f} "
                  f"{r['settled_mean'] * 100:6.1f} {r['shelf_mean'] * 100:6.1f} "
                  f"{r['shelf_within4'] * 100:6.0f}% {r['shelf_p95'] * 100:6.1f} "
                  f"{r['move_mean'] * 100:6.1f} {r['cross_mean'] * 100:6.1f}")
    if not a.no_anim:
        demo = demo_data()
        fig_summary(S, demo, OUT / "uji_jalan_ringkasan.png")
        speedup = animate(demo, OUT / "uji_jalan_animasi.gif", OUT / "uji_jalan_animasi.mp4")
        print(f"\nanimasi: {speedup:.0f}x lebih cepat dari waktu nyata")
    print("wrote", OUT / "uji_jalan.json")


if __name__ == "__main__":
    main()
