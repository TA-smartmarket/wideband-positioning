"""Walking test in the real test room (simulation only).

    python tools/room_walk.py               # 30 runs + figure + animation
    python tools/room_walk.py --runs 10 --no-anim
    python tools/room_walk.py --room 2.5x2.5 --name ruang_250x250   # another room

The room, the anchor positions and heights and the tag height come from the
server's floor plan (server/scene.json): now 2.4 x 1.85 m, an anchor in every
corner at 1.2 m, the tag at 1.1 m. --room WxD moves every anchor to the
matching corner of a W x D m room instead (same ids, heights and tag height).
A person carries the tag around the room
(a lap, the centre, a diagonal) with stops of 2-5 s, speeding up and braking
at 0.5 m/s^2, at most 0.5 m/s. Every run walks the same route; only the radio
noise differs.

Radio model, from the hardware measured in this room (firmware 1.0.37, robust
round on; tools/ranging_rate.py, sessions B and C):
  - one ranging round every 200-220 ms (4.8 rounds/s); an anchor's range
    arrives in 2 of 3 rounds (3.2 ranges per anchor per second)
  - range noise 2.5 cm (1 sigma; still tag: 1.5-3 cm), whole centimetres
  - antenna delay +50..85 cm per anchor (raw ranges before calibration)
  - within 1 m of an anchor the range reads up to 12 cm short (the
    calibration points near an anchor)
  - 2 % reflections: +30..100 cm
Two worlds: "orang", a person carries the tag in front of them (their body
adds +10..20 cm to the anchors behind), and "troli", the tag moves on its
own like the self-driving trolley (no body).

The server's own tracker (server/tracker.py, CALF-UWB, smoothing as in the
floor plan) runs twice on the same ranges: with the static calibration (each
anchor's offset known to ~2 cm) and without it (offsets learned while
walking, as the room is now). Nothing in the server changes.

While walking the shown position trails the tag: the newest range is up to a
round old and the telemetry takes 20-170 ms to arrive. "jeda" is the delay
that best explains the walking error, "bentuk" the walking error left once
the position is compared with where the tag was that long ago.

Writes docs/hasil-simulasi/<name>.json, <name>_ringkasan.png and one
animation per world, <name>_animasi_{orang,troli}.gif / .mp4 (ruang_uji by
default).
"""
import argparse
import json
import math
import random
import sys
from multiprocessing import Pool
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "server"))
sys.path.insert(0, str(ROOT / "tools"))
import scene as scene_mod  # noqa: E402
import simulate as sim  # noqa: E402
import tracker  # noqa: E402
import walk_test as wt  # noqa: E402

OUT = ROOT / "docs" / "hasil-simulasi"
SCENE = ROOT / "server" / "scene.json"

ROUTE = [                    # (x / width, y / depth, stop s): a lap, the centre, a diagonal
    (0.25, 0.27, 5.0),
    (0.75, 0.27, 3.0),
    (0.75, 0.73, 3.0),
    (0.25, 0.73, 3.0),
    (0.25, 0.27, 2.0),
    (0.50, 0.50, 4.0),
    (0.75, 0.73, 0.0),
    (0.75, 0.27, 3.0),
    (0.25, 0.73, 3.0),
    (0.50, 0.50, 5.0),
]
SPEED = 0.5                  # m/s, cruising (short legs never reach it)
ACCEL = 0.5                  # m/s^2

POLL_MS = (200, 220)         # one ranging round
ANSWER_P = 0.67              # share of rounds an anchor's range arrives in
REPORT_LOSS_P = 0.03         # the tag's own copy lost (the anchor's still arrives)
NOISE_M = 0.025
DELAY = (0.50, 0.85)         # antenna delay per anchor (m)
NEAR = (1.0, 0.4, 0.12)      # short below 1 m, by 0.12 m at 0.4 m and closer
BODY = (0.10, 0.20)          # the carrier's body (m), one value per run
BODY_JITTER = 0.03
BODY_COS = -0.35             # anchor > ~110 deg from the walking direction: behind
REFLECT_P = 0.02
REFLECT = (0.30, 1.00)
CAL_SIGMA = 0.02             # error left in a static calibration (m, per anchor)
LATENCY_MS = 18              # an anchor answers ~18 ms after its slot

WORLDS = [("orang", "tag dibawa orang"), ("troli", "tag bergerak sendiri (troli)")]
VARIANTS = [("cal", "CALF-UWB dengan kalibrasi"), ("nocal", "CALF-UWB tanpa kalibrasi (sekarang)")]
SHORT = {"cal": "dengan kalibrasi", "nocal": "tanpa kalibrasi"}
COLORS = {"cal": "#1f4e79", "nocal": "#c1121f"}
DEMO_SEED = 7
DEMO_WORLD = "orang"
LAGS = [k * 0.05 for k in range(17)]     # s, delays tried for "jeda"


# ---------------------------------------------------------------------------
# room, route, radio
# ---------------------------------------------------------------------------

def load_scene(path=SCENE):
    d = json.loads(Path(path).read_text(encoding="utf-8"))
    return d.get("scene", d)


def corner_scene(W, D, like):
    """like, with the room W x D m and every anchor moved to the corner it is
    nearest to in like's room."""
    sc = json.loads(json.dumps(like))
    w0, d0 = like["room"]["width"], like["room"]["depth"]
    sc["room"]["width"], sc["room"]["depth"] = W, D
    for a in sc["anchors"]:
        a["x"] = 0.0 if a["x"] < w0 / 2 else W
        a["y"] = 0.0 if a["y"] < d0 / 2 else D
    sc["calibration"] = {"points": [], "anchors": {}}
    return sc


def waypoints(scene):
    W, D = scene["room"]["width"], scene["room"]["depth"]
    return [(round(fx * W, 3), round(fy * D, 3), stop, SPEED) for fx, fy, stop in ROUTE]


def route(scene):
    return wt.route_segs(waypoints(scene), ACCEL)


def radio(scene, segs, rng, dur, world):
    """Every range the hardware would measure on this walk."""
    anchors, tz = scene["anchors"], float(scene.get("tag_z", 1.1))
    delay = {a["id"]: rng.uniform(*DELAY) for a in anchors}
    body = rng.uniform(*BODY) if world == "orang" else None
    ex = []
    t = rng.uniform(0, POLL_MS[0])
    while t < dur * 1000:
        for i, a in enumerate(anchors):
            if rng.random() > ANSWER_P:
                continue
            tm = t + 7 * (2 * i + 1) + LATENCY_MS
            px, py = sim.truth_at(segs, tm / 1000)
            hx, hy = sim.heading_at(segs, tm / 1000)
            ax, ay, az = float(a["x"]), float(a["y"]), float(a.get("z", 2.2))
            d = math.sqrt((px - ax) ** 2 + (py - ay) ** 2 + (tz - az) ** 2)
            r = d + delay[a["id"]] + rng.gauss(0, NOISE_M)
            if d < NEAR[0]:
                r -= NEAR[2] * min(1.0, (NEAR[0] - d) / (NEAR[0] - NEAR[1]))
            un = math.hypot(ax - px, ay - py) or 1.0
            behind = body is not None and ((ax - px) * hx + (ay - py) * hy) / un < BODY_COS
            if behind:
                r += body + abs(rng.gauss(0, BODY_JITTER))
            if rng.random() < REFLECT_P:
                r += rng.uniform(*REFLECT)
            ex.append({"t": tm, "anchor": a["id"], "r": round(r, 2), "behind": behind,
                       "tag_rx": rng.random() >= REPORT_LOSS_P})
        t += rng.uniform(*POLL_MS)
    return ex, delay


def run(scene, stream, cal=None):
    """The server's tracker on the stream: [(recv_ms, x, y, valid, sigma, still)]."""
    sc = json.loads(json.dumps(scene))
    sc["calibration"] = {"points": [], "anchors": {aid: {"bias": b, "scale": 1.0}
                                                  for aid, b in (cal or {}).items()}}
    eng = tracker.Engine()
    eng.configure(sc, True, nlos_mod=scene_mod)
    rows = []
    for recv, dev, payload in stream:
        for tag in eng.ingest(dev, payload, recv):
            e = eng.estimate(tag, recv)
            if e:
                rows.append((recv, e["x"], e["y"], e["valid"], e["sigma"], e["still"]))
    return rows


def make_case(scene, seed, world):
    segs = route(scene)
    dur = segs[-1][1] + 1.0
    rng = random.Random(seed)
    ex, delay = radio(scene, segs, rng, dur, world)
    stream = sim.telemetry_stream(scene, ex, random.Random(rng.random()), dur, "1.0.32")
    crng = random.Random(seed * 7 + 1)
    cal = {aid: v + crng.gauss(0, CAL_SIGMA) for aid, v in sorted(delay.items())}
    return segs, ex, stream, cal


# ---------------------------------------------------------------------------
# measuring
# ---------------------------------------------------------------------------

def latest(rows, t_ms, k=0):
    """Index of the newest row with time <= t_ms, searching from k (-1: none yet)."""
    if not rows or rows[0][0] > t_ms:
        return -1
    while k + 1 < len(rows) and rows[k + 1][0] <= t_ms:
        k += 1
    return k


def measure(segs, rows, warmup=3.0):
    """Every 100 ms after the warm-up: the error of the shown position, while
    walking and while standing (from 1 s after stopping); per stop the spread
    of the positions in its last 2 s; the share of positions marked valid;
    the delay of the walking positions (jeda) and their error without it."""
    out = {"all": [], "move": [], "stand": [], "spread": [], "valid": []}
    stops, k = {}, 0
    lagged = [[] for _ in LAGS]
    t, end = warmup, segs[-1][1]
    while t < end:
        k = latest(rows, t * 1000, max(k, 0))
        if k >= 0:
            x, y, valid = rows[k][1], rows[k][2], rows[k][3]
            tx, ty = sim.truth_at(segs, t)
            e = math.hypot(x - tx, y - ty)
            mv, since = wt.st.moving_at(segs, t)
            out["all"].append(e)
            out["valid"].append(1.0 if valid else 0.0)
            if mv:
                out["move"].append(e)
                for i, L in enumerate(LAGS):
                    lx, ly = sim.truth_at(segs, t - L)
                    lagged[i].append(math.hypot(x - lx, y - ly))
            elif since > 1.0:
                out["stand"].append(e)
                stops.setdefault(round(t - since, 1), []).append((since, x, y))
        t += 0.1
    for v in stops.values():
        last = max(s for s, _x, _y in v)
        pts = [(x, y) for s, x, y in v if s >= last - 2.0]
        mx, my = sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts)
        out["spread"].append(sum(math.hypot(x - mx, y - my) for x, y in pts) / len(pts))
    best = min(range(len(LAGS)), key=lambda i: mean(lagged[i]))
    out["lag"], out["shape"] = [LAGS[best]], lagged[best]
    # every time the tag sets off from a stop: how long the tracker still
    # holds it in the standing (zero-velocity) mode
    out["release"] = []
    for _t0, t1, p0, p1 in segs:
        if p0 == p1 and warmup < t1 < end:
            free = next((r[0] for r in rows if r[0] >= t1 * 1000 and not r[5]), None)
            out["release"].append((free - t1 * 1000) / 1000 if free is not None else end - t1)
    return out


def pct(v, q):
    v = sorted(v)
    return v[min(int(q * (len(v) - 1) + 0.5), len(v) - 1)] if v else float("nan")


def mean(v):
    return sum(v) / len(v) if v else float("nan")


def stats(v):
    return {"mean": mean(v), "rmse": math.sqrt(mean([e * e for e in v])) if v else float("nan"),
            "cep50": pct(v, 0.5), "cep95": pct(v, 0.95), "max": max(v) if v else float("nan")}


def one(job):
    seed, scene, world = job
    segs, _ex, stream, cal = make_case(scene, seed, world)
    return world, {key: measure(segs, run(scene, stream, cal if key == "cal" else None))
                   for key, _label in VARIANTS}


def summarize(results):
    S = {}
    for world, _wl in WORLDS:
        runs = [r for w, r in results if w == world]
        for key, _label in VARIANTS:
            pool = lambda k: [e for r in runs for e in r[key][k]]  # noqa: E731
            S.setdefault(world, {})[key] = {
                "all": stats(pool("all")), "move": stats(pool("move")), "stand": stats(pool("stand")),
                "spread": mean(pool("spread")), "valid": mean(pool("valid")),
                "lag": mean(pool("lag")), "shape": stats(pool("shape")),
                "release": mean(pool("release")), "release_max": max(pool("release"), default=0.0)}
    return S


# ---------------------------------------------------------------------------
# figures
# ---------------------------------------------------------------------------

def label_of(a):
    return a.get("label") or a["id"]


def room(ax, scene, grid=0.5):
    import matplotlib.patches as mp
    W, D = scene["room"]["width"], scene["room"]["depth"]
    ax.add_patch(mp.Rectangle((0, 0), W, D, fill=False, lw=1.6))
    k = 0.0
    while k <= W + 1e-9:
        ax.axvline(k, color="#e6e6e6", lw=0.6, zorder=0)
        k += grid
    k = 0.0
    while k <= D + 1e-9:
        ax.axhline(k, color="#e6e6e6", lw=0.6, zorder=0)
        k += grid
    for a in scene["anchors"]:
        ax.plot(a["x"], a["y"], "^", color="#2a9d8f", ms=11, mec="black", zorder=4)
        dy = 0.13 if a["y"] < D / 2 else -0.17
        ax.text(a["x"], a["y"] - dy, f"{label_of(a)}\n({a['x']:.2f}, {a['y']:.2f})",
                ha="center", va="center", fontsize=7.5)
    ax.set_xlim(-0.45, W + 0.45)
    ax.set_ylim(-0.4, D + 0.4)
    ax.set_aspect("equal")
    ax.set_xlabel(f"X (m) — garis kisi tiap {grid * 100:.0f} cm")
    ax.set_ylabel("Y (m)")


def errors_over_time(segs, rows, times):
    out, k = [], 0
    for t in times:
        k = latest(rows, t * 1000, max(k, 0))
        if k < 0:
            out.append(float("nan"))
            continue
        tx, ty = sim.truth_at(segs, t)
        out.append(math.hypot(rows[k][1] - tx, rows[k][2] - ty) * 100)
    return out


def bars(ax, S, world, title):
    groups = [("Galat saat\nberjalan", "move", "mean"),
              ("… tanpa\njeda", "shape", "mean"),
              ("Galat saat\ndiam", "stand", "mean"),
              ("CEP95\n(95% posisi)", "all", "cep95"),
              ("Goyang saat\ndiam", "spread", None)]
    w = 0.38
    for g, (glabel, part, key) in enumerate(groups):
        for k, (vk, label) in enumerate(VARIANTS):
            r = S[world][vk]
            v = (r[part] if key is None else r[part][key]) * 100
            x = g + (k - 0.5) * w
            ax.bar(x, v, w * 0.92, color=COLORS[vk], label=label if g == 0 else None)
            ax.text(x, v + 0.4, f"{v:.1f}", ha="center", fontsize=8)
    ax.set_xticks(range(len(groups)))
    ax.set_xticklabels([g[0] for g in groups], fontsize=8.5)
    ax.set_ylabel("cm")
    ax.set_ylim(0, ax.get_ylim()[1] * 1.3)          # room for the legend above the bars
    ax.grid(axis="y", alpha=0.3)
    ax.legend(fontsize=7.8, loc="upper left")
    lag = " / ".join(f"{S[world][vk]['lag']:.2f}" for vk, _ in VARIANTS)
    ax.set_title(f"{title} — jeda saat berjalan {lag} s", fontsize=10.5)


def fig_summary(scene, S, demo, runs, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    segs, _ex, tracks = demo
    wlabel = dict(WORLDS)
    fig = plt.figure(figsize=(15, 10.5))
    gs = fig.add_gridspec(2, 2, height_ratios=[1.3, 1], width_ratios=[1.1, 1])
    ax = fig.add_subplot(gs[0, 0])
    room(ax, scene)
    ax.plot([s[2][0] for s in segs] + [segs[-1][3][0]], [s[2][1] for s in segs] + [segs[-1][3][1]],
            color="black", lw=1.2, ls="--", alpha=0.6, label="rute sebenarnya")
    for key, label in VARIANTS:
        tr = tracks[key]
        ax.plot([r[1] for r in tr], [r[2] for r in tr], color=COLORS[key], lw=1.0, alpha=0.8,
                label=f"posisi {label}")
    for t0, t1, p0, p1 in segs:
        if p0 == p1 and t1 - t0 > 1.5:
            ax.plot(*p0, "s", color="#f4a261", mec="black", ms=7, zorder=5)
    ax.plot([], [], "s", color="#f4a261", mec="black", ms=7, label="titik berhenti")
    ax.set_ylim(-0.85, ax.get_ylim()[1])              # the legend goes under the room
    ax.legend(fontsize=8, loc="lower center", ncol=2, framealpha=0.9)
    ax.set_title(f"Rute demo, {wlabel[DEMO_WORLD]}: posisi sebenarnya vs yang ditampilkan", fontsize=10.5)

    bars(fig.add_subplot(gs[0, 1]), S, "orang", f"{runs}× jalan, {wlabel['orang']}")
    bars(fig.add_subplot(gs[1, 1]), S, "troli", f"{runs}× jalan, {wlabel['troli']}")

    ax = fig.add_subplot(gs[1, 0])
    end = segs[-1][1]
    times = [3.0 + 0.1 * k for k in range(int((end - 3.0) / 0.1))]
    for t0, t1, p0, p1 in segs:
        if p0 == p1:
            ax.axvspan(t0, t1, color="#eeeeee", lw=0)
    for key, label in VARIANTS:
        ax.plot(times, errors_over_time(segs, tracks[key], times), color=COLORS[key], lw=1.1, label=label)
    ax.set_xlim(times[0], times[-1])
    ax.set_ylim(bottom=0)
    ax.set_xlabel("waktu (s) — abu-abu: berhenti")
    ax.set_ylabel("galat (cm)")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc="upper right")
    ax.set_title("Galat sepanjang rute demo = jarak posisi yang ditampilkan ke posisi sebenarnya", fontsize=10.5)
    W, D = scene["room"]["width"], scene["room"]["depth"]
    fig.suptitle(f"Simulasi jalan di ruang {W:.2f} × {D:.2f} m (anchor {len(scene['anchors'])}, "
                 f"tinggi anchor {scene['anchors'][0].get('z', 0):.2f} m, tag {scene.get('tag_z', 0):.2f} m)\n"
                 "jeda = posisi tertinggal dari tag saat berjalan; \"… tanpa jeda\" = galat bila jeda itu "
                 "dihilangkan; goyang = sebaran posisi di 2 s terakhir tiap berhenti",
                 fontsize=11.5)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


def animate(scene, demo, world, path_gif, path_mp4, step=0.2, fps=10):
    """Top view to scale. step s of the walk per frame: step * fps x real time."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FFMpegWriter, FuncAnimation, PillowWriter
    segs, ex, tracks = demo
    anchors = scene["anchors"]
    end = segs[-1][1]
    times = [2.0 + k * step for k in range(int((end - 2.0) / step) + 1)]
    err = {key: errors_over_time(segs, tracks[key], times) for key, _ in VARIANTS}

    fig = plt.figure(figsize=(9.6, 9.4))
    gs = fig.add_gridspec(2, 1, height_ratios=[3, 1], top=0.78, bottom=0.07, hspace=0.28)
    ax = fig.add_subplot(gs[0])
    room(ax, scene)
    wps = waypoints(scene)
    ax.plot([w[0] for w in wps], [w[1] for w in wps], color="#9aa3ae", lw=2.4, alpha=0.6,
            solid_capstyle="round", zorder=1)
    for k, w in enumerate(wps, 1):
        if w[2] > 0:
            ax.text(w[0], w[1], str(k), ha="center", va="center", fontsize=7, color="#5b6573",
                    bbox=dict(boxstyle="circle,pad=0.2", fc="white", ec="#9aa3ae"), zorder=1)
    walked, = ax.plot([], [], color="black", lw=1.6, alpha=0.85, zorder=3)
    rays = [ax.plot([], [], color="#e76f51", lw=1.6, alpha=0.8, zorder=2)[0] for _ in anchors]
    trails, dots = {}, {}
    for z, (key, _label) in enumerate(VARIANTS):
        trails[key], = ax.plot([], [], color=COLORS[key], lw=1.4, alpha=0.7)
        dots[key], = ax.plot([], [], "o", color=COLORS[key], ms=9, mec="white", zorder=8 + z)
    has_body = world == "orang"
    carrier, = ax.plot([], [], "o", color="#f4a261", ms=15, mec="black", zorder=6, visible=has_body)
    truth, = ax.plot([], [], "s", color="white", mec="black", mew=2, ms=10, zorder=7)
    info = fig.text(0.02, 0.985, "", va="top", fontsize=9, family="monospace",
                    bbox=dict(boxstyle="round", fc="white", ec="#cccccc"))
    W, D = scene["room"]["width"], scene["room"]["depth"]
    fig.text(0.98, 0.985, f"Ruang {W:.2f} × {D:.2f} m\n{dict(WORLDS)[world]}\n"
             f"{step * fps:.0f}× waktu nyata", ha="right", va="top", fontsize=9.5)
    handles = [truth, walked] + [dots[k] for k, _ in VARIANTS]
    labels = ["tag (posisi sebenarnya)", "jalur yang sudah dilewati (abu-abu: rencana rute)"] \
        + [label for _k, label in VARIANTS]
    if has_body:
        handles += [carrier, rays[0]]
        labels += ["badan pembawa tag", "anchor terhalang badan (jarak terbaca lebih panjang)"]
    ax.legend(handles=handles, labels=labels, loc="lower right", bbox_to_anchor=(1.0, 1.01),
              ncol=1, fontsize=7.8, frameon=False)

    ax2 = fig.add_subplot(gs[1])
    for t0, t1, p0, p1 in segs:
        if p0 == p1:
            ax2.axvspan(t0, t1, color="#eeeeee", lw=0)
    for key, label in VARIANTS:
        ax2.plot(times, err[key], color=COLORS[key], lw=1.1, label=label)
    cursor = ax2.axvline(times[0], color="#e76f51", lw=1.5)
    top = max(max(v for v in err[k] if v == v) for k, _ in VARIANTS)
    ax2.set_ylim(0, min(max(top * 1.1, 10), 80))
    ax2.set_xlim(times[0], times[-1])
    ax2.set_ylabel("galat (cm)")
    ax2.set_xlabel("waktu (s) — abu-abu: berhenti")
    ax2.legend(fontsize=7.8, loc="upper right", ncol=2)
    ax2.grid(alpha=0.3)

    idx = {key: 0 for key, _ in VARIANTS}
    ei = {"k": 0}
    seen = {}
    path = [sim.truth_at(segs, t) for t in times]

    def frame(n):
        t = times[n]
        tms = t * 1000
        tx, ty = path[n]
        hx, hy = sim.heading_at(segs, t)
        mv, since = wt.st.moving_at(segs, t)
        truth.set_data([tx], [ty])
        walked.set_data([p[0] for p in path[:n + 1]], [p[1] for p in path[:n + 1]])
        carrier.set_data([tx - 0.22 * hx], [ty - 0.22 * hy])
        while ei["k"] < len(ex) and ex[ei["k"]]["t"] <= tms:
            seen[ex[ei["k"]]["anchor"]] = ex[ei["k"]]["behind"]
            ei["k"] += 1
        for i, a in enumerate(anchors):
            if seen.get(a["id"]):
                rays[i].set_data([tx, a["x"]], [ty, a["y"]])
            else:
                rays[i].set_data([], [])
        lines = [f"t = {t:5.1f} s   " + (f"berjalan {wt.speed_at(segs, t):.2f} m/s" if mv
                                         else f"berhenti {since:4.1f} s"),
                 f"{'sebenarnya':16s}: X {tx:5.2f}  Y {ty:5.2f} m"]
        for key, _label in VARIANTS:
            tr = tracks[key]
            idx[key] = k = latest(tr, tms, idx[key])
            if k < 0:
                lines.append(f"{SHORT[key]:16s}: (belum ada posisi)")
                continue
            dots[key].set_data([tr[k][1]], [tr[k][2]])
            rec = [r for r in tr[max(0, k - 60):k + 1] if tms - r[0] <= 4000]
            trails[key].set_data([r[1] for r in rec], [r[2] for r in rec])
            lines.append(f"{SHORT[key]:16s}: X {tr[k][1]:5.2f}  Y {tr[k][2]:5.2f} m   "
                         f"galat {err[key][n]:5.1f} cm")
        info.set_text("\n".join(lines))
        cursor.set_xdata([t, t])
        return []

    anim = FuncAnimation(fig, frame, frames=len(times), blit=False)
    anim.save(str(path_mp4), writer=FFMpegWriter(fps=fps, bitrate=1800), dpi=100)
    anim.save(str(path_gif), writer=PillowWriter(fps=fps), dpi=64)
    plt.close(fig)
    return step * fps


def demo_data(scene, seed=DEMO_SEED, world=DEMO_WORLD):
    segs, ex, stream, cal = make_case(scene, seed, world)
    return segs, ex, {key: run(scene, stream, cal if key == "cal" else None) for key, _ in VARIANTS}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--runs", type=int, default=30)
    ap.add_argument("--seed", type=int, default=100)
    ap.add_argument("--scene", default=str(SCENE))
    ap.add_argument("--room", help="WxD (m): an anchor in every corner of this room instead")
    ap.add_argument("--name", default="ruang_uji", help="prefix of the output files")
    ap.add_argument("--no-anim", action="store_true", help="numbers only")
    a = ap.parse_args()
    scene = load_scene(a.scene)
    if a.room:
        W, D = (float(v) for v in a.room.lower().split("x"))
        scene = corner_scene(W, D, scene)
    with Pool(8) as pool:
        results = pool.map(one, [(a.seed + k, scene, w) for w, _ in WORLDS for k in range(a.runs)])
    S = summarize(results)
    segs = route(scene)
    info = {"runs": a.runs, "seed": a.seed, "room": scene["room"], "tag_z": scene.get("tag_z"),
            "anchors": [{k: x[k] for k in ("id", "x", "y", "z")} for x in scene["anchors"]],
            "route_m": sum(math.dist(s[2], s[3]) for s in segs), "route_s": segs[-1][1],
            "stops": sum(1 for s in segs if s[2] == s[3]),
            "model": {"poll_ms": POLL_MS, "answer_p": ANSWER_P, "noise_m": NOISE_M, "delay_m": DELAY,
                      "near": NEAR, "body_m": BODY, "reflect_p": REFLECT_P, "reflect_m": REFLECT,
                      "cal_sigma_m": CAL_SIGMA}}
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / f"{a.name}.json").write_text(json.dumps({**info, "summary": S}, indent=1), encoding="utf-8")
    print(f"rute {info['route_m']:.1f} m, {info['route_s']:.0f} s, {info['stops']} titik berhenti; "
          f"{a.runs} kali jalan per dunia (cm)")
    for world, wlabel in WORLDS:
        print(f"\n== {wlabel}")
        print(f"{'':38s} {'jalan':>6s} {'-jeda':>6s} {'jeda s':>6s} {'diam':>6s} {'rata2':>6s} {'RMSE':>6s} "
              f"{'CEP50':>6s} {'CEP95':>6s} {'maks':>6s} {'goyang':>6s} {'valid':>6s} {'lepas s':>7s} {'maks':>5s}")
        for key, label in VARIANTS:
            r = S[world][key]
            print(f"{label:38s} {r['move']['mean'] * 100:6.1f} {r['shape']['mean'] * 100:6.1f} "
                  f"{r['lag']:6.2f} {r['stand']['mean'] * 100:6.1f} "
                  f"{r['all']['mean'] * 100:6.1f} {r['all']['rmse'] * 100:6.1f} {r['all']['cep50'] * 100:6.1f} "
                  f"{r['all']['cep95'] * 100:6.1f} {r['all']['max'] * 100:6.1f} {r['spread'] * 100:6.2f} "
                  f"{r['valid'] * 100:5.0f}% {r['release']:7.2f} {r['release_max']:5.1f}")
    if not a.no_anim:
        demos = {world: demo_data(scene, world=world) for world, _wl in WORLDS}
        fig_summary(scene, S, demos[DEMO_WORLD], a.runs, OUT / f"{a.name}_ringkasan.png")
        for world, _wl in WORLDS:
            speed = animate(scene, demos[world], world, OUT / f"{a.name}_animasi_{world}.gif",
                            OUT / f"{a.name}_animasi_{world}.mp4")
            print(f"animasi {world}: {speed:.0f}x waktu nyata")
    print("wrote", OUT / f"{a.name}.json")


if __name__ == "__main__":
    main()
