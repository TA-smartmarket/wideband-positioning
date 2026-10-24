"""Window study: every combination of EKF core x window on the ranges x stop
solve x window on the positions, for a self-driving trolley among shoppers.
(simulation only, no extra hardware)

    python tools/window_study.py            # ~20 min on 8 processes -> docs/hasil-simulasi/jendela.json
    python tools/window_report.py           # figures, animation, workbook from it

The world (unlike tools/accuracy_lab.py): nobody pushes the trolley, it drives
itself along the aisles (0.3-0.7 m/s, 0.3 m/s^2, stops of 3-12 s). The body
shadow comes from SHOPPERS walking and browsing around it: a person (a
cylinder 0.44 m wide, 1.70 m tall) standing on the line between the tag and
an anchor adds 10-30 cm to that range, from any direction, for as long as
they stand there. Worlds: kosong (nobody), sepi (2 shoppers), ramai (5).

A window of N with shift k (the "jendela 20" question):
  geser-1    sliding: every new sample, the newest N (1-20, 2-21, 3-22, ...);
             one output per sample, delay (N-1)/2 samples
  geser-5    hopping: every 5th sample, the newest N (1-20, 6-25, ...)
  tunggu-N   tumbling: wait for N NEW samples, one output per N samples
             (1-20, 21-40, ...), the output held in between
  adaptif    (positions only) sliding, but only while the trolley stands,
             over the outputs since it stopped (at most N): algorithm G
On the ranges a sample is one range of one anchor (2.5 per anchor per
second): N = 20 spans 8 s. On the positions a sample is one EKF output (one
per ranging round, ~3 per second, as the server counts them): N = 20 spans
about 7 s.

EKF cores (all with the validity gate):
  old   the EKF before this work (no per-anchor offsets, symmetric Huber):
        with geser-1 mean 20 on both = algorithm C
  G     per-anchor offsets + weight of long ranges (the server now)
  GZ    G + zero-velocity mode at stops (the Z of BZ-EKF)
  KGZ   GZ with the static calibration of the anchor offsets (a procedure
        with the dashboard, no hardware)
  GZL / KGZL  ... + L: while standing, a range LONGER than expected by more
        than 1 sigma is dropped instead of down-weighted (a body only ever
        lengthens a range; a short one is kept)
Outputs: the EKF, or at a stop the solve from every anchor's median range
since the stop (S), optionally excluding an inconsistent anchor (S+FDE).
"""
import argparse
import bisect
import json
import math
import random
import sys
from multiprocessing import Pool
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import accuracy_lab as lab  # noqa: E402
import tracker_lab  # noqa: E402
import walk_test as wt  # noqa: E402

sim, st, tracker = lab.sim, lab.st, lab.tracker
ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "docs" / "hasil-simulasi"

WORLDS = {"kosong": 0, "sepi": 2, "ramai": 5}       # number of shoppers
ROBOT = dict(speed=(0.3, 0.7), pause=(3.0, 12.0), accel=0.3)
SHOPPER = dict(speed=(0.8, 1.3), pause=(0.5, 8.0), accel=0.8)
BODY_R, BODY_H = 0.22, 1.70                         # m: half width, height
BODY_EXCESS = (0.10, 0.30)                          # m added to a blocked range
BODY_JITTER = 0.04
STILL_IN, STILL_OUT = 0.15, 0.30                    # m/s, as the tracker's "still"

CORES = {"old": set(), "G": set(), "GZ": {"Z"}, "KGZ": {"K", "F", "Z"},
         "GZL": {"Z", "L"}, "KGZL": {"K", "F", "Z", "L"},
         # the B of BZ-EKF (a body-shadow state for the anchors BEHIND the
         # travel direction, tools/accuracy_lab.py), here where nobody pushes
         "KBZ": {"K", "F", "B", "hd", "Z"}, "KBZL": {"K", "F", "B", "hd", "Z", "L"},
         # R: a body shadow from ANY direction, one state rho_i >= 0 per anchor
         # (tools/accuracy_lab.py RandBodyFilter)
         "KRZ": {"K", "F", "R", "Z"}, "KRZL": {"K", "F", "R", "Z", "L"}}
MAIN_CORES = ("G", "GZ", "KGZ", "GZL", "KGZL", "KBZ", "KBZL", "KRZ", "KRZL")
STILL_LONG = 1.0                                    # L: drop above this many sigma
STATS = ("mean", "median")
# (kind, N, shift, statistic)
PRE = ([("none", 1, 1, "mean")]
       + [("slide", n, 1, s) for n in (3, 5, 10, 20) for s in STATS]
       + [("hop", n, 5, s) for n in (10, 20) for s in STATS]
       + [("tumble", n, n, s) for n in (5, 10, 20) for s in STATS])
POST = ([("none", 1, 1, "mean")]
        + [("slide", n, 1, s) for n in (3, 5, 10, 20) for s in STATS]
        + [("hop", n, 5, s) for n in (10, 20) for s in STATS]
        + [("tumble", n, n, s) for n in (5, 10, 20) for s in STATS]
        + [("still", n, 1, s) for n in (5, 10, 20, 50, 100) for s in STATS])
# the old core only for the references A and C
OLD_PRE = [("none", 1, 1, "mean"), ("slide", 20, 1, "mean")]


def wkey(w):
    kind, n, k, s = w
    s = "rata" if s == "mean" else "median"
    return {"none": "tanpa", "slide": f"geser-1 {s} {n}", "hop": f"geser-{k} {s} {n}",
            "tumble": f"tunggu-{n} {s}", "still": f"adaptif-diam {s} {n}"}[kind]


# ---------------------------------------------------------------------------
# world
# ---------------------------------------------------------------------------

class Path_:
    """A route of segments (t0, t1, p0, p1) with fast lookups."""

    def __init__(self, segs):
        self.segs = segs
        self.t0 = [s[0] for s in segs]
        self.end = segs[-1][1]

    def seg(self, t):
        i = bisect.bisect_right(self.t0, t) - 1
        return self.segs[min(max(i, 0), len(self.segs) - 1)]

    def pos(self, t):
        t0, t1, p0, p1 = self.seg(t)
        a = 0.0 if t1 <= t0 else min(max((t - t0) / (t1 - t0), 0.0), 1.0)
        return p0[0] + a * (p1[0] - p0[0]), p0[1] + a * (p1[1] - p0[1])

    def state(self, t):
        """(moving, seconds since this segment began, seconds left, segment)."""
        s = self.seg(t)
        return s[2] != s[3], t - s[0], s[1] - t, s


def shopper_path(rng, duration):
    start = (rng.choice(wt.CROSS_X), rng.choice(wt.AISLES_Y))
    wps = wt.shop_route(rng, 60, SHOPPER["speed"], SHOPPER["pause"], start, rng.uniform(0, 3))
    segs = wt.route_segs(wps, SHOPPER["accel"])
    while segs[-1][1] < duration:                     # stand at the end
        segs.append((segs[-1][1], duration + 10, segs[-1][3], segs[-1][3]))
    return Path_(segs)


def blocked(tag, anchor, person, tz):
    """Does a person at person=(x, y) stand on the tag-anchor line?"""
    vx, vy = anchor[0] - tag[0], anchor[1] - tag[1]
    L2 = vx * vx + vy * vy
    if L2 < 1e-9:
        return False
    u = ((person[0] - tag[0]) * vx + (person[1] - tag[1]) * vy) / L2
    if u <= 0.03 or u >= 0.97:
        return False
    d = math.hypot(person[0] - tag[0] - u * vx, person[1] - tag[1] - u * vy)
    return d < BODY_R and tz + u * (anchor[2] - tz) < BODY_H


def make_case(world, seed):
    rng = random.Random(seed)
    robot = Path_(wt.route_segs(wt.shop_route(random.Random(seed * 13 + 5), 8, ROBOT["speed"],
                                              ROBOT["pause"]), ROBOT["accel"]))
    dur = robot.end + 1.0
    prng = random.Random(seed * 17 + 3)
    people = [shopper_path(prng, dur) for _ in range(WORLDS[world])]
    body = [prng.uniform(*BODY_EXCESS) for _ in people]
    scene = lab.scene_for(4)
    tz = scene["tag_z"]
    anchors = scene["anchors"]
    bias = {a["id"]: sim.BIAS_COMMON + rng.gauss(0, sim.BIAS_SPREAD) for a in anchors}
    ex = []
    t = rng.uniform(0, st.HW_POLL_MS)
    while t < dur * 1000:
        for i, a in enumerate(anchors):
            if rng.random() < sim.LOSS_P:
                continue
            tm = t + 7 * (2 * i + 1) + 3
            px, py = robot.pos(tm / 1000)
            r = math.sqrt((px - a["x"]) ** 2 + (py - a["y"]) ** 2 + (tz - a["z"]) ** 2)
            r += bias[a["id"]] + rng.gauss(0, sim.NOISE_M)
            for k, pp in enumerate(people):
                if blocked((px, py), (a["x"], a["y"], a["z"]), pp.pos(tm / 1000), tz):
                    r += body[k] + abs(rng.gauss(0, BODY_JITTER))
            if rng.random() < sim.OUTLIER_P:
                r += rng.uniform(*sim.OUTLIER_RANGE)
            ex.append({"t": tm, "anchor": a["id"], "r": r, "tag_rx": rng.random() >= sim.REPORT_LOSS_P})
        t += st.HW_POLL_MS + rng.uniform(0, 10)
    stream = sim.telemetry_stream(scene, ex, random.Random(rng.random()), dur, "1.0.32")
    crng = random.Random(seed * 7 + 1)
    cal = {aid: v + crng.gauss(0, lab.CAL_SIGMA) for aid, v in sorted(bias.items())}
    return scene, robot, people, stream, cal


# ---------------------------------------------------------------------------
# filters
# ---------------------------------------------------------------------------

def stat_of(vals, s):
    if s == "mean":
        return sum(vals) / len(vals)
    v = sorted(vals)
    n = len(v)
    return v[n // 2] if n % 2 else 0.5 * (v[n // 2 - 1] + v[n // 2])


class WinEngine(tracker_lab.LabEngine):
    """The tracker with a window on each anchor's raw ranges before the EKF."""

    def __init__(self, pre):
        super().__init__()
        self.pre = pre
        self.wbuf = {}

    def _process(self, tr, t, anchor, raw, rx, fp, reporter):
        kind, n, hop, s = self.pre
        if kind == "none" or not 0.05 < raw < 50.0:
            return super()._process(tr, t, anchor, raw, rx, fp, reporter)
        b = self.wbuf.setdefault((tr.id, anchor), {"v": [], "n": 0, "t": t})
        if t - b["t"] > 3000:                        # a gap: start again
            b["v"], b["n"] = [], 0
        b["t"] = t
        b["v"].append(raw)
        b["n"] += 1
        if kind == "tumble":
            if len(b["v"]) < n:
                return None
            value = stat_of(b["v"], s)
            b["v"] = []
        else:
            del b["v"][:-n]
            if b["n"] % hop:
                return None
            value = stat_of(b["v"], s)
        return super()._process(tr, t, anchor, value, rx, fp, reporter)


def post(rows, cfg):
    """A window on the positions (rows: (ms, x, y, valid, still))."""
    kind, n, hop, s = cfg
    if kind == "none":
        return rows
    out, win, cnt, held = [], [], 0, None
    for r in rows:
        ms, x, y, valid, still = r
        if not valid:                                # the gate holds: so do we
            out.append((ms, held[0], held[1], valid, still) if held else r)
            continue
        if kind == "still":
            if not still:
                win = []
                held = (x, y)
                out.append(r)
                continue
            win.append((x, y))
            del win[:-n]
        else:
            win.append((x, y))
            if kind != "tumble":
                del win[:-n]
            cnt += 1
        if kind == "tumble":
            if len(win) >= n:
                held = (stat_of([q[0] for q in win], s), stat_of([q[1] for q in win], s))
                win = []
        elif kind == "still" or cnt % hop == 0 or held is None:
            held = (stat_of([q[0] for q in win], s), stat_of([q[1] for q in win], s))
        out.append((ms, held[0], held[1], valid, still) if held else r)
    return out


def run(scene, stream, core, pre, cal):
    """One EKF over the stream. Returns {output: rows}; the stop solves only
    with the raw ranges (pre none) of the new cores."""
    tok = CORES[core]
    tracker.TagFilter = (lab.RandBodyFilter if "R" in tok else lab.BodyFilter) if tok else lab.ORIG_FILTER
    tracker.TagTrack = lab.CalTrack if "K" in tok else lab.ORIG_TRACK
    B = lab.BodyFilter
    B.USE_BETA, B.HEAD_DISP = "B" in tok, "hd" in tok
    B.IMU = B.IMU_STILL = None
    B.CALIBRATED, B.ZUPT, B.FREEZE = "K" in tok, "Z" in tok, "F" in tok
    B.STILL_LONG, B.STILL_RESET = (STILL_LONG if "L" in tok else None), 0.0
    sc = json.loads(json.dumps(scene))
    trk = sc.setdefault("tracking", {})
    trk.update({"pre_filter": "none", "post_filter": "none"})
    if core == "old":
        trk.update(st.EXTRA["old"])
    if "K" in tok:
        sc["calibration"] = {"anchors": {aid: {"bias": v, "scale": 1.0} for aid, v in cal.items()}}
    eng = WinEngine(pre)
    eng.configure(sc, True, nlos_mod=sim.scene_mod)
    if "B" in tok:
        eng.n_off = len(eng.anchor_index) + 1         # one more state: beta
    if "R" in tok:
        eng.n_off = 2 * len(eng.anchor_index)         # d_i and rho_i per anchor
    solve = pre[0] == "none" and core != "old"
    ref = lab.Refiner(eng, scene, "S") if solve else None
    rows = {"ekf": []}
    if solve:
        rows["S"], rows["S+FDE"] = [], []
    for recv, dev, payload in stream:
        for tag in eng.ingest(dev, payload, recv):
            est = eng.estimate(tag, recv)
            if not est:
                continue
            tr = eng.tracks[tag]
            sp = math.hypot(est["vx"], est["vy"])
            if tr.still_from is not None and sp > STILL_OUT:
                tr.still_from = None
            elif tr.still_from is None and sp < STILL_IN:
                tr.still_from = est["t"]
            still = tr.still_from is not None
            rows["ekf"].append((recv, est["x"], est["y"], est["valid"], still))
            if solve:
                e = dict(est, still=still)
                for mode in ("S", "S+FDE"):
                    ref.mode = mode
                    x, y = ref.refine(tag, e)
                    rows[mode].append((recv, x, y, est["valid"], still))
    tracker.TagFilter, tracker.TagTrack = lab.ORIG_FILTER, lab.ORIG_TRACK
    return rows


# ---------------------------------------------------------------------------
# metrics
# ---------------------------------------------------------------------------

def measure(robot, rows, warm=3.0, step=0.2):
    """shelf: error over the last second of every stop of >= 2 s (the position
    read at the shelf); spread: how much it wobbles there; stop: from 1 s
    after stopping; move: while driving; all: every instant; jitter: RMS
    second difference of the outputs; avail: share of time with a valid one."""
    if not rows:
        return None
    move, stop, allv, shelf, spread = [], [], [], [], []
    by_stop = {}
    j, n_ok, n = 0, 0, 0
    t = rows[0][0] / 1000 + warm
    while t < robot.end:
        while j + 1 < len(rows) and rows[j + 1][0] <= t * 1000:
            j += 1
        r = rows[j]
        tx, ty = robot.pos(t)
        e = math.hypot(r[1] - tx, r[2] - ty)
        mv, since, left, seg = robot.state(t)
        n += 1
        n_ok += r[3] and t * 1000 - r[0] <= 1500
        allv.append(e)
        if mv:
            move.append(e)
        elif since > 1.0:
            stop.append(e)
            if left <= 1.0 and seg[1] - seg[0] >= 2.0:
                by_stop.setdefault(seg[0], []).append((e, r[1], r[2]))
        t += step
    for v in by_stop.values():
        shelf.append(sum(q[0] for q in v) / len(v))
        mx = sum(q[1] for q in v) / len(v)
        my = sum(q[2] for q in v) / len(v)
        spread.append(sum(math.hypot(q[1] - mx, q[2] - my) for q in v) / len(v))
    mean = lambda v: sum(v) / len(v) if v else float("nan")  # noqa: E731
    return {"shelf": mean(shelf), "spread": mean(spread), "stop": mean(stop), "move": mean(move),
            "all": mean(allv), "jitter": sim.jitter(rows), "avail": n_ok / n if n else 0.0,
            "shelf_p95": sorted(shelf)[int(0.95 * (len(shelf) - 1))] if shelf else float("nan"),
            "shelf_le4": sum(e <= 0.04 for e in shelf) / len(shelf) if shelf else float("nan")}


_CASE = {}


def job(args):
    world, seed, core, pre = args
    key = (world, seed)
    if key not in _CASE:
        _CASE.clear()                                # one route at a time per process
        _CASE[key] = make_case(world, seed)
    scene, robot, _people, stream, cal = _CASE[key]
    out = {}
    for name, rows in run(scene, stream, core, pre, cal).items():
        for cfg in POST:
            m = measure(robot, post(rows, cfg))
            if m:
                out[f"{core}|{wkey(pre)}|{name}|{wkey(cfg)}"] = m
    return world, seed, out


def jobs_for(world, seed, cores=MAIN_CORES):
    js = [(world, seed, core, pre) for core in cores for pre in PRE]
    if cores == MAIN_CORES:
        js += [(world, seed, "old", pre) for pre in OLD_PRE]
    return js


REFS = {
    "A. EKF saja (lama)": "old|tanpa|ekf|tanpa",
    "C. A + geser-1 rata 20 (jarak) + geser-1 rata 20 (posisi)": "old|geser-1 rata 20|ekf|geser-1 rata 20",
    "G. EKF server sekarang (adaptif-diam rata 20)": "G|tanpa|ekf|adaptif-diam rata 20",
    "G + jendela C (rata 20 pada jarak dan posisi)": "G|geser-1 rata 20|ekf|geser-1 rata 20",
    "GZ (G + mode diam), tanpa jendela": "GZ|tanpa|ekf|tanpa",
    "BZ-EKF (K+B+Z): bayangan badan dianggap selalu di belakang": "KBZ|tanpa|ekf|tanpa",
    "BZ-EKF + L (K+B+Z+L)": "KBZL|tanpa|ekf|tanpa",
    "Bayangan badan arah acak + L (K+R+Z+L)": "KRZL|tanpa|ekf|tanpa",
}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--runs", type=int, default=10)
    ap.add_argument("--seed", type=int, default=1200)
    ap.add_argument("--worlds", default=",".join(WORLDS))
    ap.add_argument("--out", default=str(OUT / "jendela.json"))
    ap.add_argument("--cores", default=",".join(MAIN_CORES), help="EKF cores to run")
    ap.add_argument("--merge", action="store_true", help="add to the combinations already in --out")
    a = ap.parse_args()
    worlds = a.worlds.split(",")
    cores = tuple(a.cores.split(","))
    cores = MAIN_CORES if set(cores) == set(MAIN_CORES) else cores
    jobs = [j for w in worlds for k in range(a.runs) for j in jobs_for(w, a.seed + k, cores)]
    per = len(jobs_for(worlds[0], a.seed, cores))
    acc = {}
    done = 0
    with Pool(8) as pool:
        for world, _seed, out in pool.imap_unordered(job, jobs, chunksize=per):
            for key, m in out.items():
                acc.setdefault(world, {}).setdefault(key, []).append(m)
            done += 1
            if done % per == 0:
                print(f"  {done}/{len(jobs)}", flush=True)
    summary = {}
    if a.merge and Path(a.out).exists():
        old = json.loads(Path(a.out).read_text(encoding="utf-8"))
        assert (old["runs"], old["seed"]) == (a.runs, a.seed), "merge needs the same runs and seed"
        summary = old["summary"]
    for w, combos in acc.items():
        for key, ms in combos.items():
            agg = {}
            for f in ms[0]:
                vals = [m[f] for m in ms if m[f] == m[f]]
                agg[f] = sum(vals) / len(vals) if vals else float("nan")
            sh = [m["shelf"] for m in ms if m["shelf"] == m["shelf"]]
            mu = agg["shelf"]
            agg["shelf_sd"] = math.sqrt(sum((x - mu) ** 2 for x in sh) / max(len(sh) - 1, 1)) if sh else 0.0
            agg["runs"] = len(ms)
            summary.setdefault(w, {})[key] = agg
    Path(a.out).write_text(json.dumps({"runs": a.runs, "seed": a.seed, "worlds": WORLDS,
                                       "refs": REFS, "summary": summary}), encoding="utf-8")
    for w in worlds:
        S = summary[w]
        print(f"\n== {w}: {WORLDS[w]} pembeli ({len(S)} kombinasi; cm)")
        print(f"{'kombinasi':78s} {'rak':>5s} {'goyang':>6s} {'gerak':>6s} {'semua':>6s} {'jitter':>6s}")
        show = [(lab_, k) for lab_, k in REFS.items()]
        best = sorted(S, key=lambda k: S[k]["shelf"])[:12]
        best_all = sorted(S, key=lambda k: S[k]["all"])[:6]
        for lab_, k in show + [("", k) for k in best] + [("", k) for k in best_all]:
            r = S.get(k)
            if r:
                name = (lab_ or k)[:78]
                print(f"{name:78s} {r['shelf'] * 100:5.1f} {r['spread'] * 100:6.1f} {r['move'] * 100:6.1f} "
                      f"{r['all'] * 100:6.1f} {r['jitter'] * 100:6.2f}")
    print("\nwrote", a.out)


if __name__ == "__main__":
    main()
