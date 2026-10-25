"""Follow-up research on top of CALF-UWB (NOT used by the server):
improved two-way ranging and angle of arrival.

    python tools/riset_lanjutan.py real      # signal power on the real recordings
    python tools/riset_lanjutan.py sim       # faster ranging and AoA in the simulated market
    python tools/riset_lanjutan.py report    # figures + workbook -> docs/riset/

real  The firmware already sends, with every range, the received signal level
      (rx_power) and the first-path power (fp_power); CALF-UWB ignores both.
      Two "improved TWR" ideas use them, without new hardware:
        RSL   the DW1000 range bias depends on the signal level (Decawave
              APS011): a curve bias(rx_power) fitted from the data is
              subtracted from every range (layer 1, correction);
        FP    a first path much weaker than the whole signal hints at a
              blocked path: that range's sigma is enlarged (layer 2).
      The recordings have no ground truth, so the curve / threshold is fitted
      on one day's recordings and tested on the other day's (two folds), and
      what can be measured honestly is compared (replay_compare.measure):
      spread at stops, consistency with the ranges, jitter, availability, plus
      the RMS of the range residuals.
sim   What cannot be tested on the recordings, in the market of
      tools/window_study.py (self-driving trolley among shoppers, 3 worlds,
      the same routes):
        rate x2   one ranging round every 200 ms instead of 400 ms (firmware
                  protocol timing, no new hardware)
        AoA       every anchor also measures the direction of the tag (PDoA,
                  needs DW3000-class hardware with two antennas); angle noise
                  2.4 deg (best published) or 4 deg, 15 deg when a body blocks
                  the path (the angle then comes from a reflection)
Every variant runs the server's own CALF-UWB (server/tracker.py, calibrated,
smoothing off) and only adds the one idea on top.
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
import replay_compare as rc  # noqa: E402
import scene as scene_mod  # noqa: E402
import tracker  # noqa: E402

OUT = ROOT / "docs" / "riset"
LOGS = ROOT / "server" / "logs"
REAL_JSON = OUT / "riset_lanjutan_rekaman.json"
SIM_JSON = OUT / "riset_lanjutan_simulasi.json"


# ---------------------------------------------------------------------------
# engines
# ---------------------------------------------------------------------------

class SignalEngine(tracker.Engine):
    """CALF-UWB plus the signal-power ideas. rsl: [(rx dBm, bias m)] points of
    a curve (linear in between, flat outside); fp: (threshold dB, sigma
    factor) on dfp = (rx - fp) minus this anchor's median rx - fp in the
    recording. probe: a list that receives one row per range update."""

    def __init__(self, rsl=None, fp=None, fp_median=None, probe=None):
        super().__init__()
        self.rsl, self.fp, self.fp_median, self.probe = rsl, fp, fp_median or {}, probe
        self._rx = None
        self.flagged = 0               # ranges whose sigma the FP rule enlarged

    def correct(self, anchor, raw):
        z = super().correct(anchor, raw)
        if self.rsl and self._rx is not None:
            z -= curve(self.rsl, self._rx)
        return z

    def _process(self, tr, t, anchor, raw, rx, fp, reporter):
        self._rx = rx
        base = self.params["sigma_r"]
        dfp = None
        if rx is not None and fp is not None:
            dfp = (rx - fp) - self.fp_median.get(anchor, 0.0)
            if self.fp and dfp > self.fp[0]:
                self.params["sigma_r"] = base * self.fp[1]
                self.flagged += 1
        link = tr.links.get(anchor)
        if link is not None:
            link.pop("nu", None)
        try:
            super()._process(tr, t, anchor, raw, rx, fp, reporter)
        finally:
            self.params["sigma_r"] = base
            self._rx = None
        link = tr.links.get(anchor) or {}
        if self.probe is not None and "nu" in link and tr.f is not None:
            self.probe.append((anchor, rx, fp, dfp, link["residual"], link["nu"],
                               bool(link["accepted"]), bool(tr.f.standing), link["corrected"]))


def curve(points, x):
    if x <= points[0][0]:
        return points[0][1]
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        if x <= x1:
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    return points[-1][1]


# ---------------------------------------------------------------------------
# real recordings
# ---------------------------------------------------------------------------

def recordings():
    """(day, name, meta, rx) of every recording with raw ranges (fw >= 1.0.32)."""
    out = []
    for p in sorted(LOGS.glob("*.jsonl")):
        meta, rx = rc.load(p)
        if not any("raw" in g for o in rx[:50] for g in o["p"].get("ranges", [])):
            continue
        out.append((p.name[:8], meta.get("name") or p.stem, meta, rx))
    return out


def anchor_fp_median(rx):
    """Each anchor's median rx_power - fp_power in this recording."""
    vals = {}
    for o in rx:
        for g in o["p"].get("ranges", []):
            if g.get("rx_power") is None or g.get("fp_power") is None:
                continue
            a = g["src"] if str(g.get("src", "")).startswith("anchor") else g.get("dst")
            vals.setdefault(a, []).append(g["rx_power"] - g["fp_power"])
    return {a: rc.median(v) for a, v in vals.items()}


def replay(meta, rx, smooth=1, stats=None, **kw):
    scene = json.loads(json.dumps(meta["scene"]))
    scene["tracking"] = {"smooth": smooth}
    eng = SignalEngine(fp_median=anchor_fp_median(rx), **kw)
    eng.configure(scene, meta.get("nlos_enabled", True), nlos_mod=scene_mod)
    rows = []
    for o in rx:
        for tag in eng.ingest(o["dev"], o["p"], o["t"]):
            e = eng.estimate(tag, o["t"])
            if e:
                rows.append((o["t"], e["x"], e["y"], e["valid"]))
    if stats is not None:
        stats["flagged"] = eng.flagged
    return rows


def binned(rows, key, width, min_n=40):
    """[(bin centre, median residual, n)] of the probe rows by key(row)."""
    b = {}
    for r in rows:
        k = key(r)
        if k is None:
            continue
        b.setdefault(math.floor(k / width), []).append(r[4])
    return [((i + 0.5) * width, rc.median(v), len(v)) for i, v in sorted(b.items()) if len(v) >= min_n]


def fit_rsl(rows):
    """The bias curve from clean residuals (accepted, |e| < 2.5): the median
    residual per 2 dB of rx_power, minus their overall median (the common
    offset b takes the constant part)."""
    clean = [r for r in rows if r[6] and abs(r[5]) < 2.5 and r[1] is not None]
    pts = binned(clean, lambda r: r[1], 2.0)
    if len(pts) < 2:
        return None
    mid = rc.median([p[1] for p in pts])
    return [(x, y - mid) for x, y, _n in pts]


FP_FACTOR = 3.0                 # sigma of a flagged range: 3x (weight 1/9)


def fit_fp(rows):
    """Threshold on dfp above which a range is flagged: the lowest 1-dB bin
    (dfp >= 0, >= 25 ranges) whose share of long outliers (e > 2.5) is at
    least 3x the share over all ranges."""
    ok = [r for r in rows if r[3] is not None]
    if not ok:
        return None
    base = sum(r[5] > 2.5 for r in ok) / len(ok)
    bins = {}
    for r in ok:
        bins.setdefault(math.floor(r[3]), []).append(r[5] > 2.5)
    for k in sorted(bins):
        v = bins[k]
        if k >= 0 and len(v) >= 25 and sum(v) / len(v) >= 3 * base:
            return (float(k), FP_FACTOR)
    return None


def residual_rms(rows):
    v = [r[4] for r in rows if r[6]]
    if not v:
        return float("nan")
    m = sum(v) / len(v)
    return math.sqrt(sum((x - m) ** 2 for x in v) / len(v))


def real_main():
    recs = recordings()
    days = sorted({d for d, *_ in recs})
    print("recordings:", ", ".join(f"{n} ({d})" for d, n, *_ in recs))
    # 1) probe: residuals of CALF-UWB with the signal powers
    probes = {}
    for d, name, meta, rx in recs:
        pr = []
        replay(meta, rx, probe=pr)
        probes[name] = (d, pr)
        print(f"  {name}: {len(pr)} range updates")
    allp = [r for _d, pr in probes.values() for r in pr]
    analysis = {
        "by_rx": binned([r for r in allp if r[6] and abs(r[5]) < 2.5 and r[1] is not None],
                        lambda r: r[1], 2.0),
        "by_fp": binned([r for r in allp if r[6] and abs(r[5]) < 2.5 and r[2] is not None],
                        lambda r: r[2], 2.0),
        "by_dfp": binned([r for r in allp if r[3] is not None], lambda r: r[3], 1.0, min_n=25),
        "rx_minus_fp_median": rc.median([r[1] - r[2] for r in allp if r[1] is not None and r[2] is not None]),
        "n": len(allp),
    }
    # share of long ranges (e > 2.5) per dfp bin: does a weak first path flag them?
    long_by = {}
    for r in allp:
        if r[3] is not None:
            long_by.setdefault(math.floor(r[3]), []).append(r[5] > 2.5)
    analysis["long_share_by_dfp"] = [(k + 0.5, sum(v) / len(v), len(v)) for k, v in sorted(long_by.items())
                                     if len(v) >= 25]
    print(f"\nrx - fp median {analysis['rx_minus_fp_median']:.1f} dB over {analysis['n']} updates")
    print("residual (cm) by rx_power:", ", ".join(f"{x:.0f}:{y * 100:+.1f}({n})" for x, y, n in analysis["by_rx"]))
    print("residual (cm) by dfp:", ", ".join(f"{x:+.1f}:{y * 100:+.1f}({n})" for x, y, n in analysis["by_dfp"]))
    print("share e>2.5 by dfp:", ", ".join(f"{x:+.1f}:{s * 100:.0f}%({n})" for x, s, n in analysis["long_share_by_dfp"]))

    # 2) two folds: fit on one day, test on the other
    results = []
    folds = {}
    for test_day in days:
        train = [r for d, pr in probes.values() if d != test_day for r in pr]
        rsl, fp = fit_rsl(train), fit_fp(train)
        folds[test_day] = {"rsl": rsl, "fp": fp}
        print(f"\nfold test {test_day}: rsl {rsl and [(round(x), round(y * 100, 1)) for x, y in rsl]}, fp {fp}")
        variants = [("CALF-UWB", {}), ("+ RSL", {"rsl": rsl}), ("+ FP", {"fp": fp}),
                    ("+ RSL + FP", {"rsl": rsl, "fp": fp})]
        for d, name, meta, rx in recs:
            if d != test_day:
                continue
            ranges = rc.raw_ranges(rx)
            stops = rc.find_stops(ranges, rx[0]["t"], rx[-1]["t"])
            for vname, kw in variants:
                if any(v is None for v in kw.values()):
                    continue
                pr, st = [], {}
                rows = replay(meta, rx, probe=pr, stats=st, **kw)
                m = rc.measure(meta, rows, ranges, stops)
                m["residual_rms"] = residual_rms(pr)
                m["rejected"] = sum(1 for r in pr if not r[6]) / len(pr) if pr else float("nan")
                m["flagged"] = st["flagged"] / len(pr) if pr else float("nan")
                results.append({"day": d, "recording": name, "variant": vname, **m})
                print(f"  {name[:34]:34s} {vname:11s} CEP95 {m['cep95'] * 100:5.1f} konsis {m['consistency'] * 100:5.1f} "
                      f"goyah {m['jitter'] * 100:5.1f} residu {m['residual_rms'] * 100:5.1f} "
                      f"tolak {m['rejected'] * 100:4.1f}% tanda {m['flagged'] * 100:4.1f}% "
                      f"valid {m['available'] * 100:3.0f}%")
    OUT.mkdir(parents=True, exist_ok=True)
    REAL_JSON.write_text(json.dumps({"analysis": analysis, "folds": folds, "results": results}),
                         encoding="utf-8")
    print("\nwrote", REAL_JSON)


# ---------------------------------------------------------------------------
# simulation: faster ranging and AoA
# ---------------------------------------------------------------------------

AOA_BLOCKED_DEG = 15.0          # a body in the way: the angle of a reflection
SIM_VARIANTS = {                 # name -> (ranging round in ms, AoA sigma in deg)
    "CALF-UWB": (400, None),
    "laju x2": (200, None),
    "AoA 2.4 deg": (400, 2.4),
    "AoA 4 deg": (400, 4.0),
    "laju x2 + AoA 4 deg": (200, 4.0),
}


def angle_update(f, ax, ay, z, sigma, p):
    """One azimuth update (anchor at ax, ay sees the tag at angle z, rad):
    h = atan2(py - ay, px - ax), robust like the range update."""
    dx, dy = f.x[0] - ax, f.x[1] - ay
    r2 = dx * dx + dy * dy
    if r2 < 0.04:                     # within 20 cm of the anchor: no direction
        return
    y = (z - math.atan2(dy, dx) + math.pi) % (2 * math.pi) - math.pi
    n, P = len(f.x), f.P
    H = [0.0] * n
    H[0], H[1] = -dy / r2, dx / r2
    PHt = [P[i][0] * H[0] + P[i][1] * H[1] for i in range(n)]
    HPHt = H[0] * PHt[0] + H[1] * PHt[1]
    R = sigma * sigma
    S = HPHt + R
    nu = y / math.sqrt(S)
    if abs(nu) > p["gate"]:
        return
    if abs(nu) > p["huber_k"]:
        R *= abs(nu) / p["huber_k"]
        S = HPHt + R
    K = [PHt[i] / S for i in range(n)]
    for i in range(n):
        f.x[i] += K[i] * y
    IKH = [[(1.0 if i == j else 0.0) - K[i] * H[j] for j in range(n)] for i in range(n)]
    A = [[sum(IKH[i][k] * P[k][j] for k in range(n)) for j in range(n)] for i in range(n)]
    f.P = [[sum(A[i][k] * IKH[j][k] for k in range(n)) + K[i] * R * K[j]
            for j in range(n)] for i in range(n)]


class AoaEngine(tracker.Engine):
    """CALF-UWB plus one azimuth per range, taken from the payload's "aoa"."""

    def __init__(self, sigma_deg):
        super().__init__()
        self.aoa_sigma = math.radians(sigma_deg)
        self._aoa = {}

    def ingest(self, dev, payload, recv):
        for g in payload.get("ranges", []):
            if "aoa" in g:
                self._aoa[(g["src"], g["raw"])] = g["aoa"]
        return super().ingest(dev, payload, recv)

    def _process(self, tr, t, anchor, raw, rx, fp, reporter):
        link = tr.links.get(anchor)
        if link is not None:
            link.pop("nu", None)
        super()._process(tr, t, anchor, raw, rx, fp, reporter)
        z = self._aoa.pop((anchor, raw), None)
        if z is None or tr.f is None or "nu" not in (tr.links.get(anchor) or {}):
            return                        # no range update ran: no angle either
        ax, ay, _az = self.anchors[anchor]
        angle_update(tr.f, ax, ay, z, self.aoa_sigma, self.params)
        self._clamp_to_room(tr)


def make_market(world, seed, poll_ms, aoa_deg):
    """tools/window_study.make_case with the ranging round and AoA as options.
    The routes, shoppers and range noise are those of window_study for the same
    seed at 400 ms; the angle noise has its own generator, so the ranges of an
    AoA variant are exactly those of CALF-UWB."""
    import window_study as ws
    sim, st, lab = ws.sim, ws.st, ws.lab
    rng = random.Random(seed)
    arng = random.Random(seed * 31 + 7)
    robot = ws.Path_(ws.wt.route_segs(ws.wt.shop_route(random.Random(seed * 13 + 5), 8, ws.ROBOT["speed"],
                                                       ws.ROBOT["pause"]), ws.ROBOT["accel"]))
    dur = robot.end + 1.0
    prng = random.Random(seed * 17 + 3)
    people = [ws.shopper_path(prng, dur) for _ in range(ws.WORLDS[world])]
    body = [prng.uniform(*ws.BODY_EXCESS) for _ in people]
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
            hit = False
            for k, pp in enumerate(people):
                if ws.blocked((px, py), (a["x"], a["y"], a["z"]), pp.pos(tm / 1000), tz):
                    r += body[k] + abs(rng.gauss(0, ws.BODY_JITTER))
                    hit = True
            if rng.random() < sim.OUTLIER_P:
                r += rng.uniform(*sim.OUTLIER_RANGE)
            e = {"t": tm, "anchor": a["id"], "r": r, "tag_rx": rng.random() >= sim.REPORT_LOSS_P}
            if aoa_deg:
                s = math.radians(AOA_BLOCKED_DEG if hit else aoa_deg)
                e["aoa"] = math.atan2(py - a["y"], px - a["x"]) + arng.gauss(0, s)
            ex.append(e)
        t += poll_ms + rng.uniform(0, 10)
    stream = sim.telemetry_stream(scene, ex, random.Random(rng.random()), dur, "1.0.32")
    if aoa_deg:
        angles = {(e["anchor"], round(e["r"], 4)): e["aoa"] for e in ex}
        for _recv, _dev, payload in stream:
            for g in payload["ranges"]:
                a = angles.get((g["src"], g.get("raw")))
                if a is not None:
                    g["aoa"] = a
    crng = random.Random(seed * 7 + 1)
    cal = {aid: v + crng.gauss(0, lab.CAL_SIGMA) for aid, v in sorted(bias.items())}
    return scene, robot, stream, cal


def sim_job(args):
    import window_study as ws
    world, seed, variant = args
    poll_ms, aoa = SIM_VARIANTS[variant]
    scene, robot, stream, cal = make_market(world, seed, poll_ms, aoa)
    sc = json.loads(json.dumps(scene))
    sc["tracking"] = {"smooth": 0}
    sc["calibration"] = {"anchors": {aid: {"bias": v, "scale": 1.0} for aid, v in cal.items()}}
    eng = AoaEngine(aoa) if aoa else tracker.Engine()
    eng.configure(sc, True, nlos_mod=ws.sim.scene_mod)
    rows = []
    for recv, dev, payload in stream:
        for tag in eng.ingest(dev, payload, recv):
            e = eng.estimate(tag, recv)
            if e:
                rows.append((recv, e["x"], e["y"], e["valid"], False))
    m = ws.measure(robot, rows)
    track = rows if seed == SIM_SEED else None
    return world, seed, variant, m, track


SIM_SEED = 1200


def paired(per_route):
    """Variant minus CALF-UWB on the same routes: {world: {variant: {metric:
    (mean difference, 2 x standard error, routes better)}}}. A difference
    smaller than its 2 x SE is not shown to be real."""
    out = {}
    for w, by in per_route.items():
        base = by["CALF-UWB"]
        for v, runs in by.items():
            if v == "CALF-UWB":
                continue
            for k in ("shelf", "stop", "move"):
                d = [runs[s][k] - base[s][k] for s in runs if s in base
                     and runs[s][k] == runs[s][k] and base[s][k] == base[s][k]]
                if len(d) < 2:
                    continue
                m = sum(d) / len(d)
                se = math.sqrt(sum((x - m) ** 2 for x in d) / (len(d) - 1) / len(d))
                out.setdefault(w, {}).setdefault(v, {})[k] = (m, 2 * se, sum(x < 0 for x in d), len(d))
    return out


def sim_main(runs=20):
    import window_study as ws
    jobs = [(w, SIM_SEED + k, v) for w in ws.WORLDS for k in range(runs) for v in SIM_VARIANTS]
    acc, tracks, per_route = {}, {}, {}
    with Pool(8) as pool:
        for i, (w, seed, v, m, track) in enumerate(pool.imap_unordered(sim_job, jobs, chunksize=2), 1):
            acc.setdefault(w, {}).setdefault(v, []).append(m)
            per_route.setdefault(w, {}).setdefault(v, {})[seed] = {k: m[k] for k in ("shelf", "stop", "move")}
            if track is not None:
                tracks.setdefault(w, {})[v] = [(round(r[0]), round(r[1], 3), round(r[2], 3), r[3])
                                               for r in track]
            if i % 25 == 0:
                print(f"  {i}/{len(jobs)}", flush=True)
    summary = {}
    for w, by in acc.items():
        for v, ms in by.items():
            agg = {}
            for f in ms[0]:
                vals = [m[f] for m in ms if m[f] == m[f]]
                agg[f] = sum(vals) / len(vals) if vals else float("nan")
            agg["runs"] = len(ms)
            summary.setdefault(w, {})[v] = agg
    # the true route of the figure's seed, per world
    truth = {}
    for w in ws.WORLDS:
        robot = make_market(w, SIM_SEED, 400, None)[1]
        truth[w] = [robot.segs[0][2]] + [s[3] for s in robot.segs]
    OUT.mkdir(parents=True, exist_ok=True)
    SIM_JSON.write_text(json.dumps({"runs": runs, "seed": SIM_SEED, "variants": SIM_VARIANTS,
                                    "aoa_blocked_deg": AOA_BLOCKED_DEG, "summary": summary,
                                    "paired": paired(per_route), "tracks": tracks, "truth": truth}),
                        encoding="utf-8")
    for w in ws.WORLDS:
        print(f"\n== {w} ({runs} rute, cm)")
        print(f"{'varian':22s} {'rak':>6s} {'diam':>6s} {'gerak':>6s} {'goyah':>6s} {'valid':>6s}")
        for v in SIM_VARIANTS:
            r = summary[w][v]
            pd = paired(per_route).get(w, {}).get(v, {}).get("shelf")
            vs = f"  rak vs CALF {pd[0] * 100:+.2f} ± {pd[1] * 100:.2f} ({pd[2]}/{pd[3]} rute lebih baik)" if pd else ""
            print(f"{v:22s} {r['shelf'] * 100:6.1f} {r['stop'] * 100:6.1f} {r['move'] * 100:6.1f} "
                  f"{r['jitter'] * 100:6.2f} {r['avail'] * 100:5.0f}%{vs}")
    print("\nwrote", SIM_JSON)


# ---------------------------------------------------------------------------
# report
# ---------------------------------------------------------------------------

WORLD_LABEL = {"kosong": "kosong (0 pembeli)", "sepi": "sepi (2 pembeli)", "ramai": "ramai (5 pembeli)"}
VARIANT_LABEL = {"CALF-UWB": "CALF-UWB", "laju x2": "laju ranging ×2", "AoA 2.4 deg": "AoA 2,4°",
                 "AoA 4 deg": "AoA 4°", "laju x2 + AoA 4 deg": "laju ×2 + AoA 4°"}
VARIANT_COLOR = {"CALF-UWB": "#1f4e79", "laju x2": "#6a994e", "AoA 2.4 deg": "#c9a227",
                 "AoA 4 deg": "#e9c46a", "laju x2 + AoA 4 deg": "#bc6c25"}
REAL_VARIANTS = ("CALF-UWB", "+ RSL", "+ FP", "+ RSL + FP")
REAL_COLOR = {"CALF-UWB": "#1f4e79", "+ RSL": "#c9a227", "+ FP": "#6a994e", "+ RSL + FP": "#bc6c25"}


def short_name(name):
    return name.replace("session-10.23.2026-", "").replace("SESSION-TEST-01-", "").split("-1.")[0][:22]


def report_main():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from openpyxl import Workbook
    import window_study as ws
    from report_excel import Sheet

    real = json.loads(REAL_JSON.read_text(encoding="utf-8"))
    sim = json.loads(SIM_JSON.read_text(encoding="utf-8"))
    an = real["analysis"]
    imgs = []

    # 1) signal power vs residual on the recordings
    fig, axs = plt.subplots(1, 3, figsize=(17, 4.8))
    x = [b[0] for b in an["by_rx"]]
    axs[0].bar(x, [b[1] * 100 for b in an["by_rx"]], width=1.6, color="#c9a227")
    for b in an["by_rx"]:
        axs[0].text(b[0], 0.15 if b[1] >= 0 else -0.15, f"n={b[2]}", ha="center",
                    va="bottom" if b[1] >= 0 else "top", fontsize=7, rotation=90)
    axs[0].axhline(0, color="#555", lw=0.8)
    axs[0].set_ylim(-3, 3)
    axs[0].set_xlabel("rx_power (dBm)")
    axs[0].set_ylabel("residu jarak, nilai tengah (cm)")
    axs[0].set_title("RSL: residu hampir datar (±1 cm)\nkoreksi kuat sinyal tidak punya yang dikoreksi", fontsize=10)
    xd = [b[0] for b in an["by_dfp"]]
    axs[1].bar(xd, [b[1] * 100 for b in an["by_dfp"]], width=0.8, color="#6a994e")
    axs[1].axhline(0, color="#555", lw=0.8)
    axs[1].set_xlabel("dfp = (rx − fp) − nilai tengah anchor itu (dB)")
    axs[1].set_ylabel("residu jarak, nilai tengah (cm)")
    axs[1].set_title("Jalur pertama lemah → jarak cenderung kepanjangan", fontsize=10)
    xs = [b[0] for b in an["long_share_by_dfp"]]
    axs[2].bar(xs, [b[1] * 100 for b in an["long_share_by_dfp"]], width=0.8, color="#bc6c25")
    days = sorted(real["folds"])
    top = max(b[1] for b in an["long_share_by_dfp"]) * 100
    for day, f in real["folds"].items():
        if f.get("fp"):
            train = [d for d in days if d != day][0]
            axs[2].axvline(f["fp"][0], color="#1f4e79", ls="--", lw=1)
            axs[2].text(f["fp"][0] - 0.25, top * 0.98, f"ambang dari data {train[6:8]}-{train[4:6]}",
                        fontsize=8, color="#1f4e79", rotation=90, va="top", ha="right")
    axs[2].set_xlabel("dfp (dB)")
    axs[2].set_ylabel("bagian jarak dengan e > 2,5 (%)")
    axs[2].set_title("Bagian jarak kepanjangan (e > 2,5) naik 4–10×\nbila dfp > 5 dB", fontsize=10)
    for a in axs:
        a.grid(alpha=0.25)
    fig.suptitle(f"Rekaman asli: {an['n']} pembaruan jarak CALF-UWB dari {len({r['recording'] for r in real['results']})}"
                 f" rekaman (rx − fp nilai tengahnya {an['rx_minus_fp_median']:.1f} dB)", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    p1 = OUT / "lanjutan_1_sinyal_rekaman.png"
    fig.savefig(p1, dpi=110)
    plt.close(fig)
    imgs.append((p1, "Kuat sinyal vs residu jarak pada rekaman asli"))

    # 2) the recordings, every variant
    recs = []
    for r in real["results"]:
        if r["recording"] not in recs:
            recs.append(r["recording"])
    by = {(r["recording"], r["variant"]): r for r in real["results"]}
    fig, axs = plt.subplots(1, 3, figsize=(18, 5))
    for k, (field, lab_, scale) in enumerate((("consistency", "konsistensi jarak (cm)", 100),
                                             ("cep95", "sebaran diam CEP95 (cm)", 100),
                                             ("jitter", "kegoyahan garis (cm)", 100))):
        ax = axs[k]
        w = 0.8 / len(REAL_VARIANTS)
        for j, v in enumerate(REAL_VARIANTS):
            xs_, ys_ = [], []
            for i, rec in enumerate(recs):
                r = by.get((rec, v))
                if r and r[field] == r[field]:
                    xs_.append(i + (j - 1.5) * w)
                    ys_.append(min(r[field] * scale, 40))
            ax.bar(xs_, ys_, width=w, color=REAL_COLOR[v], label=v)
        ax.set_xticks(range(len(recs)))
        ax.set_xticklabels([short_name(n) for n in recs], rotation=60, ha="right", fontsize=8)
        ax.set_ylabel(lab_)
        ax.grid(alpha=0.25, axis="y")
        ax.set_title(lab_ + (" (dipotong di 40)" if field == "jitter" else ""), fontsize=10)
    axs[0].legend(fontsize=8)
    fig.suptitle("Rekaman asli diputar ulang: CALF-UWB vs + koreksi kuat sinyal (RSL) vs + tanda jalur pertama (FP)"
                 " — ambang dilatih di hari lain", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    p2 = OUT / "lanjutan_2_rekaman_varian.png"
    fig.savefig(p2, dpi=110)
    plt.close(fig)
    imgs.append((p2, "Rekaman asli: setiap varian"))

    # 3) simulation: bars with the paired difference
    worlds = list(WORLD_LABEL)
    fig, axs = plt.subplots(1, 3, figsize=(18, 5))
    for k, (field, lab_) in enumerate((("shelf", "bacaan di rak (cm)"), ("move", "galat bergerak (cm)"),
                                      ("jitter", "kegoyahan garis (cm)"))):
        ax = axs[k]
        w = 0.8 / len(SIM_VARIANTS)
        for j, v in enumerate(SIM_VARIANTS):
            xs_ = [i + (j - 2) * w for i in range(len(worlds))]
            ys_ = [sim["summary"][wd][v][field] * 100 for wd in worlds]
            ax.bar(xs_, ys_, width=w, color=VARIANT_COLOR[v], label=VARIANT_LABEL[v])
            for xx, yy in zip(xs_, ys_):
                ax.text(xx, yy, f"{yy:.1f}", ha="center", va="bottom", fontsize=6.5)
        ax.set_xticks(range(len(worlds)))
        ax.set_xticklabels([WORLD_LABEL[wd] for wd in worlds], fontsize=9)
        ax.set_ylabel(lab_)
        ax.grid(alpha=0.25, axis="y")
        ax.set_title(lab_, fontsize=10)
    axs[0].axhspan(2, 4, color="#6a994e", alpha=0.12, lw=0)
    axs[0].legend(fontsize=8, loc="upper left")
    fig.suptitle(f"Simulasi pasar: CALF-UWB dan riset lanjutan ({sim['runs']} rute per dunia, rute yang sama;"
                 " hijau: target 2–4 cm)", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    p3 = OUT / "lanjutan_3_simulasi.png"
    fig.savefig(p3, dpi=110)
    plt.close(fig)
    imgs.append((p3, "Simulasi: setiap varian"))

    # 4) trajectories vs ground truth (the usual figure: black = true route)
    sc = ws.lab.scene_for(4)
    W, D = sc["room"]["width"], sc["room"]["depth"]
    fig, axs = plt.subplots(len(worlds), len(SIM_VARIANTS), figsize=(4.2 * len(SIM_VARIANTS), 3.8 * len(worlds)))
    for i, wd in enumerate(worlds):
        truth = sim["truth"][wd]
        for j, v in enumerate(SIM_VARIANTS):
            ax = axs[i][j]
            ax.add_patch(plt.Rectangle((0, 0), W, D, fill=False, ec="#777", lw=1))
            for a in sc["anchors"]:
                ax.plot(a["x"], a["y"], "^", color="#c62828", ms=6)
            ax.plot([p[0] for p in truth], [p[1] for p in truth], "-", color="#111", lw=2.0)
            tr = sim["tracks"].get(wd, {}).get(v, [])
            ax.plot([r[1] for r in tr], [r[2] for r in tr], "-", color=VARIANT_COLOR[v], lw=0.9)
            ax.set_xlim(-0.5, W + 0.5)
            ax.set_ylim(-0.5, D + 0.5)
            ax.set_aspect("equal")
            ax.set_xticks([])
            ax.set_yticks([])
            m = sim["summary"][wd][v]
            ax.set_title(f"{VARIANT_LABEL[v]} — {wd}\nrak {m['shelf'] * 100:.1f} · gerak {m['move'] * 100:.1f} cm"
                         f" (rata-rata {sim['runs']} rute)", fontsize=8.5)
    fig.suptitle(f"Lintasan rute {sim['seed']} — hitam: gerak troli sebenarnya (ground truth), warna: posisi hasil"
                 " hitung", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    p4 = OUT / "lanjutan_4_lintasan.png"
    fig.savefig(p4, dpi=105)
    plt.close(fig)
    imgs.append((p4, "Lintasan vs ground truth"))

    # workbook
    wb = Workbook()
    s = Sheet(wb, "Ringkasan", [30, 14, 14, 14, 14, 14, 22], first=True)
    s.title("Riset lanjutan di atas CALF-UWB (tidak dipakai server)",
            "Improved TWR (RSL, FP, laju x2) dan AoA. cm kecuali disebut lain.")
    s.section("Simulasi pasar (rute yang sama untuk setiap varian)")
    rows = []
    for wd in worlds:
        for v in SIM_VARIANTS:
            m = sim["summary"][wd][v]
            pdiff = sim["paired"].get(wd, {}).get(v, {}).get("shelf")
            rows.append([wd, VARIANT_LABEL[v], m["shelf"] * 100, m["stop"] * 100, m["move"] * 100,
                         m["jitter"] * 100,
                         f"{pdiff[0] * 100:+.2f} ± {pdiff[1] * 100:.2f} ({pdiff[2]}/{pdiff[3]} rute)" if pdiff
                         else "pembanding"])
    s.table(["Dunia", "Varian", "Bacaan di rak", "Galat diam", "Galat bergerak", "Kegoyahan",
             "Rak vs CALF-UWB (± 2 SE)"], rows, [None, None, "cm", "cm", "cm", "cm", None])
    s.note("Rak vs CALF-UWB: selisih bacaan di rak pada rute yang sama, ± 2 kali galat baku. Selisih yang lebih "
           "kecil dari ± itu belum terbukti nyata. AoA butuh hardware baru (dua antena, keluarga DW3000).", 3)
    s.section("Rekaman asli (ambang dilatih di hari lain)")
    rows = [[r["recording"], r["variant"], r["cep95"] * 100, r["consistency"] * 100, r["jitter"] * 100,
             r["residual_rms"] * 100, r["flagged"], r["available"]] for r in real["results"]]
    s2 = Sheet(wb, "Rekaman", [34, 12, 12, 12, 12, 12, 12, 12])
    s2.title("Rekaman asli diputar ulang", "Tanpa ground truth: kestabilan, konsistensi, kegoyahan.")
    s2.table(["Rekaman", "Varian", "Diam CEP95", "Konsistensi", "Kegoyahan", "Residu RMS", "Ditandai FP",
              "Valid"], rows, [None, None, "cm", "cm", "cm", "cm", "pct", "pct"])
    s3 = Sheet(wb, "Sinyal", [16, 16, 12, 16, 16, 12])
    s3.title("Kuat sinyal vs residu jarak", "Nilai tengah residu CALF-UWB per kelompok.")
    s3.table(["rx_power (dBm)", "Residu (cm)", "n"], [[b[0], b[1] * 100, b[2]] for b in an["by_rx"]],
             ["db", "cm", "n"])
    s3.gap()
    s3.table(["dfp (dB)", "Residu (cm)", "n"], [[b[0], b[1] * 100, b[2]] for b in an["by_dfp"]], ["db", "cm", "n"])
    s3.gap()
    s3.table(["dfp (dB)", "e > 2,5", "n"], [[b[0], b[1], b[2]] for b in an["long_share_by_dfp"]],
             ["db", "pct", "n"])
    s4 = Sheet(wb, "Gambar", [12] * 12)
    s4.title("Gambar", "Semua gambar riset lanjutan.")
    for img, cap in imgs:
        s4.image(img, 1100, cap)
    path = OUT / "Hasil_Riset_Lanjutan.xlsx"
    try:
        wb.save(path)
    except PermissionError:
        path = OUT / "Hasil_Riset_Lanjutan_baru.xlsx"
        wb.save(path)
    for img, _c in imgs:
        print("wrote", img)
    print("wrote", path)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("what", choices=["real", "sim", "report"])
    ap.add_argument("--runs", type=int, default=20, help="routes per world (sim)")
    a = ap.parse_args()
    if a.what == "real":
        real_main()
    elif a.what == "sim":
        sim_main(a.runs)
    else:
        report_main()


if __name__ == "__main__":
    main()
