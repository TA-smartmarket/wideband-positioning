"""Replay real recordings through every algorithm and compare them.

    python tools/replay_compare.py server/logs/20261023-*.jsonl --out docs/laporan-akurasi

Every recorded packet is fed again through the tracker with each
algorithm's settings (tools/tracker_lab.py: the server's code with the
studies' settings adjustable). There is no ground
truth in these recordings (no marks, nothing measured), so only what can be
measured honestly is reported:

  stops         found from the RAW ranges, the same for every algorithm: every
                anchor's 1-s median range changed by < 8 cm over 2 s
  spread        CEP50 / CEP95 of the positions shown during a stop, around
                their mean (stability; 1 s after the stop began)
  consistency   at a stop: how well the shown position fits the measured
                ranges. Per anchor  median range - distance(position, anchor),
                minus the common offset (their mean), RMS over the anchors.
                Wrong anchor coordinates make it large for EVERY algorithm;
                compare the algorithms with each other, not with zero
  jitter        RMS second difference of the outputs while moving (stipple)
  outside       share of the shown positions more than 0.3 m outside the room
  available     share of outputs flagged valid by the validity gate
"""
import argparse
import json
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from openpyxl import Workbook  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "server"))
sys.path.insert(0, str(ROOT / "tools"))
import scene as scene_mod  # noqa: E402
import tracker_lab  # noqa: E402
from report_excel import Sheet  # noqa: E402

OLD = {"min_anchors": 1, "valid_sigma": 99.0, "fresh_ms": 1e9, "anchor_offsets": 0, "huber_k_long": 0.0}
ZL = {"zero_velocity": 1, "still_long": 1.0}
# name -> tracking settings on top of the defaults (= G)
ALGOS = [
    ("A. EKF saja (lama)", dict(OLD, post_filter="none")),
    ("C. jendela 20 x 20", dict(OLD, pre_filter="mean", pre_window=20, post_filter="mean", post_window=20)),
    ("G. server (default)", {}),
    ("G + jendela C (20 x 20)", dict(pre_filter="mean", pre_window=20, post_filter="mean", post_window=20)),
    ("G + mode diam (Z)", dict(zero_velocity=1, post_filter="none")),
    ("G + Z + tolak jarak panjang (L)", dict(ZL, post_filter="none")),
    ("G + Z + L + rata 3 (di web)", dict(ZL, post_filter="mean", post_window=3)),
]
COLORS = ["#bbbbbb", "#c9a227", "#8c8c8c", "#e9c46a", "#6a994e", "#6fa8dc", "#1f4e79"]
STILL_CM, STILL_SPAN_S, MIN_STOP_S, SETTLE_S = 0.08, 2.0, 3.0, 1.0


def load(path):
    meta, rx = None, []
    for line in open(path, encoding="utf-8"):
        o = json.loads(line)
        if o.get("k") == "meta":
            meta = o
        elif o.get("k") == "rx":
            rx.append(o)
    return meta, rx


def replay(meta, rx, settings):
    scene = json.loads(json.dumps(meta["scene"]))
    scene["tracking"] = dict(settings)
    eng = tracker_lab.LabEngine()
    eng.configure(scene, meta.get("nlos_enabled", True), nlos_mod=scene_mod)
    out = []
    for o in rx:
        for tag in eng.ingest(o["dev"], o["p"], o["t"]):
            e = eng.estimate(tag, o["t"])
            if e:
                out.append((o["t"], e["x"], e["y"], e["valid"]))
    return out


def raw_ranges(rx):
    """{anchor: [(server ms, raw range)]}, each measurement once."""
    seen, out = set(), {}
    for o in rx:
        for g in o["p"].get("ranges", []):
            key = (g.get("src"), g.get("dst"), g.get("seq"), g.get("ts"))
            if key in seen or g.get("raw") is None:
                continue
            seen.add(key)
            a = g["src"] if str(g.get("src", "")).startswith("anchor") else g.get("dst")
            out.setdefault(a, []).append((o["t"], float(g["raw"])))
    return out


def median(v):
    v = sorted(v)
    n = len(v)
    return v[n // 2] if n % 2 else 0.5 * (v[n // 2 - 1] + v[n // 2])


def find_stops(ranges, t0, t1):
    """Stops from the raw ranges only: (start ms, end ms)."""
    def med(a, t):
        v = [r for tt, r in ranges[a] if t - 500 <= tt <= t + 500 and 0.05 < r < 50]
        return median(v) if len(v) >= 2 else None
    still, t = [], t0 + 1000
    while t < t1 - 1000:
        ok, n = True, 0
        for a in ranges:
            m1, m0 = med(a, t), med(a, t - STILL_SPAN_S * 1000)
            if m1 is None or m0 is None:
                continue
            n += 1
            ok &= abs(m1 - m0) < STILL_CM
        still.append((t, ok and n >= 3))
        t += 250
    stops, start = [], None
    for t, s in still + [(t1, False)]:
        if s and start is None:
            start = t - STILL_SPAN_S * 1000           # it already stood that long
        elif not s and start is not None:
            if t - start >= MIN_STOP_S * 1000:
                stops.append((start, t))
            start = None
    return stops


def pct(v, q):
    v = sorted(v)
    return v[min(int(q * (len(v) - 1) + 0.5), len(v) - 1)] if v else float("nan")


def jitter(rows):
    if len(rows) < 3:
        return float("nan")
    acc = [(a[1] - 2 * b[1] + c[1]) ** 2 + (a[2] - 2 * b[2] + c[2]) ** 2
           for a, b, c in zip(rows, rows[1:], rows[2:])]
    return math.sqrt(sum(acc) / len(acc))


def measure(meta, rows, ranges, stops):
    sc = meta["scene"]
    W, D, tz = sc["room"]["width"], sc["room"]["depth"], sc.get("tag_z", 1.0)
    anchors = {a["id"]: (a["x"], a["y"], a["z"]) for a in sc["anchors"]}
    in_stop = lambda t: any(s + SETTLE_S * 1000 <= t <= e for s, e in stops)  # noqa: E731
    cep50, cep95, cons = [], [], []
    for s, e in stops:
        pts = [(x, y) for t, x, y, _v in rows if s + SETTLE_S * 1000 <= t <= e]
        if len(pts) < 3:
            continue
        mx = sum(p[0] for p in pts) / len(pts)
        my = sum(p[1] for p in pts) / len(pts)
        d = [math.hypot(p[0] - mx, p[1] - my) for p in pts]
        cep50.append(pct(d, 0.5))
        cep95.append(pct(d, 0.95))
        res = []
        for a, (ax, ay, az) in anchors.items():
            v = [r for t, r in ranges.get(a, []) if s <= t <= e and 0.05 < r < 50]
            if len(v) >= 3:
                res.append(median(v) - math.sqrt((mx - ax) ** 2 + (my - ay) ** 2 + (tz - az) ** 2))
        if len(res) >= 3:
            m = sum(res) / len(res)
            cons.append(math.sqrt(sum((r - m) ** 2 for r in res) / len(res)))
    moving = [r for r in rows if not in_stop(r[0])]
    out = sum(1 for _t, x, y, _v in rows if x < -0.3 or y < -0.3 or x > W + 0.3 or y > D + 0.3)
    mean = lambda v: sum(v) / len(v) if v else float("nan")  # noqa: E731
    return {"cep50": mean(cep50), "cep95": mean(cep95), "consistency": mean(cons),
            "jitter": jitter(moving), "outside": out / len(rows) if rows else float("nan"),
            "available": sum(r[3] for r in rows) / len(rows) if rows else float("nan"),
            "n_stops": len(cep50)}


def fig_tracks(name, meta, results, stops_xy, path):
    sc = meta["scene"]
    W, D = sc["room"]["width"], sc["room"]["depth"]
    fig, axes = plt.subplots(2, 4, figsize=(19, 8.6))
    for i, (algo, _s) in enumerate(ALGOS):
        ax = axes[i // 4][i % 4]
        ax.add_patch(plt.Rectangle((0, 0), W, D, fill=False, ec="#777", lw=1))
        for a in sc["anchors"]:
            ax.plot(a["x"], a["y"], "^", color="#c62828", ms=8)
        rows, m = results[algo]
        ax.plot([r[1] for r in rows], [r[2] for r in rows], "-", color=COLORS[i], lw=1.0)
        ax.set_xlim(-0.8, W + 0.8)
        ax.set_ylim(-0.8, D + 0.8)
        ax.set_aspect("equal")
        ax.grid(alpha=0.25)
        ax.set_title(f"{algo}\ndiam CEP95 {m['cep95'] * 100:.1f} cm · goyah {m['jitter'] * 100:.1f} cm · "
                     f"luar {m['outside'] * 100:.0f}%", fontsize=9)
    ax = axes[1][3]
    ax.axis("off")
    ax.text(0.0, 0.95,
            "Rekaman asli diputar ulang lewat tracker\nserver dengan tiap algoritma (data SAMA).\n\n"
            "TIDAK ada ground truth di rekaman ini:\nyang dibandingkan hanya kestabilan,\n"
            "kegoyahan garis dan posisi di luar ruangan.\n\n"
            "Segitiga merah: anchor; kotak: dinding\n(ukuran ruangan dari scene rekaman)\n\n"
            f"Titik berhenti (dari jarak mentah): {len(stops_xy)}", va="top", fontsize=10)
    fig.suptitle(f"Rekaman {name} — lintasan tiap algoritma", fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--out", default=str(ROOT / "docs" / "laporan-akurasi"))
    a = ap.parse_args()
    out_dir = Path(a.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    table, imgs = [], []
    for p in a.logs:
        meta, rx = load(p)
        name = meta.get("name") or Path(p).stem
        ranges = raw_ranges(rx)
        stops = find_stops(ranges, rx[0]["t"], rx[-1]["t"])
        results = {}
        for algo, settings in ALGOS:
            rows = replay(meta, rx, settings)
            results[algo] = (rows, measure(meta, rows, ranges, stops))
        short = name.replace("session-10.23.2026-", "").split("-1.")[0][:28]
        img = out_dir / f"replay_{short}.png"
        fig_tracks(name, meta, results, stops, img)
        imgs.append((img, f"Rekaman {name}"))
        dur = (rx[-1]["t"] - rx[0]["t"]) / 1000
        print(f"\n== {name}: {dur:.0f} s, {len(stops)} berhenti (dari jarak mentah)")
        print(f"{'algoritma':34s} {'CEP50':>6s} {'CEP95':>6s} {'konsis':>7s} {'goyah':>6s} {'luar':>5s} {'valid':>6s}")
        for algo, _s in ALGOS:
            m = results[algo][1]
            print(f"{algo:34s} {m['cep50'] * 100:6.1f} {m['cep95'] * 100:6.1f} {m['consistency'] * 100:7.1f} "
                  f"{m['jitter'] * 100:6.1f} {m['outside'] * 100:4.0f}% {m['available'] * 100:5.0f}%")
            table.append([name, dur, len(stops), algo, m["cep50"] * 100, m["cep95"] * 100,
                          m["consistency"] * 100, m["jitter"] * 100, m["outside"], m["available"]])
    # averages over the recordings
    avg = []
    for algo, _s in ALGOS:
        rs = [r for r in table if r[3] == algo]
        avg.append([algo] + [sum(r[i] for r in rs if r[i] == r[i]) / max(1, sum(1 for r in rs if r[i] == r[i]))
                             for i in range(4, 10)])
    print("\n== rata-rata semua rekaman")
    for r in avg:
        print(f"{r[0]:34s} CEP50 {r[1]:5.1f} CEP95 {r[2]:5.1f} konsistensi {r[3]:5.1f} goyah {r[4]:5.1f} "
              f"luar {r[5] * 100:3.0f}% valid {r[6] * 100:3.0f}%")

    wb = Workbook()
    s = Sheet(wb, "Ringkasan", [36, 12, 12, 12, 12, 12, 12], first=True)
    s.title("Replay rekaman asli dengan tiap algoritma",
            "Rekaman diputar ulang lewat server/tracker.py. TANPA ground truth: hanya kestabilan saat diam, "
            "konsistensi dengan jarak terukur, kegoyahan garis dan posisi di luar ruangan. cm kecuali disebut lain.")
    s.section("Rata-rata semua rekaman")
    s.table(["Algoritma", "Diam CEP50", "Diam CEP95", "Konsistensi jarak", "Kegoyahan garis", "Di luar ruangan",
             "Valid"], avg, [None, "cm", "cm", "cm", "cm", "pct", "pct"])
    s.note("Diam CEP50/95: sebaran posisi saat berhenti di sekitar rata-ratanya (berhenti dicari dari jarak "
           "mentah, sama untuk semua algoritma). Konsistensi jarak: RMS selisih jarak terukur dan jarak dari posisi "
           "ke tiap anchor setelah offset bersama dibuang; koordinat anchor yang salah membuatnya besar untuk SEMUA "
           "algoritma. Di luar ruangan: posisi > 0,3 m di luar dinding (pasti salah).", 4)
    s.section("Per rekaman")
    s.table(["Rekaman", "Durasi (s)", "Berhenti", "Algoritma", "Diam CEP50", "Diam CEP95", "Konsistensi",
             "Kegoyahan", "Di luar", "Valid"], table,
            [None, "s", "n", None, "cm", "cm", "cm", "cm", "pct", "pct"])
    s = Sheet(wb, "Lintasan", [12] * 12)
    s.title("Lintasan tiap algoritma per rekaman", "Tidak ada garis hitam: rekaman ini tidak punya ground truth.")
    for img, cap in imgs:
        s.image(img, 1100, cap)
    path = out_dir / "Replay_Rekaman.xlsx"
    try:
        wb.save(path)
    except PermissionError:
        path = out_dir / "Replay_Rekaman_baru.xlsx"
        wb.save(path)
    print("\nwrote", path)


if __name__ == "__main__":
    main()
