"""Simulation results report for the algorithms A-G: figures + Excel.

    python tools/smoothing_study.py --runs 10    # the numbers (docs/smoothing_study.json)
    python tools/sim_report.py                   # figures + workbook from them

Writes docs/hasil-simulasi/*.png and docs/hasil-simulasi/Hasil_Simulasi.xlsx.
The tables come from smoothing_study.json (10 Monte-Carlo runs); the figures
show one run of the same simulation so a single track can be followed.
"""
import json
import math
import random
import shutil
import sys
from multiprocessing import Pool
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from openpyxl import Workbook  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
import simulate as sim  # noqa: E402
import smoothing_study as st  # noqa: E402
from report_excel import Sheet  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "docs" / "hasil-simulasi"
SEED = 104
COLORS = ["#888888", "#c9a227", "#e76f51", "#f4a261", "#6a994e", "#6fa8dc", "#1f4e79"]
LAPS = 3


# ---------------------------------------------------------------------------
# simulation helpers
# ---------------------------------------------------------------------------

def make_data(name, seed, laps=1):
    sc = st.SCENARIOS[name]
    sim.BODY_BIAS, sim.OUTLIER_P = (0.0, 0.0) if sc.get("los") else (0.15, 0.04)
    sim.POLL_PERIOD_MS = sc.get("poll_ms", 80 + 4 * 3 * 7)
    walk = sc["walk"] + sc["walk"][1:] * (laps - 1)
    segs = st.build_segs(walk, sc["speed"], sc["pause"], sc["start"])
    dur = segs[-1][1] + 1.0
    rng = random.Random(seed)
    ex, _bias = sim.simulate_radio(sc["scene"], segs, rng, dur)
    if sc.get("dropout"):
        t0, t1, keep = sc["dropout"]
        ex = [e for e in ex if not (t0 * 1000 <= e["t"] <= t1 * 1000 and e["anchor"] != keep)]
    stream = sim.telemetry_stream(sc["scene"], ex, random.Random(rng.random()), dur, "1.0.32")
    return sc["scene"], segs, dur, stream


def track_of(key, scene, stream):
    if key == "fable-MA500":
        tr, _ = sim.run_new(scene, stream, None, st.tracking("none", "none", "old"))
        return st.fable_smooth(tr)
    tr, _ = sim.run_new(scene, stream, None, st.tracking(*key.split("|")))
    return tr


def sampled(segs, tr, dur, warm=3.0):
    """Displayed position at 10 Hz: (t_s, x, y, valid, truth_x, truth_y, moving, since)."""
    out, j = [], 0
    t = tr[0][0] + warm * 1000
    while t < dur * 1000:
        while j + 1 < len(tr) and tr[j + 1][0] <= t:
            j += 1
        tx, ty = sim.truth_at(segs, t / 1000)
        mv, since = st.moving_at(segs, t / 1000)
        out.append((t / 1000, tr[j][1], tr[j][2], tr[j][3] and t - tr[j][0] <= 1500, tx, ty, mv, since))
        t += 100
    return out


def lap_job(args):
    seed, key = args
    scene, segs, dur, stream = make_data("market-los", seed, LAPS)
    s = sampled(segs, track_of(key, scene, stream), dur)
    lap_len = segs[-1][1] / LAPS
    per = [[] for _ in range(LAPS)]
    for t, x, y, _ok, tx, ty, mv, since in s:
        if not mv and since > 1.0:
            per[min(int(t / lap_len), LAPS - 1)].append(math.hypot(x - tx, y - ty))
    return key, [sum(v) / len(v) for v in per]


# ---------------------------------------------------------------------------
# figures
# ---------------------------------------------------------------------------

def room_axes(ax, scene):
    w, d = scene["room"]["width"], scene["room"]["depth"]
    ax.add_patch(plt.Rectangle((0, 0), w, d, fill=False, ec="#777", lw=1))
    for a in scene["anchors"]:
        ax.plot(a["x"], a["y"], "^", color="#c62828", ms=8)
    ax.set_xlim(-0.6, w + 0.6)
    ax.set_ylim(-0.6, d + 0.6)
    ax.set_aspect("equal")
    ax.grid(alpha=0.25)


def fig_tracks(rows, path):
    scene, segs, dur, stream = make_data("market", SEED)
    truth = [segs[0][2]] + [s_[3] for s_ in segs]
    fig, axes = plt.subplots(2, 4, figsize=(19, 8.6))
    for i, (key, label) in enumerate(st.NAMED):
        ax = axes[i // 4][i % 4]
        room_axes(ax, scene)
        ax.plot([p[0] for p in truth], [p[1] for p in truth], "-", color="#111", lw=2.2)
        tr = track_of(key, scene, stream)
        ax.plot([p[1] for p in tr], [p[2] for p in tr], "-", color=COLORS[i], lw=1.1)
        m = rows.get(("market", key), {})
        ax.set_title(f"{label}\nMAE {m.get('mae', 0) * 100:.1f} cm · "
                     f"diam {m.get('mae_still', 0) * 100:.1f} cm · "
                     f"goyah {m.get('jitter', 0) * 100:.1f} cm", fontsize=9)
    ax = axes[1][3]
    ax.axis("off")
    ax.text(0.0, 0.95,
            "Garis hitam: lintasan sebenarnya\n(troli 0,6 m/s, berhenti 6 s di tiap sudut)\n\n"
            "Garis berwarna: posisi yang ditampilkan\nalgoritma, data simulasi yang SAMA\n\n"
            "Segitiga merah: anchor\n\n"
            "Angka di judul: rata-rata 10 simulasi\n(MAE = galat rata-rata ke posisi\n"
            "sebenarnya; goyah = patah-patahnya garis)", va="top", fontsize=10)
    fig.suptitle("Lintasan tiap algoritma — ruangan 9,5 x 5,9 m, laju anchor seperti hardware",
                 fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(path, dpi=120)
    plt.close(fig)


def fig_stop(path):
    """Displayed positions at one stop, relative to the true point."""
    scene, segs, dur, stream = make_data("market", SEED)
    stop = (8.5, 1.0)
    fig, axes = plt.subplots(2, 4, figsize=(17, 9.8))
    lim = 30
    for i, (key, label) in enumerate(st.NAMED):
        ax = axes[i // 4][i % 4]
        s = sampled(segs, track_of(key, scene, stream), dur)
        pts = [((x - tx) * 100, (y - ty) * 100) for t, x, y, ok, tx, ty, mv, since in s
               if not mv and since > 1.0 and abs(tx - stop[0]) < 1e-6 and abs(ty - stop[1]) < 1e-6]
        inside = [p for p in pts if abs(p[0]) < lim and abs(p[1]) < lim]
        for r, c in ((2, "#2e7d32"), (4, "#2e7d32")):
            ax.add_patch(plt.Circle((0, 0), r, fill=False, ec=c, ls="--", lw=1))
        ax.plot([p[0] for p in inside], [p[1] for p in inside], ".", color=COLORS[i], ms=5)
        if pts:
            mx = sum(p[0] for p in pts) / len(pts)
            my = sum(p[1] for p in pts) / len(pts)
            ax.plot(mx, my, "x", color="black", ms=10, mew=2)
            err = sum(math.hypot(*p) for p in pts) / len(pts)
            note = f"galat rata-rata {err:.1f} cm"
            if len(inside) < len(pts):
                note += f"\n{len(pts) - len(inside)} titik di luar grafik"
        else:
            note = "tidak ada titik"
        ax.plot(0, 0, "+", color="#c62828", ms=14, mew=2)
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)
        ax.set_aspect("equal")
        ax.grid(alpha=0.3)
        ax.set_title(f"{label}\n{note}", fontsize=9)
        ax.set_xlabel("ΔX (cm)", fontsize=8)
        ax.set_ylabel("ΔY (cm)", fontsize=8)
    ax = axes[1][3]
    ax.axis("off")
    ax.text(0.0, 0.95,
            "Satu titik berhenti (8,5 ; 1,0) m, ±5 detik.\n\n"
            "Plus merah: posisi sebenarnya\nSilang hitam: rata-rata yang ditampilkan\n"
            "Lingkaran hijau putus-putus: 2 cm dan 4 cm\n\n"
            "Titik yang rapat = presisi bagus.\nTitik yang dekat plus merah = akurasi bagus.\n"
            "Rapat tetapi jauh dari plus = bias\n(offset anchor / NLOS tubuh).",
            va="top", fontsize=10)
    fig.suptitle("Posisi saat troli berhenti, relatif ke posisi sebenarnya (data yang sama)",
                 fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, 0.95), h_pad=3.0)
    fig.savefig(path, dpi=120)
    plt.close(fig)


def fig_dropout(path):
    scene, segs, dur, stream = make_data("dropout", SEED)
    t0, t1, keep = st.DROPOUT
    fig, ax = plt.subplots(figsize=(12, 5.2))
    ax.axvspan(t0, t1, color="#f4a261", alpha=0.25, label=f"hanya {keep} terdengar")
    for key, label, col in (("none|none|old", "A. EKF saja (sebelum): semua posisi dianggap valid",
                             COLORS[0]),
                            ("none|still20", "G. final: hanya posisi yang valid", COLORS[6])):
        s = sampled(segs, track_of(key, scene, stream), dur)
        s = [p for p in s if 10 <= p[0] <= 40]
        # an invalid (held) output is not drawn as an error: navigation is told
        # not to use it, so it never steers the trolley
        ax.plot([p[0] for p in s],
                [math.hypot(p[1] - p[4], p[2] - p[5]) * 100 if p[3] else float("nan") for p in s],
                "-", color=col, lw=1.5, label=label)
        held = [p[0] for p in s if not p[3]]
        if held and key == "none|still20":
            ax.axvspan(min(held), max(held), ymin=0, ymax=0.06, color=col, alpha=0.8,
                       label=f"G: posisi ditahan, valid = false ({min(held):.1f}–{max(held):.1f} s)")
    ax.set_xlabel("waktu (s)")
    ax.set_ylabel("galat posisi (cm)")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=9, loc="upper left")
    ax.set_title("Saat data 3 anchor hilang: EKF lama tetap mengeluarkan posisi tebakan "
                 "(sampai ±75 cm meleset);\nversi final menahan posisi terakhir dan menandainya "
                 "tidak valid, lalu kembali valid setelah anchor terdengar lagi", fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


def fig_laps(path, laps):
    fig, ax = plt.subplots(figsize=(8.5, 5.2))
    keys = [("none|still20|v1", "F. tanpa offset per anchor", COLORS[5]),
            ("none|still20", "G. dengan offset per anchor (final)", COLORS[6])]
    w = 0.38
    for i, (key, label, col) in enumerate(keys):
        vals = [v * 100 for v in laps[key]]
        xs = [k + (i - 0.5) * w for k in range(LAPS)]
        ax.bar(xs, vals, w, color=col, label=label)
        for x, v in zip(xs, vals):
            ax.text(x, v + 0.15, f"{v:.1f}", ha="center", fontsize=9)
    ax.axhspan(2, 4, color="#2e7d32", alpha=0.12, label="target 2–4 cm")
    ax.set_xticks(range(LAPS))
    ax.set_xticklabels([f"putaran {k + 1}" for k in range(LAPS)])
    ax.set_ylabel("galat saat diam (MAE, cm)")
    ax.legend(fontsize=9, loc="upper center", bbox_to_anchor=(0.5, -0.1), ncol=3)
    ax.set_ylim(0, max(max(v) for v in laps.values()) * 100 * 1.15)
    ax.grid(axis="y", alpha=0.3)
    ax.set_title("Line of sight, tanpa kalibrasi manual: makin lama troli berjalan, offset "
                 "tiap anchor makin\nterpelajari dan galat saat diam turun (rata-rata 10 simulasi)",
                 fontsize=10)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


# ---------------------------------------------------------------------------
# workbook
# ---------------------------------------------------------------------------

GLOSSARY = [
    ("Galat / error", "Jarak antara posisi yang ditampilkan dan posisi sebenarnya (di simulasi posisi sebenarnya diketahui)."),
    ("MAE", "Mean absolute error: rata-rata galat. Semakin kecil semakin akurat."),
    ("RMSE", "Akar rata-rata kuadrat galat. Seperti MAE, tetapi galat besar lebih berpengaruh."),
    ("P95", "95% galat berada di bawah angka ini (5% sisanya lebih besar)."),
    ("Presisi", "Seberapa rapat posisi saat troli diam, tanpa melihat posisi sebenarnya."),
    ("Akurasi", "Seberapa dekat posisi dengan posisi sebenarnya."),
    ("CEP50 / CEP95", "Radius lingkaran yang memuat 50% / 95% posisi saat diam, di sekitar rata-ratanya (ukuran presisi)."),
    ("Kegoyahan", "Seberapa patah-patah garis lintasan: RMS beda-kedua posisi berurutan. Garis lurus mulus = 0."),
    ("Galat maks saat anchor hilang", "Galat terbesar di antara posisi yang dinyatakan valid, selama dan 3 s setelah data 3 anchor hilang."),
    ("Ketersediaan", "Bagian waktu posisi valid tersedia untuk navigasi."),
    ("Offset jarak (b)", "Kelebihan jarak yang sama pada semua anchor, dari delay antena DW1000 (±75–100 cm). Dihitung otomatis oleh EKF."),
    ("Offset per anchor (d)", "Selisih offset tiap anchor terhadap yang lain (beberapa cm). Dipelajari otomatis saat troli bergerak."),
    ("Margin error (±σ)", "Ketidakpastian posisi menurut EKF: sekitar 2 dari 3 posisi berada dalam radius ini."),
    ("LOS / NLOS", "Line of sight: jalur anchor–tag bebas. Non line of sight: terhalang (tubuh, rak), jarak terbaca lebih panjang."),
    ("Jendela (window)", "Sekelompok sampel terakhir yang dirata-rata / diambil mediannya."),
    ("Jendela adaptif", "Rata-rata hanya dipakai saat troli diam (kecepatan < 0,15 m/s), maksimal 20 keluaran."),
    ("Gerbang validitas", "Posisi hanya valid bila ≥ 3 anchor terdengar dalam 1,5 s dan σ ≤ 0,5 m; selain itu posisi terakhir ditahan."),
]


def build_workbook(rows, laps, imgs, path):
    def r(scn, key, cal=False):
        return rows.get((scn, key) if not cal else (scn + "#cal", key), {})

    wb = Workbook()
    s = Sheet(wb, "Ringkasan", [44, 12, 12, 12, 12, 12, 12, 14], first=True)
    s.title("Hasil Simulasi Algoritma Positioning UWB",
            "7 algoritma dijalankan pada data simulasi yang sama (10 kali Monte-Carlo). "
            "Model radio: derau DS-TWR, offset tiap anchor, outlier, NLOS tubuh pendorong, paket hilang.")
    s.section("Algoritma yang dibandingkan")
    desc = {
        "none|none|old": "EKF 5 state seperti di branch main sebelum perubahan ini",
        "fable-MA500": "rata-rata posisi 500 ms + geser sepanjang kecepatan (perubahan 22-10)",
        "mean20|mean20|old": "rata-rata 20 sampel pada jarak lalu 20 keluaran pada posisi",
        "median3|none|old": "median 3 sampel tiap anchor sebelum EKF",
        "none|linfit10|old": "garis regresi 10 keluaran terakhir pada posisi",
        "none|still20|v1": "rata-rata maks 20 keluaran hanya saat diam; posisi ditahan bila < 3 anchor",
        "none|still20": "F + EKF mempelajari offset tiap anchor + jarak yang terlalu panjang dipercaya lebih sedikit",
    }
    s.table(["Algoritma", "Isi"], [[lab, desc[k]] for k, lab in st.NAMED], [None, None])
    s.ws.column_dimensions["B"].width = 12
    for row in range(s.row - len(st.NAMED) - 1, s.row - 1):
        s.ws.merge_cells(start_row=row, start_column=2, end_row=row, end_column=8)

    s.section("Hasil utama — ruangan 9,5 × 5,9 m, laju anchor seperti hardware (cm)")
    data = []
    for k, lab in st.NAMED:
        m, d = r("market", k), r("dropout", k)
        data.append([lab, m.get("mae", 0) * 100, m.get("mae_move", 0) * 100,
                     m.get("mae_still", 0) * 100, m.get("p95_still", 0) * 100,
                     m.get("prec50", 0) * 100, m.get("jitter", 0) * 100, d.get("drop_max", 0) * 100])
    s.table(["Algoritma", "MAE semua", "MAE bergerak", "MAE diam", "P95 diam",
             "Sebaran diam CEP50", "Kegoyahan garis", "Galat maks anchor hilang"],
            data, [None] + ["cm"] * 7, bold_last=True)
    s.note("Semua angka: semakin kecil semakin baik. Baris tebal = algoritma final yang sekarang "
           "berjalan di server. C (20 × 20) tertinggal ±2 m karena jendela 20 sampel pada laju "
           "2,5 sampel/anchor/detik menunda ±4 detik.", 2)
    s.gap()

    s.section("Kondisi terbaik: line of sight (tanpa halangan tubuh), saat troli diam (cm)")
    data = []
    for k, lab in st.NAMED:
        if k == "fable-MA500":
            continue
        a, c = r("market-los", k), r("market-los", k, cal=True)
        data.append([lab, a.get("mae_still", 0) * 100, c.get("mae_still", 0) * 100 if c else None,
                     a.get("prec50", 0) * 100])
    s.table(["Algoritma", "MAE diam, tanpa kalibrasi", "MAE diam, terkalibrasi", "Sebaran CEP50"],
            data, [None, "cm", "cm", "cm"], bold_last=True)

    s.section("Target 2–4 cm")
    g, glos = r("market", "none|still20"), r("market-los", "none|still20")
    s.kv([
        ("Presisi saat diam (CEP50), dengan NLOS tubuh", g.get("prec50", 0) * 100, "cm",
         "TERCAPAI. Data nyata 16-10 (diputar ulang): 1,1–2,2 cm."),
        ("Akurasi saat diam, line of sight, tanpa kalibrasi", glos.get("mae_still", 0) * 100, "cm",
         f"Hampir: {laps['none|still20'][-1] * 100:.1f} cm setelah {LAPS} putaran (offset anchor makin terpelajari)."),
        ("Akurasi saat diam, dengan NLOS tubuh pendorong", g.get("mae_still", 0) * 100, "cm",
         "BELUM. Tubuh pendorong menambah jarak ke anchor di belakangnya; perlu tag dipasang tinggi."),
        ("Akurasi saat bergerak (MAE)", g.get("mae_move", 0) * 100, "cm",
         "Sebagian besar dari keterlambatan pengiriman WiFi (posisi dibanding waktu sekarang)."),
    ])
    s.section("Kesimpulan")
    for line in (
            "1. Algoritma final (G) paling baik di semua ukuran pada skenario ruangan besar: MAE turun "
            "16% dari EKF lama, sebaran saat diam turun 3×, garis lebih mulus, dan galat saat anchor "
            "hilang turun setengah.",
            "2. Jendela panjang yang selalu aktif (C, 20 × 20) membuat posisi tertinggal beberapa meter; "
            "jendela hanya berguna saat troli diam (F, G).",
            "3. Presisi 2–4 cm tercapai. Akurasi 2–4 cm butuh line of sight: dengan tag di atas kepala "
            "pendorong, simulasi memberi ±4–5 cm saat diam tanpa kalibrasi manual."):
        s.note(line, 2)

    # ---- scenario and model
    s = Sheet(wb, "Skenario & model", [26, 16, 50])
    s.title("Skenario dan model simulasi", "tools/simulate.py (model radio) + tools/smoothing_study.py (skenario)")
    s.section("Skenario")
    s.table(["Skenario", "Ruangan", "Keterangan"], [
        ["lab", "4,5 × 5,4 m", "persegi + diagonal 0,8 m/s, berhenti 2 s, 6 sampel/anchor/detik"],
        ["market", "9,5 × 5,9 m", "anchor seperti rekaman 23-10, tag 1,0 m, 0,6 m/s, berhenti 6 s, 2,5 sampel/anchor/detik"],
        ["dropout", "9,5 × 5,9 m", "market, detik 20–26 hanya anchor-5 terdengar"],
        ["market-los", "9,5 × 5,9 m", "market tanpa NLOS tubuh dan outlier"],
    ], [None, None, None])
    s.section("Model galat radio")
    s.table(["Komponen", "Nilai", "Keterangan"], [
        ["Derau DS-TWR (1σ)", f"{sim.NOISE_M * 100:.0f} cm", "acak, tiap sampel"],
        ["Offset bersama", f"{sim.BIAS_COMMON * 100:.0f} cm", "delay antena, sama untuk semua anchor"],
        ["Sebaran offset per anchor (1σ)", f"{sim.BIAS_SPREAD * 100:.0f} cm", "beda antar modul DW1000"],
        ["Sisa kalibrasi (1σ)", f"{sim.CAL_RESIDUAL * 100:.0f} cm", "hanya pada skenario terkalibrasi"],
        ["Outlier", f"{sim.OUTLIER_P:.0%}, +0,3–1,5 m", "pantulan (multipath)"],
        ["NLOS tubuh", "+15 cm (+ acak 5 cm)", "anchor di belakang pendorong"],
        ["Paket hilang", f"{sim.LOSS_P:.0%}", "pertukaran ranging gagal"],
        ["Pengiriman ke server", "5 paket/s, 20–170 ms", "batch WiFi dari tag"],
    ], [None, None, None])

    # ---- everything
    s = Sheet(wb, "Semua hasil", [14, 11, 24] + [10] * 13)
    s.title("Semua hasil studi", "Setiap baris: rata-rata 10 simulasi. cm kecuali disebut lain.")
    full = json.loads((ROOT / "docs" / "smoothing_study.json").read_text(encoding="utf-8"))["rows"]
    cols = ("mae", "rmse", "mae_move", "sync_move", "mae_still", "p95_still", "prec50", "prec95",
            "xt", "jitter")
    data = [[x["scenario"], "ya" if x["calibrated"] else "tidak", x["variant"]]
            + [(x.get(c) or 0) * 100 for c in cols]
            + [x.get("still_det"), x.get("avail"), (x.get("drop_max") or 0) * 100] for x in full]
    s.table(["Skenario", "Kalibrasi", "Varian (jarak|posisi|tambahan)", "MAE", "RMSE",
             "MAE bergerak", "Galat tanpa latensi", "MAE diam", "P95 diam", "CEP50", "CEP95",
             "Cross-track", "Kegoyahan", "Diam terdeteksi", "Ketersediaan", "Maks anchor hilang"],
            data, [None, None, None] + ["cm"] * 10 + ["pct", "pct", "cm"])
    s.ws.freeze_panes = "D6"

    # ---- figures
    s = Sheet(wb, "Gambar", [12] * 12)
    s.title("Gambar", "File PNG yang sama ada di docs/hasil-simulasi/.")
    for name, cap in imgs:
        s.image(OUT / name, 1000, cap)

    s = Sheet(wb, "Istilah", [30, 100])
    s.title("Istilah", "Arti setiap istilah di laporan ini.")
    s.table(["Istilah", "Arti"], [list(g) for g in GLOSSARY], [None, None])
    wb.save(path)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    data = json.loads((ROOT / "docs" / "smoothing_study.json").read_text(encoding="utf-8"))
    rows = {}
    for x in data["rows"]:
        rows[(x["scenario"] + ("#cal" if x["calibrated"] else ""), x["variant"])] = x

    fig_tracks(rows, OUT / "1_lintasan_algoritma.png")
    fig_stop(OUT / "2_titik_berhenti.png")
    st.SHOW = st.NAMED
    st.plot_summary(data["rows"], OUT / "3_ringkasan_metrik.png")
    fig_dropout(OUT / "4_anchor_hilang.png")
    with Pool(8) as pool:
        res = pool.map(lap_job, [(500 + k, key) for k in range(10)
                                 for key in ("none|still20|v1", "none|still20")])
    laps = {}
    for key in ("none|still20|v1", "none|still20"):
        vals = [v for k, v in res if k == key]
        laps[key] = [sum(v[i] for v in vals) / len(vals) for i in range(LAPS)]
    fig_laps(OUT / "5_konvergensi_offset.png", laps)
    shutil.copy(OUT / "3_ringkasan_metrik.png", ROOT / "docs" / "img" / "smoothing_study.png")

    imgs = [("1_lintasan_algoritma.png", "Gambar 1 – Lintasan tiap algoritma (data simulasi yang sama)"),
            ("2_titik_berhenti.png", "Gambar 2 – Posisi saat troli berhenti, relatif ke posisi sebenarnya"),
            ("3_ringkasan_metrik.png", "Gambar 3 – Ringkasan metrik (rata-rata 10 simulasi)"),
            ("4_anchor_hilang.png", "Gambar 4 – Saat data anchor hilang"),
            ("5_konvergensi_offset.png", "Gambar 5 – Offset per anchor terpelajari selama troli berjalan")]
    build_workbook(rows, laps, imgs, OUT / "Hasil_Simulasi.xlsx")
    print("laps", {k: [round(v * 100, 1) for v in vs] for k, vs in laps.items()})
    print("wrote", OUT)


if __name__ == "__main__":
    main()
