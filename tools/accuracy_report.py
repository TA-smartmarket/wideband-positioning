"""Figures + workbook of the accuracy research (BZ-EKF), simulation only.

    python tools/accuracy_lab.py       # the numbers (docs/hasil-simulasi/riset_akurasi.json)
    python tools/accuracy_report.py    # figures + workbook from them

Writes docs/hasil-simulasi/riset_*.png and Hasil_Riset_Akurasi.xlsx. The
tables come from riset_akurasi.json (20 Monte-Carlo runs per scenario); the
figures 3 and 5 show one run (seed 700) so single stops can be followed.
"""
import json
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from openpyxl import Workbook  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
import accuracy_lab as lab  # noqa: E402
from report_excel import Sheet  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "docs" / "hasil-simulasi"
SEED = 700
TARGET = (2.0, 4.0)                      # cm
C_G, C_FINAL, C_MID, C_LOS = "#8c8c8c", "#1f4e79", "#6fa8dc", "#6a994e"
LADDER_COL = ["#8c8c8c", "#c9a227", "#f4a261", "#1f4e79", "#9fb7d0", "#e76f51", "#6a994e"]
SHORT = {
    "G": "G: EKF sekarang",
    "K+F": "+ kalibrasi statis",
    "K+F+B+hd": "+ bayangan badan β",
    "K+F+B+hd+Z": "+ mode diam = BZ-EKF",
    "K+F+B+hd+Z+S+FDE": "BZ-EKF + FDE saat berhenti",
    "B+hd+Z": "BZ-EKF tanpa kalibrasi",
    "K+F+B+imu+Z": "BZ-EKF + IMU (gyroscope)",
}
WORLD = {"market": "Market\n(bayangan badan)", "market-los": "Market-LOS\n(tanpa halangan)",
         "robust": "Robust\n(dunia acak + NLOS rak)"}


def target_band(ax, axis="x"):
    span = ax.axvspan if axis == "x" else ax.axhspan
    span(*TARGET, color="#6a994e", alpha=0.13, lw=0, zorder=0)


# ---------------------------------------------------------------------------
# figures
# ---------------------------------------------------------------------------

def fig_ladder(S, path):
    rows = S["4"]["market"]
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(13, 5.2), gridspec_kw={"width_ratios": [1.25, 1]})
    names = [m for m, _ in lab.LADDER]
    y = list(range(len(names)))[::-1]
    for yi, m, col in zip(y, names, LADDER_COL):
        r = rows[m]
        ax.barh(yi, r["stop_mean"] * 100, color=col, height=0.62)
        ax.plot([r["stop_med"] * 100], [yi], "o", color="white", mec="black", ms=6, zorder=3)
        ax.text(r["stop_mean"] * 100 + 0.4, yi, f"{r['stop_mean'] * 100:.1f} cm  (median "
                f"{r['stop_med'] * 100:.1f})", va="center", fontsize=9)
    target_band(ax)
    ax.set_yticks(y)
    ax.set_yticklabels([SHORT[m] for m in names])
    ax.set_xlim(0, max(rows[m]["stop_mean"] for m in names) * 100 * 1.45)
    ax.set_xlabel("galat saat troli berhenti (cm) — batang: rata-rata, titik: median")
    ax.set_title("Market, 4 anchor: tiap baris = baris di atasnya + satu komponen", fontsize=11)
    ax.set_ylim(-0.6, len(names) - 0.2)
    ax.text(3.0, len(names) - 0.45, "target\n2–4 cm", color="#3d6b2c", ha="center", va="center",
            fontsize=8)
    ax.grid(axis="x", alpha=0.3)

    qs = list(range(1, 100))
    for m, col, lab_, ls in (("G", C_G, "G: EKF sekarang (4 anchor)", "-"),
                             (lab.FINAL, C_FINAL, "BZ-EKF (4 anchor)", "-")):
        ax2.plot([v * 100 for v in rows[m]["cdf"]], qs, color=col, lw=2.2, ls=ls, label=lab_)
    r6 = S.get("6", {}).get("market", {}).get(lab.FINAL)
    if r6:
        ax2.plot([v * 100 for v in r6["cdf"]], qs, color=C_MID, lw=2, ls="--", label="BZ-EKF (6 anchor)")
    ax2.plot([v * 100 for v in S["4"]["market-los"][lab.FINAL]["cdf"]], qs, color=C_LOS, lw=1.6,
             ls=":", label="BZ-EKF tanpa halangan (batas derau)")
    target_band(ax2)
    ax2.set_xlim(0, 30)
    ax2.set_ylim(0, 100)
    ax2.set_xlabel("galat saat berhenti (cm)")
    ax2.set_ylabel("% waktu berhenti dengan galat ≤ x")
    ax2.set_title("Sebaran galat saat berhenti (kurva kumulatif)", fontsize=11)
    ax2.grid(alpha=0.3)
    ax2.legend(fontsize=8.5, loc="lower right")
    fig.suptitle("Riset akurasi: komponen yang ditambahkan ke EKF (simulasi, 20 run)", fontsize=13)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def fig_time(S, data, path):
    bins = data["bins"]
    xs = [(a + b) / 2 for a, b in bins]
    fig, ax = plt.subplots(figsize=(9, 4.8))
    for key, n, w, col, lw, ls, lab_ in (
            ("G", "4", "market", C_G, 2, "-", "G: EKF sekarang"),
            ("K+F+B+hd", "4", "market", "#f4a261", 1.8, "-", "+ kalibrasi + bayangan badan β"),
            (lab.FINAL, "4", "market", C_FINAL, 2.6, "-", "BZ-EKF, 4 anchor"),
            (lab.FINAL, "6", "market", C_MID, 2, "--", "BZ-EKF, 6 anchor"),
            (lab.FINAL, "4", "market-los", C_LOS, 1.6, ":", "BZ-EKF tanpa halangan (batas derau)")):
        r = S.get(n, {}).get(w, {}).get(key)
        if not r:
            continue
        ys = [r["bins"][f"{a}-{b:g}"] * 100 for a, b in bins]
        ax.plot(xs, ys, marker="o", color=col, lw=lw, ls=ls, label=lab_)
    target_band(ax, "y")
    ax.set_xticks(xs)
    ax.set_xticklabels([f"{a}–{b:g} s" for a, b in bins])
    ax.set_xlabel("lama troli sudah berhenti")
    ax.set_ylabel("galat rata-rata (cm)")
    ax.set_ylim(0, 20)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=9)
    ax.set_title("Galat menurut lama berhenti — Market (simulasi, 20 run)", fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def one_case(mode, probe=None):
    scene, segs, dur, stream, cal, _w = lab.make_case("market", SEED, 4)
    tr = lab.run(scene, stream, mode, cal=cal, probe=probe)
    return segs, dur, tr


def stop_offsets(segs, tr):
    """(dx, dy) in cm of the shown position at every 100 ms of every stop,
    from 1 s after stopping, relative to the true stop point."""
    pts = []
    j = 0
    t = tr[0][0] + 3000
    while t < segs[-1][1] * 1000:
        while j + 1 < len(tr) and tr[j + 1][0] <= t:
            j += 1
        mv, since = lab.st.moving_at(segs, t / 1000)
        if not mv and since > 1.0:
            tx, ty = lab.sim.truth_at(segs, t / 1000)
            pts.append(((tr[j][1] - tx) * 100, (tr[j][2] - ty) * 100))
        t += 100
    return pts


def fig_stops(path):
    fig, axes = plt.subplots(1, 2, figsize=(11, 5.4))
    for ax, mode, col, title in ((axes[0], "G", C_G, "G: EKF sekarang"),
                                 (axes[1], lab.FINAL, C_FINAL, "BZ-EKF")):
        segs, _dur, tr = one_case(mode)
        pts = stop_offsets(segs, tr)
        ax.scatter([p[0] for p in pts], [p[1] for p in pts], s=7, color=col, alpha=0.45)
        for r, ls in ((2, ":"), (4, "--"), (10, "-")):
            ax.add_patch(plt.Circle((0, 0), r, fill=False, ls=ls, color="#3d6b2c" if r <= 4 else "#999999"))
            ax.text(r * 0.72, r * 0.72, f"{r} cm", fontsize=8, color="#3d6b2c" if r <= 4 else "#777777")
        ax.plot(0, 0, "k+", ms=14, mew=2)
        err = [math.hypot(*p) for p in pts]
        ax.set_title(f"{title}: rata-rata {sum(err) / len(err):.1f} cm, "
                     f"{sum(e <= 4 for e in err) / len(err) * 100:.0f}% ≤ 4 cm", fontsize=11)
        ax.set_xlim(-30, 30)
        ax.set_ylim(-30, 30)
        ax.set_aspect("equal")
        ax.set_xlabel("selisih x terhadap titik berhenti sebenarnya (cm)")
        ax.set_ylabel("selisih y (cm)")
        ax.grid(alpha=0.3)
    fig.suptitle("Posisi yang ditampilkan selama troli berhenti (11 titik berhenti, 1 run, skala asli)",
                 fontsize=12)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def fig_robust(S, path):
    worlds = ["market", "market-los", "robust"]
    fig, ax = plt.subplots(figsize=(11, 5))
    groups = [("4", "G", C_G, "G, 4 anchor"), ("4", lab.FINAL, C_FINAL, "BZ-EKF, 4 anchor"),
              ("6", "G", "#c0c0c0", "G, 6 anchor"), ("6", lab.FINAL, C_MID, "BZ-EKF, 6 anchor")]
    w = 0.2
    for k, (n, mode, col, lab_) in enumerate(groups):
        for i, wn in enumerate(worlds):
            r = S.get(n, {}).get(wn, {}).get(mode)
            if not r:
                continue
            x = i + (k - 1.5) * w
            ax.bar(x, r["stop_mean"] * 100, w * 0.92, color=col, label=lab_ if i == 0 else None)
            ax.plot([x], [r["stop_med"] * 100], "o", color="white", mec="black", ms=5, zorder=3)
            ax.text(x, r["stop_mean"] * 100 + 0.5, f"{r['stop_mean'] * 100:.1f}", ha="center", fontsize=8)
    target_band(ax, "y")
    ax.set_xticks(range(len(worlds)))
    ax.set_xticklabels([WORLD[wn] for wn in worlds])
    ax.set_ylabel("galat saat berhenti (cm) — batang: rata-rata, titik: median")
    ax.grid(axis="y", alpha=0.3)
    ax.legend(fontsize=9)
    ax.set_title("Ketahanan: tiga dunia simulasi, 4 dan 6 anchor (20 run tiap kotak)", fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def fig_beta(path):
    trace = []

    def probe(eng, tag, est, recv, x, y):
        f = eng.tracks[tag].f
        if getattr(f, "ib", None) is not None:
            trace.append((recv / 1000, f.x[f.ib] * 100, math.sqrt(max(f.P[f.ib][f.ib], 0)) * 100))

    segs, _dur, _tr = one_case(lab.FINAL, probe)
    fig, ax = plt.subplots(figsize=(10, 4.2))
    for t0, t1, p0, p1 in segs:
        if p0 == p1:
            ax.axvspan(t0, t1, color="#e8e8e8", lw=0)
    ts = [p[0] for p in trace]
    b = [p[1] for p in trace]
    sd = [p[2] for p in trace]
    ax.fill_between(ts, [v - s for v, s in zip(b, sd)], [v + s for v, s in zip(b, sd)],
                    color=C_MID, alpha=0.3, lw=0, label="± ketidakpastian (1σ)")
    ax.plot(ts, b, color=C_FINAL, lw=1.8, label="β yang dipelajari EKF")
    truth = 15 + 5 * math.sqrt(2 / math.pi)
    ax.axhline(truth, color="#e76f51", ls="--", lw=1.4,
               label=f"tambahan jarak sebenarnya di simulasi ({truth:.0f} cm rata-rata)")
    ax.set_xlabel("waktu (s) — abu-abu: troli berhenti")
    ax.set_ylabel("β: tambahan jarak oleh badan (cm)")
    ax.set_ylim(0, 35)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=9, loc="lower right")
    ax.set_title("State bayangan badan β belajar sendiri saat troli berjalan (1 run, Market)", fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


# ---------------------------------------------------------------------------
# workbook
# ---------------------------------------------------------------------------

COMPONENTS = [
    ("EKF dasar (G)", "Menggabungkan jarak dari tiap anchor menjadi posisi + kecepatan; sudah "
     "mempelajari offset jarak b dan offset per anchor d.", "Algoritma server sekarang.",
     "Kalman 1960; Singer 1970; Huber 1964"),
    ("K. Kalibrasi statis offset", "Offset tiap anchor diukur sekali dengan tag di titik yang "
     "diketahui (fitur Kalibrasi di dashboard), lalu dipakai sebagai koreksi jarak.",
     "Offset yang dipelajari sendiri ikut ‘menelan’ bayangan badan; tanpa K hasil final 9 cm.",
     "—"),
    ("F. Offset dipercaya (dibekukan)", "Setelah kalibrasi, EKF tidak lagi mengubah offset tiap anchor.",
     "Supaya tambahan jarak oleh badan hanya dijelaskan oleh β.", "—"),
    ("B. State bayangan badan β", "Satu state baru di EKF: tambahan jarak pada anchor yang "
     "berada di belakang arah jalan troli (tempat badan pendorong). Model: jarak = jarak geometri "
     "+ b + d + s·β, s = 0…1 menurut sudut anchor terhadap arah jalan.",
     "Penyebab utama galat 15 cm saat berhenti adalah bias badan, bukan derau.",
     "De Cock dkk. 2023; Tian dkk. 2019"),
    ("hd. Arah hadap dari lintasan", "Arah jalan = garis lurus terbaik melalui posisi 4 s terakhir "
     "saat bergerak (≥ 1 m); dibekukan saat berhenti.",
     "Kecepatan EKF berbelok saat mengerem / saat ada outlier; arah yang salah menandai anchor yang salah.",
     "—"),
    ("Z. Mode diam (ZUPT)", "Bila kecepatan < 0,2 m/s selama 0,4 s: kecepatan dikunci 0 dan derau "
     "proses dikecilkan; keluar bila 3 dari 4 inovasi terakhir > 1,5σ.",
     "Saat diam, satu putaran jarak yang buruk (2 outlier sekaligus) sempat menggeser posisi 20–45 cm.",
     "Foxlin 2005; Blom & Bar-Shalom 1988"),
    ("Opsi: IMU di tag", "Gyroscope memberi arah hadap (galat ±5° tetap + ±3° acak); akselerometer "
     "menyalakan / mematikan mode diam.",
     "Arah dari lintasan tidak terlihat pada leg < 1 m setelah belokan (galat 14–18 cm di titik itu); "
     "mode diam dari jarak baru dilepas ±1 s setelah troli mulai jalan.",
     "De Cock dkk. 2023; Tian dkk. 2019"),
]

REFERENCES = [
    ("De Cock, C. dkk. (2023). Robust IMU-Based Mitigation of Human Body Shadowing in UWB Indoor "
     "Positioning. Sensors 23(?).", "Model galat jarak dipilih menurut orientasi badan–tag–anchor (dari IMU)."),
    ("Tian, Q. dkk. (2019). Human Body Shadowing Effect on UWB-Based Ranging System for Pedestrian "
     "Tracking. IEEE Trans. Instrum. Meas.", "Galat jarak bergantung derajat halangan badan; arah dari gyroscope."),
    ("Naheem, K. dkk. (2024). A Robust Indoor Pedestrian Localization Approach Against Human Body "
     "Shadowing for UWB-Enabled Smartphones. IEEE Trans. Instrum. Meas.", "Pendekatan lain tanpa sensor tambahan."),
    ("Foxlin, E. (2005). Pedestrian tracking with shoe-mounted inertial sensors. IEEE Computer "
     "Graphics and Applications 25(6), 38–46.", "Zero-velocity update (ZUPT)."),
    ("Blom, H. A. P. & Bar-Shalom, Y. (1988). The interacting multiple model algorithm for systems "
     "with Markovian switching coefficients. IEEE Trans. Automatic Control 33(8), 780–783.",
     "Pergantian model diam / bergerak (IMM)."),
    ("Parkinson, B. W. & Axelrad, P. (1988). Autonomous GPS integrity monitoring using the pseudorange "
     "residual. Navigation 35(2).", "RAIM / FDE (diuji, tidak dipakai)."),
    ("Chen, P.-C. (1999). A non-line-of-sight error mitigation algorithm in location estimation. "
     "IEEE WCNC.", "Residual weighting (diuji, tidak dipakai)."),
    ("Huber, P. J. (1964). Robust estimation of a location parameter. Ann. Math. Statist. 35(1).",
     "Bobot Huber pada pembaruan EKF."),
    ("Decawave (Qorvo). APS006 Part 3: DW1000 Diagnostics for NLOS Channels.",
     "Indikator NLOS dari daya first path; di data rekaman asli hanya lemah (lihat Ringkasan)."),
]

GLOSSARY = [
    ("Galat saat berhenti", "Jarak antara posisi yang ditampilkan dan posisi sebenarnya, diambil tiap "
     "100 ms mulai 1 s setelah troli berhenti, sampai troli jalan lagi."),
    ("Rata-rata / median / P95", "Rata-rata semua galat; median = nilai tengah (50% galat di bawahnya); "
     "P95 = 95% galat di bawahnya."),
    ("≤ 4 cm", "Bagian waktu berhenti yang galatnya paling besar 4 cm."),
    ("sd antar run", "Simpangan baku rata-rata galat antar 20 simulasi: seberapa berubah hasilnya "
     "kalau datanya lain."),
    ("Galat saat bergerak", "Galat terhadap posisi sebenarnya pada saat yang sama. Didominasi "
     "keterlambatan (lag): pada 2,5 sampel/anchor/detik dan 0,6 m/s troli sudah maju ±25 cm."),
    ("Bayangan badan (body shadowing)", "Badan pendorong di belakang troli menghalangi sinyal ke "
     "anchor di belakang, jarak terbaca lebih panjang 10–35 cm."),
    ("β (beta)", "Besar tambahan jarak oleh badan, dipelajari sendiri oleh EKF saat troli berjalan."),
    ("Arah hadap", "Arah jalan troli; menentukan anchor mana yang ada di belakang (terhalang badan)."),
    ("Kalibrasi statis", "Mengukur offset jarak tiap anchor sekali, dengan tag di titik yang diketahui."),
    ("Offset jarak b / per anchor d", "Kelebihan jarak dari delay antena DW1000: b sama untuk semua "
     "anchor, d selisih tiap anchor."),
    ("Mode diam (ZUPT)", "Zero-velocity update: saat troli diam EKF menganggap kecepatan 0, sehingga "
     "posisi tidak mudah tergeser data jarak yang buruk."),
    ("FDE", "Fault detection & exclusion: membuang anchor yang jaraknya tidak cocok dengan yang lain."),
    ("NLOS rak", "Episode acak (10% waktu) anchor terhalang rak/barang: jarak +15…60 cm selama ±3 s."),
    ("LOS / NLOS", "Line of sight: jalur bebas. Non line of sight: terhalang, jarak terbaca lebih panjang."),
    ("Monte-Carlo (run)", "Simulasi diulang dengan data acak berbeda; hasil dirata-rata."),
    ("Inovasi (ν)", "Selisih jarak terukur dan jarak yang diharapkan EKF, dibagi ketidakpastiannya."),
]


def build_workbook(S, data, imgs, path):
    m4, m6 = S["4"]["market"], S.get("6", {}).get("market", {})
    los, rob = S["4"]["market-los"], S["4"]["robust"]
    F = lab.FINAL
    wb = Workbook()
    s = Sheet(wb, "Ringkasan", [36, 12, 12, 12, 12, 12, 12, 12], first=True)
    s.title("Riset Akurasi: BZ-EKF (simulasi)",
            f"EKF + state bayangan badan β + mode diam, dengan kalibrasi statis. {data['runs']} simulasi "
            "Monte-Carlo per skenario, ruangan 9,5 × 5,9 m, laju anchor seperti hardware (2,5 sampel/anchor/detik). "
            "Server & firmware TIDAK diubah: semua ini hanya di tools/accuracy_lab.py.")
    s.section("Hasil utama — galat saat troli berhenti, skenario Market (bayangan badan)")
    s.kv([
        ("BZ-EKF, 4 anchor — rata-rata", round(m4[F]["stop_mean"] * 100, 1), "cm",
         f"median {m4[F]['stop_med'] * 100:.1f} cm; setelah berhenti ≥ 2 s {m4[F]['settled_mean'] * 100:.1f} cm; "
         f"P95 {m4[F]['stop_p95'] * 100:.1f} cm"),
        ("BZ-EKF, 6 anchor — rata-rata", round(m6[F]["stop_mean"] * 100, 1) if F in m6 else None, "cm",
         (f"median {m6[F]['stop_med'] * 100:.1f} cm; 2 anchor tambahan di tengah dinding panjang"
          if F in m6 else "")),
        ("G (EKF sekarang), 4 anchor", round(m4["G"]["stop_mean"] * 100, 1), "cm",
         f"median {m4['G']['stop_med'] * 100:.1f} cm — BZ-EKF {m4['G']['stop_mean'] / m4[F]['stop_mean']:.1f}× lebih akurat"),
        ("Batas derau (tanpa halangan)", round(los[F]["stop_mean"] * 100, 1), "cm",
         "BZ-EKF di Market-LOS: tidak bisa lebih baik dari ini dengan derau & laju data yang sama"),
    ])
    s.note(
        f"KESIMPULAN. Target 2–4 cm TERCAPAI di skenario Market untuk median ({m4[F]['stop_med'] * 100:.1f} cm) "
        f"dan setelah troli berhenti ≥ 2 s ({m4[F]['settled_mean'] * 100:.1f} cm); rata-rata keseluruhan "
        f"{m4[F]['stop_mean'] * 100:.1f} cm sedikit di atas 4 cm. Dengan 6 anchor rata-rata "
        f"{m6.get(F, m4[F])['stop_mean'] * 100:.1f} cm. Syarat: kalibrasi statis offset anchor dilakukan "
        f"(tanpa kalibrasi {m4['B+hd+Z']['stop_mean'] * 100:.1f} cm).", 4, bold=True)
    s.note(
        f"BELUM TERCAPAI di dunia Robust (bayangan badan acak 8–35 cm + NLOS rak acak): "
        f"{rob[F]['stop_mean'] * 100:.1f} cm dengan 4 anchor, "
        f"{S.get('6', {}).get('robust', {}).get(F, rob[F])['stop_mean'] * 100:.1f} cm dengan 6 anchor "
        f"(G: {rob['G']['stop_mean'] * 100:.1f} cm). Saat bergerak galat semua algoritma ±25–30 cm karena "
        "keterlambatan data (lag), bukan target 2–4 cm.", 3)
    s.section("Komponen BZ-EKF")
    s.table(["Komponen", "Apa yang dilakukan", "Kenapa perlu", "Referensi"],
            [list(c) for c in COMPONENTS], [None] * 4)
    s.ws.column_dimensions["B"].width = 12
    for row in range(s.row - len(COMPONENTS) - 1, s.row - 1):
        s.ws.row_dimensions[row].height = 62
    s.note("Indikator NLOS DW1000 (rx_power − fp_power) diperiksa pada 17.650 sampel rekaman asli: "
           "median selisih jarak hanya naik dari ±2 cm ke ±6 cm; hanya > 35 dB yang jelas buruk. "
           "Terlalu lemah untuk mengoreksi bias beberapa cm, jadi tidak dipakai.", 3)

    s = Sheet(wb, "Tangga komponen", [44, 18, 11, 11, 11, 11, 11, 11, 11])
    s.title("Tangga komponen — 4 anchor",
            "Tiap baris = baris di atasnya + satu komponen. Galat saat berhenti dihitung mulai 1 s setelah "
            "berhenti. cm kecuali disebut lain.")
    for w in ("market", "market-los", "robust"):
        s.section(WORLD[w].replace("\n", " "))
        rows = []
        for mode, desc in lab.LADDER:
            r = S["4"][w][mode]
            rows.append([desc, mode, r["stop_mean"] * 100, r["stop_med"] * 100, r["stop_p95"] * 100,
                         r["settled_mean"] * 100, r["within4"], r["runs_sd"] * 100, r["move_mean"] * 100])
        s.table(["Algoritma", "Kode", "Rata-rata diam", "Median diam", "P95 diam", "Diam ≥ 2 s",
                 "≤ 4 cm", "sd antar run", "Galat bergerak"],
                rows, [None, None, "cm", "cm", "cm", "cm", "pct", "cm", "cm"])

    if "6" in S:
        s = Sheet(wb, "4 vs 6 anchor", [30, 14, 12, 12, 12, 12, 12, 12])
        s.title("4 vs 6 anchor", "6 anchor = 4 di sudut + 2 di tengah dinding panjang (4,75; 0) dan (4,75; 5,9).")
        rows = []
        for w in ("market", "market-los", "robust"):
            for n in ("4", "6"):
                for mode in ("G", F):
                    r = S[n][w].get(mode)
                    if r:
                        rows.append([WORLD[w].replace("\n", " "), f"{n} anchor", "BZ-EKF" if mode == F else "G",
                                     r["stop_mean"] * 100, r["stop_med"] * 100, r["stop_p95"] * 100,
                                     r["within4"], r["move_mean"] * 100])
        s.table(["Skenario", "Anchor", "Algoritma", "Rata-rata diam", "Median diam", "P95 diam", "≤ 4 cm",
                 "Galat bergerak"], rows, [None, None, None, "cm", "cm", "cm", "pct", "cm"])

    W = data.get("walk")
    if W:
        s = Sheet(wb, "Uji gerak jalan", [30, 12, 12, 12, 12, 12, 12, 12, 12])
        s.title("Uji gerak jalan — rute belanja acak",
                f"{W['runs']} rute per dunia, rata-rata {W['route_length_m']:.0f} m, {W['route_duration_s']:.0f} s, "
                f"{W['route_stops']:.1f} titik berhenti. Percepatan 0,5 m/s², kecepatan 0,5–1,0 m/s, berhenti "
                "2–12 s. Rute ini TIDAK dipakai saat menyetel parameter (tools/walk_test.py).")
        names = {"G": "G: EKF sekarang", F: "BZ-EKF", lab.FINAL_IMU: "BZ-EKF + IMU"}
        for wn in ("market", "market-los", "robust"):
            s.section(WORLD[wn].replace("\n", " "))
            rows = []
            for mode in ("G", F, lab.FINAL_IMU):
                r = W["summary"][wn][mode]
                rows.append([names[mode], r["stop_mean"] * 100, r["stop_med"] * 100, r["shelf_mean"] * 100,
                             r["shelf_within4"], r["shelf_p95"] * 100, r["move_mean"] * 100, r["cross_mean"] * 100])
            s.table(["Algoritma", "Rata-rata diam", "Median diam", "Bacaan di rak", "Rak ≤ 4 cm", "P95 rak",
                     "Galat bergerak", "Melenceng dari lorong"],
                    rows, [None, "cm", "cm", "cm", "pct", "cm", "cm", "cm"])
        s.note("Bacaan di rak = rata-rata posisi pada detik terakhir setiap berhenti (yang dibaca untuk tahu "
               "troli di depan rak mana). Melenceng dari lorong = jarak posisi ke garis rute saat bergerak.", 2)

    s = Sheet(wb, "Skenario & model", [30, 18, 70])
    s.title("Skenario dan model simulasi", "tools/simulate.py (model radio) + tools/accuracy_lab.py (dunia, varian)")
    s.table(["Hal", "Nilai", "Keterangan"], [
        ["Ruangan / anchor", "9,5 × 5,9 m", "anchor di 4 sudut, tinggi 1,74 m (satu 2,2 m); tag 1,0 m"],
        ["Lintasan", "2 putaran", "keliling ruangan + diagonal, 0,6 m/s, berhenti 6 s di 11 titik"],
        ["Laju data", "400 ms", "satu putaran ranging semua anchor: 2,5 sampel/anchor/detik (seperti hardware)"],
        ["Derau jarak", "6 cm (1σ)", "derau DS-TWR saat jalur bebas"],
        ["Offset antena", "20 ± 8 cm", "sama untuk semua + selisih tiap anchor"],
        ["Kalibrasi statis", "galat 2 cm", "offset tiap anchor diketahui dengan galat acak 2 cm (1σ)"],
        ["Market", "+15 cm", "anchor di belakang arah jalan (sudut > ±110°) + |N(0, 5 cm)|; outlier 4%"],
        ["Market-LOS", "—", "tanpa bayangan badan, tanpa outlier: batas derau"],
        ["Robust", "acak", "bayangan badan 8–35 cm, kerucut acak; NLOS rak +15–60 cm, 10% waktu; outlier 4%"],
        ["Paket hilang", "8% + 3%", "pertukaran hilang / laporan ke tag hilang"],
        ["Run", str(data["runs"]), f"seed {data['seed']}…; semua varian memakai data yang persis sama"],
    ], [None, None, None])

    s = Sheet(wb, "Gambar", [12] * 12)
    s.title("Gambar", "File PNG yang sama ada di docs/hasil-simulasi/.")
    for name, cap in imgs:
        s.image(OUT / name, 1000, cap)

    s = Sheet(wb, "Referensi", [90, 60])
    s.title("Referensi", "Makalah yang mendasari komponen BZ-EKF.")
    s.table(["Sumber", "Dipakai untuk"], [list(r) for r in REFERENCES], [None, None])

    s = Sheet(wb, "Istilah", [30, 100])
    s.title("Istilah", "Arti setiap istilah di laporan ini.")
    s.table(["Istilah", "Arti"], [list(g) for g in GLOSSARY], [None, None])
    wb.save(path)


def main():
    data = json.loads((OUT / "riset_akurasi.json").read_text(encoding="utf-8"))
    S = data["summary"]
    walk = OUT / "uji_jalan.json"
    if walk.exists():                        # tools/walk_test.py
        data["walk"] = json.loads(walk.read_text(encoding="utf-8"))
    imgs = [
        ("riset_1_tangga_komponen.png", "1. Komponen yang ditambahkan satu per satu, dan sebaran galat saat berhenti"),
        ("riset_2_lama_berhenti.png", "2. Galat menurut lama troli sudah berhenti"),
        ("riset_3_titik_berhenti.png", "3. Posisi yang ditampilkan selama berhenti: G vs BZ-EKF (skala asli)"),
        ("riset_4_ketahanan.png", "4. Ketahanan: tiga dunia simulasi, 4 dan 6 anchor"),
        ("riset_5_beta.png", "5. State bayangan badan β belajar sendiri"),
    ]
    if "walk" in data and (OUT / "uji_jalan_ringkasan.png").exists():
        imgs.append(("uji_jalan_ringkasan.png", "6. Uji gerak jalan: rute belanja acak (animasi: uji_jalan_animasi.gif)"))
    fig_ladder(S, OUT / imgs[0][0])
    fig_time(S, data, OUT / imgs[1][0])
    fig_stops(OUT / imgs[2][0])
    fig_robust(S, OUT / imgs[3][0])
    fig_beta(OUT / imgs[4][0])
    path = OUT / "Hasil_Riset_Akurasi.xlsx"
    try:
        build_workbook(S, data, imgs, path)
    except PermissionError:                  # open in Excel
        path = OUT / "Hasil_Riset_Akurasi_baru.xlsx"
        build_workbook(S, data, imgs, path)
    print("wrote", path, "and", ", ".join(n for n, _ in imgs))


if __name__ == "__main__":
    main()
