"""Figures, animation and workbook of the window study (tools/window_study.py).

    python tools/window_study.py --runs 20    # the numbers (docs/hasil-simulasi/jendela.json)
    python tools/window_report.py             # this: jendela_*.png, jendela_animasi.*, Hasil_Jendela.xlsx

The ranking averages the three worlds (kosong / sepi / ramai): the trolley
does not know how busy the shop will be.
"""
import bisect
import json
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from openpyxl import Workbook  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
import window_study as ws  # noqa: E402
from report_excel import Sheet  # noqa: E402

sim, st = ws.sim, ws.st
OUT = ws.OUT
WORLD_ORDER = ["kosong", "sepi", "ramai"]
WORLD_LABEL = {"kosong": "Kosong (0 pembeli)", "sepi": "Sepi (2 pembeli)", "ramai": "Ramai (5 pembeli)"}
CORE_LABEL = {"old": "EKF lama", "G": "G", "GZ": "G + mode diam", "KGZ": "kalibrasi + G + mode diam",
              "GZL": "G + mode diam + tolak jarak panjang saat diam",
              "KGZL": "kalibrasi + G + mode diam + tolak jarak panjang saat diam",
              "KBZ": "BZ-EKF: kalibrasi + G + bayangan badan di belakang + mode diam",
              "KBZL": "BZ-EKF + tolak jarak panjang saat diam",
              "KRZ": "kalibrasi + G + bayangan badan arah acak + mode diam",
              "KRZL": "kalibrasi + G + bayangan badan arah acak + mode diam + tolak jarak panjang"}
OUT_LABEL = {"ekf": "keluaran EKF", "S": "hitung ulang saat berhenti",
             "S+FDE": "hitung ulang saat berhenti + buang anchor janggal"}
DEMO_SEED = 1300
MOVE_TOL = 0.025         # a pick may be at most this much worse than G while moving (m)


def split(key):
    core, pre, out, post = key.split("|")
    return core, pre, out, post


def describe(key):
    core, pre, out, post = split(key)
    return (f"{CORE_LABEL[core]} | jarak: {pre} | {OUT_LABEL[out]} | posisi: {post}")


CORE_SHORT = {"old": "EKF lama", "G": "G", "GZ": "G+Z", "KGZ": "K+G+Z", "GZL": "G+Z+L", "KGZL": "K+G+Z+L",
              "KBZ": "K+B+Z", "KBZL": "K+B+Z+L", "KRZ": "K+R+Z", "KRZL": "K+R+Z+L"}
CODES = "K = kalibrasi statis, Z = mode diam, L = tolak jarak kepanjangan saat diam, S = hitung ulang saat berhenti"


def short(key):
    core, pre, out, post = split(key)
    return f"{CORE_SHORT[core]} · jarak: {pre} · {out.upper()} · posisi: {post}"


def averaged(S):
    """Every combination's metrics averaged over the worlds it has."""
    keys = set.intersection(*(set(S[w]) for w in WORLD_ORDER if w in S))
    avg = {}
    for k in keys:
        avg[k] = {f: sum(S[w][k][f] for w in WORLD_ORDER) / len(WORLD_ORDER)
                  for f in ("shelf", "spread", "stop", "move", "all", "jitter", "avail")}
    return avg


def choose(S, A):
    refs = {lab_: k for lab_, k in ws.REFS.items() if k in A}
    g_move = A[ws.REFS["G. EKF server sekarang (adaptif-diam rata 20)"]]["move"]
    ok = [k for k in A if A[k]["avail"] >= 0.95]
    best_shelf = min(ok, key=lambda k: A[k]["shelf"])
    best_all = min(ok, key=lambda k: A[k]["all"])
    slower = lambda k: A[k]["move"] <= g_move + MOVE_TOL  # noqa: E731
    balanced = min((k for k in ok if slower(k)), key=lambda k: A[k]["shelf"])
    nocal = min((k for k in ok if not k.startswith("K") and slower(k)), key=lambda k: A[k]["shelf"])
    return refs, {"terbaik saat berhenti": best_shelf, "terbaik semua waktu": best_all,
                  "rekomendasi (berhenti terbaik, bergerak maks. 2,5 cm lebih buruk dari G)": balanced,
                  "rekomendasi tanpa kalibrasi statis": nocal}


# ---------------------------------------------------------------------------
# figures
# ---------------------------------------------------------------------------

def fig_concept(path):
    fig, ax = plt.subplots(figsize=(14, 5.6))
    n = 42
    rows = [("geser-1 (sliding)\nC dan semua 'geser-1'", 1, 20, "#1f4e79"),
            ("geser-5 (hopping)", 5, 20, "#6fa8dc"),
            ("tunggu-20 (tumbling)\nambil 20 sampel BARU", 20, 20, "#e76f51")]
    for r, (lab_, hop, N, col) in enumerate(rows):
        y = 3 - r * 1.25
        ax.text(-0.8, y, lab_, ha="right", va="center", fontsize=9.5)
        for i in range(1, n + 1):
            ax.plot(i, y, "o", color="#bbbbbb", ms=5.5)
        k = 0
        start = 1
        while start + N - 1 <= n and k < 3:
            end = start + N - 1
            yy = y + 0.18 + 0.14 * k
            ax.plot([start - 0.35, end + 0.35], [yy, yy], color=col, lw=3, alpha=0.85)
            ax.plot(end, y, "o", color=col, ms=9)
            ax.text(end + 0.45, yy, f"keluaran #{k + 1}: sampel {start}–{end}", fontsize=8, color=col,
                    va="center")
            start += hop
            k += 1
    for i in range(1, n + 1, 1):
        ax.text(i, -0.45, str(i), ha="center", fontsize=7, color="#777777")
    ax.text(n / 2, -0.85, "nomor sampel (pada jarak: 1 sampel = 1 jarak dari 1 anchor = 0,4 s;  pada posisi: "
            "1 sampel = 1 keluaran EKF ≈ 0,35 s)", ha="center", fontsize=8.5, color="#555555")
    ax.text(n + 0.6, 3.75, "titik berwarna = saat\nkeluaran baru keluar", fontsize=8, color="#555555", va="top")
    ax.set_xlim(-6.5, n + 9)
    ax.set_ylim(-1.1, 4.0)
    ax.axis("off")
    ax.set_title("Tiga cara memakai jendela 20 sampel", fontsize=13)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def fig_heat(S, core, out, path, world="ramai"):
    pres = [ws.wkey(p) for p in ws.PRE]
    posts = [ws.wkey(p) for p in ws.POST]
    fig, axes = plt.subplots(1, 2, figsize=(19, 8.5))
    for ax, field, title, vmax in ((axes[0], "shelf", "galat bacaan di rak (cm)", 25),
                                   (axes[1], "move", "galat saat bergerak (cm)", 80)):
        grid = [[S[world].get(f"{core}|{p}|{out}|{q}", {}).get(field, float("nan")) * 100 for q in posts]
                for p in pres]
        im = ax.imshow(grid, cmap="RdYlGn_r", vmin=0, vmax=vmax, aspect="auto")
        for i, row in enumerate(grid):
            for j, v in enumerate(row):
                if v == v:
                    ax.text(j, i, f"{v:.0f}" if v >= 10 else f"{v:.1f}", ha="center", va="center",
                            fontsize=5.6, color="black")
        ax.set_xticks(range(len(posts)))
        ax.set_xticklabels(posts, rotation=70, ha="right", fontsize=7)
        ax.set_yticks(range(len(pres)))
        ax.set_yticklabels(pres, fontsize=7.5)
        ax.set_xlabel("jendela pada POSISI (sesudah EKF)")
        ax.set_ylabel("jendela pada JARAK (sebelum EKF)")
        ax.set_title(title, fontsize=11)
        fig.colorbar(im, ax=ax, fraction=0.03)
    fig.suptitle(f"Semua kombinasi jendela — inti: {CORE_LABEL[core]}, {OUT_LABEL[out]}, "
                 f"{WORLD_LABEL[world]} (rata-rata 20 rute)", fontsize=12)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def fig_rank(S, A, refs, picks, path):
    top = sorted((k for k in A if A[k]["avail"] >= 0.95), key=lambda k: A[k]["shelf"])[:12]
    keys = list(refs.values()) + [k for k in dict.fromkeys(list(picks.values()) + top) if k not in refs.values()]
    labels = [lab_ for lab_ in refs] + [short(k) for k in keys[len(refs):]]
    fig, ax = plt.subplots(figsize=(14, 0.42 * len(keys) + 1.6))
    y = list(range(len(keys)))[::-1]
    for yi, k in zip(y, keys):
        r = A[k]
        is_ref = k in refs.values()
        is_pick = k in picks.values()
        col = "#8c8c8c" if is_ref else ("#1f4e79" if is_pick else "#6fa8dc")
        ax.barh(yi, r["shelf"] * 100, color=col, height=0.66)
        ax.text(r["shelf"] * 100 + 0.3, yi, f"{r['shelf'] * 100:.1f}  (bergerak {r['move'] * 100:.0f}, "
                f"goyang {r['spread'] * 100:.1f})", va="center", fontsize=7.5)
    ax.axvspan(2, 4, color="#6a994e", alpha=0.13, lw=0)
    ax.set_yticks(y)
    ax.set_yticklabels(labels, fontsize=8)
    ax.set_xlabel("galat bacaan di rak (cm), rata-rata 3 dunia × 20 rute — abu-abu: acuan, biru tua: pilihan")
    ax.set_xlim(0, 85)
    ax.grid(axis="x", alpha=0.3)
    ax.set_title("Peringkat kombinasi (galat terkecil saat berhenti)\n" + CODES, fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


def fig_worlds(S, refs, picks, path):
    keys = [("A", refs.get("A. EKF saja (lama)")),
            ("C", refs.get("C. A + geser-1 rata 20 (jarak) + geser-1 rata 20 (posisi)")),
            ("G", refs.get("G. EKF server sekarang (adaptif-diam rata 20)")),
            ("G + jendela C", refs.get("G + jendela C (rata 20 pada jarak dan posisi)")),
            ("BZ-EKF (K+B+Z)", refs.get("BZ-EKF (K+B+Z): bayangan badan dianggap selalu di belakang")),
            ("Pilihan tanpa kalibrasi", picks["rekomendasi tanpa kalibrasi statis"]),
            ("Pilihan (rekomendasi)", picks["rekomendasi (berhenti terbaik, bergerak maks. 2,5 cm lebih buruk dari G)"])]
    cols = ["#bbbbbb", "#c9a227", "#8c8c8c", "#e9c46a", "#b5179e", "#6fa8dc", "#1f4e79"]
    fig, axes = plt.subplots(1, 3, figsize=(16, 5.6))
    for ax, field, title, cap in ((axes[0], "shelf", "Bacaan di rak (cm)", 30),
                                  (axes[1], "move", "Galat saat bergerak (cm)", 60),
                                  (axes[2], "spread", "Goyangan saat berhenti (cm)", 8)):
        w = 0.8 / len(keys)
        for k, ((lab_, key), col) in enumerate(zip(keys, cols)):
            for i, wn in enumerate(WORLD_ORDER):
                v = S[wn][key][field] * 100
                x = i + (k - (len(keys) - 1) / 2) * w
                ax.bar(x, min(v, cap), w * 0.92, color=col, label=lab_ if i == 0 and field == "shelf" else None)
                ax.text(x, min(v, cap) + cap * 0.012, f"{v:.0f}" if v >= 10 else f"{v:.1f}", ha="center",
                        va="bottom", fontsize=6.5)
        if field == "shelf":
            ax.axhspan(2, 4, color="#6a994e", alpha=0.13, lw=0)
        ax.set_ylim(0, cap * 1.1)
        ax.set_xticks(range(3))
        ax.set_xticklabels([WORLD_LABEL[w_] for w_ in WORLD_ORDER], fontsize=9)
        ax.set_title(title + f" — batang dipotong di {cap}", fontsize=10.5)
        ax.grid(axis="y", alpha=0.3)
    fig.legend(fontsize=8.5, loc="lower center", ncol=6, frameon=False)
    fig.suptitle("Acuan vs kombinasi terpilih di tiga dunia (20 rute per dunia; angka = nilai sebenarnya)",
                 fontsize=12)
    fig.tight_layout(rect=(0, 0.06, 1, 1))
    fig.savefig(path, dpi=125)
    plt.close(fig)


def demo_tracks(keys, world="ramai"):
    scene, robot, people, stream, cal = ws.make_case(world, DEMO_SEED)
    cache, tracks = {}, {}
    for name, key in keys:
        core, pre, out, post = split(key)
        pre_cfg = next(p for p in ws.PRE + ws.OLD_PRE if ws.wkey(p) == pre)
        post_cfg = next(p for p in ws.POST if ws.wkey(p) == post)
        if (core, pre) not in cache:
            cache[(core, pre)] = ws.run(scene, stream, core, pre_cfg, cal)
        tracks[name] = ws.post(cache[(core, pre)][out], post_cfg)
    return scene, robot, people, tracks


def fig_tracks(S, demo, keys, colors, world, path):
    """Same figure as 1_lintasan_algoritma.png: the black line is where the
    trolley really drove, the coloured line what each algorithm showed."""
    scene, robot, _people, tracks = demo
    truth = [robot.segs[0][2]] + [s_[3] for s_ in robot.segs]
    ncol = 5
    nrow = (len(keys) + ncol) // ncol                # room for the explanation panel
    fig, axes = plt.subplots(nrow, ncol, figsize=(4.3 * ncol, 3.4 * nrow + 0.8))
    for ax in axes.flat[len(keys) + 1:]:
        ax.axis("off")
    for i, (name, key) in enumerate(keys):
        ax = axes[i // ncol][i % ncol]
        w_, d_ = scene["room"]["width"], scene["room"]["depth"]
        ax.add_patch(plt.Rectangle((0, 0), w_, d_, fill=False, ec="#777", lw=1))
        for a in scene["anchors"]:
            ax.plot(a["x"], a["y"], "^", color="#c62828", ms=8)
        ax.set_xlim(-0.6, w_ + 0.6)
        ax.set_ylim(-0.6, d_ + 0.6)
        ax.set_aspect("equal")
        ax.grid(alpha=0.25)
        ax.plot([p[0] for p in truth], [p[1] for p in truth], "-", color="#111", lw=2.2)
        tr = tracks[name]
        ax.plot([p[1] for p in tr], [p[2] for p in tr], "-", color=colors[name], lw=1.1)
        m = S[world][key]
        ax.set_title(f"{name}\nrak {m['shelf'] * 100:.1f} cm · bergerak {m['move'] * 100:.1f} cm · "
                     f"goyah {m['jitter'] * 100:.1f} cm", fontsize=9)
    ax = axes.flat[len(keys)]
    ax.axis("off")
    ax.text(0.0, 0.95,
            "Garis hitam: gerak troli sebenarnya\n(troli robot 0,3–0,7 m/s, berhenti 3–12 s)\n\n"
            "Garis berwarna: posisi yang ditampilkan\nalgoritma, data simulasi yang SAMA\n\n"
            "Segitiga merah: anchor\n\n"
            "Angka di judul: rata-rata 20 rute\n(rak = galat bacaan di rak;\n"
            "goyah = patah-patahnya garis)", va="top", fontsize=10)
    fig.suptitle(f"Lintasan tiap algoritma — {WORLD_LABEL[world]}, ruangan 9,5 x 5,9 m, laju anchor seperti hardware",
                 fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    try:
        fig.savefig(path, dpi=120)
    except OSError:                          # open in a viewer (Windows locks it)
        path = path.with_name(path.stem + "_baru" + path.suffix)
        fig.savefig(path, dpi=120)
    plt.close(fig)
    return path


# Algorithms in the ground-truth table: (label, combination key)
GT_KEYS = [("G (server default)", "G|tanpa|ekf|adaptif-diam rata 20"),
           ("C (jendela 20 x 20)", "old|geser-1 rata 20|ekf|geser-1 rata 20"),
           ("Web: G+Z+L + rata 3", "GZL|tanpa|ekf|geser-1 rata 3"),
           ("Web + kalibrasi: K+G+Z+L + rata 3", "KGZL|tanpa|ekf|geser-1 rata 3"),
           ("+ bayangan badan di belakang (K+B+Z+L + rata 3)", "KBZL|tanpa|ekf|geser-1 rata 3"),
           ("+ bayangan badan arah acak (K+R+Z+L + rata 3)", "KRZL|tanpa|ekf|geser-1 rata 3")]


def stop_readings(robot, rows):
    """Every stop of >= 2 s: (start s, length s, true point, shown point =
    mean over its last second, error m). The same reading as "bacaan di rak"."""
    times = [r[0] for r in rows]
    out = []
    for t0, t1, p0, p1 in robot.segs:
        if p0 != p1 or t1 - t0 < 2.0 or t0 < 3.0:
            continue
        pts = []
        for k in range(5):                             # every 0.2 s of the last second
            j = max(bisect.bisect_right(times, (t1 - 1.0 + 0.2 * k) * 1000) - 1, 0)
            pts.append((rows[j][1], rows[j][2]))
        ex = sum(p[0] for p in pts) / len(pts)
        ey = sum(p[1] for p in pts) / len(pts)
        out.append((t0, t1 - t0, p0, (ex, ey), math.hypot(ex - p0[0], ey - p0[1])))
    return out


def truth_sheet(wb, S, truth):
    """Numbers: every stop of the demo route, true point vs every algorithm's
    reading, per world; then the 20-route averages."""
    names = [n for n, _k in GT_KEYS]
    s = Sheet(wb, "Ground truth vs estimasi", [6, 9, 8, 8, 8] + [8, 8, 8] * len(names))
    s.title("Ground truth vs posisi yang ditampilkan — angka per titik berhenti",
            "Ground truth = titik berhenti sebenarnya di simulasi. Estimasi = rata-rata posisi pada detik terakhir "
            "berhenti (bacaan di rak). Rute demo per dunia; di bawahnya rata-rata 20 rute. x, y dalam m; galat cm.")
    for world, (_scene, robot, _people, tracks) in truth.items():
        s.section(f"{WORLD_LABEL[world]} — rute demo (seed {DEMO_SEED})")
        reads = {n: stop_readings(robot, tracks[n]) for n in names}
        header = ["No", "Mulai (s)", "Lama (s)", "x asli", "y asli"]
        for i, n in enumerate(names, 1):
            header += [f"x [{i}]", f"y [{i}]", f"galat [{i}]"]
        rows = []
        for i, base in enumerate(reads[names[0]]):
            row = [i + 1, base[0], base[1], base[2][0], base[2][1]]
            for n in names:
                q = reads[n][i]
                row += [q[3][0], q[3][1], q[4] * 100]
            rows.append(row)
        avg = ["Rata", None, None, None, None]
        for n in names:
            avg += [None, None, sum(q[4] for q in reads[n]) / len(reads[n]) * 100]
        rows.append(avg)
        s.table(header, rows, ["n", "s", "s", "m", "m"] + ["m", "m", "cm"] * len(names), bold_last=True)
    s.note("Kolom [1]–[6]: " + "; ".join(f"[{i}] {n}" for i, n in enumerate(names, 1)), 3)
    s.section("Rata-rata 20 rute per dunia — bacaan di rak (cm)")
    rows = []
    for n, k in GT_KEYS:
        row = [n]
        for w in WORLD_ORDER:
            r = S[w].get(k)
            row += [r["shelf"] * 100, r["shelf_p95"] * 100, r["shelf_le4"]] if r else [None, None, None]
        rows.append(row)
    hdr = ["Algoritma"]
    for w in WORLD_ORDER:
        hdr += [f"{w}: rata-rata", f"{w}: P95", f"{w}: ≤ 4 cm"]
    s.table(hdr, rows, [None] + ["cm", "cm", "pct"] * 3)


def animate(scene, robot, people, tracks, colors, path_gif, path_mp4, step=0.3, fps=10):
    from matplotlib.animation import FFMpegWriter, FuncAnimation, PillowWriter
    from matplotlib.patches import Circle
    anchors = scene["anchors"]
    tz = scene["tag_z"]
    times = [3.0 + k * step for k in range(int((robot.end - 3.0) / step))]
    names = list(tracks)
    err = {}
    for nm in names:
        tr, j, e = tracks[nm], 0, []
        for t in times:
            while j + 1 < len(tr) and tr[j + 1][0] <= t * 1000:
                j += 1
            tx, ty = robot.pos(t)
            e.append(math.hypot(tr[j][1] - tx, tr[j][2] - ty) * 100)
        err[nm] = e
    fig = plt.figure(figsize=(10, 9.2))
    gs = fig.add_gridspec(2, 1, height_ratios=[3, 1.05], top=0.80, bottom=0.07, hspace=0.25)
    ax = fig.add_subplot(gs[0])
    W, D = scene["room"]["width"], scene["room"]["depth"]
    ax.add_patch(plt.Rectangle((0, 0), W, D, fill=False, lw=1.5))
    for x in range(0, int(W) + 1):
        ax.axvline(x, color="#e6e6e6", lw=0.6, zorder=0)
    for y in range(0, int(D) + 1):
        ax.axhline(y, color="#e6e6e6", lw=0.6, zorder=0)
    for a in anchors:
        ax.plot(a["x"], a["y"], "^", color="#2a9d8f", ms=10, mec="black", zorder=4)
        ax.text(a["x"], a["y"] + (0.22 if a["y"] < 1 else -0.35), a["id"].replace("anchor-", "A"),
                ha="center", fontsize=8)
    ax.plot([s[2][0] for s in robot.segs], [s[2][1] for s in robot.segs], color="black", lw=0.8,
            ls="--", alpha=0.25)
    links = [ax.plot([], [], color="#2a9d8f", lw=0.8, alpha=0.5)[0] for _ in anchors]
    bodies = [ax.add_patch(Circle((0, 0), ws.BODY_R, color="#f4a261", ec="black", zorder=5)) for _ in people]
    if not bodies:                               # legend entry even with nobody there
        bodies_legend = ax.add_patch(Circle((-9, -9), ws.BODY_R, color="#f4a261", ec="black"))
    trails, dots = {}, {}
    for z, nm in enumerate(names):
        trails[nm], = ax.plot([], [], color=colors[nm], lw=1.3, alpha=0.75)
        dots[nm], = ax.plot([], [], "o", color=colors[nm], ms=8, mec="white", zorder=8 + z)
    truth, = ax.plot([], [], "s", color="white", mec="black", mew=2, ms=11, zorder=7)
    ax.set_xlim(-0.4, W + 0.4)
    ax.set_ylim(-0.4, D + 0.4)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m) — garis kisi tiap 1 m")
    ax.set_ylabel("y (m)")
    blocked_line, = ax.plot([], [], color="#e76f51", lw=2)
    ax.legend(handles=[truth, bodies[0] if bodies else bodies_legend, blocked_line] + [dots[nm] for nm in names],
              labels=["troli robot (posisi sebenarnya)", "pembeli (lebar & posisi asli)",
                      "jalur tag–anchor terhalang"] + names,
              loc="lower right", bbox_to_anchor=(1.0, 1.01), ncol=2, fontsize=7.5, frameon=False)
    info = fig.text(0.02, 0.985, "", va="top", fontsize=9, family="monospace",
                    bbox=dict(boxstyle="round", fc="white", ec="#cccccc"))
    ax2 = fig.add_subplot(gs[1])
    for t0, t1, p0, p1 in robot.segs:
        if p0 == p1:
            ax2.axvspan(t0, t1, color="#eeeeee", lw=0)
    ax2.axhspan(2, 4, color="#6a994e", alpha=0.15, lw=0)
    for nm in names:
        ax2.plot(times, err[nm], color=colors[nm], lw=1.1, label=nm)
    cursor = ax2.axvline(times[0], color="#e76f51", lw=1.5)
    ax2.set_ylim(0, 60)
    ax2.set_xlim(times[0], times[-1])
    ax2.set_ylabel("galat (cm)")
    ax2.set_xlabel("waktu (s) — abu-abu: troli berhenti, hijau: target 2–4 cm")
    ax2.legend(fontsize=7.5, loc="upper right", ncol=2)
    ax2.grid(alpha=0.3)
    idx = {nm: 0 for nm in names}

    def frame(n):
        t = times[n]
        tx, ty = robot.pos(t)
        mv, since, _left, _seg = robot.state(t)
        truth.set_data([tx], [ty])
        pp = [p.pos(t) for p in people]
        for b, q in zip(bodies, pp):
            b.center = q
        for ln, a in zip(links, anchors):
            blk = any(ws.blocked((tx, ty), (a["x"], a["y"], a["z"]), q, tz) for q in pp)
            ln.set_data([tx, a["x"]], [ty, a["y"]])
            ln.set_color("#e76f51" if blk else "#2a9d8f")
            ln.set_linewidth(2.2 if blk else 0.8)
            ln.set_alpha(0.9 if blk else 0.35)
        for nm in names:
            tr = tracks[nm]
            k = idx[nm]
            while k + 1 < len(tr) and tr[k + 1][0] <= t * 1000:
                k += 1
            idx[nm] = k
            dots[nm].set_data([tr[k][1]], [tr[k][2]])
            rec = [r for r in tr[max(0, k - 25):k + 1] if t * 1000 - r[0] <= 6000]
            trails[nm].set_data([r[1] for r in rec], [r[2] for r in rec])
        status = "berjalan" if mv else f"berhenti {since:4.1f} s"
        info.set_text("\n".join([f"t = {t:5.1f} s   {status}"]
                                + [f"galat {nm[:22]:22s}: {err[nm][n]:5.1f} cm" for nm in names]))
        cursor.set_xdata([t, t])
        return []

    anim = FuncAnimation(fig, frame, frames=len(times), blit=False)
    anim.save(str(path_mp4), writer=FFMpegWriter(fps=fps, bitrate=1800), dpi=100)
    anim.save(str(path_gif), writer=PillowWriter(fps=fps), dpi=62)
    plt.close(fig)


# ---------------------------------------------------------------------------
# workbook
# ---------------------------------------------------------------------------

GLOSSARY = [
    ("Jendela (window) N", "Sekelompok N sampel terakhir yang diringkas jadi satu nilai (rata-rata atau median)."),
    ("geser-1 (sliding)", "Setiap ada sampel baru, jendela bergeser satu: sampel 1–20, lalu 2–21, 3–22, … Satu keluaran "
     "per sampel; terlambat (N−1)/2 sampel."),
    ("geser-5 (hopping)", "Jendela N, tetapi keluaran baru hanya tiap 5 sampel: 1–20, 6–25, 11–30, …"),
    ("tunggu-N (tumbling)", "Tunggu N sampel BARU, keluarkan satu nilai, lalu mulai kosong lagi: 1–20, 21–40, … "
     "Keluaran ditahan di antaranya."),
    ("adaptif-diam", "Jendela geser-1 yang hanya dipakai saat troli berhenti (kecepatan EKF < 0,15 m/s), atas keluaran "
     "sejak berhenti (maks N). Saat bergerak tanpa jendela. Ini cara algoritma G."),
    ("Jendela pada jarak", "Diterapkan pada jarak mentah tiap anchor SEBELUM EKF. 1 sampel = 1 jarak = 0,4 s per anchor."),
    ("Jendela pada posisi", "Diterapkan pada posisi keluaran EKF. 1 sampel = 1 keluaran ≈ 0,35 s."),
    ("Rata-rata / median", "Rata-rata ikut terseret oleh satu nilai jauh; median (nilai tengah) tidak."),
    ("Mode diam (Z, ZUPT)", "Saat troli diam EKF menganggap kecepatan 0, sehingga posisinya tidak tergeser data yang buruk."),
    ("Kalibrasi statis (K)", "Offset jarak tiap anchor diukur sekali dengan tag di titik yang diketahui (fitur dashboard); "
     "bukan hardware tambahan."),
    ("Hitung ulang saat berhenti (S)", "Saat troli berhenti, posisi dihitung ulang dari median jarak tiap anchor sejak berhenti."),
    ("Buang anchor janggal (FDE)", "Bila satu anchor tidak cocok dengan yang lain (dan jaraknya kepanjangan), anchor itu "
     "dikeluarkan dari hitungan."),
    ("Bacaan di rak", "Galat rata-rata pada detik terakhir setiap berhenti ≥ 2 s: posisi yang dibaca untuk tahu rak mana."),
    ("Goyangan saat berhenti", "Seberapa jauh posisi bergoyang di sekitar rata-ratanya pada detik itu (kestabilan)."),
    ("Galat saat bergerak", "Galat terhadap posisi troli pada saat yang sama, selama troli berjalan."),
    ("Galat semua waktu", "Rata-rata galat di setiap saat, bergerak maupun berhenti."),
    ("Ketersediaan", "Bagian waktu posisi dinyatakan valid oleh gerbang validitas."),
]


def build_workbook(S, A, refs, picks, data, imgs, path, truth=None):
    wb = Workbook()
    s = Sheet(wb, "Ringkasan", [46, 11, 11, 11, 11, 11, 11], first=True)
    s.title("Studi Jendela: semua kombinasi (simulasi)",
            f"Troli robot berjalan sendiri di antara pembeli (0 / 2 / 5 orang). {data['runs']} rute acak per dunia. "
            "Tanpa hardware tambahan. Angka = rata-rata 3 dunia kecuali disebut lain (cm).")
    s.section("Acuan dan kombinasi terpilih (rata-rata 3 dunia)")
    rows = []
    for lab_, k in list(refs.items()) + [(f"PILIHAN — {p}", k) for p, k in picks.items()]:
        r = A[k]
        rows.append([lab_ if not lab_.startswith("PILIHAN") else lab_ + "\n" + describe(k),
                     r["shelf"] * 100, r["spread"] * 100, r["stop"] * 100, r["move"] * 100, r["all"] * 100,
                     r["avail"]])
    s.table(["Kombinasi", "Bacaan di rak", "Goyangan diam", "Galat diam", "Galat bergerak", "Galat semua",
             "Ketersediaan"], rows, [None, "cm", "cm", "cm", "cm", "cm", "pct"])
    for row in range(s.row - len(rows) - 1, s.row - 1):
        s.ws.row_dimensions[row].height = 44
    s.section("Per dunia — bacaan di rak (cm)")
    rows = []
    for lab_, k in list(refs.items()) + [(f"PILIHAN — {p}", k) for p, k in picks.items()]:
        rows.append([lab_] + [S[w][k]["shelf"] * 100 for w in WORLD_ORDER] + [S[w][k]["move"] * 100 for w in WORLD_ORDER])
    s.table(["Kombinasi"] + [f"Rak — {w}" for w in WORLD_ORDER] + [f"Gerak — {w}" for w in WORLD_ORDER],
            rows, [None] + ["cm"] * 6)

    s = Sheet(wb, "Konsep jendela", [24, 60, 30])
    s.title("Konsep jendela 20", "Tiga cara memakai jendela N sampel, dan berapa terlambatnya.")
    s.table(["Cara", "Sampel yang diambil", "Keluaran baru"], [
        ["geser-1 (sliding)", "1–20, lalu 2–21, 3–22, … (dipakai C)", "setiap sampel"],
        ["geser-5 (hopping)", "1–20, lalu 6–25, 11–30, …", "setiap 5 sampel"],
        ["tunggu-20 (tumbling)", "1–20, lalu 21–40, 41–60, … (menunggu 20 sampel baru)", "setiap 20 sampel"],
        ["adaptif-diam (G)", "keluaran sejak troli berhenti, maksimal N; saat bergerak tanpa jendela", "setiap sampel"],
    ], [None, None, None])
    s.table(["Jendela", "Pada jarak (1 sampel = 0,4 s)", "Pada posisi (1 sampel ≈ 0,35 s)"], [
        ["geser-1 rata 20: terlambat", "(20−1)/2 × 0,4 s = 3,8 s → 1,9 m pada 0,5 m/s", "±3,3 s → 1,7 m pada 0,5 m/s"],
        ["tunggu-20: keluaran tiap", "20 × 0,4 s = 8 s per anchor (gerbang validitas: posisi tidak valid)", "±7 s"],
        ["geser-1 rata 5: terlambat", "0,8 s → 0,4 m", "±0,7 s → 0,35 m"],
    ], [None, None, None])

    s = Sheet(wb, "Peringkat", [92, 11, 11, 11, 11, 11, 11])
    s.title("Peringkat 60 kombinasi terbaik", "Diurutkan dari bacaan di rak (rata-rata 3 dunia), hanya yang ketersediaannya ≥ 95%.")
    top = sorted((k for k in A if A[k]["avail"] >= 0.95), key=lambda k: A[k]["shelf"])[:60]
    s.table(["Kombinasi", "Bacaan di rak", "Goyangan diam", "Galat diam", "Galat bergerak", "Galat semua",
             "Ketersediaan"],
            [[describe(k), A[k]["shelf"] * 100, A[k]["spread"] * 100, A[k]["stop"] * 100, A[k]["move"] * 100,
              A[k]["all"] * 100, A[k]["avail"]] for k in top], [None, "cm", "cm", "cm", "cm", "cm", "pct"])

    s = Sheet(wb, "Semua kombinasi", [12, 22, 30, 24, 12] + [10] * 8)
    s.title("Semua kombinasi × dunia", f"{len(A)} kombinasi × 3 dunia. Rata-rata {data['runs']} rute. cm kecuali disebut lain.")
    rows = []
    for w in WORLD_ORDER:
        for k in sorted(S[w], key=lambda k: S[w][k]["shelf"]):
            core, pre, out, post = split(k)
            r = S[w][k]
            rows.append([CORE_LABEL[core], pre, OUT_LABEL[out], post, w, r["shelf"] * 100, r["shelf_p95"] * 100,
                         r["spread"] * 100, r["stop"] * 100, r["move"] * 100, r["all"] * 100, r["jitter"] * 100,
                         r["avail"]])
    s.table(["Inti EKF", "Jendela jarak", "Keluaran", "Jendela posisi", "Dunia", "Bacaan di rak", "P95 rak",
             "Goyangan diam", "Galat diam", "Galat bergerak", "Galat semua", "Kegoyahan garis", "Ketersediaan"],
            rows, [None] * 5 + ["cm"] * 7 + ["pct"])
    s.ws.freeze_panes = "F6"
    s.ws.auto_filter.ref = f"A5:M{5 + len(rows)}"

    if truth:
        truth_sheet(wb, S, truth)
    tracks = [(n, c) for n, c in imgs if "lintasan" in n]
    if tracks:
        s = Sheet(wb, "Lintasan vs ground truth", [12] * 12)
        s.title("Lintasan tiap algoritma vs gerak troli sebenarnya",
                "Garis hitam = gerak troli sebenarnya (ground truth) dari simulasi; garis berwarna = posisi yang "
                "ditampilkan tiap algoritma, data yang sama. Angka di judul tiap kotak = rata-rata 20 rute.")
        for name, cap in tracks:
            s.image(OUT / name, 1100, cap)

    s = Sheet(wb, "Gambar", [12] * 12)
    s.title("Gambar", "File PNG yang sama ada di docs/hasil-simulasi/.")
    for name, cap in imgs:
        s.image(OUT / name, 1000, cap)
    s = Sheet(wb, "Istilah", [30, 100])
    s.title("Istilah", "Arti setiap istilah di laporan ini.")
    s.table(["Istilah", "Arti"], [list(g) for g in GLOSSARY], [None, None])
    wb.save(path)


def main():
    data = json.loads((OUT / "jendela.json").read_text(encoding="utf-8"))
    S = data["summary"]
    A = averaged(S)
    refs, picks = choose(S, A)
    print("pilihan:")
    for p, k in picks.items():
        r = A[k]
        print(f"  {p}: {k}  rak {r['shelf'] * 100:.1f}  gerak {r['move'] * 100:.1f}  semua {r['all'] * 100:.1f}")
    best_core, _pre, best_out, _post = split(picks["rekomendasi (berhenti terbaik, bergerak maks. 2,5 cm lebih buruk dari G)"])
    imgs = [("jendela_0_konsep.png", "1. Konsep jendela: geser-1, geser-5, tunggu-20"),
            ("jendela_1_peta_G.png", "2. Semua kombinasi jendela pada inti G (server sekarang), dunia ramai"),
            ("jendela_2_peta_pilihan.png", "3. Semua kombinasi jendela pada inti kombinasi pilihan, dunia ramai"),
            ("jendela_2b_peta_pilihan_kosong.png", "3b. Sama, di dunia kosong (tanpa pembeli)"),
            ("jendela_3_peringkat.png", "4. Peringkat kombinasi"),
            ("jendela_4_dunia.png", "5. Acuan vs pilihan di tiga dunia")]
    fig_concept(OUT / imgs[0][0])
    fig_heat(S, "G", "ekf", OUT / imgs[1][0])
    fig_heat(S, best_core, best_out, OUT / imgs[2][0])
    fig_heat(S, best_core, best_out, OUT / imgs[3][0], world="kosong")
    fig_rank(S, A, refs, picks, OUT / imgs[4][0])
    fig_worlds(S, refs, picks, OUT / imgs[5][0])
    best = picks["rekomendasi (berhenti terbaik, bergerak maks. 2,5 cm lebih buruk dari G)"]
    lines = [("A. EKF saja (lama)", refs["A. EKF saja (lama)"]),
             ("C. jendela 20 x 20", refs["C. A + geser-1 rata 20 (jarak) + geser-1 rata 20 (posisi)"]),
             ("G. server sekarang", refs["G. EKF server sekarang (adaptif-diam rata 20)"]),
             ("G + jendela C (20 x 20)", refs["G + jendela C (rata 20 pada jarak dan posisi)"]),
             ("G + jendela pendek (median 3 jarak)", "G|geser-1 median 3|ekf|adaptif-diam rata 20"),
             ("G + hitung ulang saat berhenti + FDE", "G|tanpa|S+FDE|tanpa"),
             ("G + mode diam (Z)", refs["GZ (G + mode diam), tanpa jendela"]),
             ("G + Z + tolak jarak panjang (L)", picks["rekomendasi tanpa kalibrasi statis"]),
             ("kalibrasi + G + Z", "KGZ|tanpa|ekf|tanpa"),
             ("REKOMENDASI: kalibrasi + G + Z + L", best),
             ("rekomendasi + adaptif-diam 20", best.rsplit("|", 1)[0] + "|adaptif-diam rata 20"),
             ("rekomendasi + geser-1 rata 3 (mulus)", best.rsplit("|", 1)[0] + "|geser-1 rata 3"),
             ("rekomendasi + tunggu-20 (posisi)", best.rsplit("|", 1)[0] + "|tunggu-20 rata"),
             ("BZ-EKF (K+B+Z), badan dianggap di belakang", "KBZ|tanpa|ekf|tanpa"),
             ("BZ-EKF + L (K+B+Z+L)", "KBZL|tanpa|ekf|tanpa"),
             ("bayangan badan arah acak + L (K+R+Z+L)", "KRZL|tanpa|ekf|tanpa")]
    line_col = dict(zip([n for n, _ in lines],
                        ["#bbbbbb", "#c9a227", "#8c8c8c", "#e9c46a", "#b5838d", "#9c6644", "#6a994e",
                         "#6fa8dc", "#2a9d8f", "#1f4e79", "#7b2cbf", "#e76f51", "#d62828", "#b5179e",
                         "#f72585", "#3a0ca3"]))
    for world, name in (("kosong", "jendela_5_lintasan_kosong.png"), ("sepi", "jendela_5b_lintasan_sepi.png"),
                        ("ramai", "jendela_5c_lintasan_ramai.png")):
        name = fig_tracks(S, demo_tracks(lines, world), lines, line_col, world, OUT / name).name
        imgs.append((name, f"{len(imgs) + 1}. Lintasan tiap algoritma vs gerak troli sebenarnya — {WORLD_LABEL[world]}"))
    truth = {w: demo_tracks(GT_KEYS, w) for w in WORLD_ORDER}
    path = OUT / "Hasil_Jendela.xlsx"
    try:
        build_workbook(S, A, refs, picks, data, imgs, path, truth)
    except PermissionError:
        path = OUT / "Hasil_Jendela_baru.xlsx"
        build_workbook(S, A, refs, picks, data, imgs, path, truth)
    print("wrote", path)
    if "--no-anim" not in sys.argv:
        keys = [("G: server sekarang", refs["G. EKF server sekarang (adaptif-diam rata 20)"]),
                ("C: jendela 20 x 20", refs["C. A + geser-1 rata 20 (jarak) + geser-1 rata 20 (posisi)"]),
                ("Pilihan tanpa kalibrasi", picks["rekomendasi tanpa kalibrasi statis"]),
                ("Pilihan (rekomendasi)", picks["rekomendasi (berhenti terbaik, bergerak maks. 2,5 cm lebih buruk dari G)"])]
        colors = {"G: server sekarang": "#8c8c8c", "C: jendela 20 x 20": "#c9a227",
                  "Pilihan tanpa kalibrasi": "#6fa8dc", "Pilihan (rekomendasi)": "#1f4e79"}
        for world, name in (("ramai", "jendela_animasi"), ("kosong", "jendela_animasi_kosong")):
            scene, robot, people, tracks = demo_tracks(keys, world)
            animate(scene, robot, people, tracks, colors, OUT / f"{name}.gif", OUT / f"{name}.mp4")
            print(f"wrote {name}.gif / .mp4")


if __name__ == "__main__":
    main()
