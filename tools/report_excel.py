"""Accuracy report (Excel + plots) from one or more recordings.

    python tools/report_excel.py server/logs/A.jsonl server/logs/B.jsonl --out docs/laporan-akurasi

Finds the stops in each recording (tag still for >= 2.5 s) and reports their
precision (spread around the stop's mean). The "Titik Diam" sheet has yellow
cells for the true X/Y measured with the laser meter; the error in cm is then
computed by Excel formulas, so accuracy appears as soon as they are filled in.
"""
import argparse
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from openpyxl import Workbook  # noqa: E402
from openpyxl.drawing.image import Image as XlImage  # noqa: E402
from openpyxl.styles import Alignment, Border, Font, PatternFill, Side  # noqa: E402
from openpyxl.utils import get_column_letter  # noqa: E402

STILL_SPEED = 0.25     # m/s, below this the tag counts as still
MIN_STOP_S = 2.5       # shortest stop that is reported
SETTLE_S = 1.0         # dropped at the start of each stop


def load(path):
    rows = [json.loads(line) for line in open(path, encoding="utf-8")]
    meta = rows[0]
    t0 = meta["t"]
    est = [dict(r, s=(r["t"] - t0) / 1000) for r in rows if r["k"] == "est"]
    seen, ranges = set(), []
    for r in rows:
        if r["k"] != "rx":
            continue
        for g in r["p"].get("ranges", []):
            key = (g["src"], g["dst"], g.get("seq"), g.get("ts"))
            if key in seen:
                continue
            seen.add(key)
            ranges.append(dict(g, s=(r["t"] - t0) / 1000))
    return meta, est, ranges


def speed_series(est):
    """Speed from positions over ~1 s, robust to the filter's velocity noise."""
    out = []
    for i, e in enumerate(est):
        j = i
        while j > 0 and e["s"] - est[j - 1]["s"] <= 1.0:
            j -= 1
        dt = e["s"] - est[j]["s"]
        out.append(math.hypot(e["x"] - est[j]["x"], e["y"] - est[j]["y"]) / dt if dt > 0.3 else 0.0)
    return out


def find_stops(est):
    spd = speed_series(est)
    stops, run = [], []
    for e, v in zip(est, spd):
        if v < STILL_SPEED:
            run.append(e)
            continue
        if run and run[-1]["s"] - run[0]["s"] >= MIN_STOP_S:
            stops.append(run)
        run = []
    if run and run[-1]["s"] - run[0]["s"] >= MIN_STOP_S:
        stops.append(run)
    return [[e for e in st if e["s"] >= st[0]["s"] + SETTLE_S] for st in stops]


def pct(vals, p):
    v = sorted(vals)
    if not v:
        return float("nan")
    k = (len(v) - 1) * p
    lo, hi = math.floor(k), math.ceil(k)
    return v[lo] + (v[hi] - v[lo]) * (k - lo)


def stop_stats(st):
    xs, ys = [e["x"] for e in st], [e["y"] for e in st]
    mx, my = sum(xs) / len(xs), sum(ys) / len(ys)
    d = [math.hypot(x - mx, y - my) for x, y in zip(xs, ys)]
    sx = math.sqrt(sum((x - mx) ** 2 for x in xs) / len(xs))
    sy = math.sqrt(sum((y - my) ** 2 for y in ys) / len(ys))
    return dict(t0=st[0]["s"], t1=st[-1]["s"], n=len(st), mx=mx, my=my, sx=sx, sy=sy,
                cep50=pct(d, 0.5), cep95=pct(d, 0.95), dmax=max(d),
                sigma=sum(e["sigma"] for e in st) / len(st))


def anchor_stats(meta, est, ranges, stops):
    scene = meta["scene"]
    tag_z = scene.get("tag_z", 0)
    anchors = {a["id"]: a for a in scene["anchors"]}
    dur = est[-1]["s"] - est[0]["s"] if est else 1
    windows = [(s[0]["s"], s[-1]["s"], stop_stats(s)) for s in stops if len(s) > 2]
    out = []
    for aid, a in anchors.items():
        rs = [g for g in ranges if aid in (g["src"], g["dst"])]
        res = []
        for g in rs:
            for t0, t1, ss in windows:
                if t0 <= g["s"] <= t1:
                    b = min(est, key=lambda e: abs(e["s"] - g["s"]))["b"]
                    d = math.sqrt((ss["mx"] - a["x"]) ** 2 + (ss["my"] - a["y"]) ** 2
                                  + (tag_z - a["z"]) ** 2)
                    res.append(g["range"] - d - b)
                    break
        mean = sum(res) / len(res) if res else float("nan")
        std = math.sqrt(sum((r - mean) ** 2 for r in res) / len(res)) if res else float("nan")
        out.append(dict(
            anchor=a.get("label") or aid, n=len(rs), hz=len(rs) / dur if dur else 0,
            rx=sum(g.get("rx_power", 0) for g in rs) / len(rs) if rs else float("nan"),
            fp=sum(g.get("fp_power", 0) for g in rs) / len(rs) if rs else float("nan"),
            nres=len(res), rmean=mean, rstd=std,
            outl=sum(abs(r) > 0.3 for r in res) / len(res) if res else float("nan")))
    return out


def path_length(est):
    return sum(math.hypot(b["x"] - a["x"], b["y"] - a["y"]) for a, b in zip(est, est[1:]))


# ---------------------------------------------------------------------------
# plots
# ---------------------------------------------------------------------------

def plot_track(name, meta, est, stops, out):
    room = meta["scene"]["room"]
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.add_patch(plt.Rectangle((0, 0), room["width"], room["depth"], fill=False, ec="#777", lw=1.2))
    for a in meta["scene"]["anchors"]:
        ax.plot(a["x"], a["y"], "^", ms=11, color="#c62828")
        ax.annotate(a.get("label") or a["id"], (a["x"], a["y"]), textcoords="offset points",
                    xytext=(6, 6), fontsize=9, color="#c62828")
    ax.plot([e["x"] for e in est], [e["y"] for e in est], "-", color="#6c8ebf", lw=1.4,
            label="lintasan estimasi")
    ax.plot(est[0]["x"], est[0]["y"], "o", color="green", ms=8, label="mulai")
    for i, st in enumerate(stops, 1):
        ss = stop_stats(st)
        ax.plot([e["x"] for e in st], [e["y"] for e in st], ".", ms=4, color="#e69500")
        ax.annotate(f"S{i}", (ss["mx"], ss["my"]), textcoords="offset points", xytext=(6, -12),
                    fontsize=10, fontweight="bold")
    ax.plot([], [], ".", color="#e69500", label="titik diam (S1..)")
    ax.set_aspect("equal")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title(f"Lintasan tag — {name}")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.1), ncol=3, fontsize=9)
    fig.tight_layout()
    fig.savefig(out, dpi=130, bbox_inches="tight")
    plt.close(fig)


def plot_stops(name, stops, out):
    n = len(stops)
    if not n:
        return False
    cols = min(n, 4)
    rows = math.ceil(n / cols)
    fig, axes = plt.subplots(rows, cols, figsize=(3.2 * cols, 3.2 * rows), squeeze=False)
    for i, st in enumerate(stops):
        ax = axes[i // cols][i % cols]
        ss = stop_stats(st)
        dx = [(e["x"] - ss["mx"]) * 100 for e in st]
        dy = [(e["y"] - ss["my"]) * 100 for e in st]
        ax.plot(dx, dy, ".", color="#6c8ebf")
        for r, c in ((ss["cep50"], "#2e7d32"), (ss["cep95"], "#c62828")):
            ax.add_patch(plt.Circle((0, 0), r * 100, fill=False, ec=c, lw=1.4))
        lim = max(10, ss["dmax"] * 100 * 1.2)
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)
        ax.set_aspect("equal")
        ax.grid(alpha=0.3)
        ax.set_title(f"S{i + 1}: CEP50 {ss['cep50'] * 100:.1f} / CEP95 {ss['cep95'] * 100:.1f} cm",
                     fontsize=9)
        ax.set_xlabel("ΔX (cm)", fontsize=8)
        ax.set_ylabel("ΔY (cm)", fontsize=8)
    for j in range(n, rows * cols):
        axes[j // cols][j % cols].axis("off")
    fig.suptitle(f"Sebaran posisi saat diam — {name}  (hijau CEP50, merah CEP95)", fontsize=10)
    fig.tight_layout()
    fig.savefig(out, dpi=130, bbox_inches="tight")
    plt.close(fig)
    return True


def plot_time(name, est, stops, out):
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(9, 5), sharex=True)
    s = [e["s"] for e in est]
    a1.plot(s, [e["x"] for e in est], label="X")
    a1.plot(s, [e["y"] for e in est], label="Y")
    for st in stops:
        a1.axvspan(st[0]["s"], st[-1]["s"], color="#e69500", alpha=0.15)
        a2.axvspan(st[0]["s"], st[-1]["s"], color="#e69500", alpha=0.15)
    a1.set_ylabel("posisi (m)")
    a1.legend(fontsize=8)
    a1.grid(alpha=0.3)
    a2.plot(s, [e["sigma"] * 100 for e in est], color="#7b1fa2", label="σ estimasi (cm)")
    a2.plot(s, [speed_series(est)[i] * 100 for i in range(len(est))], color="#999",
            lw=0.8, label="kecepatan (cm/s)")
    a2.set_xlabel("waktu (s)")
    a2.legend(fontsize=8)
    a2.grid(alpha=0.3)
    a1.set_title(f"Posisi dan ketidakpastian terhadap waktu — {name} (oranye = diam)")
    fig.tight_layout()
    fig.savefig(out, dpi=130, bbox_inches="tight")
    plt.close(fig)


# ---------------------------------------------------------------------------
# workbook
# ---------------------------------------------------------------------------

NAVY, BLUE, GREY = "1F3864", "2F5597", "595959"
F_HEAD = PatternFill("solid", fgColor="D9E2F3")
F_BAND = PatternFill("solid", fgColor="F3F6FB")
F_INPUT = PatternFill("solid", fgColor="FFF2CC")
F_NOTE = PatternFill("solid", fgColor="F2F2F2")
F_KEY = PatternFill("solid", fgColor="E2EFDA")
F_TITLE = PatternFill("solid", fgColor=NAVY)
THIN = Side(style="thin", color="BFBFBF")
BOX = Border(left=THIN, right=THIN, top=THIN, bottom=THIN)
NUM = {"cm": "0.0", "m": "0.00", "m3": "0.000", "s": "0.0", "n": "0", "hz": "0.0",
       "pct": "0%", "db": "0.0"}
KV_NUM = {"cm": '0.0 "cm"', "hz": '0.0 "Hz"', "n": "0"}


class Sheet:
    """A sheet written top to bottom with one consistent style."""

    def __init__(self, wb, name, widths, first=False):
        self.ws = wb.active if first else wb.create_sheet(name)
        self.ws.title = name
        self.ncol = len(widths)
        for i, w in enumerate(widths, 1):
            self.ws.column_dimensions[get_column_letter(i)].width = w
        self.ws.sheet_view.showGridLines = False
        ps = self.ws.page_setup
        ps.orientation, ps.fitToWidth, ps.fitToHeight = "landscape", 1, 0
        self.ws.sheet_properties.pageSetUpPr.fitToPage = True
        self.row = 1

    def _merge(self, height=None):
        self.ws.merge_cells(start_row=self.row, start_column=1, end_row=self.row,
                            end_column=self.ncol)
        if height:
            self.ws.row_dimensions[self.row].height = height

    def title(self, text, sub):
        c = self.ws.cell(self.row, 1, text)
        c.font, c.fill = Font(bold=True, size=16, color="FFFFFF"), F_TITLE
        c.alignment = Alignment(vertical="center", indent=1)
        self._merge(32)
        self.row += 1
        c = self.ws.cell(self.row, 1, sub)
        c.font = Font(italic=True, color=GREY)
        c.alignment = Alignment(wrap_text=True, vertical="top", indent=1)
        self._merge(32)
        self.row += 2

    def section(self, text):
        c = self.ws.cell(self.row, 1, text)
        c.font = Font(bold=True, size=12, color=BLUE)
        for col in range(1, self.ncol + 1):
            self.ws.cell(self.row, col).border = Border(bottom=Side(style="medium", color=BLUE))
        self.ws.row_dimensions[self.row].height = 20
        self.row += 2

    def note(self, text, lines=2, fill=F_NOTE, bold=False):
        c = self.ws.cell(self.row, 1, text)
        c.fill, c.font = fill, Font(bold=bold, color="262626")
        c.alignment = Alignment(wrap_text=True, vertical="center", indent=1)
        self._merge(16 * lines + 6)
        self.row += 1

    def gap(self, n=1):
        self.row += n

    def table(self, header, rows, fmts, key_col=0, inputs=(), bold_last=False):
        """fmts: one NUM key (or None for text) per column."""
        top = self.row
        for c, h in enumerate(header, 1):
            cell = self.ws.cell(top, c, h)
            cell.font, cell.fill, cell.border = Font(bold=True, color=NAVY), F_HEAD, BOX
            cell.alignment = Alignment(wrap_text=True, vertical="center", horizontal="center")
        self.ws.row_dimensions[top].height = 42
        for i, values in enumerate(rows):
            r = top + 1 + i
            for c, v in enumerate(values, 1):
                if isinstance(v, float) and math.isnan(v):
                    v = None
                cell = self.ws.cell(r, c, v)
                cell.border = BOX
                f = fmts[c - 1]
                if f:
                    cell.number_format = NUM.get(f, f)
                    cell.alignment = Alignment(horizontal="center", vertical="center")
                else:
                    cell.alignment = Alignment(wrap_text=True, vertical="center")
                if c - 1 in inputs:
                    cell.fill = F_INPUT
                elif i % 2:
                    cell.fill = F_BAND
                if c - 1 == key_col:
                    cell.font = Font(bold=True)
                if bold_last and i == len(rows) - 1:
                    cell.font = Font(bold=True)
        self.row = top + len(rows) + 2
        return top + 1, top + len(rows)

    def kv(self, items, label_span=3):
        """Label | value | explanation rows; value may be an Excel formula."""
        for label, value, fmt, expl in items:
            r = self.row
            for col in range(1, label_span + 3):
                self.ws.cell(r, col).border = BOX
            c = self.ws.cell(r, 1, label)
            c.font = Font(bold=True)
            c.alignment = Alignment(vertical="center", wrap_text=True, indent=1)
            self.ws.merge_cells(start_row=r, start_column=1, end_row=r, end_column=label_span)
            v = self.ws.cell(r, label_span + 1, value)
            v.number_format = KV_NUM.get(fmt, "General")
            v.font, v.fill = Font(bold=True, size=12, color=NAVY), F_KEY
            v.alignment = Alignment(horizontal="center", vertical="center")
            self.ws.cell(r, label_span + 2).fill = F_KEY
            self.ws.merge_cells(start_row=r, start_column=label_span + 1, end_row=r,
                                end_column=label_span + 2)
            e = self.ws.cell(r, label_span + 3, expl)
            e.font = Font(color=GREY)
            e.alignment = Alignment(wrap_text=True, vertical="center", indent=1)
            self.ws.merge_cells(start_row=r, start_column=label_span + 3,
                                end_row=r, end_column=self.ncol)
            self.ws.row_dimensions[self.row].height = 30
            self.row += 1
        self.row += 1

    def image(self, path, width_px, caption=None):
        if caption:
            c = self.ws.cell(self.row, 1, caption)
            c.font = Font(bold=True, color=NAVY)
            self.row += 1
        img = XlImage(str(path))
        k = width_px / img.width
        img.width, img.height = int(width_px), int(img.height * k)
        self.ws.add_image(img, f"A{self.row}")
        self.row += math.ceil(img.height / 20) + 2


def avg(vals):
    vals = [v for v in vals if v is not None]
    return sum(vals) / len(vals) if vals else None


def static_rows(cfg):
    """Per test and anchor: measured, geometric distance, residual after the common offset."""
    out = []
    for t in cfg["tests"]:
        rows = []
        for aid, (rng, n) in t["ranges"].items():
            ax, ay, az = cfg["anchors"][aid]
            d = math.sqrt((t["x"] - ax) ** 2 + (t["y"] - ay) ** 2 + (cfg["tag_z"] - az) ** 2)
            rows.append([aid, rng, n, d, rng - d])
        b = avg([r[4] for r in rows])
        out.append((t, b, [r + [b, (r[4] - b) * 100] for r in rows]))
    return out


def build(logs, out_dir, fname="Laporan_Akurasi_UWB.xlsx"):
    out_dir.mkdir(parents=True, exist_ok=True)
    recs = []
    for p in logs:
        meta, est, ranges = load(p)
        name = meta.get("name") or Path(p).stem
        stops = [s for s in find_stops(est) if len(s) >= 3]
        short = name.replace("SESSION-TEST-01-", "")
        imgs = [out_dir / f"{short}_lintasan.png", out_dir / f"{short}_titik_diam.png",
                out_dir / f"{short}_waktu.png"]
        plot_track(short, meta, est, stops, imgs[0])
        if not plot_stops(short, stops, imgs[1]):
            imgs.pop(1)
        plot_time(short, est, stops, imgs[-1])
        recs.append(dict(name=short, file=Path(p).name, meta=meta, est=est, ranges=ranges,
                         stops=stops, imgs=imgs))

    static_file, sim_file = out_dir / "uji_statis_dashboard.json", out_dir / "simulasi.json"
    dash = json.loads(static_file.read_text(encoding="utf-8")) if static_file.exists() else None
    sim = json.loads(sim_file.read_text(encoding="utf-8")) if sim_file.exists() else None
    scene = recs[0]["meta"]["scene"]
    room = f'{scene["room"]["width"]:g} × {scene["room"]["depth"]:g} m'
    stops = [(r["name"], i, stop_stats(s)) for r in recs for i, s in enumerate(r["stops"], 1)]
    rate = avg([len(r["est"]) / (r["est"][-1]["s"] - r["est"][0]["s"]) for r in recs])
    srows = static_rows(dash) if dash else []
    resid = [row[-1] for _, _, rows in srows for row in rows]

    wb = Workbook()
    W = [24, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12]

    # =============================== Ringkasan ===============================
    s = Sheet(wb, "Ringkasan", W, first=True)
    s.title("Laporan Akurasi UWB Indoor Positioning",
            f"Smart Trolley – Smart Market · ruangan {room} · 4 anchor ESP32 + DW1000 · "
            f"{len(recs)} rekaman + {len(dash['tests']) if dash else 0} uji statis dashboard"
            + (" + simulasi" if sim else ""))
    s.section("Hasil utama")
    items = [
        ("Presisi saat diam – CEP50", avg([x[2]["cep50"] for x in stops]) * 100, "cm",
         "Pengukuran. Separuh posisi berada dalam radius ini dari titik rata-ratanya."),
        ("Presisi saat diam – CEP95", avg([x[2]["cep95"] for x in stops]) * 100, "cm",
         "Pengukuran. 95% posisi berada dalam radius ini (seberapa stabil posisi saat troli diam)."),
    ]
    if resid:
        items.append(("Konsistensi jarak antar-anchor", math.sqrt(sum(r * r for r in resid) / len(resid)),
                      "cm", "Uji statis dashboard. RMS selisih jarak tiap anchor setelah offset jarak "
                      "antena dikoreksi; kecil berarti keempat anchor 'sepakat'."))
    items.append(("Laju update posisi", rate, "hz", "Pengukuran. Banyaknya posisi baru per detik."))
    if sim:
        sp = sim["points"]
        items += [
            ("Akurasi saat diam – galat rata-rata", avg([p["sim"]["mean_err"] for p in sp]) * 100,
             "cm", "Simulasi ruangan yang sama. Jarak rata-rata posisi estimasi ke posisi benar."),
            ("Akurasi saat diam – CEP95", avg([p["sim"]["cep95"] for p in sp]) * 100, "cm",
             "Simulasi. 95% posisi berada dalam radius ini dari posisi benar."),
            ("Akurasi saat berjalan – RMSE", sim["walk"]["rmse"] * 100, "cm",
             f"Simulasi, troli berjalan {sim['walk_speed']} m/s di rute seperti rekaman V2."),
            ("Akurasi lintasan – cross-track RMSE", sim["walk"]["xt_rmse"] * 100, "cm",
             "Simulasi. Jarak tegak lurus posisi ke lintasan benar (seberapa lurus jalurnya)."),
        ]
    items.append(("Akurasi dari meteran laser (hardware)",
                  "=IF(COUNT('1 Pengukuran'!M:M)=0,\"belum diukur\",AVERAGE('1 Pengukuran'!M:M))",
                  "cm", "Terisi otomatis setelah X/Y sebenarnya diisi di sheet '1 Pengukuran'."))
    s.kv(items)

    s.section("Kesimpulan")
    if sim:
        s.note(f"1. Saat troli diam, posisi stabil: 95% posisi berada dalam ±"
               f"{avg([x[2]['cep95'] for x in stops]) * 100:.0f} cm dari rata-ratanya.", 1)
        s.note(f"2. Simulasi dengan ruangan, anchor dan noise yang sama menghasilkan presisi "
               f"yang sebanding (CEP95 {avg([p['sim']['prec95'] for p in sim['points']]) * 100:.1f}"
               f" cm vs {avg([x[2]['cep95'] for x in stops]) * 100:.1f} cm), jadi model simulasi "
               "mewakili alat sebenarnya.", 2)
        s.note(f"3. Dari simulasi itu, perkiraan akurasi ±"
               f"{avg([p['sim']['mean_err'] for p in sim['points']]) * 100:.0f} cm saat diam dan "
               f"RMSE {sim['walk']['rmse'] * 100:.0f} cm saat berjalan. Penyebab utamanya offset "
               "tiap anchor yang belum dikalibrasi; kalibrasi di 2–4 titik terukur akan "
               "menurunkannya.", 2)
        s.note(f"4. Laju update hardware {rate:.1f} Hz, lebih rendah dari simulasi "
               f"({sim['walk']['rate']:.0f} Hz): sebagian paket tag terlambat atau hilang di WiFi.", 1)
    s.gap()
    s.section("Istilah")
    for t in ("Presisi: seberapa rapat posisi saat tag diam (tidak butuh posisi benar).",
              "Akurasi: seberapa dekat posisi dengan posisi benar (butuh posisi benar dari meteran "
              "laser atau simulasi).",
              "CEP50 / CEP95: radius lingkaran yang memuat 50% / 95% posisi.",
              "RMSE: akar rata-rata kuadrat galat; galat besar lebih berpengaruh.",
              "Cross-track error: jarak tegak lurus dari posisi ke lintasan yang benar."):
        s.note("• " + t, 1, fill=PatternFill(fill_type=None))
    s.gap()
    s.section("Isi file")
    for t in ("1 Pengukuran – rekaman hardware, titik diam, dan kolom kuning untuk posisi benar.",
              "2 Uji Statis – 4 uji dari dashboard: konsistensi jarak tiap anchor.",
              "3 Simulasi – perbandingan hardware vs simulasi dan perkiraan akurasi.",
              "4 Per Anchor – kualitas sinyal dan residu tiap anchor.",
              "5 Grafik – lintasan, sebaran titik diam, dan posisi terhadap waktu.",
              "Data – semua posisi hasil estimasi (data mentah)."):
        s.note("• " + t, 1, fill=PatternFill(fill_type=None))

    # ============================== 1 Pengukuran ==============================
    s = Sheet(wb, "1 Pengukuran", [16, 7, 10, 10, 10, 10, 11, 11, 11, 11, 13, 13, 12])
    s.title("1. Data Pengukuran Hardware",
            f"Rekaman dari web dashboard (Record session) di ruangan {room}. Titik diam dideteksi "
            f"otomatis: kecepatan < {STILL_SPEED} m/s selama ≥ {MIN_STOP_S:g} s.")
    s.section("A. Rekaman")
    rows = []
    for r in recs:
        est = r["est"]
        dur = est[-1]["s"] - est[0]["s"]
        rows.append([r["name"], dur, len(est), len(est) / dur, path_length(est),
                     len(r["stops"]), avg([e["sigma"] for e in est]) * 100,
                     avg([e["b"] for e in est]) * 100])
    s.table(["Rekaman", "Durasi (s)", "Jumlah posisi", "Laju (Hz)", "Panjang lintasan (m)",
             "Titik diam", "Margin error ±σ (cm)", "Offset jarak b (cm)"], rows,
            [None, "s", "n", "hz", "m", "n", "cm", "cm"])
    s.section("B. Titik diam")
    s.note("Isi kolom kuning dengan posisi tag sebenarnya yang diukur dengan meteran laser "
           "(titik (0,0) = anchor A2). Kolom 'Galat' langsung terhitung. Lokasi S1, S2, ... "
           "ada di gambar lintasan (sheet 5 Grafik).", 2, fill=F_INPUT)
    s.gap()
    rows = [[n, f"S{i}", st["t1"] - st["t0"], st["n"], st["mx"], st["my"], st["cep50"] * 100,
             st["cep95"] * 100, st["dmax"] * 100, st["sigma"] * 100, None, None, None]
            for n, i, st in stops]
    a, b = s.table(["Rekaman", "Titik", "Durasi (s)", "Sampel", "X rata-rata (m)",
                    "Y rata-rata (m)", "CEP50 (cm)", "CEP95 (cm)", "Simpangan maks (cm)",
                    "Margin error ±σ (cm)", "X sebenarnya (m)", "Y sebenarnya (m)", "Galat (cm)"],
                   rows, [None, None, "s", "n", "m3", "m3", "cm", "cm", "cm", "cm", "m3", "m3",
                          "cm"], inputs=(10, 11))
    for r in range(a, b + 1):
        c = s.ws.cell(r, 13, f'=IF(OR(K{r}="",L{r}=""),"",SQRT((E{r}-K{r})^2+(F{r}-L{r})^2)*100)')
        c.number_format, c.border, c.font = "0.0", BOX, Font(bold=True)
        c.alignment = Alignment(horizontal="center")
    s.section("C. Akurasi dari posisi benar")
    rng = f"M{a}:M{b}"
    s.kv([("Titik yang sudah diukur", f"=COUNT({rng})", "n", "jumlah baris yang kolom kuningnya terisi"),
          ("Galat rata-rata", f'=IF(COUNT({rng})=0,"-",AVERAGE({rng}))', "cm", ""),
          ("RMSE", f'=IF(COUNT({rng})=0,"-",SQRT(SUMSQ({rng})/COUNT({rng})))', "cm", ""),
          ("Galat maksimum", f'=IF(COUNT({rng})=0,"-",MAX({rng}))', "cm", "")])
    s.note(f"Konfigurasi saat merekam: tinggi tag {scene.get('tag_z')} m, tinggi anchor "
           f"{', '.join(sorted({str(x['z']) for x in scene['anchors']}))} m. Pastikan sama dengan "
           "pemasangan sebenarnya, karena tinggi yang salah menggeser posisi.", 2)

    # ============================== 2 Uji Statis ==============================
    if dash:
        s = Sheet(wb, "2 Uji Statis", [12, 12, 12, 12, 13, 13, 13, 40])
        s.title("2. Uji Statis dari Dashboard",
                "Panel 'Calibration & static test'. Tanpa posisi benar, setiap jarak terukur "
                "dicek terhadap posisi estimasi untuk melihat apakah keempat anchor konsisten.")
        s.section("A. Ringkasan per uji")
        rows = [[t["name"], t["x"], t["y"], t["sigma_cm"], b * 100,
                 math.sqrt(sum(r[-1] ** 2 for r in rr) / len(rr)), max(abs(r[-1]) for r in rr),
                 t.get("note", "")] for t, b, rr in srows]
        rows.append(["Rata-rata", None, None, avg([r[3] for r in rows]), avg([r[4] for r in rows]),
                     math.sqrt(sum(x * x for x in resid) / len(resid)),
                     max(abs(x) for x in resid), ""])
        s.table(["Uji", "X estimasi (m)", "Y estimasi (m)", "Presisi σ (cm)", "Offset jarak b (cm)",
                 "Residu RMS (cm)", "Residu maks (cm)", "Catatan"], rows,
                [None, "m3", "m3", "cm", "cm", "cm", "cm", None], bold_last=True)
        s.section("B. Detail jarak tiap anchor")
        rows = []
        for t, b, rr in srows:
            for aid, rng_, n, d, diff, bb, res in rr:
                rows.append([t["name"], aid, n, rng_, d, diff, res])
        s.table(["Uji", "Anchor", "Sampel", "Jarak terukur (m)", "Jarak geometri (m)",
                 "Selisih (m)", "Residu setelah offset (cm)"], rows,
                [None, None, "n", "m3", "m3", "m3", "cm"])
        s.section("Cara membaca")
        s.note("Selisih = jarak terukur − jarak dari posisi estimasi ke anchor. Bagian yang sama "
               "untuk semua anchor (±82 cm) adalah delay antena DW1000 dan otomatis dikoreksi EKF. "
               "Sisanya (residu) menunjukkan seberapa cocok keempat jarak satu sama lain.", 2)
        s.note("Ini bukan akurasi terhadap posisi benar. Angka 'accuracy 858 cm' di dashboard "
               "muncul karena titik acuan diisi (9,80; 6,82) = posisi anchor-3, padahal tag ada "
               "di sekitar (1,28; 5,72). Jangan tekan 'Apply calibration' dengan acuan yang salah.",
               2, fill=F_INPUT)

    # =============================== 3 Simulasi ===============================
    if sim:
        s = Sheet(wb, "3 Simulasi", [22, 11, 11, 13, 13, 13, 13, 13, 13])
        s.title("3. Perbandingan Hardware dan Simulasi",
                "Simulasi memakai ruangan, posisi anchor, tinggi tag, dan offset jarak yang sama dengan "
                "hardware. Di simulasi posisi benar diketahui, sehingga akurasi bisa dihitung.")
        s.section("Pengaturan simulasi")
        s.kv([("Offset jarak bersama (b)", sim["bias_common"] * 100, "cm", "sama dengan yang terlihat di hardware"),
              ("Noise jarak (1σ)", sim["noise"] * 100, "cm", "noise two-way ranging saat LOS"),
              ("Sebaran offset tiap anchor (1σ)", 8, "cm", "anchor belum dikalibrasi"),
              ("Outlier / paket hilang", f"{sim['outlier_p']:.0%} / {sim['loss_p']:.0%}", None,
               "lonjakan jarak akibat multipath dan paket yang tidak sampai"),
              ("Pengulangan", f"{sim['runs']} × {sim['static_s']:g} s per titik", None,
               "Monte-Carlo, hasil dirata-rata")])
        s.section("A. Titik statis – posisi sama dengan titik diam hasil pengukuran")
        rows = []
        for p in sim["points"]:
            q = p["sim"]
            rows.append([p["name"], p["x"], p["y"],
                         p["hw_prec95"] * 100 if p["hw_prec95"] is not None else None,
                         q["prec95"] * 100, q["mean_err"] * 100, q["rmse"] * 100,
                         q["cep95"] * 100, q["max"] * 100])
        rows.append(["Rata-rata", None, None] + [avg([r[c] for r in rows]) for c in range(3, 9)])
        s.table(["Titik", "X (m)", "Y (m)", "Presisi CEP95 hardware (cm)",
                 "Presisi CEP95 simulasi (cm)", "Galat rata-rata simulasi (cm)",
                 "RMSE simulasi (cm)", "CEP95 galat simulasi (cm)", "Galat maks simulasi (cm)"],
                rows, [None, "m", "m", "cm", "cm", "cm", "cm", "cm", "cm"], bold_last=True)
        s.note("Kolom presisi hardware dan simulasi dibandingkan untuk memastikan model mirip "
               "alat sebenarnya. Kolom galat hanya ada di simulasi karena posisi benarnya "
               "diketahui. Uji dashboard tidak punya data CEP95 hardware.", 2)
        s.gap()
        w = sim["walk"]
        s.section(f"B. Berjalan – rute seperti rekaman V2, {sim['walk_speed']} m/s")
        s.table(["Besaran", "Simulasi", "Hardware", "Keterangan"], [
            ["RMSE posisi (cm)", w["rmse"] * 100, None, "hardware butuh posisi benar saat berjalan"],
            ["Cross-track RMSE (cm)", w["xt_rmse"] * 100, None, "jarak tegak lurus ke lintasan benar"],
            ["Cross-track P95 (cm)", w["xt_p95"] * 100, None, ""],
            ["Cross-track maks (cm)", w["xt_max"] * 100, None, ""],
            ["Laju update (Hz)", w["rate"], rate, "hardware lebih rendah: paket tag terlambat/hilang"],
        ], [None, "cm", "cm", None])
        s.section("Grafik")
        for name, cap in zip(sim["images"], ("Gambar 3.1 – Titik statis simulasi",
                                             "Gambar 3.2 – Lintasan simulasi vs hardware (V2)")):
            s.image(out_dir / name, 860, cap)

    # ============================== 4 Per Anchor ==============================
    s = Sheet(wb, "4 Per Anchor", [16, 12, 11, 10, 12, 13, 12, 13, 12, 13])
    s.title("4. Kualitas Tiap Anchor",
            "Residu = jarak terukur − (jarak 3D dari posisi rata-rata titik diam + offset jarak). Residu "
            "besar atau porsi outlier tinggi menandakan NLOS (terhalang) pada anchor itu.")
    rows = []
    for r in recs:
        for x in anchor_stats(r["meta"], r["est"], r["ranges"], r["stops"]):
            rows.append([r["name"], x["anchor"], x["n"], x["hz"], x["rx"], x["fp"], x["nres"],
                         x["rmean"] * 100, x["rstd"] * 100, x["outl"]])
    s.table(["Rekaman", "Anchor", "Sampel jarak", "Laju (Hz)", "RX power (dBm)",
             "First-path (dBm)", "Sampel saat diam", "Residu rata-rata (cm)", "Std residu (cm)",
             "Outlier > 30 cm"], rows, [None, None, "n", "hz", "db", "db", "n", "cm", "cm", "pct"])

    # ================================ 5 Grafik ================================
    s = Sheet(wb, "5 Grafik", [12] * 10)
    s.title("5. Grafik Pengukuran", "Lintasan hasil estimasi, sebaran posisi di tiap titik diam, "
            "dan posisi terhadap waktu (area oranye = troli diam).")
    k = 0
    for r in recs:
        s.section(r["name"])
        for img, cap in zip(r["imgs"], ("lintasan", "sebaran titik diam", "posisi terhadap waktu")):
            k += 1
            s.image(img, 760, f"Gambar 5.{k} – {r['name']}: {cap}")

    # ================================== Data ==================================
    s = Sheet(wb, "Data", [22, 10, 10, 10, 13, 9, 11, 12])
    rows = []
    for r in recs:
        for e, v in zip(r["est"], speed_series(r["est"])):
            rows.append([r["name"], e["s"], e["x"], e["y"], v, e["sigma"] * 100, e["b"] * 100,
                         e.get("used")])
    s.table(["Rekaman", "Waktu (s)", "X (m)", "Y (m)", "Kecepatan (m/s)", "Margin error ±σ (cm)",
             "Offset jarak b (cm)", "Sampel dipakai"], rows,
            [None, "0.00", "m3", "m3", "m", "cm", "cm", "n"])
    s.ws.freeze_panes = "A2"

    out = out_dir / fname
    wb.save(out)
    return out, recs


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--out", default="docs/laporan-akurasi")
    ap.add_argument("--name", default="Laporan_Akurasi_UWB.xlsx")
    a = ap.parse_args()
    path, recs = build(a.logs, Path(a.out), a.name)
    for r in recs:
        print(r["name"], "stops:", [f"({stop_stats(s)['mx']:.2f},{stop_stats(s)['my']:.2f}) "
                                    f"CEP95 {stop_stats(s)['cep95'] * 100:.1f}cm {s[-1]['s'] - s[0]['s']:.0f}s"
                                    for s in r["stops"]])
    print("wrote", path)
