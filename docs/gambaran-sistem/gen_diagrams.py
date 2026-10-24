"""Generate the draw.io diagrams for the "Gambaran Sistem" document.

    python docs/gambaran-sistem/gen_diagrams.py

Writes 1_sistem_saat_ini.drawio, 2_sistem_usulan.drawio and 3_alur_ekf.drawio next
to this script and renders each to PNG with the draw.io desktop CLI. Open the
.drawio files in draw.io to edit them.

The canvases are ~850 px wide on purpose: the figures are printed 14.6 cm
wide in the Word document, which keeps 16 px text at about 8 pt on paper.
"""
import subprocess
from pathlib import Path
from xml.sax.saxutils import escape

HERE = Path(__file__).resolve().parent
FONT = "fontFamily=Arial;"

# colours by role
OURS = "fillColor=#dae8fc;strokeColor=#6c8ebf;"       # this thesis
DATA = "fillColor=#d5e8d4;strokeColor=#82b366;"       # outputs / data
EXT = "fillColor=#f5f5f5;strokeColor=#999999;fontColor=#555555;"   # other theses
OLD = "fillColor=#ffffff;strokeColor=#666666;"         # existing system parts
PALE = "fillColor=#f7f9fc;strokeColor=#9aa9c2;"        # containers
DEC = "fillColor=#fff2cc;strokeColor=#d6b656;"         # decisions
RED = "#C62828"
SMALL = 14                                             # second-line font size


def small(text):
    return f'<font style="font-size:{SMALL}px">{text}</font>'


def q(label):
    return escape(label, {'"': "&quot;"})


class Diagram:
    def __init__(self, name, width, height):
        self.name, self.w, self.h = name, width, height
        self.cells = []
        self.n = 0

    def _id(self):
        self.n += 1
        return f"c{self.n}"

    def raw(self, x, y, w, h, label, style):
        cid = self._id()
        self.cells.append(
            f'<mxCell id="{cid}" value="{q(label)}" style="{style}" vertex="1" parent="1">'
            f'<mxGeometry x="{x}" y="{y}" width="{w}" height="{h}" as="geometry"/></mxCell>')
        return cid

    def box(self, x, y, w, h, label, style, fs=16):
        return self.raw(x, y, w, h, label,
                        f"rounded=1;arcSize=10;whiteSpace=wrap;html=1;{FONT}fontSize={fs};{style}")

    def decision(self, x, y, w, h, label):
        return self.raw(x, y, w, h, label, f"rhombus;whiteSpace=wrap;html=1;{FONT}fontSize=15;{DEC}")

    def circle(self, x, y, label="A"):
        return self.raw(x, y, 40, 40, label, f"ellipse;html=1;{FONT}fontSize=16;fontStyle=1;"
                        "fillColor=#ffffff;strokeColor=#4d4d4d;")

    def container(self, x, y, w, h, title):
        return self.raw(x, y, w, h, title,
                        f"swimlane;startSize=32;rounded=1;arcSize=3;html=1;{FONT}fontSize=17;"
                        f"fontStyle=1;{PALE}")

    def text(self, x, y, w, h, label, fs=15):
        return self.raw(x, y, w, h, label,
                        f"text;html=1;whiteSpace=wrap;align=left;verticalAlign=top;{FONT}fontSize={fs};")

    def edge(self, s, t, label="", style="", lx=None):
        """lx: label position along the edge, -1 (source) .. 1 (target)."""
        cid = self._id()
        st = (f"edgeStyle=orthogonalEdgeStyle;rounded=1;html=1;{FONT}fontSize=14;"
              f"labelBackgroundColor=#ffffff;strokeColor=#4d4d4d;endArrow=block;endFill=1;{style}")
        gx = f' x="{lx}"' if lx is not None else ""
        self.cells.append(
            f'<mxCell id="{cid}" value="{q(label)}" style="{st}" edge="1" parent="1" '
            f'source="{s}" target="{t}"><mxGeometry relative="1"{gx} as="geometry"/></mxCell>')
        return cid

    def note_link(self, s, t):
        """Dashed connector without arrowheads (annotation)."""
        return self.edge(s, t, "", "dashed=1;endArrow=none;strokeColor=#888888;")

    def change(self, x, y, w, h, num, pad=7):
        """Red dashed box around an area + numbered badge (letak perubahan)."""
        self.raw(x - pad, y - pad, w + 2 * pad, h + 2 * pad, "",
                 f"rounded=1;arcSize=8;dashed=1;dashPattern=6 4;strokeWidth=2.5;"
                 f"strokeColor={RED};fillColor=none;html=1;")
        self.raw(x - pad - 13, y - pad - 13, 30, 30, str(num),
                 f"ellipse;html=1;{FONT}fontSize=16;fontStyle=1;fillColor={RED};"
                 f"strokeColor={RED};fontColor=#FFFFFF;")

    def save(self):
        body = "".join(self.cells)
        xml = (f'<mxfile host="drawio"><diagram name="{self.name}" id="{self.name}">'
               f'<mxGraphModel dx="1200" dy="900" grid="0" gridSize="10" guides="1" '
               f'tooltips="1" connect="1" arrows="1" fold="1" page="0" pageScale="1" '
               f'pageWidth="{self.w}" pageHeight="{self.h}" math="0" shadow="0" '
               f'background="#FFFFFF"><root><mxCell id="0"/><mxCell id="1" parent="0"/>'
               f'{body}</root></mxGraphModel></diagram></mxfile>')
        path = HERE / f"{self.name}.drawio"
        path.write_text(xml, encoding="utf-8")
        return path


def market_area(d, x, y, w, h, tag_label, style, tw=170):
    """Room with four corner anchors and the trolley tag in the middle."""
    d.container(x, y, w, h, "Area Smart Market")
    aw, ah = 132, 52
    pos = [(x + 14, y + 46), (x + w - aw - 14, y + 46),
           (x + 14, y + h - ah - 14), (x + w - aw - 14, y + h - ah - 14)]
    anchors = [d.box(ax, ay, aw, ah, f"Anchor {i + 1}<br>{small('ESP32 + DW1000')}", style, fs=15)
               for i, (ax, ay) in enumerate(pos)]
    th = 70
    tx, ty = x + (w - tw) / 2, y + 32 + (h - 32 - th) / 2
    tag = d.box(tx, ty, tw, th, tag_label, style, fs=15)
    for a in anchors:
        d.edge(tag, a, "", "edgeStyle=none;dashed=1;dashPattern=4 3;endArrow=none;"
               "strokeColor=#6c8ebf;")
    return tag


# ---------------------------------------------------------------------------
# Gambar 1 — kondisi saat ini (belum ada sistem penentuan posisi)
# ---------------------------------------------------------------------------
def sistem_saat_ini():
    d = Diagram("1_sistem_saat_ini", 850, 660)
    d.container(10, 10, 400, 300, "Area Smart Market")
    shelf = "fillColor=#eeeeee;strokeColor=#999999;fontColor=#555555;"
    for x, name in ((45, "A"), (170, "B"), (295, "C")):
        d.box(x, 54, 80, 96, f"Rak<br>{name}", shelf, fs=15)
    d.box(30, 180, 200, 76,
          f"<b>Troli belanja biasa</b><br>{small('didorong pelanggan,<br>tanpa perangkat elektronik')}", OLD)
    staff = d.box(250, 180, 140, 76, "Papan petunjuk<br>&amp; petugas", OLD, fs=15)
    d.raw(30, 266, 360, 38,
          "<i>belum ada perangkat penentu posisi;<br>GPS tidak menjangkau dalam ruangan</i>",
          f"text;html=1;whiteSpace=wrap;align=center;verticalAlign=middle;{FONT}fontSize=14;"
          "fontColor=#555555;")

    pos = d.box(10, 350, 400, 84,
                f"<b>Posisi troli tidak diketahui</b><br>{small('tidak ada yang menghitung koordinat troli;<br>hanya terlihat langsung oleh orang di toko')}",
                OLD)
    need = d.box(10, 500, 400, 90,
                 f"<b>Navigasi &amp; aplikasi Smart Market</b><br>{small('rencana TA anggota tim lain; memerlukan<br>koordinat troli secara real-time')}",
                 EXT)
    d.edge(pos, need, "✕ belum dapat diwujudkan",
           f"dashed=1;strokeColor={RED};fontColor={RED};")

    d.container(450, 10, 390, 400, "Proses belanja saat ini")
    s1 = d.box(475, 56, 340, 56, "Pelanggan masuk dan<br>mengambil troli", OLD, fs=15)
    s2 = d.box(475, 140, 340, 80,
               f"<b>Mencari produk sendiri</b><br>{small('papan petunjuk, berkeliling lorong,<br>atau bertanya kepada petugas')}",
               OLD)
    s3 = d.box(475, 248, 340, 56, "Mengambil produk<br>ke dalam troli", OLD, fs=15)
    s4 = d.box(475, 332, 340, 56, "Membayar di kasir", OLD, fs=15)
    d.edge(s1, s2)
    d.edge(s2, s3)
    d.edge(s3, s4)
    d.note_link(staff, s2)

    d.box(450, 500, 390, 90,
          f"<b>Pengelola toko</b><br>{small('tidak ada peta maupun pemantauan<br>posisi troli')}", OLD)

    d.text(10, 612, 830, 40,
           f"<font color=\"{RED}\"><b>1–5</b></font> letak perubahan pada sistem yang diusulkan "
           "(Gambar 2, Tabel 1)     "
           "<font color=\"#999999\">■</font> rencana TA anggota tim lain", fs=14)

    d.change(10, 10, 400, 300, 1)
    d.change(30, 180, 200, 76, 2)
    d.change(10, 350, 400, 84, 3)
    d.change(475, 140, 340, 80, 4)
    d.change(450, 500, 390, 90, 5)
    return d.save()


# ---------------------------------------------------------------------------
# Gambar 2 — sistem yang diusulkan
# ---------------------------------------------------------------------------
def sistem_usulan():
    d = Diagram("2_sistem_usulan", 860, 850)
    tag = market_area(d, 10, 10, 390, 300,
                      f"<b>Smart Trolley</b><br>{small('tag UWB (ESP32 + DW1000)')}", OURS, tw=220)

    fw = d.box(10, 350, 390, 116,
               f"<b>Firmware tag &amp; anchor</b><br>{small('setiap sampel jarak mentah dikirim sekali<br>+ timestamp + nomor urut (±5 Hz)<br>via WiFi (REST/MQTT), OTA, diagnosa')}",
               OURS)
    d.edge(tag, fw, "", "exitX=0.5;exitY=1;entryX=0.5;entryY=0;strokeColor=#6c8ebf;")

    web = d.box(450, 10, 390, 100,
                f"<b>Web Dashboard</b><br>{small('peta 3D, posisi anchor &amp; tinggi tag,<br>kalibrasi, tuning, OTA, rekam &amp; evaluasi')}",
                OURS)

    d.container(440, 140, 410, 690, "Server Positioning (Python)")
    p1 = d.box(465, 190, 360, 74,
               f"<b>Ingest &amp; sinkron waktu</b><br>{small('setiap sampel dipakai sekali,<br>waktu device dipetakan ke server')}",
               OURS)
    p2 = d.box(465, 290, 360, 74,
               f"<b>Koreksi jarak</b><br>{small('kalibrasi per anchor +<br>bias bersama diestimasi online')}",
               OURS)
    p3 = d.box(465, 390, 360, 74,
               f"<b>Penyaring outlier &amp; NLOS</b><br>{small('jarak mustahil dibuang,<br>gating 6σ, update robust Huber')}",
               OURS)
    p4 = d.box(465, 490, 360, 94,
               f"<b>Extended Kalman Filter</b><br>{small('state [x, y, vx, vy, b]<br>model jarak miring 3D (beda tinggi)<br>model kecepatan Singer')}",
               OURS)
    p5 = d.box(465, 612, 360, 62,
               f"<b>Posisi troli</b><br>{small('x, y, σ, quality, valid')}", DATA)
    api = d.box(465, 706, 360, 92,
                f"<b>API posisi &amp; peta</b><br>{small('REST /api/v1/navigation/*<br>MQTT push 10 Hz')}",
                OURS)
    d.edge(web, p1, "", "dashed=1;exitX=0.93;exitY=1;entryX=0.93;entryY=0;")
    d.edge(fw, p1, "", "exitX=1;exitY=0.5;entryX=0;entryY=0.5;")
    d.edge(p1, p2)
    d.edge(p2, p3)
    d.edge(p3, p4)
    d.edge(p4, p5)
    d.edge(p5, api)

    app = d.box(10, 606, 390, 74,
                f"<b>Aplikasi Smart Market</b><br>{small('TA anggota tim lain · peta &amp; posisi troli')}",
                EXT)
    nav = d.box(10, 712, 390, 84,
                f"<b>Navigasi Otonom Smart Trolley</b><br>{small('TA anggota tim lain · menerima<br>x, y, quality, valid')}",
                EXT)
    d.edge(p5, app, "", "exitX=0;exitY=0.5;entryX=1;entryY=0.5;strokeColor=#999999;")
    d.edge(api, nav, "", "exitX=0;exitY=0.5;entryX=1;entryY=0.5;strokeColor=#999999;")

    d.text(10, 490, 400, 110,
           "<b>Keterangan</b><br>"
           "<font color=\"#3a5a8c\">■</font> bagian TA ini   "
           "<font color=\"#999999\">■</font> TA anggota tim lain<br>"
           f"<font color=\"{RED}\"><b>1–5</b></font> letak perubahan (sama dengan Gambar 1)<br>"
           "<font color=\"#888888\">- - -</font> konfigurasi", fs=14)

    d.change(10, 10, 390, 300, 1)            # anchors installed in the market
    d.change(95, 141, 220, 70, 2)            # tag on the trolley
    d.change(465, 190, 360, 484, 3, pad=9)   # position estimation
    d.change(465, 706, 360, 92, 4)           # position interface
    d.change(450, 10, 390, 100, 5)           # web dashboard
    return d.save()


# ---------------------------------------------------------------------------
# Gambar 3 — alur estimasi posisi
# ---------------------------------------------------------------------------
def alur_ekf():
    d = Diagram("3_alur_ekf", 950, 790)
    term = f"rounded=1;arcSize=50;{DATA}"
    yes, no = "fontColor=#2D7600;", "fontColor=#B20000;"

    # main column A
    start = d.box(230, 10, 210, 60, "<b>Paket telemetri tag</b><br>(±5 Hz)", term, fs=15)
    a_in = d.circle(70, 110)
    n1 = d.box(230, 100, 210, 60, "Ambil sampel jarak<br>berikutnya", OURS, fs=15)
    d1 = d.decision(220, 190, 230, 100, "Sampel baru &amp;<br>jarak wajar?")
    n2 = d.box(230, 320, 210, 80, "Konversi waktu device<br>ke server + koreksi<br>kalibrasi", OURS, fs=15)
    d2 = d.decision(220, 430, 230, 100, "Track tag<br>sudah ada?")
    b1 = d.box(10, 210, 150, 60, "Abaikan<br>sampel", OLD, fs=15)
    c1 = d.circle(65, 300)
    b2 = d.box(10, 448, 170, 64, f"Inisialisasi track<br>{small('(multilaterasi)')}", OURS, fs=15)
    c2 = d.circle(75, 545)

    # main column B
    n3 = d.box(510, 10, 220, 64, "Prediksi EKF ke waktu<br>sampel (model Singer)", OURS, fs=15)
    n4 = d.box(510, 100, 220, 64, "Hitung residu<br>r = z − h(x)", OURS, fs=15)
    d3 = d.decision(500, 190, 240, 100, "|r| di bawah<br>batas gate?")
    n5 = d.box(510, 320, 220, 64, "Update EKF robust<br>(bobot Huber)", OURS, fs=15)
    n6 = d.box(510, 410, 220, 56, "Batasi posisi<br>dalam ruangan", OURS, fs=15)
    n7 = d.box(510, 492, 220, 64, "<b>Kirim posisi</b><br>x, y, σ, quality", DATA, fs=15)
    d4 = d.decision(500, 582, 240, 100, "Masih ada<br>sampel?")
    end = d.box(510, 712, 220, 60, "<b>Selesai</b>, tunggu<br>paket berikutnya", term, fs=15)
    b3 = d.box(780, 200, 160, 84, f"Tolak sampel<br>{small('bila &gt;1,5 s semua<br>ditolak: inisialisasi<br>ulang')}",
               OLD, fs=15)
    c3 = d.circle(840, 310)
    c4 = d.circle(840, 612)

    d.edge(start, n1)
    d.edge(a_in, n1)
    d.edge(n1, d1)
    d.edge(d1, n2, "Ya", yes)
    d.edge(d1, b1, "Tidak", no)
    d.edge(b1, c1)
    d.edge(n2, d2)
    d.edge(d2, b2, "Tidak", no)
    d.edge(b2, c2)
    d.edge(d2, n3, "Ya", yes + "exitX=1;exitY=0.5;entryX=0;entryY=0.5;")
    d.edge(n3, n4)
    d.edge(n4, d3)
    d.edge(d3, n5, "Ya", yes)
    d.edge(d3, b3, "Tidak", no)
    d.edge(b3, c3)
    d.edge(n5, n6)
    d.edge(n6, n7)
    d.edge(n7, d4)
    d.edge(d4, c4, "Ya", yes)
    d.edge(d4, end, "Tidak", no)
    d.text(10, 640, 420, 110,
           "<b>A</b> = kembali ke \"Ambil sampel jarak berikutnya\"<br>"
           "h(x) = √((x−x<sub>a</sub>)² + (y−y<sub>a</sub>)² + Δz²) + b<br>"
           "z = jarak terukur, b = bias jarak bersama,<br>Δz = beda tinggi anchor dan tag", fs=14)
    return d.save()


# ---------------------------------------------------------------------------
# docs/img/alur_algoritma — the full algorithm of server/tracker.py
# ---------------------------------------------------------------------------
def alur_algoritma():
    d = Diagram("../img/alur_algoritma", 1080, 1480)
    term = f"rounded=1;arcSize=50;{DATA}"
    opt = f"{OURS}dashed=1;dashPattern=6 4;"
    yes, no = "fontColor=#2D7600;", "fontColor=#B20000;"

    # --- column A: one range sample ---
    start = d.box(210, 10, 260, 60, f"<b>Paket telemetri tag</b><br>{small('(±3–5 paket/detik)')}", term, fs=15)
    a_in = d.circle(60, 110)
    take = d.box(210, 100, 260, 60, "Ambil sampel jarak<br>berikutnya", OURS, fs=15)
    d1 = d.decision(200, 190, 280, 110, "Sampel baru &amp;<br>jarak masuk akal?")
    skip = d.box(10, 215, 140, 60, "Abaikan<br>sampel", OLD, fs=15)
    c1 = d.circle(60, 300)
    tconv = d.box(210, 330, 260, 64, "Waktu device → server", OURS, fs=15)
    cal = d.box(210, 425, 260, 92, f"<b>Koreksi kalibrasi</b><br>{small('jarak − offset anchor dari kalibrasi<br>statis; belum dikalibrasi → offset<br>dipelajari EKF')}", OURS, fs=15)
    d2 = d.decision(200, 545, 280, 110, "Track tag<br>sudah ada?")
    init = d.box(10, 690, 270, 100, f"Inisialisasi<br>{small('median sampel 0,7 s tiap anchor<br>→ multilaterasi; tanpa kalibrasi,<br>offset jarak b ikut dicari')}", OURS, fs=15)
    c2 = d.circle(125, 822)

    # --- column B: EKF ---
    pred = d.box(580, 10, 240, 64, f"Prediksi EKF ke waktu sampel<br>{small('mode diam: kecepatan = 0')}", OURS, fs=15)
    d.text(830, 6, 245, 80,
           "<b>State EKF</b>: x, y, vx, vy<br>(+ offset jarak b dan offset<br>tiap anchor d₁…d₄ bila belum<br>dikalibrasi)", fs=13)
    res = d.box(580, 100, 240, 64, "Hitung residu ternormalisasi<br>e = (z − h(x)) / √S", OURS, fs=15)
    d3 = d.decision(570, 190, 260, 110, "|e| ≤ 6 ?")
    rej = d.box(870, 196, 180, 84, f"Tolak sampel<br>{small('1,5 s semua ditolak →<br>inisialisasi ulang')}", OLD, fs=15)
    c3 = d.circle(940, 290)
    dl = d.decision(560, 330, 280, 115, "Mode diam &amp; e &gt; 1 ?<br>(jarak kepanjangan)")
    drop = d.box(870, 352, 190, 86, f"<b>Buang</b> sampel<br>{small('badan menghalangi; hanya<br>bila ≥ 3 anchor lain ada')}", OLD, fs=15)
    c5 = d.circle(940, 462)
    upd = d.box(580, 480, 240, 64, f"Update EKF robust<br>{small('bobot turun bila e &gt; 1 atau e &lt; −2,5')}", OURS, fs=15)
    zupt = d.box(580, 572, 240, 84, f"Perbarui <b>mode diam</b><br>{small('masuk: v &lt; 0,2 m/s selama 0,4 s<br>keluar: 3 dari 4 |e| &gt; 1,5')}", OURS, fs=15)
    clamp = d.box(580, 684, 240, 56, "Batasi posisi<br>dalam ruangan", OURS, fs=15)
    d4 = d.decision(570, 765, 260, 110, "Masih ada sampel<br>di paket ini?")
    c4 = d.circle(940, 800)

    # --- output, once per packet ---
    d5 = d.decision(560, 905, 280, 120, "≥ 3 anchor diterima<br>dalam 1,5 s &amp; σ ≤ 0,5 m?")
    hold = d.box(880, 923, 180, 84, f"<b>Tahan</b> posisi valid<br>terakhir<br>{small('valid = false')}", OLD, fs=15)
    d6 = d.decision(570, 1060, 260, 110, "Haluskan jalur?<br>(centang di Setup)")
    smooth = d.box(290, 1073, 230, 84, f"<b>Rata-rata</b><br>{small('3 posisi terakhir')}", OURS, fs=15)
    pas = d.box(580, 1200, 240, 64, "Pakai keluaran EKF<br>langsung", OURS, fs=15)
    out = d.box(580, 1290, 240, 70, f"<b>Kirim posisi</b><br>{small('x, y, σ, valid → navigasi &amp; dashboard')}", DATA, fs=15)
    trail = d.box(860, 1385, 210, 84, f"Dashboard: jejak digambar<br>{small('rata-rata ±3 titik<br>(sebelum &amp; sesudah)')}", opt, fs=14)
    end = d.box(580, 1395, 240, 60, "<b>Selesai</b>, tunggu<br>paket berikutnya", term, fs=15)

    d.edge(start, take)
    d.edge(a_in, take)
    d.edge(take, d1)
    d.edge(d1, tconv, "Ya", yes)
    d.edge(d1, skip, "Tidak", no)
    d.edge(skip, c1)
    d.edge(tconv, cal)
    d.edge(cal, d2)
    d.edge(d2, init, "Tidak", no + "exitX=0;exitY=0.5;entryX=0.5;entryY=0;")
    d.edge(init, c2)
    d.edge(d2, pred, "Ya", yes + "exitX=1;exitY=0.5;entryX=0;entryY=0.5;")
    d.edge(pred, res)
    d.edge(res, d3)
    d.edge(d3, dl, "Ya", yes)
    d.edge(d3, rej, "Tidak", no)
    d.edge(rej, c3)
    d.edge(dl, drop, "Ya", yes)
    d.edge(dl, upd, "Tidak", no)
    d.edge(drop, c5)
    d.edge(upd, zupt)
    d.edge(zupt, clamp)
    d.edge(clamp, d4)
    d.edge(d4, c4, "Ya", yes)
    d.edge(d4, d5, "Tidak", no)
    d.edge(d5, d6, "Ya", yes)
    d.edge(d5, hold, "Tidak", no)
    d.edge(d6, smooth, "Ya", yes)
    d.edge(d6, pas, "Tidak", no)
    d.edge(pas, out)
    d.edge(smooth, out, "", "exitX=0.5;exitY=1;entryX=0;entryY=0.5;")
    d.edge(hold, out, "", "exitX=0.5;exitY=1;entryX=1;entryY=0.5;")
    d.edge(out, end)
    d.edge(out, trail, "", "dashed=1;exitX=1;exitY=0.85;entryX=0.5;entryY=0;")
    d.text(10, 940, 270, 190,
           "<b>A</b> = kembali ke \"Ambil sampel jarak berikutnya\"<br>"
           "kotak putus-putus = hanya tampilan<br>"
           "z = jarak terukur, h(x) = jarak dari posisi<br>estimasi ke anchor (+ offset b + d bila<br>belum dikalibrasi)<br>"
           "S = varians residu yang diharapkan,<br>e = residu ternormalisasi<br>"
           "(e &gt; 0: jarak lebih panjang dari dugaan)<br>"
           "mode diam = zero-velocity update", fs=13)
    return d.save()


def export_png(path, scale=2):
    exe = r"C:\Program Files\draw.io\draw.io.exe"
    out = path.with_suffix(".png")
    subprocess.run([exe, "-x", "-f", "png", "-s", str(scale), "-b", "10",
                    "-o", str(out), str(path)], check=True,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return out


if __name__ == "__main__":
    for fn in (sistem_saat_ini, sistem_usulan, alur_ekf, alur_algoritma):
        p = fn()
        print("wrote", p.name, "->", export_png(p).name)
