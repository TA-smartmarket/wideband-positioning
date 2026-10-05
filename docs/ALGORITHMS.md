# Algoritma Sistem — Penjelasan Mendalam

Dokumen ini menjelaskan **algoritma inti** yang dipakai di proyek ini: apa
masalahnya, bagaimana matematikanya, kenapa dipilih, dan di file/baris mana
implementasinya. Semua rumus di bawah diambil dari kode yang benar-benar jalan,
bukan dari rencana.

Fokus dokumen ini adalah algoritma yang **menentukan akurasi posisi**. Algoritma
pendukung (hash config, anti-downgrade OTA, penempatan anchor otomatis, perataan
animasi UI) tidak dibahas di sini — bukan karena tidak penting, tapi karena
tidak mempengaruhi hasil pengukuran; penjelasannya ada di komentar kode.

Konvensi: koordinat ruang 2D `(x, y)` dalam meter, `z` untuk tinggi.
Anchor = pemancar tetap dengan posisi diketahui; tag = perangkat yang dicari.

## Daftar isi

1. [Ikhtisar alur data](#1-ikhtisar-alur-data)
2. [Two-Way Ranging (TWR) — pengukuran jarak](#2-two-way-ranging-twr)
3. [Pre-filter jarak: gate lompatan + median-3 + EMA](#3-pre-filter-jarak)
4. [Konversi jarak 3D → horizontal](#4-konversi-jarak-3d--horizontal)
5. [Solver posisi (multilaterasi)](#5-solver-posisi-multilaterasi)
6. [Extended Kalman Filter (EKF)](#6-extended-kalman-filter-ekf)
7. [Klasifikasi LOS/NLOS, sigma & bias](#7-klasifikasi-losnlos-sigma--bias)
8. [Uji ketidaksamaan segitiga (geometry check)](#8-uji-ketidaksamaan-segitiga)
9. [Peta algoritma → file](#9-peta-algoritma--file)
10. [Parameter & penyetelan](#10-parameter--penyetelan)

---

## 1. Ikhtisar alur data

```
                 DW1000 (radio UWB)
                        │  TWR                      §2
                        ▼
             jarak mentah (3D, meter)
                        │
        ┌───────────────┴───────────────┐
        │  pre-filter: gate+median+EMA  │             §3
        └───────────────┬───────────────┘
                        ▼
              konversi 3D → horizontal                 §4
                        │
        ┌───────────────┴───────────────┐
        │  solver: 2 anchor → irisan     │             §5
        │  lingkaran; 3+ → least squares │
        └───────────────┬───────────────┘
                        ▼  (hanya untuk fix pertama)
        ┌───────────────┴───────────────┐
        │   EKF: predict + update/range  │             §6
        │   + innovation gate 3σ         │
        └───────────────┬───────────────┘
                        ▼
          posisi (x, y), kecepatan, sigma
                        │
        ┌───────────────┴───────────────┐
        │  clamp ruangan + uji segitiga  │             §8
        └───────────────┬───────────────┘
                        ▼
                 server → UI 3D
```

Ada **dua jalur paralel** yang menghitung hal yang sama:

| | Firmware (`src/`) | Server (`server/`) |
|---|---|---|
| Solver | `src/solver.h` | `app.py::solve_2d` |
| EKF | `src/ekf.h` | `app.py::TagEKF` |
| Geometry | `geometrySlack()` | `geometry_check()` |

Keduanya sengaja dibuat **kembar** supaya tag bisa menghitung posisinya sendiri
tanpa server, dan server tetap bisa menghitung ulang dari data mentah. Rumus di
kedua sisi harus sama; kalau salah satu diubah, yang lain wajib ikut.

---

## 2. Two-Way Ranging (TWR)

**Masalah.** Mengukur jarak dari selisih waktu tempuh radio (ToF). Tapi jam
anchor dan tag tidak tersinkronisasi, jadi selisih satu arah tidak bisa dipakai.

**Solusi.** TWR mengirim bolak-balik sehingga **empat timestamp** didapat, dan
pergeseran jam saling meniadakan:

```
Tag  ──── poll (t1) ────────────────►  Anchor (t2)
Tag  ◄─── response (t3) ─────────────  Anchor
Tag  ──── final (t5) ───────────────►  Anchor (t6)

ToF = ((t4−t1) − (t3−t2) + (t6−t3) − (t5−t4)) / 4
jarak = c · ToF          c = 299 702 547 m/s
```

**Kenapa bukan ToA/TDoA?** ToA butuh sinkronisasi jam presisi antar semua node
(1 ns error ≈ 30 cm). TDoA hanya butuh sinkronisasi antar anchor. Keduanya
menambah kompleksitas infrastruktur. TWR tidak butuh sinkronisasi sama sekali —
cocok untuk node WiFi sederhana seperti ESP32 ini. Rujukan: Bregar 2023
(ADS-TWR), Kramarić 2025 (DS-TWR).

**Kode.** Implementasi ada di library `lib/DW1000/` (`DW1000Ranging.cpp`),
dipanggil dari `src/main.cpp::newRange()`. Nilai yang diterima firmware:

```cpp
struct RangeRec {          // src/main.cpp:61
    char  src[16], dst[16];
    float range;           // meter
    float rx;              // daya terima  (dBm) — indikator NLOS
    float fp;              // daya first-path (dBm) — indikator NLOS
    float quality;
    uint32_t ts;
};
```

Jarak yang diukur adalah **slant range 3D**, bukan jarak horizontal — itu
penting untuk §4.

---

## 3. Pre-filter jarak

**Masalah.** Satu sampel TWR bisa meleset jauh (multipath, tabrakan paket).
Kalau sampel itu langsung masuk EKF, estimasi tersentak.

**Algoritma.** Tiga tahap berurutan, di `src/main.cpp::rfUpdate()` (baris 129).

### 3a. Gate lompatan (outlier gate)

```cpp
#define MAX_JUMP_M 5.0f
if (f.seeded && fabsf(raw - f.out) > MAX_JUMP_M) return f.out;
```

Manusia tidak mungkin berpindah 5 m antar sampel. Kalau lompatannya lebih
besar, sampel dibuang dan nilai terakhir dipakai. Ini menangkap spike tunggal
tanpa perlu statistik.

### 3b. Median-3

```cpp
#define RANGE_HIST 3
// gabung 3 sampel historis + sampel baru -> urutkan -> ambil tengah
const float med = s[n / 2];
```

Median dipilih, bukan rata-rata, karena **median tahan terhadap satu nilai
ekstrem** (breakdown point 50%), sedangkan rata-rata bisa ditarik jauh oleh
satu sampel liar. Pengurutan memakai insertion sort — untuk `n ≤ 4` itu
algoritma tercepat dan tidak butuh alokasi.

### 3c. EMA (Exponential Moving Average)

```cpp
#define EMA_ALPHA 0.35f
f.out = f.seeded ? (EMA_ALPHA * med + (1.0f - EMA_ALPHA) * f.out) : med;
```

```
y[k] = α·x[k] + (1−α)·y[k−1]        α = 0,35
```

EMA memberi bobot lebih besar ke sampel baru (α = 0,35 berarti ~65% bobot
masih dari riwayat). Efeknya: responsif terhadap gerakan nyata, tapi tetap
menghaluskan derau. Ukuran memori O(1) — hanya satu float per link.

**State per link.** Tiap pasangan `src>dst` punya filter sendiri (`RangeFilter`
dengan `key`), jadi anchor-2→tag-1 dan anchor-3→tag-1 tidak saling
mempengaruhi. Saat peer hilang, `filterForget()` mereset state supaya tidak
memakai riwayat basi.

**Bisa dimatikan** lewat `cfg.range_filter` (menu serial `filter on|off`).

---

## 4. Konversi jarak 3D → horizontal

**Masalah.** Anchor dipasang di langit-langit (`z = 2,2 m`), tag dipegang
setinggi pinggang (`z = 0,9 m`). TWR mengukur jarak **miring 3D**, sedangkan
EKF bekerja di bidang 2D `(x, y)`.

Kalau selisih tinggi ini diabaikan, ia menjadi galat sistematis yang
membengkakkan semua jarak.

**Solusi.** Proyeksikan secara eksak (`server/scene.py::horizontal_range`):

```
r_horizontal = √(r_3D² − Δz²)          Δz = z_anchor − z_tag
```

**Contoh nyata dari proyek ini.** `r_3D = 2,697 m`, `Δz = 1,3 m`:

```
r_h = √(2,697² − 1,3²) = √(7,274 − 1,690) = √5,584 = 2,363 m
```

Selisih **33 cm** — jauh lebih besar dari akurasi UWB sendiri (±10 cm). Tanpa
konversi ini, sistem akan tampak "tidak akurat" padahal radionya benar.

**Kasus degenerasi.** Kalau `r_3D < Δz` (secara fisik mustahil — tag lebih
dekat dari selisih tinggi), akar akan negatif. Kode mengembalikan
`max(r_3D · 0,1 ; 0,01)` alih-alih `NaN`, supaya satu pengukuran rusak tidak
meracuni seluruh solver:

```python
if r2 <= 0:
    return max(range3d * 0.1, 0.01)   # degenerate geometry
```

---

## 5. Solver posisi (multilaterasi)

**Masalah.** Diberikan `N` pasangan (posisi anchor, jarak), cari `(x, y)` tag.
Secara matematis ini **irisan lingkaran** (2D): tiap anchor memberi satu
lingkaran.

**Implementasi berbeda menurut jumlah anchor** — `src/solver.h::solvePosition()`
dan `server/app.py::solve_2d()`.

### 5a. Kasus 2 anchor — irisan dua lingkaran (closed form)

Dua lingkaran bisa berpotongan di **0, 1, atau 2 titik**. Solusinya eksak:

```
d  = |A₂ − A₁|                       jarak antar anchor
a  = (r₁² − r₂² + d²) / (2d)         proyeksi titik tengah
h  = √(r₁² − a²)                     setengah tali busur
P  = A₁ + a·(A₂−A₁)/d                titik tengah tali busur
C₁ = P + h·perp(A₂−A₁)/d             kandidat 1
C₂ = P − h·perp(A₂−A₁)/d             kandidat 2  ← cermin
```

**Ini sumber ambiguitas klasik: "flip ambiguity"** — kedua kandidat sama
validnya secara matematis. Kode menyelesaikannya dengan **uji batas ruangan**:

```python
inside = [pt for pt in c if 0 <= pt[0] <= room_w and 0 <= pt[1] <= room_h]
if len(inside) == 1:      # hanya satu kandidat di dalam ruangan
    return itu, ambiguous=False
```

Kalau **keduanya** di dalam ruangan (atau ruangan tidak diketahui), tidak ada
informasi untuk memilih — kode mengembalikan kandidat yang **terdekat ke
pusat ruangan** dan menandai `ambiguous = True`. Bendera ini diteruskan ke UI
supaya pengguna tahu posisi itu tidak bisa dipercaya.

**Kalau lingkaran tidak berpotongan** (`d > r₁+r₂` atau `d < |r₁−r₂|`), tidak
ada solusi. Kode mengembalikan titik tengah kedua anchor dengan `confidence =
0,1` dan `ambiguous = True` — jujur menyatakan "tidak tahu" alih-alih
mengarang angka. Kasus inilah yang terjadi di proyek ini saat dua anchor
saling berhadapan dan jaraknya tidak konsisten (lihat §8).

Rujukan: Park 2020 membahas flip ambiguity ini secara formal untuk multilaterasi
UWB.

### 5b. Kasus 3+ anchor — linearised least squares + Gauss-Newton

Dengan ≥3 anchor sistemnya *overdetermined* (lebih banyak persamaan daripada
variabel), jadi dicari solusi kuadrat-terkecil.

Persamaan aslinya non-linear:

```
(x − xᵢ)² + (y − yᵢ)² = rᵢ²
```

**Trik linearisasi:** kurangkan persamaan ke-0 dari persamaan ke-i, sehingga
suku `x²` dan `y²` saling menghapus dan tersisa persamaan **linear**:

```
2(xᵢ−x₀)x + 2(yᵢ−y₀)y = (rᵢ²−r₀²) − xᵢ² + x₀² − yᵢ² + y₀²
```

Disusun sebagai normal equations `A·p = B` dan diselesaikan dengan aturan
Cramer:

```
det = A₀·C − A₁²
px  = (B₀·C − B₁·A₁) / det
py  = (A₀·B₁ − A₁·B₀) / det
```

**Kenapa ada Gauss-Newton?** Linearisasi di atas mengubah masalah, jadi
solusinya sedikit menyimpang dari residual non-linear yang sebenarnya. Dua
iterasi Gauss-Newton memperbaikinya dengan menyelesaikan ulang pada residual
asli:

```
res = ‖p − aᵢ‖ − rᵢ                      residual sesungguhnya
J   = Σ (∂res/∂p)²                       matriks normal 2×2
p  -= J⁻¹ · Σ (∂res/∂p)·res              langkah Newton
```

**Guard.** Kalau `|det| < 1e-9` (anchor kolinear / berimpit) solver menyerah
dan mengembalikan `None` — geometri seperti itu memang tidak punya solusi
tunggal, jadi tidak ada gunanya memaksakan angka.

**Confidence** dihitung dari RMS residual: `conf = clamp(1 − rms, 0, 1)`, jadi
RMS 1 m berarti confidence 0.

---

## 6. Extended Kalman Filter (EKF)

**Masalah.** Solver di §5 bekerja per-kerangka dan tidak punya memori. Hasilnya
berisik dan bisa melompat. Yang kita butuhkan: estimasi yang **memanfaatkan
riwayat** dan memberi **ketidakpastian**.

**Model state.** Vektor 4 dimensi:

```
x = [ px, py, vx, vy ]ᵀ
```

**Model gerak:** kecepatan konstan + percepatan sebagai derau putih:

```
x[k+1] = F·x[k] + w,     F = [ 1 0 dt 0 ]
                             [ 0 1 0 dt ]
                             [ 0 0 1  0 ]
                             [ 0 0 0  1 ]
```

**Model pengukuran** (jarak ke anchor ke-i) — **non-linear**:

```
hᵢ(x) = √((px − axᵢ)² + (py − ayᵢ)²)
```

Karena `h` non-linear, filter ini "extended": fungsi pengukuran **dilinearisasi**
di sekitar estimasi sekarang lewat Jacobian-nya:

```
Hᵢ = [ (px−axᵢ)/d , (py−ayᵢ)/d , 0 , 0 ]        d = ‖p − aᵢ‖
```

### 6a. Dua tahap tiap siklus

**Prediksi** (`ekfPredict`, `src/ekf.h:95`):

```
x ← F·x
P ← F·P·Fᵀ + Q
```

dengan `Q` dari diskretisasi kecepatan konstan:

```
Q = σa² · [ dt⁴/4   0     dt³/2   0    ]
          [  0    dt⁴/4    0    dt³/2 ]
          [ dt³/2   0     dt²     0    ]
          [  0    dt³/2    0     dt²   ]
```

**Update** (`ekfUpdateRange`, `src/ekf.h:151`), per anchor satu per satu:

```
y  = z − h(x)                  inovasi (selisih pengukuran vs prediksi)
S  = H·P·Hᵀ + R                kovarians inovasi (skalar)
K  = P·Hᵀ / S                  Kalman gain
x ← x + K·y                    koreksi state
P ← (I − K·H)·P                koreksi kovarians
```

**Kenapa satu anchor per update, bukan sekaligus?** Setiap jarak adalah
pengukuran **skalar** yang independen. Memprosesnya berurutan menghindari
membangun matriks inovasi N×N, menghemat RAM (penting di ESP32) dan lebih
stabil secara numerik. Secara matematis hasilnya sama dengan update batch
untuk pengukuran yang tidak berkorelasi.

### 6b. Innovation gating — penolakan outlier

**Masalah.** Pengukuran NLOS bisa meleset 1–3 m. Kalau diterima, EKF akan
tertarik ke arah yang salah. Diperlukan cara otomatis untuk **menolak**
pengukuran sebelum merusak estimasi.

**Algoritma.** Uji chi-square 1 derajat kebebasan (versi praktis 3-sigma):

```
y     = z − h(x)                  inovasi
S     = H·P·Hᵀ + R                kovarians inovasi
tolak jika |y| > gate_sigma · √S
```

```cpp
const float gate = gate_sigma * sqrtf(S);
if (fabsf(innov) > gate) return false;      // ditolak
```

Logikanya: `S` adalah **varians yang diharapkan** dari inovasi. Kalau inovasi
jauh lebih besar dari yang diperkirakan filter sendiri, pengukuran itu tidak
konsisten dengan model — kemungkinan besar pantulan, bukan jalur langsung.

**Sifat penting: ambang ini adaptif.** Saat filter baru mulai (`P` besar),
gerbangnya lebar sehingga pengukuran pertama mudah diterima. Setelah filter
yakin (`P` kecil), gerbangnya menyempit sehingga outlier mudah tertangkap.
Tidak ada ambang tetap yang perlu disetel manual.

**Kembar di server** punya tambahan: `sigma_r` bisa **di-override per
pengukuran** (`update_range(..., sigma_r=None)`) supaya jalur NLOS langsung
mendapat sigma lebih besar — gerbangnya otomatis lebih longgar untuk jalur yang
memang berisik, alih-alih menolaknya mentah-mentah. Lihat §7.

### 6c. Pembatasan kovarians

```cpp
const float cap[EKF_N] = {25.0f, 25.0f, 9.0f, 9.0f};
```

Kalau pengukuran terus-menerus ditolak (NLOS berat), `P` tumbuh tanpa batas,
`sigma` menjadi tak bermakna, dan `confidence` ambruk — pernah teramati 11 m.
Cap membuat angkanya tetap bisa diinterpretasikan. `dt` juga dibatasi 2 detik:
kalau sistem sempat macet (OTA, reboot), lompatan waktu besar akan membuat
prediksi meleset jauh.

### 6d. Self-healing

**Masalah.** Kalau NLOS sangat parah (misalnya ada orang berdiri tepat di
antara anchor dan tag), **semua** pengukuran bisa ditolak. Filter lalu hanya
"coasting" — memprediksi dari kecepatan yang makin usang — dan tidak pernah
kembali ke kenyataan.

**Algoritma.** Hitung siklus berturut-turut tanpa pengukuran yang diterima;
setelah **8 siklus** (≈1,6 detik pada `update_ms = 200`), restart filter dari
solver geometris:

```python
if used == 0:
    EKF_REJECTS[tag] = EKF_REJECTS.get(tag, 0) + 1
    if EKF_REJECTS[tag] >= 8:
        sol = solve_2d(fixes, ...)          # mulai ulang dari geometri
        if sol:
            EKF[tag] = TagEKF(sol["x"], sol["y"])
            EKF_REJECTS[tag] = 0
```

**Kenapa restart, bukan memperlebar gerbang?** Memperlebar gerbang akan
membuat filter menerima data buruk — persis yang ingin dihindari. Restart dari
solver geometris (yang tidak punya memori, jadi tidak bisa "tersesat")
memberikan titik awal bersih tanpa mengorbankan selektivitas.

### 6e. Clamp batas ruangan

**Masalah.** Dengan hanya dua anchor, satu pasang jarak yang tidak konsisten
bisa mendorong estimasi sangat jauh — pernah teramati **x = −38 m**.

**Algoritma.** Batasi posisi ke persegi ruangan dengan margin 0,5 m:

```python
margin = 0.5
cx = min(max(px, -margin), room["width"] + margin)
cy = min(max(py, -margin), room["depth"] + margin)
```

**Kalau clamping terjadi**, kecepatan juga diredam dan kovarians dinaikkan:

```python
EKF[tag].x[2] *= 0.2         # vx
EKF[tag].x[3] *= 0.2         # vy
EKF[tag].P[0][0] = max(P[0][0], 0.5)    # nyatakan ketidakpastian
EKF[tag].P[1][1] = max(P[1][1], 0.5)
```

Ini penting: kalau hanya posisinya yang dijepit tapi kecepatannya dibiarkan,
filter akan terus "mendorong" ke luar tembok dan melawan clamp setiap siklus.
Meredam kecepatan + menaikkan kovarians memberi tahu filter bahwa ia baru saja
dipaksa, sehingga update berikutnya lebih berpengaruh.

**Kembar di server.** `server/app.py::TagEKF` (baris 326) mengimplementasikan
matematika yang sama. Ini bukan duplikasi yang tidak disengaja: server perlu
bisa menghitung ulang dari data mentah untuk verifikasi, dan tag perlu bisa
mandiri saat WiFi putus.

Rujukan: Yao 2021 (EKF mengungguli trilaterasi & least-squares saat data
mengandung outlier), Fan 2022 (Kalman + uji statistik untuk NLOS).

---

## 7. Klasifikasi LOS/NLOS, sigma & bias

**Masalah.** Untuk menghitung pengaruh penghalang, perlu tahu apakah ruas garis
anchor→tag **menembus** sebuah kotak (dinding/lemari) atau tidak.

### 7a. Slab method — uji ruas garis vs kotak berorientasi

Algoritma *slab method*: uji perpotongan ruas garis 3D dengan kotak berorientasi
(oriented bounding box). Namanya "slab" karena tiap dimensi dipandang sebagai
sepasang bidang paralel.

```
1. Transformasi p0, p1 ke kerangka lokal kotak (geser + rotasi −θ)
2. Untuk tiap sumbu i ∈ {x, y, z}:
       t1 = (−halfᵢ − aᵢ) / dᵢ
       t2 = ( halfᵢ − aᵢ) / dᵢ
       tmin = max(tmin, min(t1,t2))
       tmax = min(tmax, max(t1,t2))
       jika tmin > tmax → tidak berpotongan
3. Berpotongan jika tmin ≤ tmax
```

**Detail penting:**

- **Rotasi dibalik** (`cos(-rot)`) karena kita memindahkan *garis* ke kerangka
  kotak, bukan memutar kotak ke kerangka dunia. Ini menghindari alokasi matriks.
- **Kasus `|dᵢ| < 1e-9`** (garis sejajar bidang slab) ditangani terpisah:
  berpotongan hanya jika `aᵢ` sudah berada di dalam batas.
- **`tmin` dimulai dari 0 dan `tmax` dari 1**, bukan ±∞, karena kita hanya
  peduli pada segmen `p0→p1`, bukan garis tak hingga.

**Kenapa bukan bounding-sphere atau AABB saja?** Penghalang di UI bisa
**diputar** (`rot`), jadi AABB (axis-aligned) akan memberi hasil salah untuk
kotak yang dimiringkan. Slab method di kerangka lokal menangani rotasi dengan
benar dan tetap murah: O(1) per kotak, tanpa alokasi memori.

**Transparansi:** kotak dengan `atten ≤ 0` dilewati — dipakai untuk penghalang
yang sengaja ditandai tidak menghalangi.

### 7b. Sigma pengukuran membengkak

```python
def measurement_sigma(blockers, base_sigma, nlos_factor=8.0):
    if not blockers: return base_sigma
    atten = max(ob.get("atten", 1.0) for ob in blockers)
    return base_sigma * (1.0 + (nlos_factor - 1.0) * min(atten, 1.0))
```

`base_sigma = 0,15 m` (LOS), dan bisa naik sampai **8×** (1,2 m) saat terhalang
penuh. Nilai ini masuk ke `S = H·P·Hᵀ + R` di EKF, sehingga:

- Gerbang §6b otomatis **melonggar** untuk jalur itu (tidak langsung ditolak).
- Kalman gain mengecil, jadi filter **kurang memercayainya**.

Ini jauh lebih baik daripada sekadar membuang pengukuran NLOS: informasinya
tetap terpakai, hanya dengan bobot yang jujur.

**Penghalang terkuat yang menentukan** (`max(atten)`), bukan rata-rata — satu
dinding beton lebih menentukan daripada tiga tirai tipis.

### 7c. Bias positif

```python
def nlos_bias(blockers, bias_m=0.35):
    if not blockers: return 0.0
    return bias_m * min(atten, 1.0)
```

Nilai ini **ditambahkan** ke jarak sebelum masuk EKF:

```python
EKF[tag].update_range(ax, ay, rng2d + bias, sigma_r=sigma)
```

**Dasarnya fisika:** sinyal UWB NLOS tiba **lebih lambat** karena menempuh
jalur pantulan yang lebih panjang, jadi jarak terbaca **terlalu panjang**.
Mengoreksinya ke arah sebaliknya (mengurangi) akan salah arah; menambah bias
positif menggeser pengukuran kembali ke arah yang benar. Rujukan: Angarano 2021,
Yang 2024, Shalihan 2022.

---

## 8. Uji ketidaksamaan segitiga

**Masalah.** Bagaimana mengetahui bahwa **pasangan** jarak itu mustahil,
sebelum mempercayainya? Ini bukan soal derau — ini soal fisika.

**Algoritma.** Untuk setiap pasangan anchor, ketidaksamaan segitiga harus
berlaku:

```
|r₁ − r₂| ≤ d ≤ r₁ + r₂          d = jarak antar anchor
```

Pelanggaran berarti **minimal satu** dari kedua jarak itu salah. Karena UWB
NLOS selalu membuat jarak **lebih panjang** (sinyal menempuh jalur pantulan),
pelanggaran batas bawah (`r₁ + r₂ < d`) adalah tanda NLOS klasik.

```python
if r1 + r2 < d:
    worst = max(worst, d - (r1 + r2))
elif abs(r1 - r2) > d:
    worst = max(worst, abs(r1 - r2) - d)
return worst <= 0.0, worst        # (ok, slack)
```

`slack` = seberapa jauh pelanggarannya, dalam meter. Ini **bukan** untuk
membuang data (EKF yang memutuskan lewat gerbang §6b) — ini **indikator
diagnostik** yang ditampilkan di UI sebagai `geometry_ok` / `geometry_slack`.

**Kasus nyata yang terdeteksi di proyek ini:**

```
r(anchor-2) = 2,697 m      r(anchor-3) = 2,202 m
d = 5,000 m
r₁ + r₂ = 4,899 m  <  5,000 m     → pelanggaran 0,101 m
```

Setelah proyeksi horizontal (§4): `slack = 0,860 m` — dan nilai itulah yang
dilaporkan server. Artinya kedua range **tidak berpotongan sama sekali**
(`h² = −1,94`, lihat §5a). Penyebabnya bukan dinding: NLOS membuat range lebih
**panjang**, sedangkan di sini justru lebih **pendek**. Kandidat penyebabnya
adalah kalibrasi antenna delay atau posisi anchor yang belum sesuai fisik.

**Catatan penting:** uji ini memakai jarak **2D** antar anchor
(`math.hypot(dx, dy)`) sementara `r` sudah dikonversi ke horizontal, jadi
keduanya konsisten. Kalau anchor dipasang di ketinggian berbeda, perbandingan
ini perlu ditinjau ulang.

---

## 9. Peta algoritma → file

| Algoritma | File | Fungsi |
|---|---|---|
| TWR | `lib/DW1000/DW1000Ranging.cpp` | pengukuran jarak |
| Pre-filter (gate+median+EMA) | `src/main.cpp:129` | `rfUpdate()` |
| Proyeksi 3D→2D | `server/scene.py:130` | `horizontal_range()` |
| Solver 2 anchor (irisan lingkaran) | `src/solver.h:51`, `app.py:254` | `solvePosition` / `solve_2d` |
| Solver 3+ (least squares + Newton) | `src/solver.h:100`, `app.py:280` | idem |
| EKF | `src/ekf.h`, `app.py:326` | `ekfPredict`/`ekfUpdateRange`, `TagEKF` |
| Innovation gating | `src/ekf.h:175`, `app.py:387` | di dalam update |
| Self-healing | `app.py:529` | `EKF_REJECTS` |
| Clamp ruangan | `app.py:545` | di `recompute_positions` |
| Uji segitiga | `src/main.cpp:403`, `app.py:426` | `geometrySlack` / `geometry_check` |
| Slab method | `server/scene.py:85` | `segment_hits_box()` |
| Sigma NLOS | `server/scene.py:144` | `measurement_sigma()` |
| Bias NLOS | `server/scene.py:153` | `nlos_bias()` |

---

## 10. Parameter & penyetelan

| Parameter | Nilai | Di mana | Efek kalau diubah |
|---|---|---|---|
| `MAX_JUMP_M` | 5,0 m | `main.cpp` | lebih kecil = lebih agresif buang |
| `RANGE_HIST` | 3 | `main.cpp` | lebih besar = lebih halus, lebih lambat |
| `EMA_ALPHA` | 0,35 | `main.cpp` | →1 = ikut mentah, →0 = makin halus |
| `sigma_a` | 1,0 m/s² | `ekf.h`, `app.py` | lebih besar = lebih responsif, lebih berisik |
| `sigma_r` | 0,15 m | `ekf.h`, `app.py` | lebih besar = lebih percaya pengukuran |
| `gate_sigma` | 3,0 | `ekf.h`, `app.py` | lebih kecil = lebih selektif |
| `nlos_factor` | 8,0 | `scene.py` | sigma maksimum saat terhalang penuh |
| `bias_m` | 0,35 m | `scene.py` | koreksi NLOS positif |
| `DEFAULT_TAG_Z` | 0,9 m | `scene.py` | tinggi tag saat memproyeksikan jarak |
| cap kovarians | 25 / 9 | `ekf.h`, `app.py` | batas P posisi / kecepatan |
| threshold self-healing | 8 siklus | `app.py` | kecepatan pemulihan |
| `update_ms` | 200 ms | `config.h` | laju siklus; mempengaruhi `dt` EKF |

**Catatan penyetelan.** `sigma_a` dan `sigma_r` menentukan **rasio kepercayaan**
antara model gerak dan pengukuran. Menaikkan `sigma_r` membuat filter lebih
mengikuti pengukuran (responsif, tapi berisik); menaikkan `sigma_a` membuat
filter lebih mengikuti model (halus, tapi lambat bereaksi). Rasio inilah yang
paling sering perlu disetel saat pengujian lapangan, bukan nilai absolutnya.

---

## Rujukan

- **Bregar 2023** — ADS-TWR, multilateration, dataset CIR → §2, §5
- **Kramarić 2025** — DS-TWR, PDOP, anchor koplanar → §2, §8
- **Park 2020** — flip ambiguity pada multilaterasi UWB → §5a
- **Yao 2021** — EKF untuk UWB, distribusi noise LOS/NLOS → §6
- **Fan 2022** — Kalman + uji Mahalanobis untuk NLOS → §6b
- **Shalihan 2022, Angarano 2021, Yang 2024, Kram 2019** — mitigasi galat NLOS → §7
- **Krebs 2024** — arsitektur ESP32 + UWB + EKF lokal di tag → §1, §6
- **Liu 2022** — fusi UWB+IMU (pengembangan lanjutan) → §6

PDF-nya ada di `docs/papers/` (lihat `docs/papers/README.md`).
Tabel review 13 paper: `docs/Review_Jurnal_UWB_Indoor_Positioning.docx`.
