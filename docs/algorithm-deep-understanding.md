# Algoritma Positioning — Pemahaman Mendalam

Pendamping [`ALGORITHMS.md`](ALGORITHMS.md). Dokumen ringkas itu menjelaskan
*apa* yang dipakai; dokumen ini menjelaskan *mengapa*. Isinya:

- apa saja yang dicoba dari catatan bimbingan dan bagaimana hasilnya;
- teori di balik tiap bagian;
- rujukan yang mendukungnya.

Semua angka berasal dari kode dan data di repositori ini dan bisa diulang
(§11). Arti setiap istilah ada di [ALGORITHMS.md → Istilah](ALGORITHMS.md#istilah).

## Daftar isi

1. [Ringkasan](#1-ringkasan)
2. [Catatan bimbingan → apa yang diuji](#2-catatan-bimbingan--apa-yang-diuji)
3. [Audit perubahan Fable (22-10)](#3-audit-perubahan-fable-22-10)
4. [Masalah "nilai halusinasi" saat anchor hilang](#4-masalah-nilai-halusinasi-saat-anchor-hilang)
5. [Teori jendela: moving average, Nyquist–Shannon, median, garis regresi](#5-teori-jendela)
6. [Detail EKF: offset jarak, offset per anchor, bobot asimetris, inisialisasi](#6-detail-ekf)
7. [Studi simulasi](#7-studi-simulasi)
8. [Data rekaman nyata](#8-data-rekaman-nyata)
9. [Bisakah mencapai 2–4 cm?](#9-bisakah-mencapai-24-cm)
10. [Tampilan dashboard yang berskala nyata](#10-tampilan-dashboard-yang-berskala-nyata)
11. [Parameter dan cara mengulang](#11-parameter-dan-cara-mengulang)
12. [Rujukan](#12-rujukan)

---

## 1. Ringkasan

| Pertanyaan | Jawaban singkat |
|---|---|
| Apa algoritma intinya? | EKF pada jarak miring 3D dengan state **offset jarak bersama b** dan **offset tiap anchor dᵢ**, update robust yang lebih ketat untuk jarak yang terlalu panjang, **jendela adaptif** (rata-rata maks 20 keluaran hanya saat diam), dan **gerbang validitas** (posisi ditahan bila < 3 anchor) |
| Apakah moving average 20 sampel dipakai? | Ya, sebagai jendela 20 keluaran **saat diam**. Jendela 20 yang selalu aktif (pada jarak dan/atau posisi) membuat galat >2 m karena tertinggal |
| Apakah filter dua kali (raw dan sesudah) dipakai? | Kedua tahap tersedia. Tahap jarak default mati karena menambah keterlambatan; tahap posisi aktif dalam bentuk adaptif |
| Hasil pada data nyata? | Sebaran saat diam turun 3–4×: CEP50 4,8–6,1 → **1,2–2,1 cm**. Lompatan saat bergerak turun 35–85% |
| Hasil saat anchor hilang? | Galat maksimum posisi valid 85 → 44 cm (simulasi); tidak ada lagi posisi dari 1 anchor |
| Akurasi 2–4 cm tercapai? | **Presisi ya, akurasi belum.** Akurasi saat diam ±4,5–5 cm bila line of sight, ±15 cm bila tubuh pendorong menghalangi (§9) |

---

## 2. Catatan bimbingan → apa yang diuji

| Ide di catatan | Arti teknis | Diuji sebagai | Hasil |
|---|---|---|---|
| *Windowing*: "diambil dari data sebelumnya, dirata-rata" | Moving average (FIR) | `pre_filter=mean`, `post_filter=mean` | Garis lebih halus, tetapi tertinggal (N−1)/2 sampel, jadi galat naik saat bergerak (§5.1, §7) |
| "20 sampel awal untuk mencari nilai tengah" | Median / rata-rata 20 sampel | `median20`, `mean20`; median pada inisialisasi | Median pada **inisialisasi** dipakai. Median / mean 20 terus-menerus terlalu lambat |
| *Compressed sensing* (seperti MRI) | Rekonstruksi sinyal jarang dari sampel di bawah Nyquist | Dianalisis (§5.5) | Tidak diperlukan: sampel sudah di atas laju Nyquist untuk gerak troli |
| Teorema Nyquist–Shannon | Laju sampel ≥ 2× frekuensi gerak tertinggi | Menentukan panjang jendela maksimum (§5.2) | Pada 2,5 sampel/anchor/detik, jendela aman ≤ 3–5 sampel |
| Filter dua kali (pada raw dan sesudahnya) | Jendela 1 pada jarak, jendela 2 pada posisi | Semua kombinasi 8 × 8 (§7) | Terbaik: jendela 1 mati + jendela 2 adaptif. 20 × 20 terburuk |
| "Jalur dicari dari jalur sebelum dan sesudah agar lurus" | Penghalusan non-kausal (*smoother*) | Jejak dashboard ±3 titik (§5.4) | Dipakai untuk **gambar jejak**. Posisi real-time belum punya data "sesudah" |
| Galat terhadap ground truth < 20 cm | Akurasi | Simulasi (posisi benar diketahui) | **Tercapai di simulasi**: MAE 16–18 cm tanpa kalibrasi manual, ±5 cm saat diam bila line of sight |
| MAE (pesan commit 22-10) | *Mean absolute error* | Metrik utama studi | Dilaporkan per keluaran |
| "x y itu engga boleh rata-rata" (pesan commit) | Ditafsirkan: akurasi dihitung dari **setiap** keluaran x, y | MAE per keluaran 10 Hz | Bila maksudnya keluaran tidak boleh dirata-rata, matikan centang "Smooth the path" di Setup → Tracker |
| "Semua algoritma dicoba satu per satu, simulasi lalu hardware" | Studi terkontrol | `tools/smoothing_study.py` + putar ulang rekaman | §7, §8 |

---

## 3. Audit perubahan Fable (22-10)

Perubahan yang ditinggalkan (belum di-commit) di branch
`feat/moving-average-smoothing`:

- `smooth_ms = 500` **aktif secara default**: posisi = rata-rata keluaran
  filter 500 ms terakhir, digeser maju sebesar `v · umur rata-rata jendela`;
- `tools/simulate.py --compare smoothing` ditambahkan, tetapi hasilnya tidak
  dipakai untuk memilih default.

| Aspek | Temuan |
|---|---|
| Sesuai catatan bimbingan? | Sebagian: hanya satu jendela (posisi), tanpa jendela jarak, tanpa uji kombinasi, tanpa MAE |
| Akurasi (simulasi) | MAE **naik** 1,3–4,4 cm di semua skenario. Pergeseran `v·umur` memakai kecepatan EKF yang berisik |
| Kehalusan | Garis lebih halus (kegoyahan 21 → 16 cm di ruangan besar) |
| Data nyata 23-10 | Lompatan > 40 cm berkurang (45 / 30 / 35 vs 47 / 49 / 46), sebaran diam hampir tetap (CEP50 5,1 → 4,9 cm) |
| Masalah halusinasi | Tidak ditangani |
| WLS-RKF / EKF+NLOS | Masih di kode, padahal sudah diputuskan tidak dipakai (tidak lebih baik dari EKF) |

**Tindakan:**

- Rata-rata 500 ms diganti jendela adaptif (§5.3).
- Kebutuhan "garis lurus" dipenuhi di gambar jejak (§5.4).
- WLS-RKF dan EKF+NLOS dihapus; hasil perbandingannya tetap tercatat di commit
  `fe69dc2`.

> **Kenapa hasil 23-10 terlihat "makin ancur"?** Tiga hal lebih berpengaruh
> daripada smoothing:
>
> 1. Hanya **2–2,8 sampel per anchor per detik** di ruangan besar.
> 2. WiFi tag sering macet: celah sampai 19 s di `test-4`.
> 3. Geometri yang dimasukkan kemungkinan salah. Offset jarak `b` di
>    rekaman 23-10 adalah **1,1–1,3 m**, padahal hardware yang sama pada 16-10
>    memberi 0,72–0,74 m. Selain itu offset per anchor mencapai **±22–30 cm**,
>    sedangkan modul DW1000 biasanya hanya berbeda beberapa cm.
>
> Kombinasi ini hampir pasti berarti ukuran ruangan / posisi anchor / tinggi
> tag yang diketik tidak sama dengan pemasangan. Dashboard sekarang
> menampilkan peringatan ⚠ untuk kasus ini (§10).

---

## 4. Masalah "nilai halusinasi" saat anchor hilang

**Bukti** (`20261023-140111_test-4.jsonl`):

| Waktu | Kejadian |
|---|---|
| 17–36 s | Hanya anchor-5 yang terdengar |
| 36,5–37,8 s | Tag mengirim lagi, tetapi hanya jarak ke anchor-5. σ EKF naik sampai 2,4 m, posisi tetap dikirim |
| 37,8–40,3 s | Hanya anchor-4 dan anchor-5. Posisi bergeser ke y = −0,48 m (keluar ruangan, tertahan clamp) |
| Selama itu | API navigasi tetap `valid: true`, karena `valid` dulu hanya berarti "ada estimasi dalam 5 s terakhir" |

**Penyebab.** Satu jarak hanya membatasi posisi ke sebuah lingkaran. Dua
jarak memberi dua titik potong, sering keduanya di dalam ruangan. EKF tetap
menerima update karena ketidakpastiannya ikut membesar, lalu posisinya
"berjalan" sepanjang lingkaran.

**Perbaikan** (`Engine.estimate`, `app.py`):

1. `valid` hanya jika ≥ 3 anchor berbeda punya sampel diterima dalam 1,5 s,
   σ ≤ 0,5 m, dan data terbaru berumur ≤ 1,5 s.
2. Bila tidak valid, **posisi valid terakhir** dikeluarkan (`held: true`).
   Jendela 2 direset dan jejak dashboard tidak menambah titik.
3. API navigasi memakai `NAV_STALE_MS = 1500` (dulu 5000) ditambah flag di
   atas. MQTT push tidak pernah mengirim posisi tidak valid.
4. Dashboard menampilkan `HOLD (n anchors)` dan meredupkan lingkaran margin
   error.

**Hasil:**

- simulasi `dropout` (6 s hanya anchor-5): galat maksimum posisi valid
  85 → 44 cm, ketersediaan 94%;
- putar ulang `test-4`: 1,7% keluaran ditahan, tepat di episode di atas;
- gambar: [`hasil-simulasi/4_anchor_hilang.png`](hasil-simulasi/4_anchor_hilang.png).

---

## 5. Teori jendela

### 5.1 Moving average dan keterlambatannya

```
y[k] = (1/N) · Σ_{j=0}^{N−1} x[k−j]
keterlambatan = (N − 1) / (2 · f_s)          galat ≈ kecepatan × keterlambatan
```

Hardware memberi f_s ≈ **2,5 sampel per anchor per detik** (terukur 2,0–2,8 Hz
pada rekaman 16-10 dan 23-10):

| N | Keterlambatan | Galat pada 0,6 m/s |
|---|---|---|
| 3 | 0,4 s | 24 cm |
| 5 | 0,8 s | 48 cm |
| 20 | 3,8 s | **2,3 m** |

Simulasi 20 × 20 memberi MAE bergerak 2,28 m, cocok dengan rumus di atas.

### 5.2 Nyquist–Shannon: berapa panjang jendela yang boleh?

Sinyal dengan frekuensi tertinggi `f_max` bisa direkonstruksi bila
`f_s ≥ 2·f_max` (Shannon 1949).

- Troli berbelok 90° pada 0,6 m/s dalam ±1–2 s, jadi `f_max` ≈ 0,3–0,5 Hz.
- f_s = 2,5 Hz → frekuensi Nyquist 1,25 Hz: **cukup**.

Moving average N sampel adalah low-pass dengan titik −3 dB ≈ 0,443 · f_s / N:

| N | Titik −3 dB | Arti |
|---|---|---|
| 20 | 0,055 Hz | Gerak troli ikut terpotong |
| 3–5 | 0,2–0,37 Hz | Batas aman |

Jendela yang selalu aktif harus pendek. Menghaluskan lebih kuat hanya aman
saat posisi memang konstan, yaitu saat troli diam (§5.3).

### 5.3 Jendela adaptif (rata-rata saat diam)

Saat diam, merata-ratakan M keluaran mengurangi derau ±√M kali tanpa
keterlambatan. Deteksi diam memakai kecepatan EKF: masuk di bawah 0,15 m/s,
keluar di atas 0,30 m/s.

Pada laju hardware, batas 0,10 m/s hanya mengenali 66% waktu diam, sedangkan
0,15 m/s mengenali 83% dan memberi hasil terbaik. Idenya sama dengan
*zero-velocity update* pada pelacakan pejalan kaki (Foxlin 2005).

### 5.4 Data "sebelum dan sesudah": smoother non-kausal

Rata-rata simetris (titik sebelum dan sesudah) tidak punya keterlambatan,
tetapi butuh data masa depan. Versi optimalnya untuk Kalman adalah smoother
Rauch–Tung–Striebel (1965). Posisi real-time belum punya data masa depan.

Jejak yang digambar adalah riwayat, jadi titik sesudahnya sudah ada:
`buildTrail` menggambar tiap titik sebagai rata-rata ±3 tetangga.

### 5.5 Compressed sensing dan MRI

*Compressed sensing* (Donoho 2006) merekonstruksi sinyal yang jarang
(*sparse*) dari sampel yang **lebih sedikit** daripada Nyquist. Pada MRI
(Lustig dkk. 2007), ruang-k diambil sebagian agar pemindaian cepat.

Masalah positioning troli bukan kekurangan sampel, tetapi derau dan offset
pada sampel yang sudah cukup (§5.2), jadi compressed sensing tidak membantu.
"Windowing" di catatan bimbingan = moving average (§5.1). Bentuk yang tepat:
jendela adaptif (§5.3) dan smoother untuk gambar jejak (§5.4).

### 5.6 Median dan garis regresi

- **Median** (Tukey 1977): menahan spike tunggal. Median 3 pada jarak
  mengurangi lompatan di data nyata, tetapi di simulasi menambah galat
  bergerak ±20 cm. Tersedia, tidak default.
- **Garis regresi** (`linfit`, Savitzky–Golay orde 1): garis kuadrat-terkecil
  dibaca di ujung terbaru, tanpa keterlambatan untuk gerak lurus. Garisnya
  paling halus (kegoyahan 10 vs 21 cm), tetapi MAE naik 4–6 cm dan
  "kebablasan" saat troli berhenti. Tidak dipilih.

---

## 6. Detail EKF

### 6.1 State dan prediksi

```
x = [px, py, vx, vy, b, d₁ … d₄]        kovarians awal:
                                        posisi 0,5², kecepatan 1, b 0,3², dᵢ 0,10²
a = exp(−Δt/τ),  g = τ(1 − a)            τ = vel_tau_s = 1 s,  Δt ≤ 2 s
px += g·vx,  vx *= a   (sama untuk y)
Q  = σa² · [[Δt⁴/4, Δt³/2], [Δt³/2, Δt²]] per sumbu,
     + 0,003²·Δt pada b, + 0,0005²·Δt pada tiap dᵢ
```

Model Singer (1970): untuk Δt kecil praktis kecepatan konstan. Saat WiFi
macet, kecepatan meluruh sehingga estimasi tidak meluncur. Ini pernah
teramati: drift 45 cm pada tag yang diam. Varians posisi dibatasi 25 m² dan
varians kecepatan 4 m²/s².

### 6.2 Pengukuran

```
h(x) = √((px − xᵢ)² + (py − yᵢ)² + Δzᵢ²) + b + dᵢ
H    = [(px − xᵢ)/ρ, (py − yᵢ)/ρ, 0, 0, 1, 0 … 1 (posisi i) … 0]
```

Jarak miring dipakai langsung. Proyeksi `√(r² − Δz²)` memperbesar derau
dekat anchor dan gagal bila r < Δz.

### 6.3 Offset jarak b dan offset per anchor dᵢ

- **b** (dulu disebut "bias"): delay antena yang sama untuk semua anchor,
  ±0,75–1 m. Dengan 4 anchor ada lebih banyak persamaan daripada x, y, jadi b
  bisa dihitung, seperti bias jam penerima pada GPS. Dibatasi [−0,5 ; 1,5] m.
- **dᵢ**: beda tiap modul terhadap yang lain, beberapa cm.
  - Dari **satu** titik, dᵢ tidak bisa dipisahkan dari posisi: 4 jarak
    melawan 7 besaran yang tidak diketahui (x, y, b, d₁–d₄).
  - Begitu troli bergerak dan melihat anchor dari arah yang berbeda-beda,
    dᵢ menjadi teramati. Ini kalibrasi mandiri (*self-calibration*), ide yang
    terkait dengan kalibrasi mandiri anchor pada Hapsari 2025.
  - Prior σ 10 cm menahan dᵢ dekat 0 sampai datanya cukup. dᵢ dibatasi
    ±30 cm.

Hasil simulasi line of sight tanpa kalibrasi manual, galat saat diam:

| Putaran | Tanpa dᵢ | Dengan dᵢ |
|---|---|---|
| 1 | 10,2 cm | 5,8 cm |
| 2 | 9,8 cm | 4,7 cm |
| 3 | 9,6 cm | 4,5 cm |

Grafiknya: [`hasil-simulasi/5_konvergensi_offset.png`](hasil-simulasi/5_konvergensi_offset.png).

Di hardware, dᵢ > 20 cm hampir pasti berarti posisi anchor di denah salah,
bukan modulnya. Server lalu mengirim `offset_warning` dan dashboard memberi
⚠ (§10).

### 6.4 Update robust dengan bobot asimetris

```
e = (z − h)/√S
|e| > 6              → tolak
e > 1   (lebih panjang dari dugaan) → R ← R·e/1
e < −2,5 (lebih pendek)             → R ← R·|e|/2,5
```

Huber (1964): sampel menyimpang tidak dibuang, hanya bobotnya diturunkan.
Batasnya dibuat asimetris karena halangan (tubuh, rak, dinding) **hanya**
membuat jarak UWB lebih panjang. Ini sejalan dengan uji NLOS satu-sisi Fan &
Du (2022).

Di simulasi ruangan besar, bobot asimetris (bersama dᵢ) menurunkan MAE
17,7 → 16,6 cm, P95 diam 31 → 25 cm, dan kegoyahan 20 → 15 cm. Kovarians
diperbarui dengan bentuk Joseph.

### 6.5 Inisialisasi dengan pencarian offset

1. Untuk tiap anchor, ambil median sampel 0,7 s terakhir.
2. **Saat pertama kali** (b belum pernah diketahui) dan ada ≥ 4 anchor:
   - cari b dari −0,5 sampai 1,5 m dengan langkah 5 cm, lalu diperhalus
     1 cm;
   - untuk tiap b, hitung multilaterasi (x, y), lalu pilih b dengan residu
     terkecil;
   - hasil dipakai hanya bila residu ≤ 30 cm dan posisinya di dalam ruangan;
   - server menunggu hingga 5 s supaya keempat anchor terdengar.
3. Selain itu: multilaterasi biasa memakai b yang sudah diketahui (bentuk
   tertutup + 6 iterasi Gauss-Newton). Fix pertama butuh semua anchor (atau
   ≥ 3 setelah 2 s) dan residu ≤ 0,6 m.
4. Posisi belum ditampilkan sampai menyerap 2 sampel per anchor.

Kenapa pencarian, bukan Gauss-Newton tiga variabel? Gauss-Newton x, y, b
sempat dicoba dan **divergen** (b = 3 m) di ruangan 2,6 × 1,3 m dengan semua
anchor setinggi 1,74 m. Pencarian satu dimensi tidak bisa divergen.

Tanpa langkah ini, server yang di-restart di tengah sesi menganggap b = 0
(padahal ±1 m), fix pertama salah tempat, dan track terkunci di posisi salah.
Kasus ini ditemukan saat memutar ulang rekaman dari tengah.

**Inisialisasi ulang:** 1,5 s tanpa sampel diterima → mulai lagi dari
geometri, dengan b dan dᵢ terakhir dipertahankan.

---

## 7. Studi simulasi

`tools/smoothing_study.py` memakai model radio `tools/simulate.py`:

| Komponen model | Nilai |
|---|---|
| Derau DS-TWR | 6 cm |
| Offset bersama | 20 cm |
| Sebaran offset per anchor | 8 cm (sisa kalibrasi 3 cm pada skenario terkalibrasi) |
| Outlier | 4%, +0,3–1,5 m |
| NLOS tubuh | +15 cm bila anchor di belakang pendorong |
| Paket hilang | 8% |
| Pengiriman | 5 paket/s ke server, latensi 20–170 ms |

Semua algoritma dijalankan pada **data yang persis sama**, 10 kali
Monte-Carlo.

| Skenario | Isi |
|---|---|
| `lab` | 4,5 × 5,4 m, 0,8 m/s, berhenti 2 s, 6 sampel/anchor/detik |
| `market` | 9,5 × 5,9 m (anchor seperti rekaman 23-10, tag 1,0 m), 0,6 m/s, berhenti 6 s, **2,5 sampel/anchor/detik** seperti hardware |
| `dropout` | `market`, detik 20–26 hanya anchor-5 |
| `market-los` | `market` tanpa NLOS tubuh dan outlier (± versi terkalibrasi) |

Metrik: posisi yang ditampilkan disampel 10 Hz lalu dibandingkan dengan posisi
benar pada saat yang sama.

| Metrik | Arti |
|---|---|
| MAE | Galat rata-rata |
| CEP50 diam | Presisi di sekitar rata-rata tiap titik berhenti |
| Kegoyahan | RMS beda-kedua keluaran berurutan |
| Galat maks anchor hilang | Galat terbesar posisi valid selama dropout + 3 s |

**Hasil `market`** (cm):

| Algoritma | MAE | Bergerak | Diam | P95 diam | CEP50 diam | Kegoyahan |
|---|---|---|---|---|---|---|
| A. EKF saja (sebelum) | 19,7 | 20,6 | 18,2 | 35,9 | 6,6 | 21,2 |
| B. A + rata-rata 500 ms (Fable) | 21,0 | 22,8 | 18,0 | 34,8 | 6,3 | 15,8 |
| C. A + jendela 20 × 20 | 205,9 | 228,2 | 169,5 | 287,9 | 49,8 | 3,9 |
| D. A + median 3 pada jarak | 32,2 | 42,2 | 15,8 | 26,9 | 4,0 | 15,8 |
| E. A + garis regresi 10 | 25,0 | 25,2 | 24,7 | 49,6 | 11,0 | 10,0 |
| F. A + jendela adaptif + gerbang | 19,5 | 20,8 | 17,5 | 32,8 | 3,5 | 20,2 |
| **G. final** | **16,6** | **17,3** | **15,5** | **25,2** | **2,0** | 15,4 |

**Skenario lain** (G dibanding A, cm):

| Skenario | MAE | Diam | CEP50 diam | Catatan |
|---|---|---|---|---|
| `lab` (6 Hz) | 20,3 vs 21,3 | 20,0 vs 18,5 | 0,9 vs 2,6 | batas diam 0,15 dipilih untuk laju hardware |
| `dropout` | 17,7 vs 21,0 | 15,9 vs 19,9 | 1,8 vs 6,7 | galat maks valid **44** vs 85; ketersediaan 94% |
| `market-los` | 16,5 vs 19,5 | **5,0** vs 10,9 | 1,3 vs 4,8 | tanpa kalibrasi manual |
| `market-los` terkalibrasi | 16,4 vs 17,4 | **4,7** vs 7,3 | 1,3 vs 4,5 | |

**Kompensasi keterlambatan** (`extrapolate_ms=300`):

- di simulasi MAE turun 1–3 cm;
- di rekaman nyata lompatan hampir dua kali lipat (44–46 → 79–90), karena
  kecepatan EKF di hardware lebih berisik.

Default-nya 0. Tabel semua kombinasi ada di `docs/smoothing_study.json` dan
sheet "Semua hasil" di [`hasil-simulasi/Hasil_Simulasi.xlsx`](hasil-simulasi/Hasil_Simulasi.xlsx).

---

## 8. Data rekaman nyata

Tanpa ground truth, yang bisa diukur hanya presisi (saat diam) dan perilaku
(lompatan, kegoyahan, keluar ruangan). Setiap rekaman diputar ulang lewat
versi lama dan versi final.

| Rekaman | EKF saja (sebelum) | Final |
|---|---|---|
| `statis-final` (diam), CEP50 / CEP95 | 5,1 / 12,0 cm | **1,6 / 4,3 cm** |
| `statis-malam-20mnt` (diam) | 6,1 / 16,1 cm | **2,1 / 5,8 cm** |
| `verifikasi-final` (diam) | 4,8 / 11,6 cm | **1,2 / 4,2 cm** |
| `test-1` (bergerak), lompatan > 40 cm / kegoyahan / keluar ruangan | 47 / 32,6 cm / 7,4% | **28 / 20,3 cm / 2,2%** |
| `test-2` (bergerak) | 49 / 32,8 cm / 0% | **8 / 11,6 cm / 0%** |
| `test-4` (bergerak) | 46 / 27,9 cm / 2,1% | **30 / 21,3 cm / 0,5%** |

Offset yang dihitung:

- 16-10: b ≈ 0,71–0,75 m, dᵢ dalam ±10 cm, kecuali satu ±21 cm pada rekaman
  20 menit tanpa gerak, yang memang tidak bisa dipisahkan.
- 23-10: b ≈ 0,93–1,30 m, dᵢ ±22–30 cm, yang menandakan geometri salah (§3).

---

## 9. Bisakah mencapai 2–4 cm?

| Besaran | Pertanyaan | Hasil | Status |
|---|---|---|---|
| **Presisi** | Seberapa rapat posisi saat diam | **1,2–2,1 cm** CEP50 data nyata; 2,0 cm simulasi | Tercapai |
| **Akurasi**, line of sight | Seberapa dekat dengan posisi benar saat diam | **5,0 cm**, lalu **4,5 cm** setelah 3 putaran (simulasi) | Hampir |
| **Akurasi**, tubuh menghalangi | Sama, saat pendorong menutupi anchor di belakangnya | ±15 cm (simulasi) | Belum |

Rata-rata hanya membuang derau acak. Yang tersisa adalah galat yang **sama
setiap kali**:

1. **NLOS tubuh pendorong.** Jarak ke anchor di belakangnya +10–30 cm selama
   ia berdiri di sana. Bobot asimetris (§6.4) mengurangi pengaruhnya, tetapi
   tidak bisa menghapusnya.
2. **Sisa offset tiap anchor.** dᵢ terpelajari bertahap (§6.3).
3. **Geometri yang dimasukkan.** Salah 30 cm pada tinggi tag menggeser jarak
   ±13 cm.

Langkah untuk masuk ke 2–4 cm, urut dari yang paling berpengaruh:

1. **Pasang tag tinggi di tiang troli**, di atas kepala pendorong, supaya
   tubuh tidak menghalangi. Ini mengubah ±15 cm menjadi ±5 cm di simulasi.
2. Ukur ulang posisi dan tinggi anchor serta tinggi tag. Warna anchor dan
   peringatan ⚠ di dashboard membantu menemukan yang salah.
3. Biarkan troli berjalan 1–2 menit di ruangan supaya dᵢ terpelajari, atau
   kalibrasi di 3–4 titik meteran laser.
4. Naikkan laju ranging per anchor (2,5 Hz itu rendah untuk DW1000).

**Penamaan algoritma.** Syaratnya *akurasi* 2–4 cm; yang sudah terbukti baru
presisinya, dan akurasi line of sight ±4,5–5 cm di simulasi. Algoritma ini
disebut deskriptif sebagai **EKF dengan offset per anchor, jendela adaptif,
dan gerbang validitas**. Nama bisa ditetapkan setelah uji meteran laser di
ruangan memberi MAE 2–4 cm.

---

## 10. Tampilan dashboard yang berskala nyata

Sebelumnya beberapa elemen 3D tidak mengikuti angka sebenarnya. Sekarang:

| Elemen | Sebelum | Sekarang |
|---|---|---|
| Grid lantai | 1,4× ruangan, berpusat di tengah. Sel 9,5 m / 13 = 1,02 m; tidak ada garis di meter bulat | Persis ruangan; garis tiap 1 m dari pojok 0,0 (tipis tiap 0,5 m) |
| Penggaris | Tick bisa melewati dinding (tick 10 m pada dinding 9,8 m) | Berhenti di dinding; panjang persis ditulis di ujung |
| Label ukuran | Dibuat tetapi tidak pernah ditambahkan ke scene | Lebar, kedalaman, tinggi tampil |
| Lingkaran di tag | σ × 1,6, minimal 25 cm (tidak bermakna) | **Margin error ±1σ berskala nyata**; nilainya ditulis di label tag |
| Cincin "sonar" | Animasi hiasan 2,6 m | Diganti **lingkaran jarak tiap anchor** berskala nyata (jarak − offset, diproyeksikan ke tinggi tag) |
| Tiang anchor | Hijau / abu-abu (online / offline) | **Warna = kondisi**: hijau OK · kuning periksa · merah masalah · abu-abu offline; alasan di label, detail saat diklik |
| Istilah | Tanpa penjelasan | Setiap angka punya penjelasan saat kursor diarahkan, glosarium di tab Live, legenda di viewport |

Kondisi anchor (`anchorStatus` di `app.js`):

| Kondisi | Arti |
|---|---|
| **OK** | Anchor mengukur tag dalam 3 s terakhir, jalurnya LOS, residu ≤ 30 cm, offset ≤ 20 cm |
| **Periksa** (kuning) | Tidak ada jarak ke tag, residu > 30 cm, WiFi mati tetapi radio masih mengukur, atau offset > 20 cm (posisi di denah kemungkinan salah) |
| **Masalah** (merah) | Jalur melewati obstacle (NLOS) atau jaraknya ditolak filter |
| **Offline** (abu-abu) | Tidak ada laporan WiFi dan tidak ada jarak |

---

## 11. Parameter dan cara mengulang

Tabel ini adalah parameter **versi riset** (`tools/tracker_lab.py`, `LAB_DEFAULTS`; default = algoritma G). Server hanya menjalankan algoritma final CALF-UWB (K + G + Z + L) dengan nilai tetap (`server/tracker.py` `PARAMS`, lihat [ALGORITHMS.md](ALGORITHMS.md#parameter-tetap)); satu-satunya pilihan di server adalah "Smooth the path".

| Parameter | Default | Satuan | Keterangan |
|---|---|---|---|
| `sigma_a` | 0,8 | m/s² | Derau percepatan |
| `sigma_r` | 0,12 | m | Derau jarak |
| `huber_k` / `huber_k_long` | 2,5 / 1,0 | – | Batas bobot jarak pendek / panjang |
| `gate` | 6 | – | Tolak sampel |
| `max_late_ms` | 400 | ms | Sampel lebih tua dari filter dibuang |
| `reinit_ms` | 1500 | ms | Tanpa sampel diterima → inisialisasi ulang |
| `room_margin` | 0,5 | m | Batas posisi di luar ruangan |
| `vel_tau_s` | 1,0 | s | Waktu korelasi kecepatan |
| `estimate_bias` | 1 | – | Hitung offset jarak b |
| `anchor_offsets` | 1 | – | Hitung offset tiap anchor dᵢ |
| `extrapolate_ms` | 0 | ms | Prediksi ke saat paket tiba |
| `pre_filter`, `pre_window` | `none`, 5 | –, sampel | Jendela pada jarak |
| `post_filter`, `post_window` | `still`, 20 | –, keluaran | Jendela pada posisi |
| `still_speed` | 0,15 | m/s | Batas diam |
| `window_ms` | 4000 | ms | Jendela tidak melewati celah lebih lama |
| `min_anchors`, `fresh_ms`, `valid_sigma` | 3, 1500, 0,5 | –, ms, m | Gerbang validitas |
| `zero_velocity` | 0 | – | Mode diam Z (di server selalu aktif) |
| `still_long` | 0 | σ | Tolak jarak kepanjangan L saat diam (di server 1,0) |

```bash
# studi semua kombinasi (±3 menit) → docs/smoothing_study.json
python tools/smoothing_study.py --runs 10

# gambar + Excel hasil simulasi → docs/hasil-simulasi/
python tools/sim_report.py

# putar ulang rekaman dengan pengaturan lain
python tools/evaluate.py server/logs/<rekaman>.jsonl --replay --set anchor_offsets=0

# gambar ulang diagram alur (butuh draw.io desktop)
python docs/gambaran-sistem/gen_diagrams.py
```

---

## 12. Rujukan

| Rujukan | Mendukung bagian |
|---|---|
| Yao dkk. 2021, *Indoor Positioning Accuracy of UWB* (`docs/papers/06`) | EKF lebih baik daripada trilaterasi / least squares saat ada outlier (§6) |
| Krebs dkk. 2024, *ESP32 + DWM3000 UWB Positioning* (`docs/papers/01`) | Arsitektur ESP32 + UWB + Kalman (§6) |
| Fan & Du 2022, *WLS + Kalman untuk NLOS* (`docs/papers/04`) | NLOS memperpanjang jarak → uji / bobot satu-sisi (§6.4); algoritma lengkapnya dibandingkan dan tidak dipakai (§3) |
| Hapsari dkk. 2025, *Modified DW Anchor Self-Calibration* (`docs/papers/09`) | Kalibrasi mandiri anchor, terkait dengan offset per anchor (§6.3) |
| Kramarić dkk. 2025, *Anchor Placement* (`docs/papers/07`) | Geometri anchor, ≥ 3 anchor untuk fix 2D (§4) |
| Vandermeeren & Steendam 2021, IEEE Sensors J. 21(9) | Troli belanja + UWB + Kalman: 62,6 → 34,1 cm (konteks) |
| R. A. Singer 1970, IEEE Trans. AES-6(4), 473–483 | Model kecepatan (§6.1) |
| P. J. Huber 1964, Ann. Math. Stat. 35(1), 73–101 | Update robust (§6.4) |
| H. E. Rauch, F. Tung, C. T. Striebel 1965, AIAA J. 3(8), 1445–1450 | Smoother sebelum & sesudah (§5.4) |
| C. E. Shannon 1949, Proc. IRE 37(1), 10–21 | Teorema sampling (§5.2) |
| S. W. Smith 1997, *The Scientist and Engineer's Guide to DSP*, bab 15 | Respons frekuensi moving average (§5.1–5.2) |
| J. W. Tukey 1977, *Exploratory Data Analysis* | Median (§5.6) |
| A. Savitzky & M. J. E. Golay 1964, Anal. Chem. 36(8), 1627–1639 | Garis regresi berjendela (§5.6) |
| E. Foxlin 2005, IEEE CG&A 25(6), 38–46 | Memanfaatkan momen diam (§5.3) |
| D. L. Donoho 2006, IEEE Trans. Inf. Theory 52(4), 1289–1306 | Compressed sensing (§5.5) |
| M. Lustig, D. Donoho, J. M. Pauly 2007, Magn. Reson. Med. 58(6), 1182–1195 | Compressed sensing pada MRI (§5.5) |
