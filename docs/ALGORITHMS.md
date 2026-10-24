# Algoritma Positioning

Dokumen ini menjelaskan algoritma yang **benar-benar jalan** di server
(`server/tracker.py`). Fokusnya algoritma inti; algoritma pendukung cukup
digambarkan garis besarnya.

- Penurunan rumus, hasil uji lengkap, pembahasan catatan bimbingan, dan
  rujukan per bagian: [`algorithm-deep-understanding.md`](algorithm-deep-understanding.md).
- Gambar dan Excel hasil simulasi: [`hasil-simulasi/`](hasil-simulasi/).
- Arti setiap istilah: [Istilah](#istilah) di akhir dokumen.

Konvensi: koordinat lantai `(x, y)` dalam meter dari pojok 0,0 (anchor A2),
`z` = tinggi. Anchor = node tetap; tag = node di troli.

---

## Masalah yang diselesaikan

1. **Jarak UWB berisik dan kadang meleset jauh.** Derau satu sampel DS-TWR
   ±6–12 cm. Tubuh orang atau pantulan bisa menambah puluhan sentimeter, dan
   sesekali modul melaporkan angka mustahil.
2. **Semua jarak kelebihan jarak yang sama.** Delay antena DW1000 menambah
   ±75–100 cm ke setiap jarak. Selain itu tiap modul masih berbeda beberapa
   sentimeter dari modul lain.
3. **Data datang tidak serempak dan kadang terlambat.**
   - Laju sampel hanya ±2,5 per anchor per detik.
   - Data dikirim lewat WiFi secara berkelompok.
   - Kadang data macet beberapa detik.
4. **Saat troli diam, posisi bergetar beberapa sentimeter;** saat bergerak,
   garis lintasannya patah-patah.
5. **Saat data anchor berhenti, server dulu tetap mengeluarkan posisi.**
   Posisi itu tebakan dari 1–2 anchor dan bisa melenceng keluar ruangan
   (rekaman 23-10 `test-4`).

## Algoritma inti

**EKF + kalibrasi statis + mode diam + tolak jarak kepanjangan + gerbang
validitas** (di hasil studi disebut "Rekomendasi" = K + G + Z + L).

Ini satu-satunya algoritma di server. Nilainya tetap (tabel
[Parameter](#parameter-tetap)). Yang bisa diatur pengguna hanya:

- ukuran ruangan, tinggi tag, posisi anchor, dan kalibrasi (Setup → Room,
  Live → Inspector, Setup → Calibration);
- centang **Smooth the path** (Setup → Tracker), lihat §6.

Varian lain yang dipakai saat riset (jendela pada jarak, jendela posisi,
rata-rata C 20 × 20, bayangan badan B/R, dan seterusnya) dipindah ke
`tools/tracker_lab.py`. File itu hanya dipakai skrip simulasi di `tools/`,
tidak oleh server.

![Alur algoritma](img/alur_algoritma.png)

*Sumber yang bisa diedit: [`img/alur_algoritma.drawio`](img/alur_algoritma.drawio)
(dibuat oleh `docs/gambaran-sistem/gen_diagrams.py`).*

### 1. EKF pada jarak miring, dengan offset jarak

State dan model (`TagFilter`):

```
x = [px, py, vx, vy, b, d₁, d₂, d₃, d₄]
      b  = offset jarak bersama semua anchor (delay antena, ±0,75–1 m)
      dᵢ = offset tiap anchor terhadap yang lain (beberapa cm)

prediksi (model kecepatan Singer, τ = 1 s):
  a = e^(−Δt/τ),  g = τ(1 − a)
  px += g·vx,  py += g·vy,  vx *= a,  vy *= a
  P  = F P Fᵀ + Q(σa = 0,8 m/s²)

pengukuran satu anchor i (jarak miring 3D):
  h(x) = √((px − xᵢ)² + (py − yᵢ)² + Δzᵢ²) + b + dᵢ      Δzᵢ = z_anchor − z_tag
  e    = (z − h(x)) / √S                                   S = H P Hᵀ + R

update robust:
  |e| > 6             → sampel ditolak
  e > 1  atau e < −2,5 → R diperbesar (bobot turun, sampel tidak dibuang)
  K = P Hᵀ / S,  x += K·(z − h),  P = (I − KH) P (I − KH)ᵀ + K R Kᵀ
```

Poin-poin penting:

- **Setiap sampel dipakai satu kali,** pada waktu ukurnya sendiri.
- **Offset diambil dari kalibrasi statis (K) bila ada.** Bila setiap anchor
  punya titik kalibrasi (Setup → Calibration), jarak dikoreksi
  `r = (raw − offset) / scale` sebelum masuk EKF. Offset `b` dan `dᵢ` lalu
  dikunci 0 dan tidak dipelajari lagi. Di simulasi, kalibrasi menurunkan
  galat di rak dari 5,2 menjadi 3,9 cm.
- **Tanpa kalibrasi, offset dipelajari sendiri.**
  - `b` dihitung EKF, mirip bias jam penerima GPS: 4 anchor memberi lebih
    banyak persamaan daripada yang dibutuhkan untuk x dan y saja.
  - `dᵢ` dipelajari selama troli bergerak. Dari satu titik saja, offset ini
    tidak bisa dipisahkan dari posisi. Begitu troli melihat setiap anchor dari
    arah yang berbeda-beda, offsetnya bisa dipisahkan (*self-calibration*).
  - Server memilih sendiri: `calibrated = true` di `/api/v1/tracking` bila
    semua anchor sudah dikalibrasi.
- **Jarak yang terlalu panjang dipercaya lebih sedikit.** Halangan (tubuh,
  rak) hanya bisa membuat jarak **lebih panjang**, jadi batas Huber untuk
  e > 0 dibuat lebih ketat (1) daripada untuk e < 0 (2,5).

### 2. Mode diam (Z) dan tolak jarak kepanjangan (L)

Keduanya selalu aktif. Hasil studi semua kombinasi pada troli robot di antara
pembeli ada di [RISET_JENDELA.md](hasil-simulasi/RISET_JENDELA.md).

```
masuk mode diam : kecepatan EKF < 0,2 m/s selama 0,4 s
selama diam     : σa 0,8 → 0,05 m/s²; pengukuran semu v = 0 (σ 0,02 m/s)
                  jarak dengan e > 1 dibuang, bila ≥ 3 anchor lain diterima
                  dalam 1,5 s terakhir
keluar          : 3 dari 4 jarak terakhir |e| > 1,5
```

- **Mode diam (Z, *zero-velocity update*).** Saat troli berhenti, kecepatan
  dikunci 0 dan derau proses dikecilkan. Satu putaran jarak yang buruk tidak
  lagi menggeser posisi. Keluaran `still` = sedang dalam mode diam.
- **Tolak jarak kepanjangan (L).** Selama mode diam, jarak yang lebih panjang
  dari perkiraan lebih dari 1σ dibuang, bukan hanya diturunkan bobotnya.
  Alasannya, badan orang hanya bisa memperpanjang jarak.
- **Pengaman L.** Penolakan hanya terjadi selama anchor lain masih cukup
  untuk menghitung posisi. Bila koordinat anchor di denah salah, satu anchor
  bisa terus terbaca kepanjangan. Tanpa pengaman ini posisi menjadi tidak valid
  (rekaman `test-2`: ketersediaan turun ke 78%, dengan pengaman kembali 98%).
- **Saat bergerak** posisi = keluaran EKF langsung, tanpa jeda tambahan.

Mode ini menggantikan "jendela adaptif" versi sebelumnya (rata-rata sampai 20
keluaran saat diam, varian G). Di simulasi bacaan di rak turun dari
4,4 / 6,7 / 10,0 cm (G) ke 3,0 / 3,4 / 5,4 cm (K + G + Z + L), untuk dunia
kosong / sepi / ramai.

### 3. Gerbang validitas dan tahan posisi

```
valid ⇔ ≥ 3 anchor berbeda punya sampel diterima dalam 1,5 s terakhir
        dan margin error σ ≤ 0,5 m dan data terbaru berumur ≤ 1,5 s
tidak valid → keluarkan posisi valid TERAKHIR (held = true, valid = false)
```

Satu atau dua anchor tidak bisa menentukan posisi 2D. API navigasi ikut
menyatakan `valid: false`, sehingga troli tidak dikemudikan dengan posisi
tebakan.

### 4. Inisialisasi dengan pencarian offset

Waktu server baru menyala, offset `b` belum diketahui. Kalau `b` dianggap 0,
padahal aslinya ±1 m, fix pertama salah tempat dan track terkunci di posisi
yang salah. Kesalahan ini terlihat saat rekaman diputar ulang dari
tengah-tengah.

Langkah inisialisasi:

1. Ambil median sampel 0,7 s terakhir tiap anchor.
2. Cari `b` dari −0,5 sampai 1,5 m:
   - langkah 5 cm, lalu diperhalus 1 cm;
   - untuk tiap nilai `b`, hitung multilaterasi (x, y);
   - pilih `b` dengan residu terkecil.
3. Hasilnya hanya dipakai bila residu ≤ 30 cm dan posisinya di dalam ruangan.

### 5. Jendela (hanya untuk riset)

Jendela pada jarak (median / mean / garis regresi N sampel sebelum EKF) dan
jendela pada posisi (rata-rata 20 × 20 versi catatan bimbingan, varian C)
tidak ada lagi di server. Pada laju ±2,5 sampel per anchor per detik, jendela
N sampel menunda ±(N−1)/5 detik, dan di simulasi varian C meleset ±63 cm.
Kodenya tetap ada di `tools/tracker_lab.py` (`window_fit`, `LabEngine`) supaya
hasil studi bisa diulang.

### 6. Haluskan jalur (satu-satunya pilihan, Setup → Tracker)

```
centang "Smooth the path" → posisi = rata-rata 3 keluaran EKF terakhir
                            (tidak melewati jeda data > 4 s)
tidak dicentang           → posisi = keluaran EKF langsung
```

- Default: **dicentang**. Disimpan di `scene.json` (`"tracking": {"smooth": 1}`)
  dan bisa diubah lewat `PUT /api/v1/tracking {"smooth": true|false}`.
- Simulasi, bacaan di rak (kosong / sepi / ramai), dengan kalibrasi:
  - tanpa haluskan: 3,0 / 3,4 / 5,4 cm, galat bergerak 23,2 cm;
  - dengan haluskan: 3,1 / 3,5 / 5,5 cm, galat bergerak 34,5 cm.
- Garis ±3× lebih mulus, tapi tertinggal ±11 cm lebih banyak saat troli
  berjalan. Untuk navigasi yang butuh posisi paling baru, matikan centangnya.

---

## Algoritma pendukung (garis besar)

| Algoritma | Fungsi | Di mana |
|---|---|---|
| Dedup sampel | Setiap pengukuran dipakai sekali | `Engine.ingest` |
| Sinkronisasi jam | `millis()` device dipetakan ke waktu server | `ClockSync` |
| Satu sumber per tag | Salinan jarak dari anchor hanya dipakai bila tag diam > 10 s | `Engine.ingest` |
| Uji jarak mustahil | Jarak > sudut terjauh ruangan + 1,5 m dibuang | `Engine._process` |
| Kalibrasi manual per anchor | `r = (raw − offset) / scale` dari titik yang diukur meteran laser | `Engine.correct`, `/api/v1/calibration` |
| Inisialisasi ulang | 1,5 s tanpa sampel diterima → mulai lagi dari geometri | `_process` |
| Batas ruangan | Posisi dijepit ke ruangan ± 0,5 m | `_clamp_to_room` |
| NLOS dari denah | Jalur yang menembus obstacle mendapat σ lebih besar | `scene.py` |
| Peringatan posisi anchor | `dᵢ` > 20 cm → anchor itu kemungkinan salah posisi di denah | `estimate` (`offset_warning`) |

## Tampilan di dashboard

Semua elemen 3D sekarang mengikuti angka sebenarnya. Legenda di pojok kanan
bawah viewport menjelaskan elemen mana yang berskala nyata.

| Elemen | Arti |
|---|---|
| Lantai dan grid | Persis ukuran ruangan; garis tiap 1 m dari pojok 0,0 (tipis tiap 0,5 m). Penggaris di keempat sisi berakhir dengan panjang sebenarnya |
| Lingkaran di sekitar tag | Margin error ±1σ, **berskala nyata** (±2 dari 3 posisi ada di dalamnya) |
| Lingkaran putus-putus | Jarak yang diukur tiap anchor, **berskala nyata**. Titik temunya = posisi |
| Tiang anchor | Dari lantai sampai tinggi anchor sebenarnya |
| Warna tiang anchor | Kondisi anchor: hijau OK · kuning periksa (tidak ada jarak, residu besar, WiFi mati, atau offset > 20 cm) · merah masalah (terhalang / jarak ditolak) · abu-abu offline. Alasannya tertulis di label; klik anchor untuk detail |
| Garis jejak | Rata-rata ±3 titik sebelum dan sesudah (hanya gambar; posisi live tidak ditunda) |
| Kepala tag dan anchor | Penanda, bukan ukuran asli |

---

## Hasil

### Simulasi

Ruangan 9,5 × 5,9 m, laju anchor seperti hardware, 10 kali Monte-Carlo
(`tools/smoothing_study.py`, gambar di [`hasil-simulasi/`](hasil-simulasi/)).

| Algoritma | Galat bergerak (MAE) | Galat diam (MAE) | Sebaran diam (CEP50) | Kegoyahan garis | Galat maks saat anchor hilang |
|---|---|---|---|---|---|
| A. EKF saja (sebelum) | 20,6 cm | 18,2 cm | 6,6 cm | 21,2 cm | 85 cm |
| B. A + rata-rata 500 ms (Fable) | 22,8 cm | 18,0 cm | 6,3 cm | 15,8 cm | 84 cm |
| C. A + jendela 20 × 20 (catatan bimbingan) | 228 cm | 170 cm | 50 cm | 3,9 cm | 298 cm |
| D. A + median 3 pada jarak | 42,2 cm | 15,8 cm | 4,0 cm | 15,8 cm | 87 cm |
| E. A + garis regresi 10 pada posisi | 25,2 cm | 24,7 cm | 11,0 cm | 10,0 cm | 92 cm |
| F. A + jendela adaptif + gerbang | 20,8 cm | 17,5 cm | 3,5 cm | 20,2 cm | 50 cm |
| **G. F + offset per anchor + bobot jarak panjang** | **17,3 cm** | **15,5 cm** | **2,0 cm** | **15,4 cm** | **44 cm** |

Studi ini berhenti di G. Algoritma final (G + kalibrasi + mode diam + tolak
jarak kepanjangan) diuji di dunia troli robot di antara pembeli, dengan semua
kombinasi lain: [RISET_JENDELA.md](hasil-simulasi/RISET_JENDELA.md).

| Algoritma (dunia troli robot) | Bacaan di rak: kosong / sepi / ramai |
|---|---|
| G | 4,4 / 6,7 / 10,0 cm |
| G + Z + L (tanpa kalibrasi) | 3,5 / 4,7 / 7,4 cm |
| **K + G + Z + L (final)** | **3,0 / 3,4 / 5,4 cm** |
| K + G + Z + L + haluskan jalur | 3,1 / 3,5 / 5,5 cm |
| C / G + C (jendela 20) | ±63 cm |

Kondisi line of sight (tanpa halangan tubuh), saat diam:

- G tanpa kalibrasi manual: **5,0 cm**, lalu **4,5 cm** setelah tiga putaran
  karena offset tiap anchor makin terpelajari.
- G dengan kalibrasi: 4,7 cm.

### Data rekaman nyata

Diputar ulang lewat versi lama dan versi final.

| Rekaman | Sebelum | Final |
|---|---|---|
| `statis-final` (diam), CEP50 / CEP95 | 5,1 / 12,0 cm | **1,6 / 4,3 cm** |
| `statis-malam-20mnt` (diam) | 6,1 / 16,1 cm | **2,1 / 5,8 cm** |
| `verifikasi-final` (diam) | 4,8 / 11,6 cm | **1,2 / 4,2 cm** |
| `test-1` (bergerak), lompatan > 40 cm / kegoyahan | 47 / 32,6 cm | **28 / 20,3 cm** |
| `test-2` (bergerak) | 49 / 32,8 cm | **8 / 11,6 cm** |
| `test-4` (bergerak) | 46 / 27,9 cm | **30 / 21,3 cm** |

### Target 2–4 cm

| Ukuran | Hasil | Status |
|---|---|---|
| **Presisi** (sebaran saat diam) | CEP95 1,4 cm (rekaman 23-10, algoritma final) | Tercapai |
| **Akurasi** saat diam, tanpa orang | 3,0 cm (simulasi, final) | Tercapai |
| **Akurasi** saat diam, sedikit pembeli | 3,4 cm (simulasi, final) | Tercapai |
| **Akurasi** saat diam, ramai pembeli | 5,4 cm (simulasi, final) | Hampir |

Halangan tubuh pembeli membuat jarak lebih panjang ke anchor di balik badan.
Mode diam dan tolak jarak kepanjangan mengurangi sebagian besarnya tanpa
hardware tambahan. Sisanya lebih ke pemasangan, yaitu tag dipasang tinggi di
tiang troli. Detail di dokumen mendalam §9.

---

## Parameter tetap

Nilai algoritma tetap dan tidak muncul di Setup, supaya pengguna hanya
mengisi yang memang harus diukur. Alasan tiap nilai tertulis di
`server/tracker.py` (`PARAMS`).

| Parameter | Nilai | Arti |
|---|---|---|
| `sigma_a` | 0,8 m/s² (0,05 saat diam) | Seberapa cepat troli boleh berubah kecepatan |
| `sigma_r` | 0,12 m | Derau satu jarak |
| `vel_tau_s` | 1,0 s | Berapa lama kecepatan dipertahankan saat data berhenti |
| `huber_k` / `huber_k_long` / `gate` | 2,5 / 1,0 / 6 | Mulai turunkan bobot (jarak pendek / jarak panjang) / tolak sampel |
| Mode diam | masuk < 0,2 m/s selama 0,4 s; keluar 3 dari 4 \|e\| > 1,5 | `ZUPT_*` di `tracker.py` |
| `still_long` | 1,0σ | Ambang buang jarak kepanjangan saat diam |
| `min_anchors`, `fresh_ms`, `valid_sigma` | 3, 1500 ms, 0,5 m | Gerbang validitas |
| `reinit_ms`, `room_margin` | 1500 ms, 0,5 m | Mulai ulang tanpa data / batas ruangan |
| `smooth_window`, `window_ms` | 3 posisi, 4000 ms | Ukuran rata-rata "Smooth the path" |

Yang bisa diubah pengguna:

| Pengaturan | Di mana | Catatan |
|---|---|---|
| Ukuran ruangan, tinggi tag `tag_z` | Setup → Room | Ukur dengan meteran; salah ukur `tag_z` menggeser semua posisi |
| Posisi dan tinggi anchor | Live → Inspector (klik anchor) | Offset > 20 cm memunculkan peringatan "periksa posisi" |
| Titik kalibrasi | Setup → Calibration | Taruh tag di titik yang diukur, isi x, y, lalu Apply |
| Haluskan jalur | Setup → Tracker | Satu centang, lihat §6 |

## Peta kode

| Bagian | File / fungsi |
|---|---|
| Nilai tetap + alasannya | `server/tracker.py` `PARAMS`, `tracking_params` |
| EKF, mode diam, tolak jarak kepanjangan | `tracker.py` `TagFilter` (`predict`, `update`, `_standing_mode`) |
| Kalibrasi atau offset dipelajari | `tracker.py` `Engine.correct`, `Engine._offset_policy` |
| Inisialisasi dengan pencarian offset | `tracker.py` `multilaterate_with_offset`, `Engine._bootstrap` |
| Haluskan jalur | `tracker.py` `Engine._reported` |
| Gerbang validitas, posisi ditahan | `tracker.py` `Engine.estimate` |
| Satu-satunya pengaturan (`smooth`) | `server/app.py` `api_tracking` |
| `valid` untuk navigasi | `server/app.py` `_nav_position_doc` (`NAV_STALE_MS`) |
| Tampilan 3D, kondisi anchor, istilah | `server/static/app.js` (`buildRoom`, `anchorStatus`, `TIP`) |
| Varian riset (jendela, C, B/R, dst.) | `tools/tracker_lab.py` `LabEngine` (hanya dipakai `tools/`) |
| Studi semua kombinasi | `tools/window_study.py` + `tools/window_report.py` → `docs/hasil-simulasi/` |
| Studi simulasi awal (A–G) | `tools/smoothing_study.py` → `docs/smoothing_study.json` |
| Putar ulang rekaman | `tools/evaluate.py LOG --replay` (algoritma server) atau `--set …` (varian riset) |

---

## Istilah

| Istilah | Arti |
|---|---|
| **Jarak (range)** | Jarak tag ke satu anchor hasil UWB, garis lurus 3D (termasuk beda tinggi) |
| **Offset jarak (b)** | Kelebihan jarak yang sama di semua anchor karena delay antena DW1000 (±75–100 cm). Dihitung otomatis dan dikurangkan. Dulu disebut "bias" |
| **Offset anchor (d)** | Selisih satu anchor terhadap anchor lain (beberapa cm). Lebih dari 20 cm = posisi anchor di denah kemungkinan salah |
| **Margin error (±σ)** | Ketidakpastian posisi menurut EKF: ±2 dari 3 posisi berada dalam radius ini. Bukan galat sebenarnya; galat dari posisi anchor yang salah ketik tidak termasuk |
| **Residu** | Jarak terukur − jarak yang diharapkan dari posisi dan offset. Mendekati 0 = anchor itu sepakat dengan yang lain |
| **e (residu ternormalisasi)** | Residu dibagi simpangan yang diharapkan; dipakai untuk menolak / menurunkan bobot sampel |
| **LOS / NLOS** | Jalur bebas / terhalang |
| **Presisi** | Seberapa rapat posisi saat diam (tidak butuh posisi benar) |
| **Akurasi** | Seberapa dekat posisi dengan posisi benar (butuh meteran laser atau simulasi) |
| **MAE** | Rata-rata galat (*mean absolute error*) |
| **RMSE** | Seperti MAE, tetapi galat besar lebih berpengaruh |
| **CEP50 / CEP95** | Radius yang memuat 50% / 95% posisi |
| **Kegoyahan** | Seberapa patah-patah garis (RMS beda-kedua posisi berurutan) |
| **Jendela (window)** | Sekelompok sampel terakhir yang dirata-rata / diambil mediannya |
| **HOLD** | Posisi ditahan karena data kurang (< 3 anchor); ditandai tidak valid |
| **EKF** | Extended Kalman Filter: menggabungkan model gerak troli dengan setiap jarak baru |
| **Huber** | Cara menurunkan bobot sampel yang menyimpang tanpa membuangnya |
| **Model Singer** | Model gerak di mana kecepatan pelan-pelan meluruh bila tidak ada data baru |

## Rujukan singkat

| Bagian | Rujukan |
|---|---|
| EKF untuk UWB | Yao 2021, Krebs 2024 |
| Huber | Huber 1964 |
| Model Singer | Singer 1970 |
| Offset per anchor (self-calibration) | Hapsari 2025 (paper 09) |
| Bobot asimetris NLOS | Fan & Du 2022 (NLOS selalu memperpanjang jarak) |
| Rata-rata saat diam | Ide *zero-velocity update* (Foxlin 2005) |
| Jendela, Nyquist, garis regresi | Shannon 1949, Savitzky–Golay 1964 |
| Data sebelum & sesudah | Rauch–Tung–Striebel 1965 |
| Penempatan anchor | Kramarić 2025 |

Detail dan bagian mana yang didukung tiap rujukan ada di
[`algorithm-deep-understanding.md`](algorithm-deep-understanding.md#12-rujukan).
