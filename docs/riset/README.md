# Riset dan pembanding

Semua yang ada di sini adalah **riset**: varian pembanding, sejarah perbaikan,
dan ide yang **tidak dijalankan server**. Algoritma yang dipakai, CALF-UWB,
dijelaskan di [`../ALGORITHMS.md`](../ALGORITHMS.md). Dokumen ini dipisah
supaya dokumen utama hanya berisi yang benar-benar dipakai.

## Daftar dokumen riset

| Dokumen | Isi |
|---|---|
| [`RISET_LANJUTAN.md`](RISET_LANJUTAN.md) | Improved TWR (koreksi kuat sinyal, tanda NLOS dari jalur pertama, laju ranging ×2) dan AoA, diuji di atas CALF-UWB |
| [`../hasil-simulasi/RISET_JENDELA.md`](../hasil-simulasi/RISET_JENDELA.md) | Studi semua kombinasi inti EKF × jendela pada troli robot di antara pembeli; asal pilihan K + G + Z + L |
| [`../hasil-simulasi/RISET_AKURASI.md`](../hasil-simulasi/RISET_AKURASI.md) | Riset akurasi 2–4 cm: bayangan badan, IMU, anchor tambahan |
| [`../algorithm-deep-understanding.md`](../algorithm-deep-understanding.md) | Penurunan rumus, studi windowing, pembahasan catatan bimbingan |
| [`../PERBAIKAN_AKURASI.md`](../PERBAIKAN_AKURASI.md) | Penulisan ulang pipeline server (Oktober 2026) |

## Kode riset (tidak dipakai server)

| File | Isi |
|---|---|
| `tools/tracker_lab.py` | `LabEngine`: tracker server dengan semua nilai studi bisa diubah (jendela, offset, mode diam) |
| `tools/riset_lanjutan.py` | Studi improved TWR dan AoA → `docs/riset/` |
| `tools/ranging_rate.py` | Laju ranging hardware dari rekaman (jarak/anchor/detik, putaran, jarak mustahil) |
| `lib/DW1000` `useRobustRound`, firmware 1.0.37 | Ronde ranging robust di tag (eksperimen, default mati; `server/app.py --robust-round`) |
| `tools/window_study.py`, `tools/window_report.py` | Studi semua kombinasi jendela → `docs/hasil-simulasi/` |
| `tools/accuracy_lab.py`, `tools/accuracy_report.py` | Riset akurasi (bayangan badan B/R, IMU, anchor tambahan) |
| `tools/smoothing_study.py`, `tools/sim_report.py` | Studi awal A–G |
| `tools/simulate.py` | Pipeline lama vs baru |
| `tools/replay_compare.py` | Rekaman asli diputar ulang dengan tiap varian |
| `tools/walk_test.py`, `tools/web_walk_video.py`, `tools/sim_feed.py` | Video dan simulasi uji jalan |

---

## Sejarah: dari pipeline lama sampai CALF-UWB

### Pipeline lama vs baru (Oktober 2026)

Simulasi 30 run, galat radio sama untuk semua pipeline (`tools/simulate.py`):

| Pipeline | RMSE | Galat lintasan (XT-RMSE) |
|---|---|---|
| Lama (server `1aff68c` + FW 1.0.31) | 144 cm | 53 cm |
| Baru + FW ≥ 1.0.32 | 23 cm | 13 cm |
| Baru + FW ≥ 1.0.32 + kalibrasi | 21 cm | 11 cm |

### Studi awal A–G

Ruangan 9,5 × 5,9 m, laju anchor seperti hardware, 10 kali Monte-Carlo
(`tools/smoothing_study.py`, gambar di [`../hasil-simulasi/`](../hasil-simulasi/)).

| Algoritma | Galat bergerak (MAE) | Galat diam (MAE) | Sebaran diam (CEP50) | Kegoyahan garis | Galat maks saat anchor hilang |
|---|---|---|---|---|---|
| A. EKF saja (sebelum) | 20,6 cm | 18,2 cm | 6,6 cm | 21,2 cm | 85 cm |
| B. A + rata-rata 500 ms | 22,8 cm | 18,0 cm | 6,3 cm | 15,8 cm | 84 cm |
| C. A + jendela 20 × 20 (catatan bimbingan) | 228 cm | 170 cm | 50 cm | 3,9 cm | 298 cm |
| D. A + median 3 pada jarak | 42,2 cm | 15,8 cm | 4,0 cm | 15,8 cm | 87 cm |
| E. A + garis regresi 10 pada posisi | 25,2 cm | 24,7 cm | 11,0 cm | 10,0 cm | 92 cm |
| F. A + jendela adaptif + gerbang | 20,8 cm | 17,5 cm | 3,5 cm | 20,2 cm | 50 cm |
| G. F + offset per anchor + bobot jarak panjang | 17,3 cm | 15,5 cm | 2,0 cm | 15,4 cm | 44 cm |

Kondisi line of sight (tanpa halangan tubuh), saat diam:

- G tanpa kalibrasi manual: 5,0 cm, lalu 4,5 cm setelah tiga putaran karena
  offset tiap anchor makin terpelajari.
- G dengan kalibrasi: 4,7 cm.

Rekaman asli diputar ulang lewat versi sebelum G dan G:

| Rekaman | Sebelum | G |
|---|---|---|
| `statis-final` (diam), CEP50 / CEP95 | 5,1 / 12,0 cm | 1,6 / 4,3 cm |
| `statis-malam-20mnt` (diam) | 6,1 / 16,1 cm | 2,1 / 5,8 cm |
| `verifikasi-final` (diam) | 4,8 / 11,6 cm | 1,2 / 4,2 cm |
| `test-1` (bergerak), lompatan > 40 cm / kegoyahan | 47 / 32,6 cm | 28 / 20,3 cm |
| `test-2` (bergerak) | 49 / 32,8 cm | 8 / 11,6 cm |
| `test-4` (bergerak) | 46 / 27,9 cm | 30 / 21,3 cm |

### Dari G ke CALF-UWB

G masih memakai "jendela adaptif": rata-rata sampai 20 keluaran saat troli
diam. Studi semua kombinasi pada troli robot di antara pembeli
([RISET_JENDELA.md](../hasil-simulasi/RISET_JENDELA.md)) menggantinya dengan
mode diam (Z), tolak jarak kepanjangan (L), dan kalibrasi statis (K):

| Algoritma (dunia troli robot) | Bacaan di rak: kosong / sepi / ramai |
|---|---|
| G | 4,4 / 6,7 / 10,0 cm |
| G + Z + L (tanpa kalibrasi) | 3,5 / 4,7 / 7,4 cm |
| **K + G + Z + L = CALF-UWB** | **3,0 / 3,4 / 5,4 cm** |
| C / G + C (jendela 20) | ±63 cm |

### Jendela (tidak dipakai server)

- **Jendela pada jarak:** median / mean / garis regresi N sampel sebelum EKF.
- **Jendela pada posisi:** termasuk rata-rata 20 × 20 versi catatan bimbingan
  (varian C).
- **Kenapa tidak dipakai:**
  - pada laju ±2,5 sampel per anchor per detik, jendela N sampel menunda
    ±(N−1)/5 detik;
  - di simulasi varian C meleset ±63 cm.
- Kodenya ada di `tools/tracker_lab.py` (`window_fit`, `LabEngine`) supaya hasil
  studi bisa diulang.
- Yang tersisa di server hanya "Smooth the path": rata-rata 3 posisi terakhir
  (geser-1).

---

## Istilah riset

Istilah yang hanya muncul di riset. Istilah yang dipakai sistem ada di
[`../ALGORITHMS.md`](../ALGORITHMS.md#istilah).

| Istilah | Arti |
|---|---|
| **A–F** | Versi awal sebelum G (A = EKF saja, F = + jendela adaptif + gerbang) |
| **C** | Jendela rata-rata 20 × 20 dari catatan bimbingan |
| **B / R** | Model bayangan badan: badan selalu di belakang troli / dari arah acak |
| **Jendela adaptif** | Rata-rata sampai 20 keluaran sejak troli berhenti (varian G) |
| **Geser-5 (*hopping*)** | Jendela maju lima sampel tiap kali |
| **Tunggu-N (*tumbling*)** | Jendela tidak tumpang tindih: 1–20, 21–40, … |
| **SS-TWR** (*Single-Sided TWR*) | Satu kali kirim-balas. Cepat, tetapi peka terhadap beda kecepatan jam |
| **DS-TWR** (*Double-Sided TWR*) | Dua kali kirim-balas, galat beda jam saling membatalkan. ADS-TWR yang dipakai firmware adalah salah satu bentuknya |
| **Improved TWR** | Sebutan umum perbaikan TWR (koreksi beda jam, koreksi kuat sinyal, laju lebih tinggi) |
| **RSL / `rx_power`** (*Received Signal Level*) | Kuat sinyal yang diterima (dBm). Dikirim firmware, tidak dipakai CALF-UWB |
| **FP power / `fp_power`** (*First Path*) | Kuat sinyal pada jalur pertama (jalur langsung). Dikirim firmware, tidak dipakai CALF-UWB |
| **dfp** | `rx_power − fp_power` dikurangi nilai tengahnya untuk anchor itu; besar = jalur pertama lemah |
| **CIR** (*Channel Impulse Response*) | "Rekaman" sinyal yang tiba lewat berbagai jalur; bahan riset NLOS |
| **Multipath** | Sinyal yang tiba lewat pantulan, bukan jalur langsung |
| **ToA / TDoA** | *Time of Arrival* / *Time Difference of Arrival*: cara lain memakai waktu tiba sinyal |
| **AoA** (*Angle of Arrival*) | Arah datang sinyal |
| **PDoA** (*Phase Difference of Arrival*) | Beda fase sinyal di dua antena; dari sini AoA dihitung |
| **DW3000** | Generasi chip UWB berikutnya; versi dua antena bisa mengukur AoA |
| **MAE** (*Mean Absolute Error*) | Rata-rata galat; dipakai di tabel studi awal A–G |
| **QoS** (*Quality of Service*) | Tingkat jaminan pengiriman MQTT; firmware memakai 0 (tanpa kirim ulang) |

## Rujukan riset

| Bagian | Rujukan |
|---|---|
| Jendela, Nyquist, garis regresi | Shannon 1949, Savitzky–Golay 1964 |
| Data sebelum & sesudah | Rauch–Tung–Striebel 1965 |
| Koreksi beda jam dan kuat sinyal | Sidorenko dkk. 2019 ([arXiv:1902.11085](https://arxiv.org/abs/1902.11085)); Decawave APS011 |
| NLOS dari daya jalur pertama | [arXiv:2403.19706](https://arxiv.org/abs/2403.19706) |
| AoA dengan PDoA | [arXiv:2312.13672](https://arxiv.org/abs/2312.13672) (ETH Zürich, ±2,4° rata-rata) |
