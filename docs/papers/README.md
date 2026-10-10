# Daftar Paper (docs/papers)

Kumpulan paper rujukan untuk proyek indoor positioning UWB (DW1000/ESP32).
Semua paper **open access**, PDF-nya sudah diunduh lokal dan judulnya sudah
diverifikasi satu per satu dengan membaca halaman pertamanya.

Semua tautan di bawah sudah dicoba dan berhasil diunduh (status 200, header `%PDF`).

| # | Berkas | Tahun | Terbitan | Unduh | Halaman |
|---|---|---|---|---|---|
| 1 | `01_Krebs_2024_ESP32_DWM3000_UWB_Positioning.pdf` | 2024 | arXiv:2403.10194 | [PDF](https://arxiv.org/pdf/2403.10194) · [abs](https://arxiv.org/abs/2403.10194) | — |
| 2 | `02_Shalihan_2022_NLOS_NN_Ranging_Mitigation.pdf` | 2022 | arXiv:2206.09607 | [PDF](https://arxiv.org/pdf/2206.09607) · [abs](https://arxiv.org/abs/2206.09607) | — |
| 3 | `03_Angarano_2021_Robust_UWB_Range_Error_DL_Edge.pdf` | 2021 | arXiv:2011.14684 | [PDF](https://arxiv.org/pdf/2011.14684) · [abs](https://arxiv.org/abs/2011.14684) | — |
| 4 | `04_Fan_2022_WLS_RKF_NLOS_Kalman.pdf` | 2022 | arXiv:2205.05939 | [PDF](https://arxiv.org/pdf/2205.05939) · [abs](https://arxiv.org/abs/2205.05939) | — |
| 5 | `05_Bregar_2023_Indoor_UWB_Positioning_Dataset.pdf` | 2023 | Scientific Data 10:744 | [PDF](https://www.nature.com/articles/s41597-023-02639-5.pdf) · [DOI](https://doi.org/10.1038/s41597-023-02639-5) | [PMC10603152](https://pmc.ncbi.nlm.nih.gov/articles/PMC10603152/) |
| 6 | `06_Yao_2021_Indoor_Positioning_Accuracy_UWB.pdf` | 2021 | Sensors 21(17):5731 | [PDF](https://res.mdpi.com/d_attachment/sensors/sensors-21-05731/article_deploy/sensors-21-05731.pdf) · [DOI](https://doi.org/10.3390/s21175731) | [PMC8433727](https://pmc.ncbi.nlm.nih.gov/articles/PMC8433727/) |
| 7 | `07_Kramaric_2025_Anchor_Placement_Localization.pdf` | 2025 | Sensors 25(16):5115 | [PDF](https://res.mdpi.com/d_attachment/sensors/sensors-25-05115/article_deploy/sensors-25-05115.pdf) · [DOI](https://doi.org/10.3390/s25165115) | [PMC12389748](https://pmc.ncbi.nlm.nih.gov/articles/PMC12389748/) |
| 8 | `08_Yang_2024_CIR_Feature_NLOS_Identification.pdf` | 2024 | Sensors 24(5):1703 | [PDF](https://res.mdpi.com/d_attachment/sensors/sensors-24-01703/article_deploy/sensors-24-01703.pdf) · [DOI](https://doi.org/10.3390/s24051703) | [PMC10934496](https://pmc.ncbi.nlm.nih.gov/articles/PMC10934496/) |
| 9 | `09_Hapsari_2025_Modified_DW_Anchor_Self_Calibration.pdf` | 2025 | IJIES 18(5):212–224 | [PDF](https://inass.org/wp-content/uploads/2025/01/2025063016-2.pdf) · [DOI](https://doi.org/10.22266/ijies2025.0630.16) | 212–224 |
| 10 | `10_Hapsari_2025_UWB_Indoor_Tag_Localization_SMS_SLR.pdf` | 2025 | IEEE Access 13:21827–21852 | [PDF](https://ieeexplore.ieee.org/document/10528315) · [DOI](https://doi.org/10.1109/ACCESS.2024.3399476) | 21827–21852 |

Total: 10 PDF (~32 MB + ~3 MB, tidak ikut di-commit — lihat `.gitignore`).

## Catatan tentang tautan

- **MDPI** memblokir permintaan langsung ke `mdpi.com/.../pdf` (403). Tautan
  `res.mdpi.com/d_attachment/...` di tabel ini yang berhasil.
- **Tautan `doi.org` ke MDPI juga 403** kalau diakses otomatis; itu normal
  (MDPI memblokir bot). Di browser biasa tautan DOI-nya tetap terbuka. Untuk
  mengunduh otomatis, pakai kolom **Unduh**.
- **PMC/Europe PMC** juga memblokir unduhan otomatis (403), jadi untuk paper
  MDPI dipakai CDN penerbitnya, dan untuk Bregar 2023 dipakai `nature.com`.

## Ringkasan relevansi ke proyek

| Kelompok | Paper | Dipakai untuk |
|---|---|---|
| Hardware ESP32 + UWB | #1 | Arsitektur paling mirip: ESP32 + modul UWB + EKF |
| Pelacakan (EKF) | #6, #4 | EKF, gate inovasi, uji Mahalanobis untuk menolak pengukuran |
| Penanganan NLOS | #2, #3, #8 | Deteksi NLOS, pembobotan, koreksi galat ranging |
| Geometri anchor | #7 | PDOP, anchor koplanar, pengaruh penempatan pada akurasi |
| Self-calibration anchor | #9 | Konsensus Modified Deffuant-Weisbuch + klasifikasi LOS/NLOS/Multipath untuk posisi anchor otomatis |
| Tren & peta riset | #10 | SMS + SLR, bibliometrik, taksonomi metrik, celah riset UWB indoor tag localization |
| Dataset & protokol | #5 | ADS-TWR, dataset CIR, arsitektur MQTT |

Tabel review lengkap (format 8 kolom): [`../Jurnal_UWB_Indoor_Positioning.docx`](../Jurnal_UWB_Indoor_Positioning.docx)

## Unduh ulang semua PDF

```bash
cd docs/papers
python - <<'EOF'
import re, requests, pathlib
md = pathlib.Path("README.md").read_text(encoding="utf-8")
H = {"User-Agent": "Mozilla/5.0", "Accept": "application/pdf,*/*",
     "Referer": "https://www.mdpi.com/"}
for row in md.splitlines():
    m = re.match(r"\|\s*\d+\s*\|\s*`([^`]+)`.*?\[PDF[^\]]*\]\(([^)]+)\)", row)
    if not m: continue
    name, url = m.group(1), m.group(2)
    r = requests.get(url, headers=H, timeout=120)
    if r.status_code == 200 and r.content[:4] == b"%PDF":
        pathlib.Path(name).write_bytes(r.content)
        print(f"OK   {name}  {len(r.content)//1024} KB")
    else:
        print(f"FAIL {name}  status={r.status_code}")
EOF
```
