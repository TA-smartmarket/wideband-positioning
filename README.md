# 📡 ESP32 UWB Positioning — Makerfabs ESP32 UWB Pro with Display

Real-time indoor **tag tracking**: boards placed in the room corners
(**anchors**) measure the distance to a moving **tag**, and a server solves
the tag's 2D position, displayed live on a web map.

**One firmware runs on every board.** Role (`tag`/`anchor`) and ID (1–10) are
chosen at runtime — from the web UI, a setup Wi-Fi portal, or the serial
menu. The UWB address is generated automatically, so there is **no manual
address editing anywhere**.

```
┌──────────┐  UWB range   ┌──────────┐  REST/MQTT   ┌────────────────────┐
│ anchor-1 │◀────────────▶│   tag-1  │─────────────▶│  server (Flask)    │
│  (0,0)   │              │ (moves)  │              │  solver → (x,y)    │
└──────────┘              └──────────┘              │  web UI live map   │
┌──────────┐                                        └────────────────────┘
│ anchor-2 │
│ (W,0)    │
└──────────┘
```

---

## ✅ Fitur

- **Satu firmware untuk semua board** — role & ID (1–10) dipilih runtime
  (web UI / setup portal / serial menu), disimpan di NVS.
- **EUI & short address otomatis** dari role+ID — tidak perlu edit kode.
- **Konfigurasi dari server** — WiFi, MQTT, posisi anchor (x, y dalam meter),
  ukuran ruangan, mode UWB — device mengambilnya via REST poll **dan** MQTT
  push (retained, jadi config tidak hilang saat device restart).
- **2 jalur data** — REST **dan** MQTT; MQTT dipakai kalau ada, REST fallback.
- **Position solver 2D** — di server (dan fallback di tag standalone) dari
  ≥2 jangkauan anchor, dengan disambiguasi batas ruangan.
- **Web UI** — live map (anchor, tag, garis jarak), tabel tag, form setup.
- **Multi-device** — sampai 10 tag + 10 anchor per site.

---

## 🧰 Hardware & Software

### Hardware
| Item | Jumlah | Catatan |
|---|---|---|
| Makerfabs ESP32 UWB Pro with Display (DW1000) | **3×** | 2 anchor + 1 tag (bisa lebih) |
| Kabel USB-C (data, bukan cuma charge) | 3× | Untuk flash & serial |
| Router / Wi-Fi | 1 | Board + server harus satu jaringan |

### Software
| Tool | Cara dapat |
|---|---|
| **PlatformIO** | VS Code + ekstensi *PlatformIO IDE*, atau CLI: `pip install platformio` |
| **Python 3.9+** | Untuk server (CLI install butuh pip) |
| **(Opsional) MQTT broker** | Mosquitto — kalau mau pakai jalur MQTT (FTP: Windows & Ubuntu punya paket `mosquitto`) |

Semua library (ESP32 Arduino framework, Adafruit SSD1306/GFX, PubSubClient,
ArduinoJson) diunduh otomatis oleh PlatformIO saat build pertama.

---

## 📁 Struktur Project

```
wideband-positioning/
├── platformio.ini        # 1 env: esp32uwb (partisi min_spiffs: app0 + app1)
├── src/
│   ├── main.cpp          # firmware: ranging, OLED UI, serial menu, portal AP,
│   │                     #          REST+MQTT, terapkan config dari server
│   ├── config.h          # model config + NVS + derivasi EUI + anchor map
│   ├── net.h             # WiFi, HTTP client, MQTT (PubSubClient), setup portal
│   ├── ekf.h             # Extended Kalman Filter 2D (tracking tag)
│   ├── solver.h          # multilaterasi 2D (fix pertama / mode standalone)
│   └── ota.h             # OTA: pull dari server + web updater (push)
├── lib/
│   └── DW1000/           # library Makerfabs DW1000 (di-vendor, include guard difix)
├── docs/
│   ├── API.md            # kontrak API/MQTT + model payload
│   ├── ARCHITECTURE.md   # cara kerja sistem + trade-off desain
│   ├── ALGORITHMS.md     # algoritma inti (EKF + kalibrasi + mode diam + tolak jarak kepanjangan)
│   ├── algorithm-deep-understanding.md  # detail, studi windowing, catatan bimbingan, rujukan
│   ├── hasil-simulasi/   # gambar + Excel perbandingan algoritma (tools/sim_report.py)
│   ├── Jurnal_UWB_Indoor_Positioning.docx   # tabel review 10 jurnal
│   └── papers/           # PDF 10 paper rujukan + README berisi link
├── tools/                # build_firmware.py (build + staging OTA), ota_debug.py,
│                         # simulate.py (sebelum/sesudah), smoothing_study.py (studi algoritma),
│                         # sim_report.py (gambar + Excel hasil simulasi),
│                         # evaluate.py (analisis rekaman), report_excel.py (laporan Excel),
│                         # tracker_lab.py (varian algoritma riset, tidak dipakai server),
│                         # window_study.py + window_report.py (studi semua kombinasi)
└── server/
    ├── app.py            # Flask: REST + MQTT ingest + web UI + kalibrasi + rekam
    ├── tracker.py        # engine tracking: dedup sampel, clock sync, algoritma final (EKF)
    ├── recorder.py       # perekam sesi (server/logs/*.jsonl)
    ├── scene.py          # geometri ruangan, penghalang, klasifikasi LOS/NLOS
    ├── static/           # web UI 3D (Three.js di-vendor di static/vendor/)
    ├── firmware/         # gambar OTA yang sudah di-build
    ├── requirements.txt  # flask, paho-mqtt
    ├── README.md         # panduan server (ringkas)
    └── mosquitto.test.conf
```

---

## 🚀 Setup Lengkap

### Langkah 1 — Install PlatformIO

**VS Code**: buka *Extensions* → cari *PlatformIO IDE* → *Install* → reload.
**CLI**:

```bash
pip install platformio
```

### Langkah 2 — Jalankan Server

Munculkan terminal di folder `server/`:

```bash
cd server
pip install -r requirements.txt
python app.py
```

Server jalan di **`http://0.0.0.0:8080`**. Buka di browser:
`http://127.0.0.1:8080` (mesin sendiri) — atau `http://<IP-LAN-mesin>:8080`
kalau diakses board/HP lain dalam jaringan.

**Opsi server:**

```bash
python app.py --port 9000              # port beda
python app.py --token rahasia123       # proteksi API dengan bearer token
python app.py --mqtt-host 192.168.1.20 # host broker MQTT (default 127.0.0.1)
python app.py --mqtt-port 1883         # port broker
python app.py --mqtt-base uwb/ruangku  # base topic MQTT
python app.py --mqtt 0                 # nonaktifkan MQTT (khusus REST)
```

### Langkah 3 — Build & Flash Firmware

Dari folder root project:

```bash
pio run                     # build saja
pio run -t upload           # build + upload ke board yang tersambung
```

Flash **ketiga board dengan firmware yang sama**. Kalau beberapa port
tersambung sekaligus, pilih port:

```bash
pio run -t upload --upload-port COM5
```

> **Gagal di "Connecting..."?** Tahan **BOOT**, tekan **RST/EN** sebentar,
> lepas BOOT, upload, lalu tekan RST sekali biar sketch jalan.

### Langkah 4 — Konfigurasi Tiap Board

Board belum dikonfigur → OLED menampilkan `NOT CONFIGURED`. Ada **3 cara**:

#### 4a. Lewat Web UI (paling gampang) ✅
1. Board terhubung Wi-Fi (WiFi & server URL sudah diset sewaktu pertama kali;
   kalau belum, pakai 4b atau 4c dulu satu kali).
2. Buka `http://<IP-server>:8080` → form **Setup**.
3. Contoh Konfigurasi Anchor dan Tag di ruangan:
   - **Anchor 1** → role `anchor`, ID `1`, Position X `0`, Position Y `0`
     (taruh board di pojok ruangan), Room width/height → misal `5` × `4`.
   - **Anchor 2** → role `anchor`, ID `2`, Position X `5` (lebar ruangan),
     Position Y `0` (pojok seberang).
   - **Tag** → role `tag`, ID `1`.
   - WiFi SSID/password & Server URL diisi sekali (sama untuk semua board).
4. Klik **💾 Save**. Server push config via MQTT → device terapkan & reboot.

#### 4b. Lewat Serial Menu
Flash dulu, buka monitor:

```bash
pio device monitor           # baud otomatis 115200
```

Ketik `?` untuk daftar semua perintah:

```
show                       lihat config saat ini
role tag|anchor            set role
id <1-10>                  set ID device
site <nama>                nama site/lokasi
wifi <ssid> <pass>         set WiFi
server <url>               mis. server http://192.168.1.10:8080
mqtt <host> [port]         aktifkan MQTT (port default 1883)
mqtt off                   nonaktifkan MQTT
base <topic>               base topic MQTT (default uwb/home)
pos <x> <y> [z]            posisi anchor (meter) — khusus anchor
room <w> <h>               ukuran ruangan (meter)
mode <nama>                mode PHY UWB (lihat 'show')
filter on|off              filter smoothing jarak
rate <ms>                  interval telemetry
save                       simpan ke NVS
portal [off]               buka/tutup setup portal (sama dengan tahan BOOT 3 s)
reboot                     restart
reset                      hapus config & reboot
```

Contoh minimal:

```
role anchor
id 1
wifi NamaWifi Password123
server http://192.168.1.10:8080
pos 0 0
room 5 4
save
```

#### 4c. Lewat Setup Portal (AP)
Board membuka **AP `UWB-Setup-XXXX`** (4 digit unik per board, tertulis di
OLED) bila:
- belum ada config sama sekali (pertama boot), atau
- Wi-Fi yang tersimpan tidak bisa disambungi selama ~20 detik
  (mis. pindah ke jaringan lain), atau
- dibuka manual: **tahan tombol BOOT 3 detik**, atau ketik `portal` di serial
  (berguna untuk memindah satu board ke Wi-Fi baru tanpa mematikan yang lain).

1. HP/laptop → Wi-Fi → join **UWB-Setup-XXXX** sesuai OLED board itu.
2. Buka **`http://192.168.4.1`** → form serupa dengan web UI → isi → Save.
3. Board reboot dengan config baru.

**Semua board boleh dinyalakan bersamaan** (firmware ≥ 1.0.35). Selama portal
terbuka, ranging UWB board itu dijeda dan board berhenti mencoba Wi-Fi lama
selama ada HP tersambung ke portal. Sebelumnya, percobaan konek ulang tiap
10 detik membuat access point pindah kanal sehingga HP terus terputus, dan tag
yang sedang ranging membuat anchor crash (≤ 1.0.32), sehingga setup harus
dilakukan satu per satu. Portal manual menutup sendiri setelah Save,
`portal off`, atau 10 menit.

### Langkah 5 — Lihat Tracking-nya

Buka web UI server. Tag muncul di peta ruangan dan bergerak real-time sesuai
gerakanmu. Tiap tag menampilkan koordinat, confidence, dan jarak ke tiap
anchor. Anchor abu-abu = offline.

---

## 🔢 Skema ID & Alamat UWB (otomatis)

Role+ID → EUI & short address (2 byte pertama = short address):

| Role | ID | EUI | Short address |
|---|---|---|---|
| anchor | 1 | `01:A0:5B:D5:A9:9A:E2:9C` | `0xA001` |
| anchor | 2 | `02:A0:5B:D5:A9:9A:E2:9C` | `0xA002` |
| anchor | 10 | `0A:A0:5B:D5:A9:9A:E2:9C` | `0xA00A` |
| tag | 1 | `01:7D:00:22:EA:82:60:3B` | `0x7D01` |

Log/telemetry memakai label `anchor-2`, `tag-1`, dst.

> **Penting**: tiap board wajib punya **ID unik**. 2 anchor di dua sudut ruangan
> + tag sudah cukup. Maks 10 anchor + 10 tag per site.

---

## 🌐 REST API

Semua body JSON. Base URL: `http://<server>:8080`.

| Method | Path | Fungsi |
|---|---|---|
| `POST` | `/api/v1/telemetry` | device kirim data jarak/status |
| `GET` | `/api/v1/config/device?role=anchor&id=1` | device ambil config-nya |
| `PUT` | `/api/v1/config` | web UI/API simpan config |
| `GET` | `/api/v1/state` | state dunia (tag, anchor, links) |
| `GET` | `/api/v1/devices` | daftar device yang dikenal |
| `POST` | `/api/v1/position` | paksa hitung ulang posisi (debug) |
| `GET` | `/` | web UI |

Contoh (curl):

```bash
# set anchor 1 di pojok (0,0), ruangan 5×4 m
curl -X PUT localhost:8080/api/v1/config -H 'Content-Type: application/json' \
     -d '{"role":"anchor","id":1,"position":{"x":0,"y":0},"room":{"width":5,"height":4}}'

# set anchor 2 di (5,0)
curl -X PUT localhost:8080/api/v1/config -H 'Content-Type: application/json' \
     -d '{"role":"anchor","id":2,"position":{"x":5,"y":0}}'

# kirim jarak dari anchor-1 ke tag-1
curl -X POST localhost:8080/api/v1/telemetry -H 'Content-Type: application/json' \
     -d '{"device_id":"anchor-1","ranges":[{"src":"anchor-1","dst":"tag-1","range":2.236,"rx_power":-60}]}'

# lihat hasil
curl localhost:8080/api/v1/state
```

Respons `state`:

```json
{
  "site": "home",
  "ts": 1727880000123,
  "room": {"width": 5, "height": 4},
  "anchors": [{"id": "anchor-1", "x": 0, "y": 0, "online": true, "last_seen": 1727880000123}],
  "tags": [{"id": "tag-1", "x": 2.0, "y": 1.0, "confidence": 0.8, "ambiguous": false,
            "online": true, "last_seen": 1727880000123,
            "ranges": {"anchor-1": 2.236, "anchor-2": 3.162}}],
  "links": [{"src": "anchor-1", "dst": "tag-1", "range": 2.236, "rx_power": -60, "ts": 1727880000123}]
}
```

Detail lengkap & model payload: [`docs/API.md`](docs/API.md).

---

## 📡 MQTT

Base topic default: `uwb/home` (bisa diganti lewat `base <topic>` / `--mqtt-base`).

| Topic | Arah | Payload |
|---|---|---|
| `uwb/home/telemetry` | device → server | batch telemetry |
| `uwb/home/status/<device_id>` | device → server | status online (retained) |
| `uwb/home/config/<device_id>` | server → device | config push (retained) |
| `uwb/home/cmd/<device_id>` | server → device | perintah (`{"cmd":"reboot"}`) |
| `uwb/home/state` | server → semua | state dunia (retained) |

Uji cepat:

```bash
# subscribe semua topic
mosquitto_sub -h 127.0.0.1 -t 'uwb/home/#'

# kirim jarak tiruan
mosquitto_pub -h 127.0.0.1 -t 'uwb/home/telemetry' -m \
 '{"device_id":"anchor-1","ranges":[{"src":"anchor-1","dst":"tag-1","range":2.1,"rx_power":-58}]}'
```

Device pakai MQTT kalau `mqtt on` dan broker terjangkau; kalau tidak, otomatis
fallback ke REST.

---

## 🔋 Hemat daya: layar OLED node

Layar OLED adalah konsumen daya terbesar yang selalu menyala di board. Sekarang
bisa dimatikan **tanpa memutus apa pun** — node tetap ranging, tetap melayani
web UI, dan tetap mengirim telemetri.

Tiga mode:

| Mode | Perilaku |
|---|---|
| `always` | layar selalu menyala |
| `dim` | **redup** (kontras rendah) setelah idle — default |
| `off` | **mati total** (panel off, `SSD1306_DISPLAYOFF`) setelah idle |

Timeout idle diatur dalam detik (`0` = tidak pernah tidur, default 60 s).
Layar **otomatis bangun** saat ada aktivitas: input serial, jarak baru terukur,
atau pesan MQTT/REST.

**Cara mengatur:**

1. **Web UI** → tab **Setup → Device screen (OLED)** → pilih mode + timeout →
   **Apply to all devices**. Dikirim sebagai config biasa (retained MQTT), jadi
   node menerapkannya sendiri.
2. **Serial menu:**
   ```
   screen on            layar selalu nyala
   screen dim           redup setelah timeout
   screen off           mati setelah timeout
   screen auto 120      ubah timeout idle jadi 120 detik
   ```
3. **REST:** `PUT /api/v1/config` dengan `{"role":"anchor","id":2,"display":{"mode":"off","timeout_s":120}}`

---

## 🔄 OTA (update firmware dari server)

Firmware **tidak perlu dicabut** lagi. Setiap node menjalankan web updater
sendiri begitu tersambung jaringan:

```
POST http://<ip-node>:3232/update?key=<key-per-device>   (multipart, .bin)
```

**Model keamanan** — server membuat **key unik per device**
(`secrets.token_urlsafe`), menyimpannya, lalu mengirimkannya ke node lewat
channel config yang biasa. Node **menolak** upload apa pun tanpa key itu, jadi
host lain di LAN tidak bisa menulis ulang firmware. Key tersimpan di NVS;
`reset` pada serial menghapusnya sekaligus menonaktifkan OTA sampai server
memprovision ulang.

Dari web UI → tab **Setup → OTA firmware**:

1. Taruh image di `server/firmware/` (mis. `uwb-node-1.0.0.bin`).
   Folder ini di-`.gitignore` karena isinya binary besar.
2. Pilih image → **⬆ Push to all online** (atau tombol **OTA** per device di
   tab **Devices**).
3. Node menerima, menulis partisi OTA, lalu reboot. Progress terlihat di panel
   hasil.

Lewat CLI:

```bash
# lihat device + key-nya
curl -s localhost:8080/api/v1/ota | python -m json.tool

# push ke satu device (atau "all")
curl -X POST localhost:8080/api/v1/ota/push -H 'Content-Type: application/json' \
     -d '{"device_id":"anchor-2","firmware":"uwb-node-1.0.0.bin"}'

# rotasi key (device ambil key baru saat config sync berikutnya)
curl -X POST localhost:8080/api/v1/ota/key -H 'Content-Type: application/json' \
     -d '{"device_id":"anchor-2"}'
```

Atau langsung ke node (berguna saat server mati):

```bash
curl -F "firmware=@firmware.bin" "http://192.168.0.109:3232/update?key=<key>"
```

Halaman status node juga tersedia di `http://<ip-node>:3232/`.

> Catatan: token di query string bisa tercatat di log. Untuk jaringan yang
> tidak dipercaya, taruh di belakang reverse proxy TLS.

---

## 🖥️ Web UI 3D (editor ruangan + live tracking)

Buka `http://<server>:8080`. Ada **4 tab**:

| Tab | Isi |
|---|---|
| **Live** | inspector objek + daftar tag (posisi, σ, IP, confidence) |
| **Setup** | ukuran ruangan + tinggi tag, kalibrasi, centang *Smooth the path*, tampilan, rekam, **OTA firmware** |
| **Devices** | tiap node: IP, RSSI, firmware, tombol **OTA** + **🔑 key** |
| **API** | daftar endpoint REST (dibaca live dari `/api/v1/meta`) + topik MQTT |

Semua **dinamis**: anchor yang baru masuk otomatis muncul di daftar device,
dapat key OTA sendiri, dan langsung ditempatkan di sudut ruangan yang masih
kosong di 3D view (tinggal digeser ke posisi sebenarnya). Tidak perlu restart
server atau ubah kode.

Tombol di viewport: **⌂** reset view · **3D** / **Top** · **＋** **－** zoom ·
**⏸** pause render (hemat CPU, default aktif) · **🔊** suara · **☀** tema
terang/gelap · **⟲** hapus trail.

Suara (Web Audio, tanpa file aset): blip saat device online, nada turun saat
terputus, chime saat update firmware di-push, dan peringatan saat geometri
NLOS terdeteksi.

Saat halaman dibuka: overlay boot + kamera menyapu masuk dengan satu putaran.

| Aksi | Cara |
|---|---|
| Putar kamera | drag area kosong (1 jari di HP) |
| **Zoom** | scroll (1 notch ≈ 8%) · **＋/－** · pinch di HP · `+`/`-` |
| **Keliling ruangan** | **WASD** / **panah** · `Q`/`E` turun/naik · **Shift** = cepat |
| **Balik ke viewport awal** | tombol **⌂** atau tekan **H** |
| **Pause render** | tombol **⏸** atau **Space** |
| Pindah anchor | drag bola hijau — otomatis dikunci di dalam ruangan |
| Ubah ukuran ruangan | isi Width/Depth/Height di tab Setup |
| Tambah anchor / obstacle | tombol **Anchor** / **Obstacle**, lalu klik lantai |
| Geser obstacle | drag badannya |
| Ubah ukuran obstacle | drag **bola di sudut** (dua sumbu) atau **bola di tepi** (satu sumbu) |
| Ubah tinggi obstacle | drag **cone hijau di atas** |
| Hapus | pilih lalu `Delete` (atau tombol Delete di inspector) |
| Simpan | **💾 Save room & anchors** — langsung di-push ke device via MQTT |

Di HP: **1 jari** = putar, **2 jari** = geser + pinch zoom. Layout otomatis
bertumpuk (viewport di atas, panel di bawah) di layar sempit.

Yang terlihat di scene:

- **Bola hijau** = anchor (redup = offline), dengan tiang setinggi `z`
- **Bola kuning** = tag live + **cincin 1σ** dari kovarians EKF
- **Jejak kuning** = riwayat gerak tag
- **Garis** anchor→tag: **hijau penuh = LOS**, **merah putus-putus = NLOS**
  (terhalang obstacle)
- **Kotak ungu** = obstacle; opacity mengikuti nilai attenuation

### Obstacle benar-benar dihitung (bukan hiasan)

Setiap jalur anchor→tag diuji terhadap obstacle (`segment_hits_box`, slab
method). Kalau terhalang:

1. **σ pengukuran dinaikkan** (`base × (1 + 7·atten)`) → EKF lebih tidak
   percaya pada anchor itu;
2. **bias positif** ditambahkan (UWB NLOS cenderung terbaca lebih jauh);
3. kalau inovasi tetap di luar gerbang 3σ, pengukuran **ditolak** dan filter
   lanjut memakai model gerak.

Toggle **Obstacle-aware NLOS correction** mematikan/menyalakan seluruh
mekanisme ini. Scene disimpan ke `server/scene.json` sehingga restart server
tidak menghilangkan denah ruangan.

### Endpoint scene

| Method | Path | Fungsi |
|---|---|---|
| `GET` | `/api/v1/scene` | denah (room + anchors + obstacles) + status NLOS |
| `PUT` | `/api/v1/scene` | simpan denah; posisi anchor otomatis di-push ke device |

```bash
# contoh: ruangan 5×4×2.7 m, 2 anchor di sudut, 1 dinding di tengah
curl -X PUT localhost:8080/api/v1/scene -H 'Content-Type: application/json' -d '{
  "scene": {
    "room": {"width":5,"depth":4,"height":2.7},
    "anchors": [{"id":"anchor-2","x":0,"y":0,"z":2.2},
                {"id":"anchor-3","x":5,"y":0,"z":2.2}],
    "obstacles": [{"id":"obstacle-1","label":"Wall","x":1.5,"y":1.5,"z":1.35,
                   "sx":3.0,"sy":0.2,"sz":2.7,"rot":0,"atten":1.0}]
  },
  "nlos_enabled": true }'
```

---

## ⚙️ Cara Kerja & Logika Penting

> Pipeline server ditulis ulang pada Oktober 2026 karena lintasan zig-zag.
> Akar masalah, bukti simulasi sebelum/sesudah, dan protokol uji ada di
> [`docs/PERBAIKAN_AKURASI.md`](docs/PERBAIKAN_AKURASI.md).

### 1. Pengukuran (firmware ≥ 1.0.32)
Setiap hasil two-way ranging masuk antrean dan dikirim **sekali** ke server,
lengkap dengan nilai mentah (`raw`), waktu ukur (`ts`, `millis()` device) dan
nomor urut (`seq`). Tag mengirim tiap 200 ms, anchor tiap 1 s.

Filter bawaan library DW1000 **dimatikan sengaja** — itu low-pass yang
menyimpan nilai sebelumnya di dalam `DW1000Device`, jadi satu sampel buruk
diumpan-balik terus (terbukti di hardware: 1,5 m → 248 m). Device masih
menghitung versi ter-filter (gate 5 m + median-3 + EMA 0,35) untuk OLED dan
solver lokal; server memakai nilai mentah.

### 2. Server: setiap sampel sekali, pada waktunya (`server/tracker.py`)
- Sampel di-dedup per link berdasarkan timestamp device (firmware lama yang
  mengirim ulang nilai yang sama juga tertangani).
- Jam tiap device dipetakan ke jam server (offset latensi minimum), dan EKF
  memprediksi ke waktu ukur sebelum update.
- Tag dan anchor sama-sama melaporkan hasil TWR yang sama; salinan tag yang
  dipakai.

### 3. Tracking — **Extended Kalman Filter** dengan bias jarak

```
State     : x = [ px, py, vx, vy, b ]ᵀ     posisi, kecepatan, offset jarak bersama
Gerak     : constant velocity (σa = 0,8 m/s²); b random walk lambat
Pengukuran: h(x) = √((px-ax)² + (py-ay)² + dz²) + b     dz = z_anchor − z_tag
Jacobian  : H = [ (px-ax)/ρ , (py-ay)/ρ , 0 , 0 , 1 ]
Update    : y = z − h(x) ; S = H P Hᵀ + σr² ; K = P Hᵀ/S   (σr = 0,12 m)
            x ← x + K y ; P ← (I−KH) P (I−KH)ᵀ + K σr² Kᵀ   (Joseph)
```

- **Jarak miring langsung** dipakai sebagai pengukuran (tanpa proyeksi
  `√(r² − dz²)` yang memperbesar noise dekat anchor).
- **`b`** = offset yang sama di semua jarak (delay antena DW1000 yang belum
  dikalibrasi). Dengan ≥ 3 anchor ia teramati dan diestimasi online, seperti
  *clock bias* pada GPS.
- **Robust (Huber)**: inovasi ternormalisasi > 2,5 → noise dibesarkan;
  > 6 → ditolak. Tidak ada sampel diterima 1,5 s → inisialisasi ulang.
- **Bootstrap**: multilaterasi Gauss-Newton dari semua anchor.
- **Kalibrasi per anchor** (`bias`, `scale`) dari titik statis yang diukur
  dengan meteran laser — lihat bagian 🎯 di bawah.
- **Confidence** = `1/(1+σ)` dengan `σ = √(P₀₀+P₁₁)`.
- **Mode diam + tolak jarak kepanjangan**: saat troli berhenti kecepatan
  dikunci 0 dan jarak yang > 1σ terlalu panjang (badan pembeli) dibuang.
  Satu-satunya pilihan pengguna: *Smooth the path* (rata-rata 3 posisi).
  Detail dan hasil: [`docs/ALGORITHMS.md`](docs/ALGORITHMS.md).

Hasil simulasi (30 run, error radio sama untuk semua pipeline,
`python tools/simulate.py`):

| Pipeline | RMSE | Galat lintasan (XT-RMSE) |
|---|---|---|
| Lama (server `1aff68c` + FW 1.0.31) | 144 cm | 53 cm |
| Baru + FW ≥ 1.0.32 | 23 cm | 13 cm |
| Baru + FW ≥ 1.0.32 + kalibrasi | 21 cm | 11 cm |

| Tahap | Device (tag) | Server |
|---|---|---|
| pre-filter | ya (OLED / fallback) | — (pakai `raw`) |
| bootstrap | ya (standalone) | ya |
| EKF tracking | ya (fallback, model lama) | **ya (utama)** |

### 4. Tinggi tag — setelan yang paling mudah salah

Radio mengukur jarak **miring 3D** antara anchor di langit-langit dan tag yang
kamu bawa. Posisi dicari di lantai (2D), jadi selisih tinggi masuk ke model:

```
r_3D = √(dx² + dy² + (z_anchor − z_tag)²)
```

`z_anchor` kamu isi di editor 3D. **`z_tag` kamu ukur sendiri** di
**Setup → Tag height above floor** (default 1,3 m) — sistem tidak bisa
menebaknya. Nilainya disimpan sebagai `tag_z` di scene dan dikirim ke device,
supaya tag yang menghitung mandiri memakai angka yang sama dengan server.

**Kenapa ini penting:** salah tinggi 30 cm menggeser tiap jarak ~13 cm, dan
pada dua anchor itu berubah menjadi puluhan sentimeter pada posisi akhir.
Gejalanya khas: posisi **konsisten** terlalu jauh dari anchor, atau tag tampak
"di belakang" padahal secara fisik dekat. Kalau gejalanya seperti itu, curigai
tinggi tag lebih dulu sebelum menuduh radionya.

Sensitivitasnya, dihitung dari range nyata 3,689 m:

| Tinggi tag | Posisi yang dihasilkan |
|---|---|
| 0,4 m | y = 1,52 m |
| 0,6 m | y = 1,73 m |
| **1,3 m (sebenarnya)** | **y = 2,23 m** |
| 1,6 m | y = 2,36 m |

Detail matematis lengkap: [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) §5.

> Karena jarak radio ±10–30 cm noise di dalam ruangan (pantulan), EKF +
> update robust sangat membantu. Kalau posisi masih melompat, tambahkan
> anchor ke-3 (posisi non-kolinear) untuk geometri yang sehat.

---

## 🎯 Kalibrasi & Uji Akurasi

Tanpa motion-capture, ground truth yang bisa dipakai adalah **titik yang
diukur** (uji statis) dan **lintasan yang diketahui** (garis lakban; galat
diukur tegak lurus terhadap garis). Protokol lengkap:
[`docs/PERBAIKAN_AKURASI.md`](docs/PERBAIKAN_AKURASI.md) §4.

1. **Kalibrasi** — Setup → *Calibration & static test*: tag diam di titik yang
   diukur, isi X/Y → **Measure here** (2–4 titik) → **Apply calibration**.
2. **Uji statis** — sama, tapi *Keep as calibration point* tidak dicentang;
   hasilnya mean error, σ, RMSE, CEP50/95.
3. **Uji lintasan** — Setup → *Record session* → jalan di garis lakban → Stop →

   ```bash
   python tools/evaluate.py server/logs/<file>.jsonl --path 0.8,1.0 3.6,4.4 --min-speed 0.1
   ```

   `--replay` memutar ulang paket mentah lewat algoritma server; `--replay --set
   sigma_a=1.2` memakai varian riset (`tools/tracker_lab.py`) dengan parameter
   lain pada data yang sama.

| Method | Path | Fungsi |
|---|---|---|
| `POST` | `/api/v1/calibration/point` | `{"x","y","seconds","store"}` — kumpulkan di titik terukur |
| `POST` | `/api/v1/calibration/apply` | hitung bias/scale per anchor |
| `POST` | `/api/v1/calibration/clear` | hapus kalibrasi |
| `GET`/`PUT` | `/api/v1/tracking` | `GET`: status (`smooth`, `calibrated`) + nilai tetap; `PUT {"smooth": true\|false}`: satu-satunya pilihan |
| `POST` | `/api/v1/record/start` · `stop` · `mark` | rekam sesi ke `server/logs/` |

---

## 🔧 Troubleshooting

| Masalah | Solusi |
|---|---|
| OLED tulisan `NOT CONFIGURED` | Set role+ID (serial `role tag` + `id 1` + `save`, atau AP `UWB-Setup`) |
| Tidak ada jarak sama sekali | Pastikan anchor & tag ID unik, jarak masih jangkauan, power USB cukup |
| Web map kosong | Cek `/api/v1/devices` — device online? Posisi anchor sudah diset? |
| Posisi tag loncat/cermin | 2 anchor → mirror (tanda `ambiguous`). Pastikan kotak ruangan di web sama dengan ruangan asli, sebaiknya anchor tidak sebaris. Tambah anchor ke-3 untuk menghilangkannya |
| Lintasan zig-zag / tertinggal | Pastikan semua board firmware ≥ 1.0.32 (tab Devices) — firmware lama hanya mengirim 1 sampel/detik. Inspector tag harus menunjukkan *Samples* ≈ 20–25/s dengan 4 anchor |
| Lintasan melengkung / melebar | Koordinat atau tinggi anchor di scene tidak sesuai aslinya, atau belum kalibrasi. Lihat *Residual* tiap anchor di inspector tag: yang konsisten besar = anchor yang salah |
| **Posisi tag konsisten terlalu jauh / terlihat "di belakang"** | Tinggi tag salah. Set **Setup → Tag height above floor** sesuai ukuran meteran (jangan ditebak), lalu Save. Salah 30 cm menggeser posisi puluhan cm |
| Server tidak bisa diakses board | Board & server harus satu Wi-Fi; pakai IP LAN (`192.168.x.x`), bukan `127.0.0.1` |
| MQTT tidak terima | Broker harus jalan (`mosquitto -v`); REST tetap jalan tanpa MQTT |
| Flash stuck "Connecting..." | BOOT + RST manual (lihat Langkah 3) |
| Posisi beku | Server drop range lebih 5 detik tanpa data baru (device offline) |
| Boros flash/RAM? | Partisi `min_spiffs` sudah diset (app0+app1, perlu untuk OTA); pakai `mode shortdata_fast_accuracy` kalau mau hemat |

---

## 📚 Dokumentasi Tambahan

- [`docs/API.md`](docs/API.md) — kontrak API/MQTT lengkap + model payload.
- [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) — cara kerja sistem, skema
  alamat UWB, matematika solver, trade-off.
- [`docs/ALGORITHMS.md`](docs/ALGORITHMS.md) — algoritma inti yang berjalan di
  server (EKF + kalibrasi + mode diam + tolak jarak kepanjangan + gerbang validitas),
  diagram alur, hasil, parameter, dan arti setiap istilah.
- [`docs/hasil-simulasi/`](docs/hasil-simulasi/) — gambar dan
  `Hasil_Simulasi.xlsx`: tujuh algoritma pada data simulasi yang sama.
- [`docs/algorithm-deep-understanding.md`](docs/algorithm-deep-understanding.md)
  — penurunan rumus, studi semua kombinasi windowing, pembahasan catatan
  bimbingan (moving average, Nyquist–Shannon, compressed sensing), audit
  perubahan 22-10, dan rujukan per bagian.
- [`docs/Jurnal_UWB_Indoor_Positioning.docx`](docs/Jurnal_UWB_Indoor_Positioning.docx)
  — tabel review 10 jurnal (AUTHOR | JUDUL | TAHUN | METODE | ALGORITMA | ALAT |
  KEKURANGAN | KELEBIHAN) + daftar pustaka APA.
- [`docs/papers/`](docs/papers/README.md) — 10 PDF paper rujukan; README-nya
  memuat tautan unduh tiap paper dan ringkasan relevansinya ke proyek.
- [`server/README.md`](server/README.md) — panduan server saja.

## ⚠️ Catatan & Keterbatasan

- **Bukan WiFi CSI / radar**: radio board ini DW1000 (UWB), bukan WiFi — tidak
  bisa deteksi manusia tanpa tag (fitur CSI ada di board ESP32-S3 + WiFi).
- Akurasi ±10–30 cm indoor; noise karena pantulan radio.
- State server **in-memory**: config device hilang saat server restart
  (device akan ambil ulang config dalam 15 detik setelah online kembali).
- Server dev Flask cocok untuk LAN; untuk produksi pakai gunicorn
  (`gunicorn -w 4 -b 0.0.0.0:8080 app:APP`) di belakang reverse proxy.
