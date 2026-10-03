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
├── platformio.ini        # 1 env: esp32uwb (partisi huge_app)
├── src/
│   ├── main.cpp          # firmware: ranging, OLED UI, serial menu, portal AP,
│   │                     #          REST+MQTT, terapkan config dari server
│   ├── config.h          # model config + NVS + derivasi EUI + anchor map
│   ├── net.h             # WiFi, HTTP client, MQTT (PubSubClient), setup portal
│   └── solver.h          # multilaterasi 2D (dipakai tag standalone)
├── lib/
│   └── DW1000/           # library Makerfabs DW1000 (di-vendor, include guard difix)
└── server/
    ├── app.py            # Flask: REST + MQTT ingest + solver + web UI (1 file)
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
3. Isi per board:
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
Saat pertama boot (belum ada config), board membuka **AP `UWB-Setup`**:
1. HP/laptop → Wi-Fi → join **UWB-Setup**.
2. Buka **`http://192.168.4.1`** → form serupa dengan web UI → isi → Save.
3. Board reboot dengan config baru.

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
| `uwb/home/range` | device → server | satu baris jarak |
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
| **Setup** | ukuran ruangan, NLOS, trail, auto-orbit, **OTA firmware** |
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

Pipeline lokalisasi punya **3 tahap** (identik di device dan server):

### 1. Pre-filter (buang sampel liar)
Filter bawaan library DW1000 **dimatikan sengaja** — itu low-pass yang menyimpan
nilai sebelumnya di dalam `DW1000Device`, jadi satu sampel buruk diumpan-balik
terus dan jarak melenceng jauh (terbukti di hardware: 1,5 m → 248 m). Diganti
filter sendiri:
- **outlier gate** — tolak lompatan > 5 m dari estimasi sekarang
- **median-of-3** — buang spike satu sampel
- **EMA ringan** (α = 0,35) untuk penghalusan

### 2. Bootstrap (fix pertama)
Geometri tertutup:
- **2 anchor** → perpotongan dua lingkaran = 2 kandidat cermin; dipilih yang
  masuk batas ruangan. Kalau dua-duanya di dalam/luar → `ambiguous`.
- **3+ anchor** → least-squares + 2 iterasi Gauss-Newton.

### 3. Tracking — **Extended Kalman Filter** (`src/ekf.h`, `server/app.py`)

Setelah fix pertama ada, **EKF** yang jalan. Disebut *Extended* karena model
pengukurannya non-linear (jarak = akar kuadrat posisi):

```
State     : x = [ px, py, vx, vy ]ᵀ        posisi (m) + kecepatan (m/s)
Gerak     : constant velocity,  x ← F x ,  P ← F P Fᵀ + Q
Pengukuran: h(x) = √((px-ax)² + (py-ay)²)          ← non-linear
Jacobian  : H = [ (px-ax)/d , (py-ay)/d , 0 , 0 ]  ← linearisasi
Update    : y = z - h(x) ; S = H P Hᵀ + σr² ; K = P Hᵀ/S
            x ← x + K y ; P ← (I - K H) P
```

- **Innovation gating**: pengukuran ditolak bila `|y| > 3·√S` (outlier NLOS /
  pantulan), jadi estimasi tidak rusak — filter lanjut pakai model gerak.
- **Tuning**: `σa = 1,0 m/s²` (proses), `σr = 0,15 m` (noise jarak).
- **Confidence** = `1/(1+σ)` dengan `σ = √(P₀₀+P₁₁)` — makin banyak data
  masuk, makin kecil σ, makin tinggi confidence (otomatis).
- **Bonus**: EKF membawa state sebelumnya, jadi masalah **cermin 2-anchor tidak
  bisa membalik** estimasi antar-siklus.

Verifikasi (server, lintasan sintetis y = 1,0 → 1,6 m):

```
step 0  true_y=1.0 -> x=1.99 y=1.12  vy=0.00  sigma=0.270  used=2
step 1  true_y=1.2 -> x=2.01 y=1.03  vy=-0.28 sigma=0.263  used=2
step 2  true_y=1.4 -> x=2.01 y=1.46  vy=0.73  sigma=0.257  used=2
step 3  true_y=1.6 -> x=2.01 y=1.64  vy=0.68  sigma=0.202  used=2
```

| Tahap | Device (tag) | Server |
|---|---|---|
| pre-filter | ya | — |
| bootstrap | ya (standalone) | ya |
| EKF tracking | ya (fallback) | **ya (utama)** |

Detail matematis lengkap: [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) §5.

> Karena jarak radio ±10–30 cm noise di dalam ruangan (pantulan), EKF +
> innovation gate sangat membantu. Kalau posisi masih melompat, tambahkan
> anchor ke-3 (posisi non-kolinear) untuk geometri yang sehat.

---

## 🔧 Troubleshooting

| Masalah | Solusi |
|---|---|
| OLED tulisan `NOT CONFIGURED` | Set role+ID (serial `role tag` + `id 1` + `save`, atau AP `UWB-Setup`) |
| Tidak ada jarak sama sekali | Pastikan anchor & tag ID unik, jarak masih jangkauan, power USB cukup |
| Web map kosong | Cek `/api/v1/devices` — device online? Posisi anchor sudah diset? |
| Posisi tag loncat/cermin | 2 anchor → mirror (tanda `ambiguous`). Tambah anchor ke-3 |
| Server tidak bisa diakses board | Board & server harus satu Wi-Fi; pakai IP LAN (`192.168.x.x`), bukan `127.0.0.1` |
| MQTT tidak terima | Broker harus jalan (`mosquitto -v`); REST tetap jalan tanpa MQTT |
| Flash stuck "Connecting..." | BOOT + RST manual (lihat Langkah 3) |
| Posisi beku | Server drop range lebih 5 detik tanpa data baru (device offline) |
| Boros flash/RAM? | Partisi huge_app sudah diset; pakai `mode shortdata_fast_accuracy` kalau mau hemat |

---

## 📚 Dokumentasi Tambahan

- [`docs/API.md`](docs/API.md) — kontrak API/MQTT lengkap + model payload.
- [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) — cara kerja sistem, skema
  alamat UWB, matematika solver, trade-off.
- [`server/README.md`](server/README.md) — panduan server saja.

## ⚠️ Catatan & Keterbatasan

- **Bukan WiFi CSI / radar**: radio board ini DW1000 (UWB), bukan WiFi — tidak
  bisa deteksi manusia tanpa tag (fitur CSI ada di board ESP32-S3 + WiFi).
- Akurasi ±10–30 cm indoor; noise karena pantulan radio.
- State server **in-memory**: config device hilang saat server restart
  (device akan ambil ulang config dalam 15 detik setelah online kembali).
- Server dev Flask cocok untuk LAN; untuk produksi pakai gunicorn
  (`gunicorn -w 4 -b 0.0.0.0:8080 app:APP`) di belakang reverse proxy.