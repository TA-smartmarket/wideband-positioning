# 📡 UWB Positioning — ESP32 UWB Pro with Display

Real-time indoor positioning: **tag** (moving device) measures distance to
**anchors** placed in room corners, and the position is tracked live on a web
map. Everything is configurable from a web UI — no recompiling, no serial
editing.

One firmware runs on **every** board. Role (`tag`/`anchor`) and ID (1–10) are
chosen at runtime from the web UI, the setup portal, or the serial menu — the
UWB address is generated automatically (no more manual `7D:00:22:…` editing).

```
┌────────┐  range  ┌────────┐        ┌──────────────────┐
│ anchor1├────────▶│  tag   │        │  server (Flask)  │
│ (0,0)  │◀────────│ (moves)│──REST/MQTT──▶ REST + MQTT  │
└────────┘         └───┬────┘        │  solver → (x,y)  │
┌────────┐             │             │  web UI          │
│ anchor2│◀────────────┘             └──────────────────┘
│ (5,0)  │
└────────┘
```

## Features

- **One binary, every board** — role & ID 1–10 via web UI / AP portal / serial
  menu; UWB EUI + short address auto-derived.
- **Config from the server** — set WiFi, MQTT, anchor position (`x,y`),
  room size, UWB mode; devices poll + receive MQTT pushes (retained).
- **Two transports** — HTTP REST *and* MQTT; MQTT preferred, REST fallback.
- **Position solver** — server (and device, standalone) computes 2D position
  from ≥2 anchor ranges with room-bound disambiguation.
- **Live web map** — anchors, tag position, ranges, confidence, online status.
- **Multi-anchor ready** — up to 10 tags and 10 anchors per site.

## Quick start

### 1. Run the server (your computer / Ubuntu later)

```bash
cd server
pip install -r requirements.txt
python app.py                     # http://<your-ip>:8080
```

### 2. Flash the firmware (one board)

```bash
pio run -t upload                 # PlatformIO, ESP32 UWB Pro with Display
```

### 3. Configure each board

- **First boot** → the board starts a setup AP `UWB-Setup` (WiFi) →
  open `http://192.168.4.1` → pick role (anchor/tag), ID, WiFi, server URL.
- **From the web UI** → open `http://<server>:8080`, choose role+ID, set
  anchor position in metres (`x`, `y` from room corner), save. The server
  pushes the config over MQTT (retained) — device applies and reboots.
- **Serial menu** → open `pio device monitor`, type `?` for the full menu
  (`role tag`, `id 1`, `wifi SSID PASS`, `server http://192.168.1.10:8080`, …).

Minimal setup to track a tag:

| Board | Role | ID | Position |
|---|---|---|---|
| Board A | anchor | 1 | x=0, y=0 (room corner) |
| Board B | anchor | 2 | x=room width, y=0 |
| Board C | tag | 1 | — |

> 2 anchors give 2 mirror candidates; the solver picks the one inside the room
> (you set room width/height in the UI). 3+ anchors resolve it exactly.

## Project layout

```
├── platformio.ini      # one env: esp32uwb (huge_app partition)
├── src/
│   ├── main.cpp        # firmware: ranging, UI, config, REST+MQTT
│   ├── config.h        # config model + NVS + auto EUI
│   ├── net.h           # WiFi, AP portal, REST, MQTT
│   └── solver.h        # 2D multilateration (mirrored on server)
├── lib/DW1000/         # Makerfabs DW1000 library (vendored, patched guard)
└── server/
    ├── app.py          # Flask: REST + MQTT ingest + solver + web UI
    ├── requirements.txt
    └── README.md
```

## Documentation

- [`docs/API.md`](docs/API.md) — frozen REST/MQTT contract, config & telemetry
  models, solver rules.
- [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) — how the pieces fit, UWB
  short-address scheme, 2-anchor mirror math.
- [`docs/README.md`](docs/README.md) — step-by-step setup guide (family
  friendly).
- [`server/README.md`](server/README.md) — run & configure the server.

## Notes & limitations

- **No WiFi CSI / radar**: the DW1000 UWB radio cannot do WiFi CSI (that
  feature belongs to an ESP32-S3 + WiFi). This repo does UWB ranging.
- Accuracy ≈ ±10–30 cm indoor; reflections cause noise — enable the range
  filter (default on) for smoother tracking.
- Server state is in-memory: device configs are lost on server restart
  (persistence is a later step).