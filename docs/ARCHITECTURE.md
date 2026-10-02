# Architecture

How the pieces fit together, and the important logic.

## 1. Components

```
┌────────────────────────── ESP32 (one binary) ──────────────────────────┐
│ main.cpp    — ranging loop, callbacks, telemetry buffer, OLED UI,      │
│             — serial menu, setup portal, apply-server-config           │
│ config.h    — Config struct, NVS store, role/id → EUI derivation       │
│ net.h       — WiFi, HTTP client, PubSubClient MQTT, AP portal          │
│ solver.h    — 2D multilateration (also used standalone by the tag)     │
└───────┬────────────────────────────────────────────────────────────────┘
        │ REST POST /api/v1/telemetry   ·  MQTT uwb/home/...
        ▼
┌──────────────────────────── server/app.py ─────────────────────────────┐
│ Flask REST (telemetry, config, state)  ·  paho MQTT listener           │
│ solver (mirror of solver.h)  ·  state model  ·  web UI (map + setup)   │
└─────────────────────────────────────────────────────────────────────────┘
```

## 2. Identity & UWB addressing

Every board stores **role + id** in NVS. The 8-byte EUI and the 2-byte
network short address are derived, never typed:

| role | id | EUI | short address |
|---|---|---|---|
| anchor | 1 | `01:A0:5B:D5:A9:9A:E2:9C` | `0xA001` |
| anchor | 10 | `0A:A0:5B:D5:A9:9A:E2:9C` | `0xA00A` |
| tag | 3 | `03:7D:00:22:EA:82:60:3B` | `0x7D03` |

Derivation (`config.h` `deviceEui()`):

- anchor: `NN:A0:5B:D5:A9:9A:E2:9C`
- tag:    `NN:7D:00:22:EA:82:60:3B`

Both `startAsTag`/`startAsAnchor` are called with `randomShortAddress=false`,
so the short address = first EUI bytes = `0xA0NN` / `0x7DNN`. This lets any
node derive the peer's role+id from its short address (`shortToDeviceId()`
in `main.cpp`), which the telemetry uses to label `src`/`dst`.

## 3. Configuration flow

```
server (source of truth)
    │  GET /api/v1/config/device?role=&id=   (on boot, on wifi connect, every 15 s)
    ▼
device ──────────────── applies partial JSON → NVS → reboots if role/id changed
    ▲
    │ MQTT uwb/home/config/<dev> (retained) — server pushes on web-UI save
    └─────────────────────────────────────── device subscribes on connect
```

- Devices poll every 15 s and also receive pushes via retained MQTT so config
  survives device reboots and offline periods.
- Anchor positions are broadcast to tags inside the config (`anchors[]`), which
  lets a tag solve its own position without the server (standalone mode).

## 4. Telemetry & transports

- Every completed range → buffered (`RangeRec` ring) with `src`, `dst`,
  `range`, `rx_power`, `fp_power`, `quality`.
- Every `update_ms * 5` the device publishes:
  - MQTT: `<base>/range` (per range, live) + `<base>/telemetry` (batch)
  - REST:  `POST /api/v1/telemetry` (batch; used as fallback when MQTT is off/down)
- Server stores latest range per `(anchor, tag)` and recomputes positions.

## 5. Solver (identical on device & server)

Input: anchor positions + ranges. Output: `(x, y)`, `confidence`, `ambiguous`.

- **0–1 anchors** → no fix.
- **2 anchors** → circle intersection, two candidates. Pick the candidate
  inside the room rect `[0,room_w]×[0,room_h]`. If both/neither → `ambiguous`
  and nearest to room centre is returned. No intersection → midpoint with
  low confidence.
- **3+ anchors** → linearised least squares (subtract eq. 0), then 2
  Gauss-Newton iterations; `confidence = 1 − RMS residual`.

Server normalises to the same function (`solve_2d` in `app.py`); the device
uses `solver.h` for standalone operation.

## 6. State model

`GET /api/v1/state` (and `uwb/home/state` MQTT, retained):

```json
{ "site": "home", "ts": ..., "room": {...},
  "anchors": [{"id":"anchor-1","x":0,"y":0,"online":true, ...}],
  "tags":    [{"id":"tag-1","x":2.00,"y":1.00,"confidence":0.8,
               "ambiguous":false,"ranges":{"anchor-1":2.236,...}}],
  "links":   [{"src":"anchor-1","dst":"tag-1","range":2.236,"rx_power":-60}] }
```

Staleness: ranges older than 5 s are ignored; `online` flips off after 5 s.

## 7. Firmware partition layout

WiFi + HTTP + MQTT + ArduinoJson + display exceed the default 1.3 MB app
slot, so `platformio.ini` sets `board_build.partitions = huge_app.csv`
(3 MB app / 1 MB SPIFFS-ish). Measured: 32.7% flash, 15.7% RAM.

## 8. Known trade-offs

- **In-memory server state** — device configs vanish on server restart until
  devices re-report (they re-fetch config each 15 s anyway).
- **2-anchor mirror ambiguity** — inherent to two range circles; mitigated by
  room bounds, resolved fully with 3+ anchors in non-collinear positions.
- **Clock sync** — device epochs are `millis()`-based; the server stamps
  its own `recv_ts`. Cross-device time tags are relative only.