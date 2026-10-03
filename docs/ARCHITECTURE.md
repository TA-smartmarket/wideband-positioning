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

## 5. Localisation pipeline (device and server agree)

Three stages, in order:

### 5.1 Pre-filter — `src/main.cpp` (`RangeFilter`, `filterRange()`)

The DW1000 library's own `useRangeFilter()` is **disabled on purpose**: it is a
plain low-pass that stores its previous output *inside* `DW1000Device`, so one
bad sample is fed back forever and the range walks away (observed on hardware:
1.5 m drifting to 248 m). It is replaced by:

1. **outlier gate** — reject a sample that jumps more than `MAX_JUMP_M` (5 m)
   from the current estimate;
2. **median-of-3** over the recent history — kills single-sample spikes;
3. **light EMA** (`EMA_ALPHA = 0.35`) for smoothing.

Filter state is per link (`"src>dst"`) and dropped when the peer is lost
(`filterForget()`). The `filter on|off` setting controls this pre-filter.

### 5.2 Bootstrap — `src/solver.h` / `server/app.py::solve_2d()`

Closed-form geometry gives the first fix:

- **2 anchors** → circle intersection, two mirror candidates; the one inside
  the room rect wins. Both/neither inside → `ambiguous: true`, nearest to room
  centre returned. No intersection → midpoint, low confidence.
- **3+ anchors** → linearised least squares + 2 Gauss-Newton iterations.

### 5.3 Tracking — Extended Kalman Filter, `src/ekf.h` / `server/app.py::TagEKF`

Once a first fix exists, the **EKF** takes over (this is the estimator the
supervisor asked for — the measurement model is non-linear, hence *Extended*).

**State** `x = [px, py, vx, vy]ᵀ` — position (m) and velocity (m/s).

**Motion model** (constant velocity, discrete white-acceleration noise):

```
x_{k+1} = F x_k ,  F = [[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]]
Q = sigma_a^2 * [[dt^4/4, 0, dt^3/2, 0],
                 [0, dt^4/4, 0, dt^3/2],
                 [dt^3/2, 0, dt^2,   0],
                 [0, dt^3/2, 0, dt^2  ]]
```

**Measurement model** — range to anchor *i* at `(ax_i, ay_i)`:

```
h_i(x) = sqrt((px-ax_i)^2 + (py-ay_i)^2)          <- NON-LINEAR
H_i    = [ (px-ax_i)/d , (py-ay_i)/d , 0 , 0 ]    <- Jacobian (linearisation)
```

**Update** (one scalar measurement per anchor, processed sequentially):

```
y = z - h(x)                    innovation
S = H P Hᵀ + sigma_r^2          innovation covariance
K = P Hᵀ / S                    Kalman gain
x = x + K y
P = (I - K H) P
```

**Innovation gating**: a measurement is rejected when `|y| > 3·sqrt(S)`
(NLOS / reflection outlier). Rejected samples are simply skipped, so the filter
keeps coasting on the motion model instead of being corrupted.

**Tuning**: `sigma_a = 1.0 m/s²` (process), `sigma_r = 0.15 m` (range noise).
`confidence` reported to the UI is `1 / (1 + sigma)` where
`sigma = sqrt(P00 + P11)` — it shrinks as measurements accumulate, so a
converged track shows high confidence automatically.

**Why it also fixes the "mirror" problem**: the EKF carries the previous state
forward, so the 2-anchor ambiguity cannot flip the estimate between cycles.

Verification (server-side, synthetic walk at y = 1.0 → 1.6 m):

```
step 0  true_y=1.0 -> x=1.99 y=1.12  vy=0.00  sigma=0.270  used=2
step 1  true_y=1.2 -> x=2.01 y=1.03  vy=-0.28 sigma=0.263  used=2
step 2  true_y=1.4 -> x=2.01 y=1.46  vy=0.73  sigma=0.257  used=2
step 3  true_y=1.6 -> x=2.01 y=1.64  vy=0.68  sigma=0.202  used=2
```

### 5.4 NLOS from the 3D scene (`server/scene.py`)

The room, anchor placements and obstacles are edited in the 3D web UI and
stored in `server/scene.json` (the server is the source of truth; anchor
positions are mirrored onto the devices through the normal config push).

Coordinate system is right-handed with **Z up** (`x` width, `y` depth,
`z` height); the UI maps this to Three.js as `(x, z, y)`.

For every anchor→tag path the server runs a segment/oriented-box test
(`segment_hits_box`, slab method in the box's yaw frame). A blocked path is
treated as **NLOS**:

| Effect | Formula |
|---|---|
| measurement noise | `sigma = base * (1 + 7 * atten)` |
| range bias | `z = z + 0.35 * atten` (UWB NLOS reads long) |
| rejection | innovation gate `|y| > 3*sqrt(S)` still applies |

`atten` is per-obstacle (0 = transparent, 1 = solid wall), so a glass partition
and a concrete wall behave differently. The flag `nlos_enabled` turns the whole
mechanism off for comparison.

Vertical geometry is handled exactly: the EKF is 2D, so each 3D range is
projected with `r_2d = sqrt(r^2 - dz^2)` using the anchor height and the
assumed tag height (`DEFAULT_TAG_Z = 0.9 m`) — the vertical offset is removed
instead of being absorbed as error.

Verified on hardware + synthetic input:

```
wall at y=1.5 (x 0..3): anchor-2 path blocked=True, anchor-3 blocked=False
tag at (2.5, 3.0) -> estimate (2.61, 3.09), sigma 0.79 (inflated by NLOS)
```

### 5.5 Where each stage runs

| Stage | Device (tag) | Server |
|---|---|---|
| pre-filter | yes | — (ranges arrive already filtered) |
| bootstrap | yes (standalone) | yes |
| EKF tracking | yes (standalone fallback) | yes (primary) |

The tag publishes its own EKF estimate with `"source": "device"`; the server's
estimate is authoritative and published as `"source": "server"`.

### 5.6 Timestamps

Device clocks are `millis()`-based (uptime) and cannot be compared with the
server's epoch, so ranges carry both: `ts` (device, for display) and
`recv_ts` (server, **used for staleness**). Mixing them silently discards every
measurement as "stale" — a bug that cost an afternoon.

---

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