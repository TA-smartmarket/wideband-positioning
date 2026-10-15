# API & Protocol Contract (frozen)

Frozen interface between **devices** (ESP32 UWB) and the **server**.
Both REST and MQTT carry the same payload objects.

---

## 1. Identity scheme

A device is identified by `role` + `id`. The UWB EUI is **derived automatically** —
no manual address editing anywhere.

| Role | id range | EUI (auto) | Short address |
|---|---|---|---|
| `anchor` | 1–10 | `0N:A0:5B:D5:A9:9A:E2:9C` | `0xA00N` |
| `tag` | 1–10 | `0N:7D:00:22:EA:82:60:3B` | `0x7D0N` |

`N` = id as a single hex nibble (`1` → `01`, `10` → `0A`).

Examples: `anchor` id 1 → EUI `01:A0:5B:...` short `0xA001`;
`tag` id 3 → EUI `03:7D:00:...` short `0x7D03`.

Device ID string used everywhere: `"<role>-<id>"` → `"anchor-1"`, `"tag-3"`.

---

## 2. Config object

Source of truth: **server**. A device fetches it on boot, on WiFi connect, and
every `config_poll_s`. A locally edited config (AP portal / serial) is kept in
NVS and used until the server provides one.

```json
{
  "device_id": "anchor-1",
  "role": "anchor",
  "id": 1,
  "site": "home",

  "wifi":   { "ssid": "MyWiFi", "password": "secret" },
  "server": { "url": "http://192.168.1.10:8080", "token": "" },
  "mqtt":   { "enabled": true, "host": "192.168.1.10", "port": 1883,
              "user": "", "password": "", "base_topic": "uwb/home" },

  "position": { "x": 0.0, "y": 0.0, "z": 2.2 },
  "room":     { "width": 5.0, "height": 4.0 },

  "uwb": { "mode": "longdata_range_lowpower",
           "range_filter": true,
           "update_ms": 200 }
}
```

### Field notes
- `position` — anchor placement in metres, origin at room corner (0,0).
  Ignored for tags. **This is what makes corner placement configurable.**
- `room` — used by the solver to disambiguate the 2-anchor mirror solution
  (pick the intersection that falls inside the room).
- `uwb.mode` — one of `longdata_range_lowpower`, `shortdata_fast_lowpower`,
  `longdata_fast_lowpower`, `shortdata_fast_accuracy`, `longdata_fast_accuracy`,
  `longdata_range_accuracy`.
- `server.token` / `mqtt.user`+`password` — optional auth; empty = disabled.

### Partial config
Any subset of fields may be sent. Omitted fields keep their current value.
`{"room": {"width": 6}}` changes only the width.

---

## 3. Telemetry object (device → server)

Sent as a batch. `ranges` and `status` may both be present.

```json
{
  "site": "home",
  "device_id": "anchor-1",
  "ts": 1727880000123,
  "ranges": [
    {
      "ts": 1727880000123,
      "seq": 5121,
      "src": "anchor-1",
      "dst": "tag-1",
      "range": 2.731,
      "raw": 2.744,
      "rx_power": -71.04,
      "fp_power": -74.20,
      "quality": 0.87
    }
  ],
  "positions": [
    {
      "ts": 1727880000123,
      "id": "tag-1",
      "x": 2.31, "y": 1.02, "z": 0.90,
      "confidence": 0.80,
      "source": "server"
    }
  ],
  "status": {
    "ip": "192.168.1.57",
    "rssi": -60,
    "uptime_s": 1284,
    "anchors_seen": 2,
    "tags_seen": 1,
    "fw": "1.0.0"
  }
}
```

- `range` — metres, pre-filtered on the device (gate + median-3 + EMA).
  `raw` — the unfiltered two-way-ranging result (firmware ≥ 1.0.32); the
  server tracker uses `raw` when present. `rx_power`/`fp_power` — dBm.
  `quality` — 0..1.
- `ranges` holds **every sample measured since the previous packet**, each
  sent exactly once (firmware ≥ 1.0.32). Older firmware sent the latest value
  per link again in every packet; the server deduplicates by `ts`, so both
  work.
- `ts` (per range) — the device's `millis()` when the sample was measured.
  `ts` (packet) — `millis()` when the packet was built. The server maps each
  device clock to its own (minimum-latency offset) and applies every sample at
  the time it was measured.
- `seq` — per-device sample counter (diagnostics: gaps = lost samples).
- `status.samples_dropped` — samples lost because the uplink was down.
- `status.reset` — why the chip last restarted (`poweron`, `software`,
  `panic`, `int_wdt`, `task_wdt`, `brownout`, …); `status.heap_min` — lowest
  free heap since boot. Shown per device in `/api/v1/state` → `devices[]`.
- `positions` — only the **tag** may include locally solved positions
  (`"source": "device"`); the server shows them only while it has no track of
  its own for that tag.
- Rates: the tag sends every `update_ms` (default 200 ms), anchors every
  `5 × update_ms`. The tag's copies win; the anchors' copies of the same
  exchanges are only used when the tag has sent nothing for 10 s.

---

## 4. REST API (server)

Base: `http://<server>:8080`. All bodies JSON. `Authorization: Bearer <token>`
required on `/api/v1/*` **only if** the server was started with a token.

| Method | Path | Purpose |
|---|---|---|
| `POST` | `/api/v1/telemetry` | Ingest telemetry (section 3) |
| `GET`  | `/api/v1/config?device_id=anchor-1` | Fetch config (section 2) |
| `GET`  | `/api/v1/state` | Current world state (section 6) |
| `GET`  | `/api/v1/devices` | List known devices + last seen |
| `PUT`  | `/api/v1/config` | Web UI writes config for a device |
| `POST` | `/api/v1/position` | Current tracker estimate of every tag (debug) |
| `GET`  | `/api/v1/calibration` | Stored calibration points + per-anchor correction |
| `POST` | `/api/v1/calibration/point` | `{"x","y","seconds":15,"store":true}` — tag held still on a measured spot; returns range error per anchor + position error (mean error, σ, RMSE, CEP50/95). Blocks for `seconds` |
| `POST` | `/api/v1/calibration/apply` | Fit `measured = scale·true + bias` per anchor from the stored points |
| `POST` | `/api/v1/calibration/clear` | Forget points and/or corrections |
| `GET`/`PUT` | `/api/v1/tracking` | Tracker tuning (`sigma_a`, `sigma_r`, `huber_k`, `gate`, …) |
| `GET`  | `/api/v1/record` | Recorder status + recorded sessions |
| `POST` | `/api/v1/record/start` | `{"name","note"}` — record every packet + estimate to `server/logs/*.jsonl` |
| `POST` | `/api/v1/record/stop` | Stop recording |
| `POST` | `/api/v1/record/mark` | `{"label","x","y"}` — marker in the recording |
| `GET`  | `/api/v1/record/file/<name>` | Download a recording |
| `GET`  | `/api/v1/navigation/position` | Trolley position in the navigation contract (section 8) |
| `GET`  | `/api/v1/navigation/scene` | Static obstacles in the navigation contract (section 8) |
| `GET`  | `/` | Web UI (live map + config) |

Responses: `200` with `{"ok": true, ...}`, errors `4xx/5xx` with
`{"ok": false, "error": "..."}`.

`GET /api/v1/config` returns `404` when the device is unknown → device keeps
its local config.

---

## 5. MQTT

Base topic from config: `base_topic` (e.g. `uwb/home`).

| Topic | Dir | Retained | Payload |
|---|---|---|---|
| `<base>/telemetry` | device → server | no | full telemetry object |
| `<base>/config/<device_id>` | server → device | **yes** | config object |
| `<base>/cmd/<device_id>` | server → device | no | `{"cmd":"reboot"}` |
| `<base>/state` | server → all | yes | world state (section 6) |
| `<base>/status/<device_id>` | device → server | yes | status object |

Device behaviour:
1. On connect, publish its `status` (retained) and subscribe to
   `<base>/config/<device_id>` + `<base>/cmd/<device_id>`.
2. Publish batches of fresh samples to `<base>/telemetry` (tag every
   `update_ms`, anchors every `5 × update_ms`); REST when MQTT is down or the
   publish fails. The MQTT client buffer is 4 kB (PubSubClient's 256-byte
   default silently dropped every telemetry batch). The old per-range
   `<base>/range` messages are no longer sent — they carried no `device_id`,
   so the server could never map their timestamps.
3. On a `config/<device_id>` message → apply partial config, persist to NVS
   (reboot when role/id changed). On `cmd/<device_id>` → run the command.

If `mqtt.enabled` is false the device uses REST only. Both paths are always
available; MQTT is used when reachable, REST otherwise.

---

## 6. World state (server → clients)

```json
{
  "site": "home",
  "ts": 1727880000123,
  "room": { "width": 5.0, "height": 4.0 },
  "anchors": [
    { "id": "anchor-1", "x": 0.0, "y": 0.0, "z": 2.2,
      "online": true, "last_seen": 1727880000123, "rssi": -60 }
  ],
  "tags": [
    { "id": "tag-1", "x": 2.31, "y": 1.02, "z": 0.90,
      "confidence": 0.80, "ambiguous": false,
      "online": true, "last_seen": 1727880000123,
      "ranges": { "anchor-1": 2.731, "anchor-2": 3.104 } }
  ],
  "links": [
    { "src": "anchor-1", "dst": "tag-1", "range": 2.731,
      "rx_power": -71.04, "ts": 1727880000123 }
  ]
}
```

`ambiguous: true` means only 2 anchors produced ranges and the solver could
not pick a room-interior solution — the position is a mirror guess.

---

## 7. Localisation rules (server)

Implemented in `server/tracker.py`; the reasoning and the before/after
evidence are in `docs/PERBAIKAN_AKURASI.md`.

1. **Each sample once, at its own time.** Samples are deduplicated per
   `(reporter, anchor, tag)` by their device `ts`, ordered, and applied at
   `device_ts + clock_offset(reporter)`.
2. **One source per tag** — the tag's own reports are used (all its ranges
   share one clock); anchors' copies only when the tag has sent nothing for
   10 s. Ranges longer than the farthest room corner + 1.5 m are dropped.
3. **Range correction** — `r = (raw − bias_i) / scale_i` from the calibration
   (`scene.calibration.anchors`), then the NLOS bias of a blocked path.
4. **Bootstrap** — slant-range multilateration (closed form + Gauss-Newton on
   `sqrt(dx² + dy² + dz²)`) from all anchors heard in the last 0.7 s (3 are
   enough after 2 s). Not shown until it has absorbed 2 samples per anchor.
5. **EKF** — state `[px, py, vx, vy, b]`, Singer velocity model (`sigma_a`
   0.8 m/s², velocity correlation time `vel_tau_s` 1 s), `b` = range offset shared by all anchors (estimated online when
   there are ≥ 3 anchors). Measurement `h = sqrt((px−ax)² + (py−ay)² + dz²) + b`
   with `dz = anchor_z − tag_z` — no projection of the range onto the floor.
6. **Robust update** — normalised innovation `|ν| > 2.5` → measurement noise
   inflated (Huber); `|ν| > 6` → rejected. No accepted sample for 1.5 s →
   re-initialised from multilateration.
7. `confidence = 1 / (1 + sigma)`, `sigma = sqrt(P00 + P11)`; the tag state
   also carries `range_bias` (b), `rate_hz` (samples/s), `used` (accepted/s)
   and per-link `residual` / `accepted`.

---

## 8. Navigation integration (server → trolley)

The `wideband-navigation` trolley consumes the **processed position** and the
**static scene** — never raw ranges. Three read-only endpoints expose those in
the exact contract navigation documents (`navigation_core/docs/data_contract.md`
+ `integration.md`). Position is polled periodically (or pushed over MQTT); the
map and scene are fetched once per navigation session / map change (they are not
re-sent with every position).

### `GET /api/v1/navigation/position`

Polled by the trolley at ~10 Hz. Optional `?tag=<id>` selects a specific tag;
otherwise the freshest online tag wins.

```json
{
  "schema_version": 1,
  "trolley_id": "TROLLEY_01",
  "frame_id": "smart_market_map",
  "timestamp_ms": 204621234,
  "position": { "x_m": 1.495, "y_m": 0.991 },
  "quality": 0.925,
  "valid": true
}
```

- `timestamp_ms` is **monotonic** (`time.monotonic_ns()`), not wall clock —
  navigation uses it only for staleness detection.
- `quality` is the EKF confidence, clamped to `[0, 1]`.
- `valid` is `false` when the tag position is stale/offline (`STALE_MS = 5000`).
- Returns `404` only when no tag position has ever been seen.

### `GET /api/v1/navigation/map`

Fetched once at navigation start; the map rectangle is sourced from the 3D
scene so navigation never hard-codes room dimensions that drift from the scene.

```json
{
  "map_id": "SMART_MARKET_MAIN",
  "map_version": 1,
  "frame_id": "smart_market_map",
  "width_m": 3.9,
  "height_m": 2.5,
  "origin_x_m": 0.0,
  "origin_y_m": 0.0
}
```

### `GET /api/v1/navigation/scene`

Fetched once at navigation start (and re-fetched when the map changes).

```json
{
  "schema_version": 1,
  "map_id": "SMART_MARKET_MAIN",
  "map_version": 1,
  "frame_id": "smart_market_map",
  "scene": {
    "room": { "width": 3.9, "depth": 2.5 },
    "obstacles": [
      { "id": "obstacle-1", "label": "Wall", "x": 1.0, "y": 2.0, "z": 0.24,
        "sx": 1.79, "sy": 0.97, "sz": 0.39, "rot": 1.5708, "atten": 1.0 }
    ]
  },
  "nlos_enabled": true
}
```

- `map_id` / `map_version` / `frame_id` let navigation version-check the map
  against the `GET /api/v1/navigation/map` response before motion.
- `x`, `y` are the obstacle centre, `sx`, `sy` the full size, in metres.
- **`rot` is in radians** here (positioning stores degrees; the server converts
  at this boundary). navigation's obstacle parser reads `rot` directly as
  radians, so the units now match end-to-end.

### MQTT push (position)

When MQTT is enabled the server also publishes the position-contract document to
`<base>/navigation/position` at 10 Hz (non-retained). `valid:false` samples are
**not** published — a drop-out means no sample, which navigation treats as
stale and enters `POSITION_LOST` safely.

### Coordinate frame

Origin bottom-left, `+X` right, `+Y` up, metres, `frame_id = smart_market_map`
— matching `wideband-navigation/config/map.json`.

