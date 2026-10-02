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
      "src": "anchor-1",
      "dst": "tag-1",
      "range": 2.731,
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

- `range` — metres. `rx_power`/`fp_power` — dBm. `quality` — 0..1.
- `positions` — only the **tag** may include locally solved positions
  (`"source": "device"`); the server ignores/merges them.
- `ts` — device `millis()`-based epoch ms. Server stamps `recv_ts` itself.

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
| `POST` | `/api/v1/position` | Solve now from current ranges (debug) |
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
| `<base>/range` | device → server | no | one range object (section 3) |
| `<base>/telemetry` | device → server | no | full telemetry object |
| `<base>/config/<device_id>` | server → device | **yes** | config object |
| `<base>/cmd/<device_id>` | server → device | no | `{"cmd":"reboot"}` |
| `<base>/state` | server → all | yes | world state (section 6) |
| `<base>/status/<device_id>` | device → server | yes | status object |

Device behaviour:
1. On connect, publish its `status` (retained) and subscribe to
   `<base>/config/<device_id>` + `<base>/cmd/<device_id>`.
2. Publish every range to `<base>/range` plus a batch to `<base>/telemetry`;
   batch to REST every `update_ms * 5` or when MQTT is down.
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

The pipeline is **pre-filter → bootstrap → EKF tracking** (full maths in
`docs/ARCHITECTURE.md` §5).

1. Collect the most recent range per `(anchor, tag)` pair (age < `stale_ms`,
   default 2000 ms). Ranges arrive already pre-filtered by the device
   (median + outlier gate; the library's own low-pass is disabled).
2. Need ≥ 2 anchor ranges for a tag, with known anchor positions.
3. **Bootstrap (first fix only)** — closed-form geometry:
   - **2 anchors** → circle intersection gives two candidates; pick the one
     inside `room` bounds. If both or neither are inside → `ambiguous: true`,
     the one closer to room centre is used.
   - **3+ anchors** → least-squares multilateration (linearised, 2 unknowns).
4. **Tracking (after the first fix)** — **Extended Kalman Filter**
   (`class TagEKF`), state `[px, py, vx, vy]`:
   - `predict(dt)` with the constant-velocity model + process noise `sigma_a`;
   - one scalar `update_range(ax, ay, z)` per anchor, linearised Jacobian
     `H = [(px-ax)/d, (py-ay)/d, 0, 0]`;
   - **innovation gating**: `|z - h(x)| > 3·sqrt(S)` → measurement rejected
     (NLOS/outlier) and skipped.
5. `confidence` = `1 / (1 + sigma)` with `sigma = sqrt(P00 + P11)` from the
   EKF covariance (shrinks as measurements accumulate). `vx`/`vy` (m/s) and
   `sigma` are included in the tag state.
6. Positions are recomputed on every ingest and broadcast on `<base>/state`.
