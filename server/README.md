# Server — UWB Positioning

REST + MQTT ingest, position solver, and a web UI (live map + device config).

## Requirements
- Python 3.9+
- `pip install -r requirements.txt` (Flask, paho-mqtt)
- A MQTT broker (Mosquitto) on the network — **optional**, the server works
  with REST-only too.

## Run

```bash
cd server
pip install -r requirements.txt
python app.py                          # http://0.0.0.0:8080
```

Options:

```bash
python app.py --port 9000              # different port
python app.py --token mysecret         # require Bearer token on /api/*
python app.py --mqtt-host 192.168.1.5  # MQTT broker address (default 127.0.0.1)
python app.py --mqtt-port 1883
python app.py --mqtt-base uwb/myroom   # base MQTT topic
python app.py --mqtt 0                 # disable MQTT entirely
```

## Endpoints

| Method | Path | Purpose |
|---|---|---|
| `POST` | `/api/v1/telemetry` | devices send ranges/status here |
| `GET`  | `/api/v1/config/device?role=anchor&id=1` | device fetches its config |
| `PUT`  | `/api/v1/config` | web UI / API saves config |
| `GET`  | `/api/v1/state` | world state (tags, anchors, links) |
| `GET`  | `/api/v1/devices` | registered devices |
| `POST` | `/api/v1/position` | force re-solve (debug) |
| `GET`  | `/` | web UI |

Full contract: `docs/API.md`.

## Example session (curl)

```bash
# 1. Place anchor 1 at room corner (0,0), anchor 2 at (5,0)
curl -X PUT localhost:8080/api/v1/config -H 'Content-Type: application/json' \
     -d '{"role":"anchor","id":1,"position":{"x":0,"y":0},"room":{"width":5,"height":4}}'
curl -X PUT localhost:8080/api/v1/config -H 'Content-Type: application/json' \
     -d '{"role":"anchor","id":2,"position":{"x":5,"y":0}}'

# 2. Ingest a range pair (anchor → tag)
curl -X POST localhost:8080/api/v1/telemetry -H 'Content-Type: application/json' \
     -d '{"device_id":"anchor-1","ranges":[{"src":"anchor-1","dst":"tag-1","range":2.236,"rx_power":-60}]}'
curl -X POST localhost:8080/api/v1/telemetry -H 'Content-Type: application/json' \
     -d '{"device_id":"anchor-2","ranges":[{"src":"anchor-2","dst":"tag-1","range":3.162,"rx_power":-61}]}'

# 3. Tag position appears in state
curl localhost:8080/api/v1/state
# -> tags: [{"id":"tag-1","x":2.00,"y":1.00,...}]
```

## MQTT topics

| Topic | Direction | Payload |
|---|---|---|
| `uwb/home/range` | device → server | single range |
| `uwb/home/telemetry` | device → server | batch telemetry |
| `uwb/home/status/<dev>` | device → server | online status (retained) |
| `uwb/home/config/<dev>` | server → device | config push (retained) |
| `uwb/home/state` | server → all | world state (retained) |

## Production notes

- Flask dev server is fine for a LAN deployment. For production behind a
  reverse proxy use `gunicorn -w 4 -b 0.0.0.0:8080 app:APP`.
- State is kept in memory; add SQLite/Postgres persistence if restarts must
  not lose device configs.
- `--token mysecret` protects `/api/*`; the UI page itself stays open.