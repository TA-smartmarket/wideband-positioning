# Setup Guide — UWB Positioning (step by step)

This is the friendly walkthrough. You need:

- **3× Makerfabs ESP32 UWB Pro with Display** (2 anchors + 1 tag)
- **A computer** (server): your PC now, Ubuntu later
- PlatformIO (VS Code extension, or `pip install platformio`)

---

## Step 1 — Run the server

On your computer:

```bash
cd server
pip install -r requirements.txt
python app.py
```

You should see: `http://0.0.0.0:8080`. Open `http://127.0.0.1:8080` in a
browser → the **UWB positioning** page appears (empty map).

> For multiple boards to reach the server, they must be on the same Wi-Fi;
> the server listens on all interfaces, so use your computer's LAN IP
> (e.g. `http://192.168.1.10:8080`), not `127.0.0.1`.

## Step 2 — Flash all three boards

```bash
pio run -t upload          # plug board 1, upload
```

Do the same for the other two boards. **Same firmware on all three.**

## Step 3 — Configure each board (web UI — easiest)

1. **Anchor 1** — on the config page choose:
   - Role: **anchor**, ID: **1**
   - Position X: **0**, Position Y: **0**  (put the board in the room corner)
   - Room width/height: e.g. 5 × 4 m
   - WiFi SSID/password + Server URL → same for all boards
   - **Save** — the server pushes it to the board over MQTT; the board
     applies it and reboots.
2. **Anchor 2** — Role **anchor**, ID **2**, position **X = room width**,
   **Y = 0** (opposite corner).
3. **Tag** — Role **tag**, ID **1**, nothing else.

If a board is already on the network it receives the config automatically.
If not, configure it later from the serial menu or the `UWB-Setup` AP.

## Step 4 — Watch it track

Look at the web map: the tag appears inside the room and moves as you walk.
Each tag shows its coordinates, confidence, and the distances used
(`ranges`). Anchors turn grey when offline.

## Alternative setup — serial menu

`pio device monitor`, then type commands:

```
role anchor      → set role
id 1             → set id (1–10)
wifi MySSID pass → join Wi-Fi
server http://192.168.1.10:8080
pos 0 0          → anchor position (metres)
room 5 4         → room size
save             → store to memory, reboot
```

Or on first boot, join the **UWB-Setup** Wi-Fi AP from a phone and open
`http://192.168.4.1` for the same form.

---

## Troubleshooting

| Problem | Fix |
|---|---|
| Board stuck at "NOT CONFIGURED" | Set role+id (serial `role tag` / `id 1`, then `save`) |
| No distance shown | Both anchors and tag must have unique IDs; keep tag within range; check power |
| Web map empty | Check devices are online (`/api/v1/devices`); anchor positions must be set |
| Tag position jumps/flips | 2 anchors → mirror ambiguity (`ambiguous` in state). Add a 3rd anchor |
| No MQTT | MQTT is optional — REST works; verify broker: `mosquitto_sub -t uwb/home/#` |
| Flashing fails at "Connecting..." | Hold BOOT, tap RST, release BOOT, upload, then RST |
| Position frozen | Server keeps last ranges 5 s; if device offline longer it drops |

## Next steps

- Add a **3rd anchor** for unambiguous positions (web UI, ID 3, corner 3).
- Move the server to **Ubuntu** (`pip install -r requirements.txt`, same app).
- Add database persistence so configs survive server restarts.