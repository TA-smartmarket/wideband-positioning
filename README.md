# Wideband Positioning — ESP32 UWB (Distance)

UWB (Ultra-Wideband) distance measurement firmware for the
**Makerfabs ESP32 UWB Pro with Display** boards, built with **PlatformIO**,
based on the official [Makerfabs-ESP32-UWB](https://github.com/Makerfabs/Makerfabs-ESP32-UWB) library.

Two firmwares are provided (one PlatformIO environment each):

| Firmware | Environment | Role | What it does |
|---|---|---|---|
| [`src/tag/main.cpp`](src/tag/main.cpp) | `tag` | **Tag** (moving) | Measures distance to every anchor; prints + shows the latest distance on the OLED |
| [`src/anchor/main.cpp`](src/anchor/main.cpp) | `anchor` | **Anchor #1** (fixed) | Replies to ranging requests from tags; prints each measured distance and shows its status on the OLED |
| [`src/anchor/main.cpp`](src/anchor/main.cpp) | `anchor2` | **Anchor #2** (fixed) | Same firmware, but a **different UWB address** (`3D:4E:…`), so two anchors can run side by side |

> The tag initiates ranging, so the distance value appears on **both** the
> tag and the anchor's Serial output.

---

## Project structure

```
wideband-positioning/
├── platformio.ini          # PIO config: tag, anchor, anchor2 environments
├── lib/
│   └── DW1000/             # Makerfabs DW1000 library (vendored)
│       └── src/            # DW1000, DW1000Ranging, DW1000Device, ...
├── src/
│   ├── tag/main.cpp        # Tag firmware (env: tag)
│   └── anchor/main.cpp     # Anchor firmware (env: anchor, anchor2)
```

The DW1000 library is committed into `lib/` so the project builds without
network access (no Arduino Library Manager needed).

---

## Requirements

### Hardware
- 3× Makerfabs ESP32 UWB Pro with Display (1 tag + 2 anchors), or any
  combination — multi-anchor works out of the box
- USB-C cables for programming

### Software
- **PlatformIO** (IDE extension for VS Code, or CLI: `pip install platformio`)
- Python 3.9+ (only for the CLI install)

The ESP32 platform, Arduino framework and Adafruit SSD1306/GFX libraries are
downloaded automatically by PlatformIO on the first build.

---

## Wiring / Pinout

All connections are already fixed on the Makerfabs ESP32 UWB Pro with Display
(SPI for the DW1000, I2C for the OLED) — **no extra wiring needed**:

| Signal | ESP32 pin |
|---|---|
| SPI SCK  | GPIO 18 |
| SPI MISO | GPIO 19 |
| SPI MOSI | GPIO 23 |
| UWB CS (SS)   | GPIO 21 |
| UWB Reset (RST) | GPIO 27 |
| UWB IRQ | GPIO 34 |
| OLED SDA | GPIO 4 |
| OLED SCL | GPIO 5 |
| OLED address | `0x3C` |

---

## Getting Started (PlatformIO)

### 1. Open the project
- **VS Code**: install the *PlatformIO IDE* extension, then
  `File → Open Folder` this project. PlatformIO automatically activates the
  project (look for the 🚀 task bar at the bottom).
- **CLI**: `cd wideband-positioning`

### 2. Unique addresses (very important!)
Every board must have its **own unique 8-byte UWB address**. The **first two
bytes** of that address become the *short address* (the ID shown in the logs
and on the OLED), so they must differ between anchors.

| Board | Environment | UWB address (EUI) | Short address |
|---|---|---|---|
| Tag | `tag` | `7D:00:22:EA:82:60:3B:9B` | `0x7D00` |
| Anchor #1 | `anchor` | `86:17:5B:D5:A9:9A:E2:9C` | `0x8617` |
| Anchor #2 | `anchor2` | `3D:4E:7C:2A:08:5F:B1:C3` | `0x3D4E` |

- **Anchor #1** address is the default in `src/anchor/main.cpp`.
- **Anchor #2** is the *same* firmware with a different address, injected by
  the `anchor2` environment — no file editing needed:

  ```ini
  [env:anchor2]
  build_flags = ... -DANCHOR_EUI="3D:4E:7C:2A:08:5F:B1:C3"
  ```

To add a **third** anchor, copy the `[env:anchor2]` block, rename it
(`[env:anchor3]`) and give it a new EUI — just keep the first two bytes unique.

To change the **tag** address, edit `TAG_ADDR` in `src/tag/main.cpp`.

### 3. Build

**PlatformIO IDE**: click the ✔ (build) icon for the environment you want.
**CLI**:

```bash
pio run -e tag         # build tag firmware
pio run -e anchor      # build anchor #1 firmware
pio run -e anchor2     # build anchor #2 firmware (unique address)
pio run                # build the default env (tag)
```

### 4. Flash the firmware
Upload the matching environment to each board (one board connected at a time):

| Board | Command |
|---|---|
| Anchor #1 | `pio run -e anchor -t upload` |
| Anchor #2 | `pio run -e anchor2 -t upload --upload-port COM11` |
| Tag | `pio run -e tag -t upload` |

If multiple boards are connected, pick the port with `--upload-port`:

```bash
pio run -e anchor2 -t upload --upload-port COM11
```

(On Windows the ports look like `COM3`, `COM5`, `COM11`, …)

> **Stuck at "Connecting..." during upload?** The board resets automatically
> most of the time, but if it doesn't: hold **BOOT**, tap **RST/EN**, release
> BOOT, then upload — and press RST once after uploading.

### 5. Monitor
Open the Serial Monitor (115200 baud) on either board:

```bash
pio device monitor -e tag -b 115200
```

---

## Expected Serial output

**Tag** (`env:tag`) — with two anchors running, the tag alternates between them:

```
[UWB] Tag starting...
[UWB] Tag 7D:00:22:EA:82:60:3B:9B started. Waiting for anchors...
anchor added -> short: 0x8617
anchor added -> short: 0x3D4E
from: 0x8617	Range: 1.484 m	RX power: -67.19 dBm
from: 0x3D4E	Range: 2.731 m	RX power: -71.04 dBm
from: 0x8617	Range: 1.486 m	RX power: -67.31 dBm
from: 0x3D4E	Range: 2.728 m	RX power: -70.98 dBm
```

**Anchor #1** (`env:anchor`):

```
[UWB] Anchor starting...
[UWB] Anchor 86:17:5B:D5:A9:9A:E2:9C started. Waiting for tags...
tag added -> short: 0x7D00
from: 0x7D00	Range: 1.484 m	RX power: -67.19 dBm
```

**Anchor #2** (`env:anchor2`) — same output, different IDs:

```
[UWB] Anchor starting...
[UWB] Anchor 3D:4E:7C:2A:08:5F:B1:C3 started. Waiting for tags...
tag added -> short: 0x7D00
from: 0x7D00	Range: 2.731 m	RX power: -71.04 dBm
```

## OLED display
- **Tag**: shows `No Anchor — waiting...` until the first anchor is seen,
  then the latest distance (big), the measured anchor, RX power, and count of
  active anchors.
- **Anchor**: shows its own address; when a tag ranges, the last measured
  distance and the tag's short address appear.

---

## Alternative: Arduino IDE

If you prefer the Arduino IDE over PlatformIO, copy `src/tag/main.cpp` and
`src/anchor/main.cpp` into sketch folders (rename them `main.ino`; the Arduino
IDE auto-generates prototypes, so the explicit forward declarations are
harmless).

1. Install the **ESP32** board package (Boards Manager: `esp32 by Espressif Systems`).
2. Install libraries (Sketch → Include Library → Manage Libraries):
   - **Adafruit SSD1306** + **Adafruit GFX Library**
   - **Makerfabs DW1000** from ZIP (`mf_DW1000.zip`, from the Makerfabs repo)
     — or copy the vendored `lib/DW1000` folder into your Arduino `libraries/`
3. Board: `Tools → Board → ESP32 Arduino → ESP32 Dev Module`
4. Open the sketch, select the port, click Upload.

For a second anchor in the Arduino IDE, just change the default in
`src/anchor/main.cpp` (the `#define` inside the `#ifndef ANCHOR_EUI` block):

```cpp
#define ANCHOR_EUI "3D:4E:7C:2A:08:5F:B1:C3"
```

---

## How it works

1. The **tag** broadcasts ranging polls; each **anchor** replies.
2. `DW1000Ranging` measures the round-trip time of flight → distance.
3. `newRange()` fires on every completed measurement (tag and anchor) — the
   values are printed, and the tag refreshes the OLED ~2×/second.

```
Tag ──poll──▶ Anchor │ Anchor ──ack──▶ Tag │ Tag computes range
```

## Notes & tips
- Move the tag **slowly** while testing; UWB picks up reflections in cluttered
  rooms, so expect ±10–30 cm noise.
- Try `DW1000.MODE_LONGDATA_FAST_ACCURACY` (or enable
  `DW1000Ranging.useRangeFilter(true)`) for smoother results.
- For positioning (2D/3D coordinates from multiple anchors), see the
  Makerfabs `IndoorPositioning` / `OutdoorPositioning_display` examples.

## Reference
- Makerfabs repo: https://github.com/Makerfabs/Makerfabs-ESP32-UWB
- Board product page: https://www.makerfabs.com/esp32-uwb-pro-with-display.html