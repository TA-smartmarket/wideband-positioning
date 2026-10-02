# Wideband Positioning — ESP32 UWB (Distance)

UWB (Ultra-Wideband) distance measurement examples for the
**Makerfabs ESP32 UWB Pro with Display** boards, based on the official
[Makerfabs-ESP32-UWB](https://github.com/Makerfabs/Makerfabs-ESP32-UWB) library.

Two role sketches are provided:

| Sketch | Role | What it does |
|---|---|---|
| [`examples/uwb_tag_distance`](examples/uwb_tag_distance/uwb_tag_distance.ino) | **Tag** (moving) | Measures distance to every anchor; prints + shows the latest distance on the OLED |
| [`examples/uwb_anchor_distance`](examples/uwb_anchor_distance/uwb_anchor_distance.ino) | **Anchor** (fixed) | Replies to ranging requests from tags; prints each measured distance and shows its status on the OLED |

> The tag initiates ranging, so the distance value appears on **both** the
> tag and the anchor's Serial output.

---

## Requirements

### Hardware
- 2× Makerfabs ESP32 UWB Pro with Display (or 1 tag + as many anchors as you like — multi-anchor works out of the box)
- USB-C cables for programming

### Software (Arduino IDE)
1. **Arduino IDE 2.x** with the **ESP32 board package** (`arduino:esp32`)
   - Preferences → Additional boards manager URLs: `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
   - Boards Manager → install **esp32 by Espressif Systems**
2. **Libraries** (Sketch → Include Library → Manage Libraries):
   - **Makerfabs DW1000** — install from ZIP: `https://github.com/Makerfabs/Makerfabs-ESP32-UWB` → download `mf_DW1000.zip`
   - **Adafruit SSD1306**
   - **Adafruit GFX Library** (dependency, installed automatically with SSD1306)

### Boards select
`Tools → Board → ESP32 Arduino → ESP32 Dev Module`

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

## Getting Started

### 1. Unique addresses (very important!)
Every board must have its **own unique 8-byte UWB address**.

- Tag: edit `TAG_ADDR` in `examples/uwb_tag_distance/uwb_tag_distance.ino`
- Anchor: edit `ANCHOR_ADDR` in `examples/uwb_anchor_distance/uwb_anchor_distance.ino`

Keep the same value across both files for each role you flash. Example:

```cpp
#define TAG_ADDR    "7D:00:22:EA:82:60:3B:9B"   // tag
#define ANCHOR_ADDR "86:17:5B:D5:A9:9A:E2:9C"   // anchor
```

### 2. Flash the firmware
1. Flash **`uwb_anchor_distance`** on the fixed anchor board.
2. Flash **`uwb_tag_distance`** on the mobile tag board.
3. Open **Serial Monitor** (115200 baud) on either board.

### 3. Expected Serial output
**Tag** (`uwb_tag_distance`):

```
[UWB] Tag starting...
[UWB] Tag 7D:00:22:EA:82:60:3B:9B started. Waiting for anchors...
anchor added -> short: 0xE29C
from: 0xE29C	Range: 1.484 m	RX power: -67.19 dBm
from: 0xE29C	Range: 1.486 m	RX power: -67.31 dBm
```

**Anchor** (`uwb_anchor_distance`):

```
[UWB] Anchor starting...
[UWB] Anchor 86:17:5B:D5:A9:9A:E2:9C started. Waiting for tags...
tag added -> short: 0x3B9B
from: 0x3B9B	Range: 1.484 m	RX power: -67.19 dBm
from: 0x3B9B	Range: 1.486 m	RX power: -67.31 dBm
```

### 4. OLED display
- **Tag**: shows `No Anchor — waiting...` until the first anchor is seen,
  then the latest distance (big), the measured anchor, RX power, and count of
  active anchors.
- **Anchor**: shows its own address; when a tag ranges, the last measured
  distance and the tag's short address appear.

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