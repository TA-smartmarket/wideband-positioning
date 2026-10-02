/*
 * ============================================================
 *  UWB Anchor - Distance Measurement, fixed node (PlatformIO)
 *  Board : Makerfabs ESP32 UWB Pro with Display (DW1000)
 *  Role  : ANCHOR (the fixed node the tag ranges against)
 *  Build : pio run -e anchor
 * ============================================================
 *
 *  This firmware turns the board into a UWB anchor.
 *  It answers ranging requests from the tag and:
 *    - prints every measured distance to Serial (115200 baud)
 *    - shows its own address + last distance on the OLED
 *
 *  Flash the companion firmware (tag) on the board that
 *  should move around and measure distance.
 *  Any number of anchors can be added - the tag shows the
 *  distance to anchors one by one.
 */

#include <SPI.h>
#include "DW1000Ranging.h"

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/*
 * UWB address of this anchor.
 * Every UWB node on the network MUST have its own unique address
 * (the first two bytes become the short address used in the network).
 *
 * The default below is anchor #1. For the second anchor (env:anchor2)
 * platformio.ini overrides it with -DANCHOR_EUI="..." so each board
 * gets a different address without touching this file.
 */
#ifndef ANCHOR_EUI
#define ANCHOR_EUI "86:17:5B:D5:A9:9A:E2:9C"
#endif

// Non-const array: the DW1000 library takes char* (writable), so a plain
// string literal would trigger -Wwrite-strings.
char ANCHOR_ADDR[] = ANCHOR_EUI;

/*
 * Pinout of the ESP32 UWB Pro with Display.
 * (Already wired on the board — do not change unless necessary.)
 */
// DW1000 via SPI
#define SPI_SCK  18
#define SPI_MISO 19
#define SPI_MOSI 23
#define UWB_RST  27 // reset pin
#define UWB_IRQ  34 // interrupt pin
#define UWB_SS   21 // chip-select pin
// SSD1306 OLED via I2C
#define I2C_SDA 4
#define I2C_SCL 5
#define OLED_ADDR 0x3C

Adafruit_SSD1306 display(128, 64, &Wire, -1);

// ------------------------------------------------------------------
// Callbacks & helpers — declared here because setup()/loop() call them
// and (unlike the Arduino IDE) PlatformIO compiles each file directly.
// ------------------------------------------------------------------
void newRange();
void newBlink(DW1000Device *device);
void inactiveDevice(DW1000Device *device);
void showLogo();
void updateDisplay(uint16_t tagAddr, float range, float rxPower, unsigned long now);
void drawScreen();

// ------------------------------------------------------------------
// Setup
// ------------------------------------------------------------------
void setup()
{
    Serial.begin(115200);
    delay(500);
    Serial.println(F("[UWB] Anchor starting..."));

    // OLED
    Wire.begin(I2C_SDA, I2C_SCL);
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR))
    {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;)
            ; // stop here, display hardware problem
    }
    display.clearDisplay();
    showLogo();

    // DW1000
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    DW1000Ranging.initCommunication(UWB_RST, UWB_SS, UWB_IRQ);

    DW1000Ranging.attachNewRange(newRange);             // distance measured
    DW1000Ranging.attachBlinkDevice(newBlink);          // tag announced itself
    DW1000Ranging.attachInactiveDevice(inactiveDevice); // tag disappeared

    // Start as ANCHOR. The tag initiates the ranging.
    DW1000Ranging.startAsAnchor(ANCHOR_ADDR, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);

    Serial.print(F("[UWB] Anchor "));
    Serial.print(ANCHOR_ADDR);
    Serial.println(F(" started. Waiting for tags..."));
}

// ------------------------------------------------------------------
// Main loop
// ------------------------------------------------------------------
void loop()
{
    DW1000Ranging.loop(); // keep the ranging protocol running
    drawScreen();         // refresh OLED ~2x per second
}

// ------------------------------------------------------------------
// DW1000Ranging callbacks
// ------------------------------------------------------------------

// Called whenever the tag finished a distance measurement with us.
void newRange()
{
    DW1000Device *d = DW1000Ranging.getDistantDevice();

    Serial.print(F("from: 0x"));
    Serial.print(d->getShortAddress(), HEX);
    Serial.print(F("\tRange: "));
    Serial.print(d->getRange(), 3);
    Serial.print(F(" m\tRX power: "));
    Serial.print(d->getRXPower());
    Serial.println(F(" dBm"));

    updateDisplay(d->getShortAddress(), d->getRange(), d->getRXPower(), millis());
}

// A tag (or another anchor) blinked and joined the network.
void newBlink(DW1000Device *device)
{
    Serial.print(F("tag added -> short: 0x"));
    Serial.println(device->getShortAddress(), HEX);
}

// A tag stopped responding and was removed.
void inactiveDevice(DW1000Device *device)
{
    Serial.print(F("tag removed -> short: 0x"));
    Serial.println(device->getShortAddress(), HEX);
}

// ------------------------------------------------------------------
// OLED screen
// ------------------------------------------------------------------

// Values copied out of the DW1000 callback so they stay valid in loop()
float    lastRange     = 0.0f;
float    lastRXPower   = 0.0f;
uint16_t lastTagAddr   = 0;
unsigned long lastRangeMillis = 0;

void showLogo(void)
{
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("Makerfabs"));
    display.setTextSize(1);
    display.setCursor(0, 20);
    display.println(F("UWB Anchor"));
    display.setCursor(0, 40);
    display.println(ANCHOR_ADDR);
    display.display();
    delay(1500);
}

unsigned long lastScreen = 0;

void updateDisplay(uint16_t tagAddr, float range, float rxPower, unsigned long now)
{
    lastTagAddr    = tagAddr;
    lastRange      = range;
    lastRXPower    = rxPower;
    lastRangeMillis = now;
}

// Small helper so loop() stays tiny but the screen still refreshes:
// called from loop() below via DW1000Ranging; kept on a timer here.
void drawScreen()
{
    if (millis() - lastScreen < 500)
        return;
    lastScreen = millis();

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    if (lastTagAddr == 0 || millis() - lastRangeMillis > 3000)
    {
        display.setTextSize(1);
        display.setCursor(0, 0);
        display.println(F("Anchor"));
        display.setCursor(0, 12);
        display.println(ANCHOR_ADDR);
        display.setCursor(0, 40);
        display.println(F("Distance: --.- m"));
        display.display();
        return;
    }

    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println(F("Anchor"));
    display.setCursor(0, 12);
    display.println(ANCHOR_ADDR);

    display.setTextSize(2);
    display.setCursor(0, 28);
    display.print(lastRange, 2);
    display.println(F(" m"));

    display.setTextSize(1);
    display.setCursor(0, 50);
    display.print(F("tag 0x"));
    display.print(lastTagAddr, HEX);

    display.display();
}