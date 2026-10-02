/*
 * ============================================================
 *  UWB Tag - Distance Measurement (PlatformIO)
 *  Board : Makerfabs ESP32 UWB Pro with Display (DW1000)
 *  Role  : TAG  (the moving node that measures distance)
 *  Build : pio run -e tag
 * ============================================================
 *
 *  This firmware turns the board into a UWB tag.
 *  It ranges against every nearby anchor (fixed node) and:
 *    - prints distance + RX power to Serial (115200 baud)
 *    - shows the latest distance on the built-in SSD1306 OLED
 *
 *  Flash the companion firmware (anchor) on at least one
 *  other board so there is an anchor to range with.
 */

#include <SPI.h>
#include "DW1000Ranging.h"

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/*
 * UWB address of this tag.
 * Every UWB node on the network MUST have its own unique address.
 * Keep the same address as the one in src/anchor/main.cpp.
 */
// Non-const array: the DW1000 library takes char* (writable), so a plain
// string literal would trigger -Wwrite-strings.
char TAG_ADDR[] = "7D:00:22:EA:82:60:3B:9B";

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
void newDevice(DW1000Device *device);
void inactiveDevice(DW1000Device *device);
void showLogo();
void updateDisplay();

// ------------------------------------------------------------------
// Ranging state (updated from DW1000 interrupts, read from loop)
// ------------------------------------------------------------------
float    lastRange      = 0.0f;   // measured distance in meters
float    lastRXPower    = 0.0f;   // received signal strength in dBm
uint16_t lastAnchorAddr = 0;      // short address of the anchor
int      anchorCount    = 0;      // number of anchors currently active
unsigned long lastRangeMillis = 0;

// ------------------------------------------------------------------
// Setup
// ------------------------------------------------------------------
void setup()
{
    Serial.begin(115200);
    delay(500);
    Serial.println(F("[UWB] Tag starting..."));

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
    DW1000Ranging.attachNewDevice(newDevice);           // new anchor appeared
    DW1000Ranging.attachInactiveDevice(inactiveDevice); // anchor disappeared

    // Start as TAG (the node that measures distance).
    // MODE_LONGDATA_RANGE_LOWPOWER gives the best range at low power.
    // randomShortAddress = false -> short address is derived from the first
    // two bytes of TAG_ADDR (0x7D00), so the tag keeps a stable ID instead
    // of a new random one on every boot.
    DW1000Ranging.startAsTag(TAG_ADDR, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);

    Serial.print(F("[UWB] Tag "));
    Serial.print(TAG_ADDR);
    Serial.println(F(" started. Waiting for anchors..."));
}

// ------------------------------------------------------------------
// Main loop
// ------------------------------------------------------------------
void loop()
{
    DW1000Ranging.loop(); // keep the ranging protocol running
    updateDisplay();      // refresh OLED ~2x per second
}

// ------------------------------------------------------------------
// DW1000Ranging callbacks
// ------------------------------------------------------------------

// Called whenever a distance measurement with an anchor completes.
void newRange()
{
    DW1000Device *d = DW1000Ranging.getDistantDevice();

    lastAnchorAddr   = d->getShortAddress();
    lastRange        = d->getRange();
    lastRXPower      = d->getRXPower();
    lastRangeMillis  = millis();

    Serial.print(F("from: 0x"));
    Serial.print(lastAnchorAddr, HEX);
    Serial.print(F("\tRange: "));
    Serial.print(lastRange, 3);
    Serial.print(F(" m\tRX power: "));
    Serial.print(lastRXPower);
    Serial.println(F(" dBm"));
}

// A new anchor joined the network.
void newDevice(DW1000Device *device)
{
    anchorCount++;
    Serial.print(F("anchor added -> short: 0x"));
    Serial.println(device->getShortAddress(), HEX);
}

// An anchor stopped responding and was removed.
void inactiveDevice(DW1000Device *device)
{
    if (anchorCount > 0)
        anchorCount--;
    Serial.print(F("anchor removed -> short: 0x"));
    Serial.println(device->getShortAddress(), HEX);
}

// ------------------------------------------------------------------
// OLED screen
// ------------------------------------------------------------------

void showLogo(void)
{
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("Makerfabs"));
    display.setTextSize(1);
    display.setCursor(0, 20);
    display.println(F("UWB TAG"));
    display.setCursor(0, 40);
    display.println(TAG_ADDR);
    display.display();
    delay(1500);
}

unsigned long lastScreen = 0;

void updateDisplay()
{
    if (millis() - lastScreen < 500)
        return;
    lastScreen = millis();

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    if (anchorCount == 0)
    {
        display.setTextSize(2);
        display.setCursor(0, 0);
        display.println(F("No Anchor"));
        display.setTextSize(1);
        display.setCursor(0, 40);
        display.println(F("waiting..."));
        display.display();
        return;
    }

    // Latest distance — big and clear
    display.setTextSize(2);
    display.setCursor(0, 2);
    display.print(lastRange, 2);
    display.println(F(" m"));

    display.setTextSize(1);
    display.setCursor(0, 26);
    display.print(F("Anchor 0x"));
    display.print(lastAnchorAddr, HEX);

    display.setCursor(0, 38);
    display.print(F("RX "));
    display.print(lastRXPower, 1);
    display.println(F(" dBm"));

    display.setCursor(0, 52);
    display.print(F("active: "));
    display.print(anchorCount);

    display.display();
}