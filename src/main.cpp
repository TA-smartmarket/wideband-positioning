// ============================================================================
//  ESP32 UWB positioning node — single firmware for tag and anchor
//
//  Role (tag/anchor), id (1..10), WiFi, server and MQTT settings are runtime
//  configuration stored in NVS, so one binary runs on every board.
//
//  Configuration can be changed three ways, all equivalent:
//    1. web UI on the server        (recommended)
//    2. setup portal AP "UWB-Setup" (http://192.168.4.1) — used on first boot
//    3. serial menu                 (type '?' in the monitor)
//
//  Data goes to the server over REST and/or MQTT (docs/API.md).
// ============================================================================

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "DW1000Ranging.h"

#include "config.h"
#include "solver.h"
#include "ekf.h"
#include "net.h"
#include "ota.h"

// ---------------------------------------------------------------------------
// globals (declared extern in net.h)
// ---------------------------------------------------------------------------
Config cfg;
Net    net;

WiFiClient   g_wifi_client;
PubSubClient g_mqtt(g_wifi_client);
WebServer    g_portal(80);
WebServer    g_ota(3232);
bool         ota_busy = false;   // set while an OTA download is running


// ---------------------------------------------------------------------------
// board pinout (fixed on the Makerfabs ESP32 UWB Pro with Display)
// ---------------------------------------------------------------------------
#define SPI_SCK  18
#define SPI_MISO 19
#define SPI_MOSI 23
#define UWB_RST  27
#define UWB_IRQ  34
#define UWB_SS   21
#define I2C_SDA  4
#define I2C_SCL  5
#define OLED_ADDR 0x3C

Adafruit_SSD1306 display(128, 64, &Wire, -1);

// ---------------------------------------------------------------------------
// telemetry buffer
// ---------------------------------------------------------------------------
#define MAX_RANGES 16

struct RangeRec {
    char  src[16];
    char  dst[16];
    float range;
    float rx;
    float fp;
    float quality;
    uint32_t ts;
};

RangeRec ranges[MAX_RANGES];
uint8_t  range_count = 0;

struct PosRec {
    char  id[16];
    float x, y;
    float confidence;
    bool  ambiguous;
    bool  geometry_ok;
    float geometry_slack;
};
PosRec positions[MAX_TAGS];
uint8_t pos_count = 0;

uint32_t last_telem_ms = 0;
uint8_t  anchors_seen = 0, tags_seen = 0;
unsigned long boot_ms = 0;

// ---------------------------------------------------------------------------
// UWB helpers
// ---------------------------------------------------------------------------

// Short address layout: 0xA0NN = anchor NN, 0x7DNN = tag NN (see config.h).
inline void shortToDeviceId(uint16_t shortAddr, char *out, size_t n)
{
    uint8_t hi = (shortAddr >> 8) & 0xFF;
    uint8_t lo = shortAddr & 0xFF;
    const char *role = (hi == 0xA0) ? "anchor" : (hi == 0x7D ? "tag" : "dev");
    snprintf(out, n, "%s-%u", role, (unsigned)lo);
}

// ---------------------------------------------------------------------------
// Robust per-link range filter
//
// The DW1000 library's own `useRangeFilter()` is a plain low-pass that stores
// its previous output inside DW1000Device. When one sample is wrong the bad
// value is fed back forever, so the range walks away (observed: 1.5 m drifting
// to 248 m). It is therefore DISABLED and replaced by this filter:
//   * reject physically impossible jumps (outlier gate)
//   * median-of-3 over the recent history (kills single-sample spikes)
//   * light EMA for smoothing
// ---------------------------------------------------------------------------
#define MAX_LINKS  MAX_RANGES
#define RANGE_HIST 3
#define MAX_JUMP_M 5.0f      // a real person cannot jump 5 m between samples
#define EMA_ALPHA  0.35f

struct RangeFilter {
    float   hist[RANGE_HIST];
    uint8_t n;
    float   out;
    bool    seeded;
    char    key[24];         // "src>dst", identifies the link
    bool    used;
};

RangeFilter filters[MAX_LINKS];

inline float rfUpdate(RangeFilter &f, float raw)
{
    // outlier gate: ignore jumps that are not physically plausible
    if (f.seeded && fabsf(raw - f.out) > MAX_JUMP_M) return f.out;

    // median of the last RANGE_HIST samples (+ the new one)
    float s[RANGE_HIST + 1];
    uint8_t n = 0;
    for (uint8_t i = 0; i < f.n; i++) s[n++] = f.hist[i];
    s[n++] = raw;
    for (uint8_t i = 0; i < n; i++)                 // insertion sort, n <= 4
        for (uint8_t j = i + 1; j < n; j++)
            if (s[j] < s[i]) { float t = s[i]; s[i] = s[j]; s[j] = t; }
    const float med = s[n / 2];

    // keep a short history
    if (f.n < RANGE_HIST) f.hist[f.n++] = raw;
    else {
        memmove(&f.hist[0], &f.hist[1], sizeof(float) * (RANGE_HIST - 1));
        f.hist[RANGE_HIST - 1] = raw;
    }

    f.out = f.seeded ? (EMA_ALPHA * med + (1.0f - EMA_ALPHA) * f.out) : med;
    f.seeded = true;
    return f.out;
}

// Filter the raw range for this link (creating state on first use).
inline float filterRange(const char *src, const char *dst, float raw)
{
    char key[24];
    snprintf(key, sizeof(key), "%s>%s", src, dst);

    for (uint8_t i = 0; i < MAX_LINKS; i++)
        if (filters[i].used && !strcmp(filters[i].key, key))
            return rfUpdate(filters[i], raw);

    for (uint8_t i = 0; i < MAX_LINKS; i++) {
        if (!filters[i].used) {
            memset(&filters[i], 0, sizeof(RangeFilter));
            snprintf(filters[i].key, sizeof(filters[i].key), "%s", key);
            filters[i].used = true;
            return rfUpdate(filters[i], raw);
        }
    }
    return raw;   // no slot: pass through
}

// Forget a link's filter state (called when the peer is lost).
inline void filterForget(const char *id)
{
    for (uint8_t i = 0; i < MAX_LINKS; i++) {
        if (!filters[i].used) continue;
        if (strstr(filters[i].key, id)) {
            filters[i].used = false;
            filters[i].seeded = false;
            filters[i].n = 0;
        }
    }
}

void pushRange(const char *src, const char *dst, float range, float rx, float fp, float q)
{
    // replace the entry for the same pair, else append
    int8_t slot = -1;
    for (uint8_t i = 0; i < range_count; i++)
        if (!strcmp(ranges[i].src, src) && !strcmp(ranges[i].dst, dst)) { slot = i; break; }
    if (slot < 0) {
        if (range_count >= MAX_RANGES) {
            memmove(&ranges[0], &ranges[1], sizeof(RangeRec) * (MAX_RANGES - 1));
            slot = MAX_RANGES - 1;
        } else {
            slot = range_count++;
        }
    }
    RangeRec &r = ranges[slot];
    strncpy(r.src, src, sizeof(r.src) - 1); r.src[sizeof(r.src) - 1] = 0;
    strncpy(r.dst, dst, sizeof(r.dst) - 1); r.dst[sizeof(r.dst) - 1] = 0;
    r.range = cfg.range_filter ? filterRange(src, dst, range) : range;
    r.rx = rx; r.fp = fp; r.quality = q;
    r.ts = millis();
}

// Build the telemetry JSON (docs/API.md section 3).
String buildTelemetry()
{
    JsonDocument doc;
    char me[16];
    deviceId(cfg, me, sizeof(me));

    doc["site"] = cfg.site;
    doc["device_id"] = me;
    doc["ts"] = (uint32_t)millis();

    JsonArray ra = doc["ranges"].to<JsonArray>();
    for (uint8_t i = 0; i < range_count; i++) {
        JsonObject o = ra.add<JsonObject>();
        o["ts"] = ranges[i].ts;
        o["src"] = ranges[i].src;
        o["dst"] = ranges[i].dst;
        jsonPutFloat(o, "range", ranges[i].range);
        jsonPutFloat(o, "rx_power", ranges[i].rx);
        jsonPutFloat(o, "fp_power", ranges[i].fp);
        jsonPutFloat(o, "quality", ranges[i].quality);
    }

    if (pos_count) {
        JsonArray pa = doc["positions"].to<JsonArray>();
        for (uint8_t i = 0; i < pos_count; i++) {
            JsonObject o = pa.add<JsonObject>();
            o["ts"] = (uint32_t)millis();
            o["id"] = positions[i].id;
            jsonPutFloat(o, "x", positions[i].x);
            jsonPutFloat(o, "y", positions[i].y);
            jsonPutFloat(o, "confidence", positions[i].confidence);
            o["ambiguous"] = positions[i].ambiguous;
            o["geometry_ok"] = positions[i].geometry_ok;
            jsonPutFloat(o, "geometry_slack", positions[i].geometry_slack);
            o["source"] = "device";
        }
    }

    JsonObject st = doc["status"].to<JsonObject>();
    if (net.wifi_up) st["ip"] = WiFi.localIP().toString();
    st["rssi"] = net.wifi_up ? WiFi.RSSI() : 0;
    st["uptime_s"] = (millis() - boot_ms) / 1000;
    st["anchors_seen"] = anchors_seen;
    st["tags_seen"] = tags_seen;
    st["fw"] = FW_VERSION;

    String out;
    serializeJson(doc, out);
    return out;
}

// ---------------------------------------------------------------------------
// telemetry uplink (non-blocking)
//
// HTTP POSTs used to run inline in loop(), which stalls DW1000Ranging.loop()
// for up to the HTTP timeout (1.5 s). The ranging protocol then misses its
// reply window and the peer is dropped as "inactive" (1 s timeout) — the
// visible symptom is a link that keeps dropping.
//
// The uplink therefore runs in its own FreeRTOS task on the other core.
// ---------------------------------------------------------------------------
String  pending_json;
bool    pending_rest = false;
SemaphoreHandle_t pending_lock = nullptr;

void uplinkTask(void *)
{
    for (;;) {
        String body;
        bool   do_rest = false;

        if (xSemaphoreTake(pending_lock, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (pending_rest) { body = pending_json; do_rest = true; pending_rest = false; }
            xSemaphoreGive(pending_lock);
        }

        // Cooperative: stand down completely while an OTA owns the network.
        if (ota_busy) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }
        if (do_rest && net.wifi_up && cfg.server_url[0])
            httpPostJson("/api/v1/telemetry", body);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// Queue a batch for the uplink task. Never blocks the ranging loop.
//
// The mutex may not exist yet during early boot, and taking a FreeRTOS
// semaphore before the scheduler is running trips
//   assert failed: xQueueSemaphoreTake queue.c:1554
// so bail out instead of crashing.
void queueRest(const String &body)
{
    if (pending_lock == nullptr) return;
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) return;
    if (xSemaphoreTake(pending_lock, 0) == pdTRUE) {
        pending_json = body;
        pending_rest = true;
        xSemaphoreGive(pending_lock);
    }
}

void publishTelemetry()
{
    String body = buildTelemetry();

    // MQTT: non-blocking publishes, safe to do inline
    if (net.mqtt_up) {
        char t[80];
        topic(t, sizeof(t), "telemetry");
        mqttPublish(t, body);
        for (uint8_t i = 0; i < range_count; i++) {
            JsonDocument d;
            d["ts"] = ranges[i].ts;
            d["src"] = ranges[i].src;
            d["dst"] = ranges[i].dst;
            jsonPutFloat(d, "range", ranges[i].range);
            jsonPutFloat(d, "rx_power", ranges[i].rx);
            jsonPutFloat(d, "fp_power", ranges[i].fp);
            jsonPutFloat(d, "quality", ranges[i].quality);
            String s; serializeJson(d, s);
            topic(t, sizeof(t), "range");
            mqttPublish(t, s);
        }
    }

    // REST: hand off to the uplink task (also the fallback when MQTT is down)
    if (!net.mqtt_up || !cfg.mqtt_enabled) queueRest(body);
    else queueRest("");          // nothing pending, keeps the slot clear

    // NOTE: the range buffer is NOT cleared here. It used to be, which emptied
    // `range_count` every telemetry cycle (~1 s). drawUi() then hit its
    // "no range yet..." branch on the next frame, so the OLED flickered
    // between the distance and that placeholder forever. Staleness is handled
    // per record via the timestamp instead (see drawUi).
}

// ---------------------------------------------------------------------------
// standalone solver (tag only): uses ranges collected from anchors
// ---------------------------------------------------------------------------
AnchorFix fixes[MAX_ANCHORS];

// Display power saving (defined with drawUi(); used from the ranging callback)
void screenWake();
void screenPowerLoop();

// ---------------------------------------------------------------------------
// Localisation: bootstrap with the geometric solver, then track with the EKF.
//
//  * The closed-form solver gives the first fix (and a sanity check).
//  * From then on the EKF fuses every anchor range over time: it smooths the
//    noise, estimates velocity, and its innovation gate rejects NLOS/outlier
//    measurements (see src/ekf.h for the model and Jacobians).
// ---------------------------------------------------------------------------
Ekf tag_ekf;
float ekf_last_solve_ms = 0;
uint8_t ekf_rejects = 0;      // consecutive cycles with no accepted measurement

// Triangle-inequality test on the collected fixes. A violation means at least
// one anchor is measuring a reflection (NLOS) rather than the direct path.
// Mirrors geometry_check() in server/app.py.
float geometrySlack(const AnchorFix *f, uint8_t n)
{
    float worst = 0.0f;
    for (uint8_t i = 0; i < n; i++) {
        for (uint8_t j = i + 1; j < n; j++) {
            const float dx = f[j].x - f[i].x, dy = f[j].y - f[i].y;
            const float d = sqrtf(dx * dx + dy * dy);
            if (d < 1e-6f) continue;
            if (f[i].range + f[j].range < d)
                worst = fmaxf(worst, d - (f[i].range + f[j].range));
            else if (fabsf(f[i].range - f[j].range) > d)
                worst = fmaxf(worst, fabsf(f[i].range - f[j].range) - d);
        }
    }
    return worst;
}

void solveLocally()
{
    if (cfg.role != ROLE_TAG) return;

    uint8_t n = 0;
    for (uint8_t i = 0; i < range_count && n < MAX_ANCHORS; i++) {
        if (strncmp(ranges[i].src, "anchor", 6)) continue;
        uint8_t aid = (uint8_t)atoi(ranges[i].src + 7);
        if (aid < 1 || aid > MAX_ANCHORS) continue;

        // anchor positions come from the config pushed by the server
        float ax, ay;
        if (!anchorPos(cfg, aid, ax, ay)) continue;
        fixes[n].x = ax; fixes[n].y = ay;
        fixes[n].range = ranges[i].range;
        fixes[n].valid = true;
        n++;
    }
    if (n < 2) { pos_count = 0; return; }

    char me[16];
    deviceId(cfg, me, sizeof(me));

    // --- bootstrap: first fix from the geometric solver --------------------
    if (!tag_ekf.init) {
        Position p = solvePosition(fixes, n, cfg.room_w, cfg.room_h);
        if (!p.valid) { pos_count = 0; return; }
        ekfInit(tag_ekf, p.x, p.y);
        ekf_last_solve_ms = millis();
    } else {
        // --- track: predict + one range update per anchor ------------------
        const uint32_t now = millis();
        ekfPredict(tag_ekf, (now - ekf_last_solve_ms) / 1000.0f);
        ekf_last_solve_ms = now;
    }

    uint8_t used = 0;
    for (uint8_t i = 0; i < n; i++)
        if (ekfUpdateRange(tag_ekf, fixes[i].x, fixes[i].y, fixes[i].range))
            used++;

    // Self-healing: if every measurement keeps being rejected the filter has
    // lost the track (its covariance is at the cap). Restart it from the
    // current geometry instead of coasting on a stale estimate. Mirrors the
    // EKF_REJECTS logic in server/app.py.
    if (used == 0) {
        ekf_rejects++;
        if (ekf_rejects >= 8) {
            Position p = solvePosition(fixes, n, cfg.room_w, cfg.room_h);
            if (p.valid) {
                ekfInit(tag_ekf, p.x, p.y);
                ekf_rejects = 0;
                for (uint8_t i = 0; i < n; i++)
                    if (ekfUpdateRange(tag_ekf, fixes[i].x, fixes[i].y, fixes[i].range))
                        used++;
            }
        }
    } else {
        ekf_rejects = 0;
    }

    float px, py, vx, vy;
    ekfPosition(tag_ekf, px, py, vx, vy);

    // Never publish a position outside the room: with two anchors an
    // inconsistent range pair can throw the estimate far away. Mirrors the
    // clamp in server/app.py.
    const float margin = 0.5f;
    const float cx = fminf(fmaxf(px, -margin), cfg.room_w + margin);
    const float cy = fminf(fmaxf(py, -margin), cfg.room_h + margin);
    if (fabsf(cx - px) > 1e-6f || fabsf(cy - py) > 1e-6f) {
        tag_ekf.x[0] = cx; tag_ekf.x[1] = cy;
        tag_ekf.x[2] *= 0.2f; tag_ekf.x[3] *= 0.2f;
        if (tag_ekf.P[0][0] < 0.5f) tag_ekf.P[0][0] = 0.5f;
        if (tag_ekf.P[1][1] < 0.5f) tag_ekf.P[1][1] = 0.5f;
        px = cx; py = cy;
    }

    const float sigma = ekfPositionSigma(tag_ekf);
    const float slack = geometrySlack(fixes, n);

    snprintf(positions[0].id, sizeof(positions[0].id), "%s", me);
    positions[0].x = px;
    positions[0].y = py;
    positions[0].confidence = 1.0f / (1.0f + sigma);   // 0..1, from the covariance
    positions[0].ambiguous = false;                     // EKF resolves the mirror
    positions[0].geometry_ok = (slack <= 0.0f);
    positions[0].geometry_slack = slack;
    pos_count = 1;

    Serial.printf("[ekf] x %.2f y %.2f v(%.2f,%.2f) sigma %.2f used %u/%u%s\n",
                  px, py, vx, vy, sigma, used, n,
                  positions[0].geometry_ok ? "" : "  NLOS-geometry!");
}

// ---------------------------------------------------------------------------
// DW1000 callbacks
// ---------------------------------------------------------------------------
void newRange()
{
    DW1000Device *d = DW1000Ranging.getDistantDevice();
    if (!d) return;

    char me[16], other[16];
    deviceId(cfg, me, sizeof(me));
    shortToDeviceId(d->getShortAddress(), other, sizeof(other));

    const float range = d->getRange();
    const float rx    = d->getRXPower();
    const float fp    = DW1000.getFirstPathPower();
    const float q     = DW1000.getReceiveQuality();

    // The smoothing filter can overshoot below zero on the first samples;
    // a negative distance is meaningless and breaks the solver.
    if (range <= 0.01f) return;

    // A live measurement is NOT treated as activity for the display: ranging
    // runs continuously, so waking on it kept the screen on forever and the
    // dim/off timeout never fired. The panel wakes on serial input and on
    // server traffic instead (see handleSerial / onServerConfig).
    if (cfg.role == ROLE_TAG) pushRange(other, me, range, rx, fp, q);
    else                      pushRange(me, other, range, rx, fp, q);

    Serial.printf("range %s -> %s  %.3f m  rx %.1f dBm  fp %.1f dBm\n",
                  me, other, range, rx, fp);
}

void newDevice(DW1000Device *d)
{
    char id[16]; shortToDeviceId(d->getShortAddress(), id, sizeof(id));
    if (!strncmp(id, "anchor", 6)) anchors_seen++;
    Serial.printf("[uwb] device added: %s\n", id);
}

void blinkDevice(DW1000Device *d)
{
    char id[16]; shortToDeviceId(d->getShortAddress(), id, sizeof(id));
    if (!strncmp(id, "tag", 3)) tags_seen++;
    Serial.printf("[uwb] tag blinked: %s\n", id);
}

void inactiveDevice(DW1000Device *d)
{
    char id[16]; shortToDeviceId(d->getShortAddress(), id, sizeof(id));
    if (!strncmp(id, "anchor", 6) && anchors_seen) anchors_seen--;
    if (!strncmp(id, "tag", 3) && tags_seen) tags_seen--;

    filterForget(id);   // drop the pre-filter state for this link

    // drop the stale range so the solver does not use a dead link
    for (uint8_t i = 0; i < range_count; i++) {
        if (!strcmp(ranges[i].src, id) || !strcmp(ranges[i].dst, id)) {
            memmove(&ranges[i], &ranges[i + 1], sizeof(RangeRec) * (range_count - i - 1));
            range_count--;
            i--;
        }
    }
    Serial.printf("[uwb] device lost: %s\n", id);
}

// ---------------------------------------------------------------------------
// OLED UI
// ---------------------------------------------------------------------------
unsigned long last_ui = 0;

// ---------------------------------------------------------------------------
// Display power saving
//
// The OLED is the only always-on consumer besides the radio, so it can be
// dimmed or switched off entirely. The node keeps ranging, serving the web UI
// and pushing telemetry the whole time — only the panel sleeps.
//
//   mode 0 : always on
//   mode 1 : dim after screen_timeout_s  (low contrast, still readable)
//   mode 2 : off after screen_timeout_s  (panel off, SSD1306_DISPLAYOFF)
//
// Any activity (serial input, a new range, an MQTT/REST message) wakes it.
// ---------------------------------------------------------------------------
bool          screen_off = false;
bool          screen_dim = false;
unsigned long last_activity_ms = 0;

void screenSetBrightness(uint8_t contrast)
{
    display.ssd1306_command(SSD1306_SETCONTRAST);
    display.ssd1306_command(contrast);
}

// Wake the panel; called from anywhere that counts as activity.
void screenWake()
{
    last_activity_ms = millis();
    if (screen_off) {
        display.ssd1306_command(SSD1306_DISPLAYON);
        screenSetBrightness(0xCF);
        screen_off = false;
        screen_dim = false;
        last_ui = 0;                 // force a redraw
    } else if (screen_dim) {
        screenSetBrightness(0xCF);
        screen_dim = false;
        last_ui = 0;
    }
}

// Apply the configured policy. Called from the main loop.
void screenPowerLoop()
{
    if (cfg.screen_mode == 0 || cfg.screen_timeout_s == 0) return;
    if (screen_off || screen_dim) return;
    if (millis() - last_activity_ms < (unsigned long)cfg.screen_timeout_s * 1000UL) return;

    if (cfg.screen_mode == 2) {
        display.ssd1306_command(SSD1306_DISPLAYOFF);
        screen_off = true;
        Serial.println(F("[oled] display off (device stays online)"));
    } else {
        screenSetBrightness(0x0A);    // very dim but still visible
        screen_dim = true;
        Serial.println(F("[oled] display dimmed"));
    }
}

void drawUi()
{
    if (screen_off) return;           // nothing to draw while the panel sleeps
    if (millis() - last_ui < 400) return;
    last_ui = millis();

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    if (cfg.role == ROLE_NONE) {
        display.setTextSize(1);
        display.setCursor(0, 0);
        display.println(F("NOT CONFIGURED"));
        display.setCursor(0, 16);
        display.println(F("1) join WiFi 'UWB-Setup'"));
        display.setCursor(0, 28);
        display.println(F("2) open 192.168.4.1"));
        display.setCursor(0, 40);
        display.println(F("   or use serial menu"));
        display.display();
        return;
    }

    char id[16];
    deviceId(cfg, id, sizeof(id));

    // header: role-id + link status
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print(id);
    display.setCursor(96, 0);
    display.print(net.portal_on ? F("AP") : (net.mqtt_up ? F("MQTT") : (net.wifi_up ? F("REST") : F("----"))));

    if (net.portal_on) {
        display.setCursor(0, 20);
        display.print(F("Join WiFi:"));
        display.setCursor(0, 30);
        display.print(SSID_AP);
        display.setCursor(0, 44);
        display.print(F("open 192.168.4.1"));
        display.display();
        return;
    }

    if (range_count == 0) {
        display.setCursor(0, 28);
        display.print(F("no range yet..."));
        display.display();
        return;
    }

    // most recent range, large. Records are kept across telemetry cycles, so
    // ignore anything older than a few seconds instead of showing a stale
    // number as if it were live.
    int8_t newest = -1;
    for (uint8_t i = 0; i < range_count; i++)
        if (newest < 0 || ranges[i].ts > ranges[newest].ts) newest = i;
    if (newest < 0 || (millis() - ranges[newest].ts) > 5000UL) {
        display.setCursor(0, 28);
        display.print(F("no range yet..."));
        display.display();
        return;
    }

    const RangeRec &r = ranges[newest];
    display.setTextSize(2);
    display.setCursor(0, 14);
    display.print(r.range, 2);
    display.print(F(" m"));

    display.setTextSize(1);
    display.setCursor(0, 36);
    display.print(strncmp(r.src, "anchor", 6) ? F("to ") : F("from "));
    display.print(strncmp(r.src, "anchor", 6) ? r.src : r.dst);

    display.setCursor(0, 48);
    display.print(F("rx "));
    display.print(r.rx, 0);
    display.print(F(" dBm"));

    // tag: solved position when available
    if (cfg.role == ROLE_TAG && pos_count) {
        display.setCursor(0, 56);
        display.print(F("x"));
        display.print(positions[0].x, 1);
        display.print(F(" y"));
        display.print(positions[0].y, 1);
        if (positions[0].ambiguous) display.print(F(" ?"));
    }
    display.display();
}

// ---------------------------------------------------------------------------
// serial menu
// ---------------------------------------------------------------------------
void printHelp()
{
    Serial.println(F(
        "\n=== UWB node serial menu ===\n"
        "  show                 print current config\n"
        "  role tag|anchor      set role\n"
        "  id <1-10>            set device id\n"
        "  site <name>          set site name\n"
        "  wifi <ssid> <pass>   set WiFi credentials\n"
        "  server <url>         e.g. server http://192.168.1.10:8080\n"
        "  mqtt <host> [port]   enable MQTT (port default 1883)\n"
        "  mqtt off             disable MQTT\n"
        "  base <topic>         MQTT base topic (default uwb/home)\n"
        "  pos <x> <y> [z]      anchor position in metres (anchor only)\n"
        "  room <w> <h>         room size in metres\n"
        "  mode <name>          uwb phy mode (see 'show')\n"
        "  filter on|off        range smoothing filter\n"
        "  screen on|dim|off    OLED power saving (device stays online)\n"
        "  screen auto <sec>    idle seconds before dim/off (0 = never)\n"
        "  rate <ms>            telemetry interval\n"
        "  save                 persist to NVS\n"
        "  reboot               restart\n"
        "  reset                erase config and reboot\n"
        "============================\n"));
}

void printConfig()
{
    char eui[40];
    deviceEui(cfg, eui, sizeof(eui));
    Serial.printf("role       : %s\n", roleName(cfg.role));
    Serial.printf("id         : %u\n", (unsigned)cfg.id);
    Serial.printf("eui        : %s\n", eui);
    Serial.printf("site       : %s\n", cfg.site);
    Serial.printf("wifi       : %s\n", cfg.wifi_ssid[0] ? cfg.wifi_ssid : "(not set)");
    Serial.printf("server     : %s\n", cfg.server_url[0] ? cfg.server_url : "(not set)");
    Serial.printf("mqtt       : %s %s:%u\n", cfg.mqtt_enabled ? "on" : "off",
                  cfg.mqtt_host, (unsigned)cfg.mqtt_port);
    Serial.printf("base topic : %s\n", cfg.mqtt_base);
    Serial.printf("position   : %.2f %.2f %.2f\n", cfg.pos_x, cfg.pos_y, cfg.pos_z);
    Serial.printf("room       : %.2f x %.2f m\n", cfg.room_w, cfg.room_h);
    Serial.printf("uwb mode   : %s\n", uwbModeName(cfg.uwb_mode));
    Serial.printf("filter     : %s\n", cfg.range_filter ? "on" : "off");
    Serial.printf("screen     : %s after %us\n",
                  cfg.screen_mode == 0 ? "always on" :
                  (cfg.screen_mode == 1 ? "dim" : "off"),
                  (unsigned)cfg.screen_timeout_s);
    Serial.printf("rate       : %u ms\n", (unsigned)cfg.update_ms);
    Serial.printf("wifi state : %s\n", net.wifi_up ? WiFi.localIP().toString().c_str() : "down");
}

void handleSerial()
{
    static char buf[128];
    static uint8_t len = 0;

    if (Serial.available()) screenWake();   // typing wakes the panel

    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\r') continue;
        if (c != '\n') { if (len < sizeof(buf) - 1) buf[len++] = c; continue; }

        buf[len] = 0;
        len = 0;
        if (!strlen(buf)) continue;

        char *cmd = strtok(buf, " ");
        if (!cmd) continue;
        char *a1 = strtok(NULL, " ");
        char *a2 = strtok(NULL, " ");

        if (!strcmp(cmd, "?") || !strcmp(cmd, "help")) printHelp();
        else if (!strcmp(cmd, "show")) printConfig();
        else if (!strcmp(cmd, "role") && a1) {
            cfg.role = !strcmp(a1, "tag") ? ROLE_TAG : ROLE_ANCHOR;
            Serial.printf("role = %s (save + reboot to apply)\n", roleName(cfg.role));
        }
        else if (!strcmp(cmd, "id") && a1) {
            int v = atoi(a1);
            if (v >= 1 && v <= MAX_DEVICES) { cfg.id = v; Serial.printf("id = %d\n", v); }
            else Serial.println("id must be 1..10");
        }
        else if (!strcmp(cmd, "site") && a1) snprintf(cfg.site, sizeof(cfg.site), "%s", a1);
        else if (!strcmp(cmd, "wifi") && a1 && a2) {
            snprintf(cfg.wifi_ssid, sizeof(cfg.wifi_ssid), "%s", a1);
            snprintf(cfg.wifi_pass, sizeof(cfg.wifi_pass), "%s", a2);
            Serial.println("wifi set");
        }
        else if (!strcmp(cmd, "server") && a1) snprintf(cfg.server_url, sizeof(cfg.server_url), "%s", a1);
        else if (!strcmp(cmd, "mqtt") && a1 && !strcmp(a1, "off")) { cfg.mqtt_enabled = false; Serial.println("mqtt off"); }
        else if (!strcmp(cmd, "mqtt") && a1) {
            snprintf(cfg.mqtt_host, sizeof(cfg.mqtt_host), "%s", a1);
            if (a2) cfg.mqtt_port = (uint16_t)atoi(a2);
            cfg.mqtt_enabled = true;
            Serial.println("mqtt set");
        }
        else if (!strcmp(cmd, "base") && a1) snprintf(cfg.mqtt_base, sizeof(cfg.mqtt_base), "%s", a1);
        else if (!strcmp(cmd, "pos") && a1 && a2) {
            cfg.pos_x = atof(a1); cfg.pos_y = atof(a2);
            char *a3 = strtok(NULL, " ");
            if (a3) cfg.pos_z = atof(a3);
            Serial.printf("pos = %.2f %.2f %.2f\n", cfg.pos_x, cfg.pos_y, cfg.pos_z);
        }
        else if (!strcmp(cmd, "room") && a1 && a2) {
            cfg.room_w = atof(a1); cfg.room_h = atof(a2);
            Serial.printf("room = %.2f x %.2f\n", cfg.room_w, cfg.room_h);
        }
        else if (!strcmp(cmd, "mode") && a1) {
            cfg.uwb_mode = uwbModeFromName(a1);
            Serial.printf("mode = %s (save + reboot to apply)\n", uwbModeName(cfg.uwb_mode));
        }
        else if (!strcmp(cmd, "screen")) {
            // screen on | dim | off | auto <seconds>
            if (a1 && !strcmp(a1, "on")) {
                cfg.screen_mode = 0;
                screenWake();
                Serial.println(F("screen: always on"));
            } else if (a1 && !strcmp(a1, "dim")) {
                cfg.screen_mode = 1;
                Serial.println(F("screen: dims after the timeout"));
            } else if (a1 && !strcmp(a1, "off")) {
                cfg.screen_mode = 2;
                Serial.println(F("screen: turns off after the timeout"));
            } else if (a1 && !strcmp(a1, "auto") && a2) {
                cfg.screen_timeout_s = (uint16_t)atoi(a2);
                Serial.printf("screen: timeout %us\n", (unsigned)cfg.screen_timeout_s);
            } else {
                Serial.println(F("usage: screen on|dim|off|auto <seconds>"));
            }
        }
        else if (!strcmp(cmd, "filter") && a1) cfg.range_filter = !strcmp(a1, "on");
        else if (!strcmp(cmd, "rate") && a1) cfg.update_ms = (uint16_t)atoi(a1);
        else if (!strcmp(cmd, "save")) { configSave(cfg); Serial.println("saved"); }
        else if (!strcmp(cmd, "reboot")) { Serial.println("rebooting"); delay(100); ESP.restart(); }
        else if (!strcmp(cmd, "reset")) { configClear(); Serial.println("cleared, rebooting"); delay(200); ESP.restart(); }
        else Serial.println("unknown command, type ? for help");
    }
}

// ---------------------------------------------------------------------------
// setup portal handlers
// ---------------------------------------------------------------------------
void portalRegister()
{
    g_portal.on("/", HTTP_GET, []() {
        char id[16], eui[40];
        deviceId(cfg, id, sizeof(id));
        deviceEui(cfg, eui, sizeof(eui));
        String h = F("<!doctype html><meta name=viewport content='width=device-width,initial-scale=1'>"
                     "<style>body{font:15px system-ui;max-width:520px;margin:20px auto;padding:0 14px}"
                     "input,select{width:100%;padding:7px;margin:4px 0 12px;box-sizing:border-box}"
                     "label{font-weight:600}button{padding:10px 18px;font-size:16px}</style>"
                     "<h2>UWB node setup</h2><form method=post action=/save>");
        h += "<label>Role</label><select name=role>";
        h += String("<option value=anchor") + (cfg.role == ROLE_ANCHOR ? " selected" : "") + ">anchor</option>";
        h += String("<option value=tag") + (cfg.role == ROLE_TAG ? " selected" : "") + ">tag</option></select>";
        h += "<label>ID (1-10)</label><input name=id type=number min=1 max=10 value=" + String(cfg.id) + ">";
        h += "<label>Site</label><input name=site value='" + String(cfg.site) + "'>";
        h += "<label>WiFi SSID</label><input name=wssid value='" + String(cfg.wifi_ssid) + "'>";
        h += "<label>WiFi password</label><input name=wpass type=password value='" + String(cfg.wifi_pass) + "'>";
        h += "<label>Server URL</label><input name=surl placeholder='http://192.168.1.10:8080' value='" + String(cfg.server_url) + "'>";
        h += "<label>MQTT host</label><input name=mqhost value='" + String(cfg.mqtt_host) + "'>";
        h += "<label>MQTT port</label><input name=mqport type=number value=" + String(cfg.mqtt_port) + ">";
        h += "<label>MQTT base topic</label><input name=mqbase value='" + String(cfg.mqtt_base) + "'>";
        h += "<label>Anchor X (m)</label><input name=px type=number step=0.01 value=" + String(cfg.pos_x, 2) + ">";
        h += "<label>Anchor Y (m)</label><input name=py type=number step=0.01 value=" + String(cfg.pos_y, 2) + ">";
        h += "<label>Room width (m)</label><input name=rw type=number step=0.01 value=" + String(cfg.room_w, 2) + ">";
        h += "<label>Room height (m)</label><input name=rh type=number step=0.01 value=" + String(cfg.room_h, 2) + ">";
        h += "<button type=submit>Save &amp; reboot</button></form>";
        h += "<p style='color:#666'>device id <b>" + String(id) + "</b>, EUI <code>" + String(eui) + "</code></p>";
        g_portal.send(200, "text/html", h);
    });

    g_portal.on("/save", HTTP_POST, []() {
        if (g_portal.hasArg("role")) cfg.role = g_portal.arg("role") == "tag" ? ROLE_TAG : ROLE_ANCHOR;
        if (g_portal.hasArg("id")) {
            int v = g_portal.arg("id").toInt();
            if (v >= 1 && v <= MAX_DEVICES) cfg.id = v;
        }
        snprintf(cfg.site, sizeof(cfg.site), "%s", g_portal.arg("site").c_str());
        snprintf(cfg.wifi_ssid, sizeof(cfg.wifi_ssid), "%s", g_portal.arg("wssid").c_str());
        snprintf(cfg.wifi_pass, sizeof(cfg.wifi_pass), "%s", g_portal.arg("wpass").c_str());
        snprintf(cfg.server_url, sizeof(cfg.server_url), "%s", g_portal.arg("surl").c_str());
        snprintf(cfg.mqtt_host, sizeof(cfg.mqtt_host), "%s", g_portal.arg("mqhost").c_str());
        cfg.mqtt_enabled = cfg.mqtt_host[0] != 0;
        if (g_portal.arg("mqport").toInt() > 0) cfg.mqtt_port = (uint16_t)g_portal.arg("mqport").toInt();
        snprintf(cfg.mqtt_base, sizeof(cfg.mqtt_base), "%s", g_portal.arg("mqbase").c_str());
        cfg.pos_x = g_portal.arg("px").toFloat();
        cfg.pos_y = g_portal.arg("py").toFloat();
        if (g_portal.arg("rw").toFloat() > 0) cfg.room_w = g_portal.arg("rw").toFloat();
        if (g_portal.arg("rh").toFloat() > 0) cfg.room_h = g_portal.arg("rh").toFloat();
        configSave(cfg);

        g_portal.send(200, "text/html",
                      F("<h3>Saved. Rebooting...</h3><script>setTimeout(()=>location='/',4000)</script>"));
        delay(300);
        ESP.restart();
    });

    // NOTE: g_portal.begin() is intentionally NOT called here — opening the
    // listening socket before WiFi is up crashes lwIP. portalStart() does it
    // after the AP is running.
    Serial.println("[ap] portal handlers registered");
}

// ---------------------------------------------------------------------------
// server-pushed config / commands (REST + MQTT both land here)
// ---------------------------------------------------------------------------

// FNV-1a over the serialized config. Used as a safety net: if the exact same
// config arrives again after a reboot, we never reboot a second time. This
// guards against any field that is pushed on every sync (the wifi SSID and the
// OTA token both did exactly that, which caused an endless reboot loop).
static uint32_t configHash(JsonObjectConst doc)
{
    String s;
    serializeJson(doc, s);
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < s.length(); i++) {
        h ^= (uint8_t)s[i];
        h *= 16777619u;
    }
    return h;
}

static uint32_t loadAppliedHash()
{
    Preferences p;
    if (!p.begin("uwbcfg", true)) return 0;
    uint32_t h = p.getUInt("cfghash", 0);
    p.end();
    return h;
}

static void storeAppliedHash(uint32_t h)
{
    Preferences p;
    if (!p.begin("uwbcfg", false)) return;
    p.putUInt("cfghash", h);
    p.end();
}

void onServerConfig(JsonObjectConst doc)
{
    const uint32_t incoming = configHash(doc);
    if (incoming != 0 && incoming == loadAppliedHash()) {
        // identical config already applied -> nothing to do, and above all
        // do NOT reboot again
        Serial.println("[cfg] unchanged, ignoring");
        return;
    }

    bool changed = false;

    if (doc["role"].is<const char *>()) {
        Role r = !strcmp(doc["role"].as<const char *>(), "tag") ? ROLE_TAG : ROLE_ANCHOR;
        if (r != cfg.role) { cfg.role = r; changed = true; }
    }
    if (doc["id"].is<int>()) {
        int v = doc["id"].as<int>();
        if (v >= 1 && v <= MAX_DEVICES && v != cfg.id) { cfg.id = v; changed = true; }
    }
    if (doc["site"].is<const char *>()) snprintf(cfg.site, sizeof(cfg.site), "%s", doc["site"].as<const char *>());

    if (doc["wifi"].is<JsonObjectConst>()) {
        JsonObjectConst w = doc["wifi"].as<JsonObjectConst>();
        if (w["ssid"].is<const char *>()) {
            const char *v = w["ssid"].as<const char *>();
            // Only reboot when the value actually differs. Setting `changed`
            // unconditionally made the node reboot on every config sync, since
            // the server sends the SSID each time -> infinite reboot loop.
            if (strcmp(v, cfg.wifi_ssid) != 0) {
                snprintf(cfg.wifi_ssid, sizeof(cfg.wifi_ssid), "%s", v);
                changed = true;
            }
        }
        if (w["password"].is<const char *>()) {
            const char *v = w["password"].as<const char *>();
            if (strcmp(v, cfg.wifi_pass) != 0) {
                snprintf(cfg.wifi_pass, sizeof(cfg.wifi_pass), "%s", v);
                changed = true;
            }
        }
    }
    if (doc["server"].is<JsonObjectConst>()) {
        JsonObjectConst s = doc["server"].as<JsonObjectConst>();
        if (s["url"].is<const char *>()) snprintf(cfg.server_url, sizeof(cfg.server_url), "%s", s["url"].as<const char *>());
        if (s["token"].is<const char *>()) snprintf(cfg.api_token, sizeof(cfg.api_token), "%s", s["token"].as<const char *>());
    }
    if (doc["mqtt"].is<JsonObjectConst>()) {
        JsonObjectConst m = doc["mqtt"].as<JsonObjectConst>();
        if (m["enabled"].is<bool>()) cfg.mqtt_enabled = m["enabled"].as<bool>();
        if (m["host"].is<const char *>()) snprintf(cfg.mqtt_host, sizeof(cfg.mqtt_host), "%s", m["host"].as<const char *>());
        if (m["port"].is<int>()) cfg.mqtt_port = (uint16_t)m["port"].as<int>();
        if (m["user"].is<const char *>()) snprintf(cfg.mqtt_user, sizeof(cfg.mqtt_user), "%s", m["user"].as<const char *>());
        if (m["password"].is<const char *>()) snprintf(cfg.mqtt_pass, sizeof(cfg.mqtt_pass), "%s", m["password"].as<const char *>());
        if (m["base_topic"].is<const char *>()) snprintf(cfg.mqtt_base, sizeof(cfg.mqtt_base), "%s", m["base_topic"].as<const char *>());
    }
    if (doc["position"].is<JsonObjectConst>()) {
        JsonObjectConst p = doc["position"].as<JsonObjectConst>();
        if (p["x"].is<float>()) cfg.pos_x = p["x"].as<float>();
        if (p["y"].is<float>()) cfg.pos_y = p["y"].as<float>();
        if (p["z"].is<float>()) cfg.pos_z = p["z"].as<float>();
    }
    if (doc["room"].is<JsonObjectConst>()) {
        JsonObjectConst r = doc["room"].as<JsonObjectConst>();
        if (r["width"].is<float>()) cfg.room_w = r["width"].as<float>();
        if (r["height"].is<float>()) cfg.room_h = r["height"].as<float>();
    }
    if (doc["anchors"].is<JsonArrayConst>()) {          // anchor map for tag-side solving
        char map[sizeof(cfg.anchor_map)] = "";
        for (JsonObjectConst a : doc["anchors"].as<JsonArrayConst>()) {
            if (!a["id"].is<int>() || !a["x"].is<float>() || !a["y"].is<float>()) continue;
            char part[24];
            snprintf(part, sizeof(part), "%d:%.2f,%.2f;", a["id"].as<int>(),
                     a["x"].as<float>(), a["y"].as<float>());
            if (strlen(map) + strlen(part) < sizeof(map)) strcat(map, part);
        }
        if (strcmp(map, cfg.anchor_map)) {
            snprintf(cfg.anchor_map, sizeof(cfg.anchor_map), "%s", map);
            changed = true;
        }
    }
    if (doc["display"].is<JsonObjectConst>()) {
        JsonObjectConst d = doc["display"].as<JsonObjectConst>();
        if (d["mode"].is<const char *>()) {
            const char *m = d["mode"].as<const char *>();
            cfg.screen_mode = !strcmp(m, "off") ? 2 : (!strcmp(m, "always") ? 0 : 1);
            screenWake();
        }
        if (d["timeout_s"].is<int>()) cfg.screen_timeout_s = (uint16_t)d["timeout_s"].as<int>();
    }
    if (doc["ota"].is<JsonObjectConst>()) {
        JsonObjectConst o = doc["ota"].as<JsonObjectConst>();
        if (o["enabled"].is<bool>()) cfg.ota_enabled = o["enabled"].as<bool>();
        if (o["port"].is<int>()) cfg.ota_port = (uint16_t)o["port"].as<int>();
        if (o["token"].is<const char *>()) {
            const char *v = o["token"].as<const char *>();
            // Same trap as the SSID: the token is present in every config
            // response, so compare first or the node reboots forever.
            if (strcmp(v, cfg.ota_token) != 0) {
                snprintf(cfg.ota_token, sizeof(cfg.ota_token), "%s", v);
                changed = true;
            }
        }
    }
    if (doc["uwb"].is<JsonObjectConst>()) {
        JsonObjectConst u = doc["uwb"].as<JsonObjectConst>();
        if (u["mode"].is<const char *>()) cfg.uwb_mode = uwbModeFromName(u["mode"].as<const char *>());
        if (u["range_filter"].is<bool>()) cfg.range_filter = u["range_filter"].as<bool>();
        if (u["update_ms"].is<int>()) cfg.update_ms = (uint16_t)u["update_ms"].as<int>();
    }

    configSave(cfg);
    storeAppliedHash(incoming);
    Serial.println("[cfg] applied config from server");
    if (changed) { delay(200); ESP.restart(); }
}

void onServerCommand(const char *cmd, JsonObjectConst args)
{
    (void)args;
    if (!strcmp(cmd, "reboot")) { ESP.restart(); }
    else if (!strcmp(cmd, "identify")) { Serial.println("[cmd] identify"); }
}

// ---------------------------------------------------------------------------
// setup / loop
// ---------------------------------------------------------------------------
void setup()
{
    Serial.begin(115200);
    delay(400);
    boot_ms = millis();

    Serial.println(F("\n=== ESP32 UWB positioning node ==="));
    Serial.printf("firmware %s\n", FW_VERSION);

    configLoad(cfg);

    Wire.begin(I2C_SDA, I2C_SCL);
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println(F("[oled] init failed"));
    }
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("Makerfabs UWB"));
    display.println(F(FW_VERSION));
    display.display();

    // Handlers are registered on every boot (cheap, no socket yet) so the
    // portal can be started later — e.g. when WiFi credentials are wrong.
    portalRegister();

    // Uplink runs on its own task so an unreachable server can never stall
    // the ranging protocol (see uplinkTask()).
    pending_lock = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(uplinkTask, "uplink", 8192, nullptr, 1, nullptr, 0);

    if (cfg.role == ROLE_NONE) {
        Serial.println(F("no role configured -> starting setup portal"));
        printHelp();
        portalStart();
        return;
    }

    char eui[40];
    deviceEui(cfg, eui, sizeof(eui));
    Serial.printf("role %s id %u  eui %s\n", roleName(cfg.role), (unsigned)cfg.id, eui);

    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    DW1000Ranging.initCommunication(UWB_RST, UWB_SS, UWB_IRQ);
    DW1000Ranging.attachNewRange(newRange);
    DW1000Ranging.attachNewDevice(newDevice);
    DW1000Ranging.attachBlinkDevice(blinkDevice);
    DW1000Ranging.attachInactiveDevice(inactiveDevice);
    // The library's own low-pass filter is disabled on purpose: it stores its
    // previous output inside DW1000Device and feeds a bad sample back forever
    // (observed drift 1.5 m -> 248 m). src/main.cpp does the filtering instead
    // (median + outlier gate), and src/ekf.h does the tracking.
    DW1000Ranging.useRangeFilter(false);

    if (cfg.role == ROLE_TAG)
        DW1000Ranging.startAsTag(eui, uwbModeBytes(cfg.uwb_mode), false);
    else
        DW1000Ranging.startAsAnchor(eui, uwbModeBytes(cfg.uwb_mode), false);

    wifiBegin();
    printHelp();
}

void loop()
{
    handleSerial();
    portalLoop();

    if (cfg.role == ROLE_NONE) { delay(10); return; }

    DW1000Ranging.loop();
    screenPowerLoop();
    wifiLoop();
    otaBegin();
    otaLoop();
    // While an image is being written, stay off the network stack from this
    // task: concurrent TCP use during the flash write is what tripped
    // 'assert failed: xQueueSemaphoreTake' inside lwIP.
    if (ota_busy) { drawUi(); return; }
    mqttEnsureConnected();
    mqttLoop();
    syncConfigFromServer();

    if (millis() - last_telem_ms > (unsigned long)cfg.update_ms * 5) {
        last_telem_ms = millis();
        if (range_count) { solveLocally(); publishTelemetry(); }
    }

    drawUi();
}