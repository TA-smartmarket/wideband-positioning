// ============================================================================
//  net.h — WiFi, setup portal, REST client and MQTT client
//
//  Both transports are always available; MQTT is preferred when connected and
//  REST is the fallback. The server is the source of truth for configuration.
//  Wire formats: docs/API.md.
// ============================================================================
#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "config.h"

// ---- JSON helpers (shared with main.cpp) ----------------------------------

inline void jsonPutFloat(JsonDocument doc, const char *k, float v)
{
    doc[k] = v;   // v7: JsonDocument root is a JsonVariant
}

// ---- network state --------------------------------------------------------

struct Net {
    bool  wifi_up   = false;
    bool  mqtt_up   = false;
    bool  portal_on = false;
    unsigned long last_retry = 0;
    unsigned long last_cfg   = 0;
    uint32_t cfg_version = 0;
};

extern Config cfg;
extern Net    net;

extern WiFiClient   g_wifi_client;
extern PubSubClient g_mqtt;
extern WebServer    g_portal;

// implemented in main.cpp
void onServerConfig(JsonObjectConst doc);
void onServerCommand(const char *cmd, JsonObjectConst args);

// ---- MQTT topics ----------------------------------------------------------

inline void topic(char *out, size_t n, const char *suffix)
{
    snprintf(out, n, "%s/%s", cfg.mqtt_base, suffix);
}

inline void topicDevice(char *out, size_t n, const char *kind, const char *suffix)
{
    snprintf(out, n, "%s/%s/%s-%u/%s", cfg.mqtt_base, kind,
             roleName(cfg.role), (unsigned)cfg.id, suffix);
}

// Config topic for this device: "<base>/config/<role>-<id>" (must match the
// server's publish topic in server/app.py).
inline void topicConfig(char *out, size_t n)
{
    snprintf(out, n, "%s/config/%s-%u", cfg.mqtt_base,
             roleName(cfg.role), (unsigned)cfg.id);
}

// ---- REST -----------------------------------------------------------------

// POST json to "<server_url><path>". Returns true on HTTP 2xx.
inline bool httpPostJson(const char *path, const String &body)
{
    if (!net.wifi_up || cfg.server_url[0] == 0) return false;
    HTTPClient http;
    String url = String(cfg.server_url) + path;
    if (!http.begin(url)) return false;
    http.setTimeout(1500);
    http.addHeader("Content-Type", "application/json");
    if (cfg.api_token[0]) http.addHeader("Authorization", String("Bearer ") + cfg.api_token);
    int code = http.POST(body);
    http.end();
    return code >= 200 && code < 300;
}

// GET json from "<server_url><path>" into doc. Returns true on HTTP 2xx.
inline bool httpGetJson(const char *path, JsonDocument &doc)
{
    if (!net.wifi_up || cfg.server_url[0] == 0) return false;
    HTTPClient http;
    String url = String(cfg.server_url) + path;
    if (!http.begin(url)) return false;
    http.setTimeout(1500);
    if (cfg.api_token[0]) http.addHeader("Authorization", String("Bearer ") + cfg.api_token);
    int code = http.GET();
    bool ok = false;
    if (code >= 200 && code < 300) {
        DeserializationError e = deserializeJson(doc, http.getStream());
        ok = !e;
    }
    http.end();
    return ok;
}

// ---- MQTT -----------------------------------------------------------------

inline void mqttPublish(const char *topic_name, const String &payload, bool retained = false)
{
    if (!net.mqtt_up) return;
    g_mqtt.publish(topic_name, payload.c_str(), retained);
}

inline void mqttCallback(char *t, byte *payload, unsigned int len)
{
    char cfg_topic[80], cmd_topic[80];
    topicConfig(cfg_topic, sizeof(cfg_topic));
    snprintf(cmd_topic, sizeof(cmd_topic), "%s/cmd/%s-%u", cfg.mqtt_base,
             roleName(cfg.role), (unsigned)cfg.id);

    JsonDocument doc;
    if (deserializeJson(doc, payload, len)) return;

    if (!strcmp(t, cfg_topic)) {
        onServerConfig(doc.as<JsonObjectConst>());
    } else if (!strcmp(t, cmd_topic)) {
        onServerCommand(doc["cmd"] | "", doc["args"].as<JsonObjectConst>());
    }
}

inline void mqttEnsureConnected()
{
    if (!cfg.mqtt_enabled || !net.wifi_up || cfg.mqtt_host[0] == 0) return;
    if (g_mqtt.connected()) { net.mqtt_up = true; return; }
    if (millis() - net.last_retry < 3000) return;
    net.last_retry = millis();

    char id[24];
    deviceId(cfg, id, sizeof(id));
    g_mqtt.setServer(cfg.mqtt_host, cfg.mqtt_port);
    g_mqtt.setCallback(mqttCallback);
    bool ok = cfg.mqtt_user[0]
                  ? g_mqtt.connect(id, cfg.mqtt_user, cfg.mqtt_pass)
                  : g_mqtt.connect(id);

    net.mqtt_up = ok;
    if (!ok) return;

    char t[80];
    topicConfig(t, sizeof(t));
    g_mqtt.subscribe(t);
    snprintf(t, sizeof(t), "%s/cmd/%s-%u", cfg.mqtt_base, roleName(cfg.role), (unsigned)cfg.id);
    g_mqtt.subscribe(t);

    // retained online status (LWT marks offline)
    snprintf(t, sizeof(t), "%s/status/%s-%u", cfg.mqtt_base, roleName(cfg.role), (unsigned)cfg.id);
    String st = String("{\"online\":true,\"fw\":\"") + FW_VERSION + "\"}";
    g_mqtt.publish(t, st.c_str(), true);
    Serial.printf("[mqtt] connected to %s:%u\n", cfg.mqtt_host, cfg.mqtt_port);
}

inline void mqttLoop()
{
    if (cfg.mqtt_enabled && net.wifi_up) g_mqtt.loop();
}

// ---- server configuration sync -------------------------------------------

inline void syncConfigFromServer(bool force = false)
{
    if (!net.wifi_up) return;
    if (!force && millis() - net.last_cfg < 15000) return;
    net.last_cfg = millis();

    char path[96];
    snprintf(path, sizeof(path), "/api/v1/config/device?role=%s&id=%u",
             roleName(cfg.role), (unsigned)cfg.id);

    JsonDocument doc;
    if (!httpGetJson(path, doc)) return;
    onServerConfig(doc.as<JsonObjectConst>());
}

// ---- setup portal (captive AP) -------------------------------------------
//
// IMPORTANT: the WebServer socket may only be opened AFTER the network
// interface exists, otherwise lwIP aborts with
//   "assert failed: tcpip_send_msg_wait_sem (Invalid mbox)".
// So: bring up WiFi (AP) first, give lwIP a moment, then begin() the server.
// WIFI_AP_STA keeps the STA connection attempt alive while the portal is up.

inline void portalStart()
{
    if (net.portal_on) return;

    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(SSID_AP);
    delay(300);                 // let the AP + lwIP come up before opening the socket
    g_portal.begin();
    net.portal_on = true;
    Serial.printf("[ap] setup portal at http://192.168.4.1  (ssid %s)\n", SSID_AP);
}

inline void portalStop()
{
    if (!net.portal_on) return;
    g_portal.stop();
    WiFi.softAPdisconnect(true);
    net.portal_on = false;
    Serial.println("[ap] portal closed");
}

inline void portalLoop()
{
    if (net.portal_on) g_portal.handleClient();
}

// ---- wifi -----------------------------------------------------------------

inline void wifiBegin()
{
    if (cfg.wifi_ssid[0] == 0) {
        portalStart();          // nothing to join -> go straight to the portal
        return;
    }
    WiFi.mode(WIFI_AP_STA);     // STA now, AP only if the portal is needed
    WiFi.begin(cfg.wifi_ssid, cfg.wifi_pass);
    Serial.printf("[wifi] connecting to %s\n", cfg.wifi_ssid);
}

inline void wifiLoop()
{
    static unsigned long last = 0;
    static uint8_t fails = 0;

    if (net.wifi_up) {
        if (WiFi.status() != WL_CONNECTED) {
            net.wifi_up = false;
            Serial.println("[wifi] lost");
        }
        return;
    }
    if (WiFi.status() == WL_CONNECTED) {
        net.wifi_up = true;
        fails = 0;
        Serial.printf("[wifi] connected, ip %s\n", WiFi.localIP().toString().c_str());
        if (net.portal_on) portalStop();     // reached the network, close the portal
        syncConfigFromServer(true);
        return;
    }
    if (millis() - last > 10000) {           // retry; after 3 tries open the portal
        last = millis();
        if (cfg.wifi_ssid[0]) { WiFi.disconnect(); WiFi.begin(cfg.wifi_ssid, cfg.wifi_pass); }
        if (++fails >= 3 && !net.portal_on) {
            Serial.println(F("[wifi] cannot connect -> starting setup portal"));
            portalStart();
        }
    }
}