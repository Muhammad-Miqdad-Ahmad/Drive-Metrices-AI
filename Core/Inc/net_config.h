#pragma once
/* Network + MQTT configuration.
 * FILL IN the WiFi credentials and the broker IP before flashing. */

/* ── WiFi (2.4 GHz only — the ISM43362 does not support 5 GHz) ──
 * Networks are tried in order; the first successful join is used. */
#define NET_WIFI_NETWORKS                                                      \
  { "Iphone", "mahum1234" },                                                   \
  { "vivo s1", "maheenzahid" },                                                \
  { "Cyber Surge", "cout<<INTERNETpassword" },                                 \
  { "Faculty-ITU", "faculty@itu1234*" },                                       \
  { "ITU-Library", "library@itu1234*" }
/* Security (applies to all networks): WIFI_ECN_OPEN / WIFI_ECN_WEP /
 * WIFI_ECN_WPA_PSK / WIFI_ECN_WPA2_PSK / WIFI_ECN_WPA_WPA2_PSK */
#define NET_WIFI_SECURITY  WIFI_ECN_WPA2_PSK

/* ── Home zone (upload trigger) ──
 * When the GPS fix enters this circle, the whole SD log is uploaded and cleared.
 * Coordinates: Plus Code GF88+553 Lahore → 8J3PGF88+553 → 31.5154, 74.4654. */
#define HOME_LAT           31.5154
#define HOME_LON           74.4654
#define HOME_RADIUS_M      200.0f

/* ── MQTT broker (Mosquitto on your PC, plain TCP on the LAN) ──
 * The board drains its SD log to the broker when it reaches the home zone
 * (or on a collision). No TLS — the ISM43362 can't do SNI, and a LAN broker
 * doesn't need it. A Python bridge on the PC turns these messages into trips.
 * Set MQTT_BROKER_IP to the PC's LAN IPv4 (find it with `ip addr`). */
#define MQTT_BROKER_IP    { 192, 168, 1, 50 }
#define MQTT_BROKER_PORT  1883
#define MQTT_CLIENT_ID    "stm32-drive"
#define MQTT_TOPIC        "drivemetrics/readings"
#define DEVICE_TOKEN      "stm32-001"  /* identifies this board in the DB */
