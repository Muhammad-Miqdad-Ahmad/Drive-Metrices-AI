#pragma once
/* Network + ThingSpeak configuration.
 * FILL IN the WiFi credentials and ThingSpeak channel before flashing. */

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

/* ── Upload cadence ──
 * Upload the SD log to ThingSpeak after every N predictions. */
#define UPLOAD_EVERY_N     15

/* ── ThingSpeak (plain HTTP, direct from the board) ── */
#define THINGSPEAK_HOST        "api.thingspeak.com"
#define THINGSPEAK_PORT        80
#define THINGSPEAK_CHANNEL_ID  "3408345"          /* <-- your channel ID  */
#define THINGSPEAK_WRITE_KEY   "H8VDD3ARZ9O8KQSV" /* <-- your Write API key */
