#pragma once
/* Network + Supabase configuration.
 * FILL IN the WiFi credentials before flashing. */

/* ── WiFi (2.4 GHz only — the ISM43362 does not support 5 GHz) ──
 * Networks are tried in order; the first successful join is used. */
#define NET_WIFI_NETWORKS                                                      \
  { "Cyber Surge", "cout<<INTERNETpassword" },                                 \
  { "Faculty-ITU", "faculty@itu1234*" },                                       \
  { "ITU-Library", "library@itu1234*" }
/* Security (applies to all networks): WIFI_ECN_OPEN / WIFI_ECN_WEP /
 * WIFI_ECN_WPA_PSK / WIFI_ECN_WPA2_PSK / WIFI_ECN_WPA_WPA2_PSK */
#define NET_WIFI_SECURITY  WIFI_ECN_WPA2_PSK

/* ── Home zone (upload trigger) ──
 * When the GPS fix is inside this circle, the SD log is uploaded.
 * FILL IN your home/parking coordinates (decimal degrees). */
#define HOME_LAT           0.0
#define HOME_LON           0.0
#define HOME_RADIUS_M      100.0f

/* ── Upload cadence ──
 * Upload the SD log to Supabase after every N predictions. */
#define UPLOAD_EVERY_N     15

/* ── ThingSpeak ──
 * Plain HTTP on port 80 — works directly from the board (the ISM43362
 * module TLS cannot send SNI, so HTTPS services behind CDNs are
 * unreachable; ThingSpeak officially supports plain HTTP).
 * Create a channel at thingspeak.com with 5 fields:
 *   field1=prediction, field2=confidence, field3=lat, field4=lon, field5=speed
 * then FILL IN the channel ID and Write API key. */
#define THINGSPEAK_HOST        "api.thingspeak.com"
#define THINGSPEAK_PORT        80
#define THINGSPEAK_CHANNEL_ID  "0000000"          /* <-- your channel ID  */
#define THINGSPEAK_WRITE_KEY   "XXXXXXXXXXXXXXXX" /* <-- your Write API key */
