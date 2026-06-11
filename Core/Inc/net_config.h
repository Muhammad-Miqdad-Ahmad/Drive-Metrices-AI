#pragma once
/* Network + Supabase configuration.
 * FILL IN the WiFi credentials before flashing. */

/* ── WiFi (2.4 GHz only — the ISM43362 does not support 5 GHz) ── */
#define NET_WIFI_SSID      "ITU-Library"
#define NET_WIFI_PASSWORD  "library@itu1234*"
/* Security: WIFI_ECN_OPEN / WIFI_ECN_WEP / WIFI_ECN_WPA_PSK /
 *           WIFI_ECN_WPA2_PSK / WIFI_ECN_WPA_WPA2_PSK */
#define NET_WIFI_SECURITY  WIFI_ECN_WPA2_PSK

/* ── Home zone (upload trigger) ──
 * When the GPS fix is inside this circle, the SD log is uploaded.
 * FILL IN your home/parking coordinates (decimal degrees). */
#define HOME_LAT           0.0
#define HOME_LON           0.0
#define HOME_RADIUS_M      100.0f

/* ── Supabase ── */
#define SUPABASE_HOST      "gsgmyvdszgejcadgrvgs.supabase.co"
#define SUPABASE_PORT      443
#define SUPABASE_ANON_KEY                                                      \
  "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9."                                      \
  "eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6ImdzZ215dmRzemdlamNhZGdydmdzIiwicm9sZSI6Im" \
  "Fub24iLCJpYXQiOjE3ODA5MTk3NTIsImV4cCI6MjA5NjQ5NTc1Mn0."                     \
  "2IJohd6T1WK1QpXD_yvWHpTaCmh6U30qrgeMbzVQYys"
