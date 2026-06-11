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

/* ── Relay mode ──
 * The ISM43362 module TLS cannot send SNI, which Supabase (Cloudflare)
 * requires — so direct HTTPS from the board is impossible. Instead the
 * board POSTs plain HTTP to a relay (tools/relay.py) running on a PC in
 * the same network, which forwards to Supabase over HTTPS.
 * Set SB_USE_RELAY 0 only if you have an SNI-capable TLS path. */
#define SB_USE_RELAY       1
#define RELAY_IP           {10, 119, 158, 48}  /* laptop on Cyber Surge hotspot */
#define RELAY_PORT         8787

/* ── Supabase ── */
#define SUPABASE_HOST      "gsgmyvdszgejcadgrvgs.supabase.co"
#define SUPABASE_PORT      443
#define SUPABASE_ANON_KEY                                                      \
  "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9."                                      \
  "eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6ImdzZ215dmRzemdlamNhZGdydmdzIiwicm9sZSI6Im" \
  "Fub24iLCJpYXQiOjE3ODA5MTk3NTIsImV4cCI6MjA5NjQ5NTc1Mn0."                     \
  "2IJohd6T1WK1QpXD_yvWHpTaCmh6U30qrgeMbzVQYys"
