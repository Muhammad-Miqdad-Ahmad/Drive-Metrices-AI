/**
 * MQTT batch uploader — plain TCP to a local Mosquitto broker.
 *
 * The SD log accumulates one JSON line per inference. On upload, the board
 * opens one TCP connection to the broker, sends an MQTT 3.1.1 CONNECT, then
 * one QoS-0 PUBLISH per log record to MQTT_TOPIC, then DISCONNECT. The log is
 * cleared only after every record was published over a healthy connection.
 *
 * A Python bridge subscribed to the broker turns these messages into Supabase
 * trips/route_points/trip_events. No TLS: the ISM43362 can't do SNI, and a LAN
 * broker doesn't need it.
 *
 * Published payload (one per reading):
 *   {"dev":"stm32-001","ts":1718086496,"pred":2,"conf":95.3,"lat":31.5156,
 *    "lon":74.4654,"spd":45.2,"gmax":2.156,"collision":0,"gpeak":0.0,"fix":1}
 *   ts = Unix epoch SECONDS (uint32; newlib-nano has no %llu, and seconds fit
 *   until 2106). Kept strictly increasing so each reading maps to a unique
 *   wall-clock second (the bridge keys route points/events on recorded_at).
 *   Without a GPS fix, ts starts at 1970 — the bridge drops those.
 */
#include "mqtt.h"
#include "app_log.h"
#include "net_config.h"
#include "sd_log.h"

#include "es_wifi.h"
#include "fatfs.h"
#include "wifi.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

extern ES_WIFIObject_t EsWifiObj; /* defined in Drivers/WiFi/Src/wifi.c */

#define MQTT_SOCKET     0
#define MQTT_TIMEOUT_MS 10000

static uint8_t wifi_joined = 0;
static uint8_t was_outside = 1;  /* geofence hysteresis */
static uint32_t ts_last_sec = 0; /* keep created_at seconds strictly increasing */

static ES_WIFI_Conn_t conn;
static uint8_t pub_buf[512];

/* ── tiny JSON field extractor for our own SD log lines ─────────────────── */

static int json_num(const char *line, const char *key, float *out) {
    const char *p = strstr(line, key);
    if (!p) return -1;
    *out = strtof(p + strlen(key), NULL);
    return 0;
}

/* ── raw TCP send helper (chunks to ES_WIFI_PAYLOAD_SIZE) ───────────────── */

static int send_all(const uint8_t *data, int len) {
    int off = 0;
    while (off < len) {
        uint16_t chunk = (uint16_t)(len - off);
        if (chunk > ES_WIFI_PAYLOAD_SIZE)
            chunk = ES_WIFI_PAYLOAD_SIZE;
        uint16_t sent = 0;
        if (ES_WIFI_SendData(&EsWifiObj, MQTT_SOCKET, (uint8_t *)data + off,
                             chunk, &sent, MQTT_TIMEOUT_MS) != ES_WIFI_STATUS_OK ||
            sent == 0)
            return -1;
        off += sent;
    }
    return 0;
}

/* ── MQTT 3.1.1 packet helpers ──────────────────────────────────────────── */

/* Encode an MQTT "remaining length" variable-byte integer. Returns #bytes. */
static int enc_remlen(uint8_t *out, int len) {
    int i = 0;
    do {
        uint8_t byte = (uint8_t)(len % 128);
        len /= 128;
        if (len > 0)
            byte |= 0x80;
        out[i++] = byte;
    } while (len > 0);
    return i;
}

/* Open TCP, send CONNECT (anonymous, clean session), check CONNACK. */
static int mqtt_connect(void) {
    memset(&conn, 0, sizeof(conn));
    conn.Number     = MQTT_SOCKET;
    conn.Type       = ES_WIFI_TCP_CONNECTION;
    conn.RemotePort = MQTT_BROKER_PORT;
    uint8_t ip[4]   = MQTT_BROKER_IP;
    memcpy(conn.RemoteIP, ip, 4);

    if (ES_WIFI_StartClientConnection(&EsWifiObj, &conn) != ES_WIFI_STATUS_OK) {
        DEBUG_PRINTF("[MQTT] TCP connect failed (module says: %.100s)\n",
                     (char *)EsWifiObj.CmdData);
        return -1;
    }

    const char *cid = MQTT_CLIENT_ID;
    int cid_len = (int)strlen(cid);

    /* variable header: protocol name + level 4 + clean-session + keepalive 60 */
    uint8_t vh[10] = {0x00, 0x04, 'M', 'Q', 'T', 'T', 0x04, 0x02, 0x00, 0x3C};
    int remlen = (int)sizeof(vh) + 2 + cid_len;

    uint8_t pkt[64];
    int o = 0;
    pkt[o++] = 0x10; /* CONNECT */
    o += enc_remlen(pkt + o, remlen);
    memcpy(pkt + o, vh, sizeof(vh));
    o += (int)sizeof(vh);
    pkt[o++] = (uint8_t)(cid_len >> 8);
    pkt[o++] = (uint8_t)(cid_len & 0xFF);
    memcpy(pkt + o, cid, cid_len);
    o += cid_len;

    if (send_all(pkt, o) != 0) {
        ES_WIFI_StopClientConnection(&EsWifiObj, &conn);
        return -2;
    }

    uint8_t ack[4] = {0};
    uint16_t rcv = 0;
    if (ES_WIFI_ReceiveData(&EsWifiObj, MQTT_SOCKET, ack, sizeof(ack), &rcv,
                            MQTT_TIMEOUT_MS) != ES_WIFI_STATUS_OK ||
        rcv < 4 || ack[0] != 0x20 || ack[3] != 0x00) {
        DEBUG_PRINTF("[MQTT] bad CONNACK (rcv=%u, code=%u)\n", rcv, ack[3]);
        ES_WIFI_StopClientConnection(&EsWifiObj, &conn);
        return -3;
    }
    return 0;
}

/* Publish one QoS-0 message to MQTT_TOPIC. */
static int mqtt_publish(const char *payload, int plen) {
    const char *topic = MQTT_TOPIC;
    int tlen = (int)strlen(topic);
    int remlen = 2 + tlen + plen;

    int o = 0;
    pub_buf[o++] = 0x30; /* PUBLISH, QoS 0, no retain */
    o += enc_remlen(pub_buf + o, remlen);
    pub_buf[o++] = (uint8_t)(tlen >> 8);
    pub_buf[o++] = (uint8_t)(tlen & 0xFF);
    if (o + tlen + plen > (int)sizeof(pub_buf))
        return -1; /* message too big for the buffer */
    memcpy(pub_buf + o, topic, tlen);
    o += tlen;
    memcpy(pub_buf + o, payload, plen);
    o += plen;

    return send_all(pub_buf, o);
}

static void mqtt_disconnect(void) {
    uint8_t d[2] = {0xE0, 0x00}; /* DISCONNECT */
    send_all(d, sizeof(d));
    ES_WIFI_StopClientConnection(&EsWifiObj, &conn);
}

/* ── network bring-up (lazy, once; no DNS — broker is a raw LAN IP) ──────── */

static int net_ensure(void) {
    if (wifi_joined)
        return 0;

    static const struct {
        const char *ssid;
        const char *pass;
    } networks[] = {NET_WIFI_NETWORKS};
    const int n_networks = sizeof(networks) / sizeof(networks[0]);

    if (WIFI_Init() != WIFI_STATUS_OK) {
        DEBUG_PRINTF("[MQTT] WiFi module init failed\n");
        return -1;
    }

    for (int i = 0; i < n_networks && !wifi_joined; i++) {
        DEBUG_PRINTF("[MQTT] joining \"%s\"...\n", networks[i].ssid);
        if (WIFI_Connect(networks[i].ssid, networks[i].pass,
                         NET_WIFI_SECURITY) == WIFI_STATUS_OK) {
            wifi_joined = 1;
            uint8_t ip[4] = {0};
            if (WIFI_GetIP_Address(ip, 4) == WIFI_STATUS_OK)
                SUCCESS_PRINTF("[MQTT] WiFi up on \"%s\", IP %d.%d.%d.%d\n",
                               networks[i].ssid, ip[0], ip[1], ip[2], ip[3]);
        }
    }
    if (!wifi_joined) {
        DEBUG_PRINTF("[MQTT] all %d WiFi networks failed\n", n_networks);
        return -2;
    }
    return 0;
}

/* ── upload ─────────────────────────────────────────────────────────────── */

/* Build one MQTT payload from an SD log line. Returns length, or -1 if the
   line isn't a record. */
static int build_payload(const char *line, char *out, int outsz) {
    float tick = 0, lat = 0, lon = 0, spd = 0, pred = 0, conf = 0, gmax = 0,
          collision = 0, gpeak = 0, fix = 0;
    if (json_num(line, "\"time\":", &tick) != 0)
        return -1;
    json_num(line, "\"lat\":", &lat);
    json_num(line, "\"lon\":", &lon);
    json_num(line, "\"spd\":", &spd);
    json_num(line, "\"pred\":", &pred);
    json_num(line, "\"conf\":", &conf);
    json_num(line, "\"gmax\":", &gmax);
    json_num(line, "\"collision\":", &collision);
    json_num(line, "\"gpeak\":", &gpeak);
    json_num(line, "\"fix\":", &fix);

    uint64_t epoch_ms = GPS_TickToEpochMs((uint32_t)tick);
    uint32_t sec = (uint32_t)(epoch_ms / 1000ULL);
    if (sec <= ts_last_sec) /* force strictly increasing, unique seconds */
        sec = ts_last_sec + 1;
    ts_last_sec = sec;

    int n = snprintf(out, outsz,
                     "{\"dev\":\"" DEVICE_TOKEN "\",\"ts\":%lu,\"pred\":%d,"
                     "\"conf\":%.1f,\"lat\":%.6f,\"lon\":%.6f,\"spd\":%.1f,"
                     "\"gmax\":%.3f,\"collision\":%d,\"gpeak\":%.2f,\"fix\":%d}",
                     (unsigned long)sec, (int)pred, (double)conf, (double)lat,
                     (double)lon, (double)spd, (double)gmax, (int)collision,
                     (double)gpeak, (int)fix);
    if (n <= 0 || n >= outsz)
        return -1;
    return n;
}

static int upload_log(void) {
    ts_last_sec = 0; /* restart the monotonic sequence for this upload */
    if (net_ensure() != 0)
        return -1;

    if (SD_Log_PauseForUpload() != 0) {
        DEBUG_PRINTF("[MQTT] SD log not available\n");
        return -2;
    }

    FIL rd;
    if (f_open(&rd, SD_LOG_PATH, FA_READ) != FR_OK) {
        DEBUG_PRINTF("[MQTT] cannot open log for reading\n");
        SD_Log_ResumeAfterUpload(0);
        return -2;
    }

    if (mqtt_connect() != 0) {
        DEBUG_PRINTF("[MQTT] broker connect failed\n");
        f_close(&rd);
        SD_Log_ResumeAfterUpload(0); /* keep the log — retry next time */
        return -1;
    }

    int rc = -1;
    {
        char line[256];
        char payload[256];
        int sent = 0, ok = 1;
        while (f_gets(line, sizeof(line), &rd)) {
            int n = build_payload(line, payload, sizeof(payload));
            if (n < 0)
                continue; /* not a record line — skip */
            if (mqtt_publish(payload, n) != 0) {
                DEBUG_PRINTF("[MQTT] publish failed after %d events\n", sent);
                ok = 0;
                break;
            }
            sent++;
        }
        if (ok) {
            rc = 0;
            SUCCESS_PRINTF("[MQTT] upload complete: %d events\n", sent);
        }
    }

    mqtt_disconnect();
    f_close(&rd);
    SD_Log_ResumeAfterUpload(rc == 0); /* clear the log only on full success */
    return rc;
}

/* ── public API ─────────────────────────────────────────────────────────── */

int Mqtt_UploadNow(void) {
    if (!GPS_TimeKnown())
        DEBUG_PRINTF("[MQTT] WARNING: no GPS time — timestamps start at 1970\n");
    return upload_log();
}

void Mqtt_Process(const GPS_Fix_t *fix) {
    if (!fix->valid || !GPS_TimeKnown())
        return;

    const float m_per_deg_lat = 111320.0f;
    float m_per_deg_lon =
        111320.0f * cosf((float)HOME_LAT * 3.1415926f / 180.0f);
    float dy = (fix->lat - (float)HOME_LAT) * m_per_deg_lat;
    float dx = (fix->lon - (float)HOME_LON) * m_per_deg_lon;
    float dist = sqrtf(dx * dx + dy * dy);

    if (dist > HOME_RADIUS_M * 1.5f) {
        was_outside = 1;
        return;
    }

    if (dist <= HOME_RADIUS_M && was_outside) {
        SUCCESS_PRINTF("[MQTT] home zone reached (%.0f m) — uploading\n",
                       (double)dist);
        /* Disarm only on a fully successful upload, so a failed drain retries
           on the next pass while still parked. */
        if (upload_log() == 0)
            was_outside = 0;
    }
}
