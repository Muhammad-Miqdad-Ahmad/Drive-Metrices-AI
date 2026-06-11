/**
 * Supabase batch uploader.
 *
 * The board logs every inference to the SD card during a trip. When the
 * GPS fix enters the home zone, this module joins WiFi, creates a trip
 * row, streams the whole SD log to trip_events in batched JSON-array
 * POSTs, fixes the trip's end_time, then clears the log.
 *
 * Timestamps: the SD log stores HAL_GetTick() ms. The GPS time anchor
 * (gps.c) converts any tick to UTC epoch ms, which is formatted here as
 * ISO 8601 for the timestamptz columns.
 *
 * TLS: the es-WiFi module's built-in TLS (P9=2, no cert verification).
 */
#include "supabase.h"
#include "app_log.h"
#include "net_config.h"
#include "sd_log.h"

#include "wifi.h"
#include "es_wifi.h"
#include "fatfs.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern ES_WIFIObject_t EsWifiObj; /* defined in Drivers/WiFi/Src/wifi.c */

#define SB_SOCKET      0
#define SB_TIMEOUT_MS  10000
#define SB_HDR_SIZE    768
#define SB_BODY_SIZE   3072  /* ~12 events per batch */
#define SB_RESP_SIZE   512
#define SB_EVENT_SIZE  280   /* worst-case single event JSON */

static uint8_t server_ip[4];
static char    trip_id[40];
static uint8_t wifi_joined  = 0;
static uint8_t dns_resolved = 0;
static uint8_t was_outside  = 1; /* geofence hysteresis: arm at boot */

static char hdr_buf[SB_HDR_SIZE];
static char body_buf[SB_BODY_SIZE];
static char resp_buf[SB_RESP_SIZE];

/* ── time formatting ────────────────────────────────────────────────── */

/* Howard Hinnant's civil_from_days */
static void civil_from_days(int64_t z, int *y, int *m, int *d) {
    z += 719468;
    int64_t era = (z >= 0 ? z : z - 146096) / 146097;
    int doe = (int)(z - era * 146097);
    int yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    int doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    int mp  = (5 * doy + 2) / 153;
    *d = doy - (153 * mp + 2) / 5 + 1;
    *m = mp + (mp < 10 ? 3 : -9);
    *y = (int)(yoe + era * 400) + (*m <= 2);
}

/* epoch ms → "2026-06-10T14:23:45.123Z" (out must hold 32 bytes) */
static void epoch_to_iso(uint64_t ms, char *out) {
    uint64_t sec = ms / 1000;
    int msec = (int)(ms % 1000);
    int y, mo, d;
    civil_from_days((int64_t)(sec / 86400), &y, &mo, &d);
    int rem = (int)(sec % 86400);
    snprintf(out, 32, "%04d-%02d-%02dT%02d:%02d:%02d.%03dZ", y, mo, d,
             rem / 3600, (rem % 3600) / 60, rem % 60, msec);
}

/* ── tiny JSON field extractors for our own SD log lines ────────────── */

static int json_num(const char *line, const char *key, float *out) {
    const char *p = strstr(line, key);
    if (!p) return -1;
    *out = strtof(p + strlen(key), NULL);
    return 0;
}

static int json_str(const char *line, const char *key, char *out, int max) {
    const char *p = strstr(line, key);
    if (!p) return -1;
    p += strlen(key);
    int i = 0;
    while (*p && *p != '"' && i < max - 1)
        out[i++] = *p++;
    out[i] = '\0';
    return 0;
}

/* ── HTTP over the module TLS socket ────────────────────────────────── */

/* Send `body` to `path` with `method`. Returns HTTP status or <0. */
static int http_request(const char *method, const char *path,
                        const char *body) {
    ES_WIFI_Conn_t conn;
    memset(&conn, 0, sizeof(conn));
    conn.Number     = SB_SOCKET;
    conn.Type       = ES_WIFI_TCP_SSL_CONNECTION;
    conn.RemotePort = SUPABASE_PORT;
    memcpy(conn.RemoteIP, server_ip, 4);

    int body_len = (int)strlen(body);
    int hdr_len = snprintf(hdr_buf, sizeof(hdr_buf),
        "%s %s HTTP/1.1\r\n"
        "Host: " SUPABASE_HOST "\r\n"
        "apikey: " SUPABASE_ANON_KEY "\r\n"
        "Authorization: Bearer " SUPABASE_ANON_KEY "\r\n"
        "Content-Type: application/json\r\n"
        "Prefer: return=minimal\r\n"
        "Content-Length: %d\r\n"
        "Connection: close\r\n"
        "\r\n",
        method, path, body_len);
    if (hdr_len <= 0 || hdr_len >= (int)sizeof(hdr_buf))
        return -1;

    if (ES_WIFI_StartClientConnection(&EsWifiObj, &conn) != ES_WIFI_STATUS_OK) {
        DEBUG_PRINTF("[SB] TLS connect failed\n");
        return -2;
    }

    int rc = -3;
    const char *parts[2] = {hdr_buf, body};
    int part_lens[2] = {hdr_len, body_len};
    for (int p = 0; p < 2; p++) {
        int off = 0;
        while (off < part_lens[p]) {
            uint16_t chunk = (uint16_t)(part_lens[p] - off);
            if (chunk > ES_WIFI_PAYLOAD_SIZE)
                chunk = ES_WIFI_PAYLOAD_SIZE;
            uint16_t sent = 0;
            if (ES_WIFI_SendData(&EsWifiObj, SB_SOCKET,
                                 (uint8_t *)parts[p] + off, chunk, &sent,
                                 SB_TIMEOUT_MS) != ES_WIFI_STATUS_OK ||
                sent == 0)
                goto out;
            off += sent;
        }
    }

    {
        uint16_t rcv = 0;
        if (ES_WIFI_ReceiveData(&EsWifiObj, SB_SOCKET, (uint8_t *)resp_buf,
                                sizeof(resp_buf) - 1, &rcv,
                                SB_TIMEOUT_MS) != ES_WIFI_STATUS_OK || rcv == 0)
            goto out;
        resp_buf[rcv] = '\0';

        int code = 0;
        if (sscanf(resp_buf, "HTTP/1.%*c %d", &code) == 1)
            rc = code;
    }

out:
    ES_WIFI_StopClientConnection(&EsWifiObj, &conn);
    return rc;
}

/* ── network bring-up (lazy, once) ──────────────────────────────────── */

static int net_ensure(void) {
    if (!wifi_joined) {
        if (WIFI_Init() != WIFI_STATUS_OK) {
            DEBUG_PRINTF("[SB] WiFi module init failed\n");
            return -1;
        }
        DEBUG_PRINTF("[SB] joining \"%s\"...\n", NET_WIFI_SSID);
        if (WIFI_Connect(NET_WIFI_SSID, NET_WIFI_PASSWORD,
                         NET_WIFI_SECURITY) != WIFI_STATUS_OK) {
            DEBUG_PRINTF("[SB] WiFi join failed\n");
            return -2;
        }
        uint8_t ip[4] = {0};
        if (WIFI_GetIP_Address(ip, 4) == WIFI_STATUS_OK)
            SUCCESS_PRINTF("[SB] WiFi up, IP %d.%d.%d.%d\n", ip[0], ip[1],
                           ip[2], ip[3]);
        wifi_joined = 1;
    }
    if (!dns_resolved) {
        if (WIFI_GetHostAddress(SUPABASE_HOST, server_ip, 4) !=
            WIFI_STATUS_OK) {
            DEBUG_PRINTF("[SB] DNS lookup failed\n");
            return -3;
        }
        dns_resolved = 1;
    }
    return 0;
}

/* ── upload ─────────────────────────────────────────────────────────── */

/* Append one converted event to body_buf at offset *off. The SD line
   format is written by SD_Log_Write():
   {"time":..,"lat":..,"lon":..,"spd":..,"fix":..,"pred":..,"label":"..","conf":..} */
static int append_event(const char *line, int *off) {
    float tick = 0, lat = 0, lon = 0, spd = 0, pred = 0, conf = 0;
    char  label[32] = "?";
    if (json_num(line, "\"time\":", &tick) != 0)
        return -1; /* not a log line */
    json_num(line, "\"lat\":", &lat);
    json_num(line, "\"lon\":", &lon);
    json_num(line, "\"spd\":", &spd);
    json_num(line, "\"pred\":", &pred);
    json_num(line, "\"conf\":", &conf);
    json_str(line, "\"label\":\"", label, sizeof(label));

    char iso[32];
    epoch_to_iso(GPS_TickToEpochMs((uint32_t)tick), iso);

    int n = snprintf(body_buf + *off, SB_BODY_SIZE - *off,
                     "%s{\"trip_id\":\"%s\",\"event_label\":%d,"
                     "\"event_name\":\"%s\",\"confidence\":%.1f,"
                     "\"latitude\":%.6f,\"longitude\":%.6f,"
                     "\"speed_kmh\":%.1f,\"recorded_at\":\"%s\"}",
                     (*off > 1) ? "," : "", trip_id, (int)pred, label,
                     (double)conf, (double)lat, (double)lon, (double)spd, iso);
    if (n <= 0 || *off + n >= SB_BODY_SIZE)
        return -2; /* batch full — caller flushes and retries */
    *off += n;
    return 0;
}

static int upload_log(void) {
    if (net_ensure() != 0)
        return -1;

    if (SD_Log_PauseForUpload() != 0) {
        DEBUG_PRINTF("[SB] SD log not available\n");
        return -2;
    }

    int rc = -1;
    FIL rd;
    char line[256];
    uint32_t first_tick = 0, last_tick = 0;
    int have_first = 0, total = 0;

    if (f_open(&rd, SD_LOG_PATH, FA_READ) != FR_OK) {
        DEBUG_PRINTF("[SB] cannot open log for reading\n");
        SD_Log_ResumeAfterUpload(0);
        return -2;
    }

    /* pass 1: find first/last tick for the trip row */
    while (f_gets(line, sizeof(line), &rd)) {
        float t;
        if (json_num(line, "\"time\":", &t) == 0) {
            last_tick = (uint32_t)t;
            if (!have_first) {
                first_tick = (uint32_t)t;
                have_first = 1;
            }
            total++;
        }
    }
    if (!have_first) {
        DEBUG_PRINTF("[SB] log empty, nothing to upload\n");
        f_close(&rd);
        SD_Log_ResumeAfterUpload(0);
        return 0;
    }
    DEBUG_PRINTF("[SB] uploading %d events...\n", total);

    /* create the trip row */
    {
        uint8_t mac[6] = {0};
        WIFI_GetMAC_Address(mac, 6);
        snprintf(trip_id, sizeof(trip_id),
                 "%02x%02x%02x%02x-%02x%02x-4000-8000-%012lx", mac[0], mac[1],
                 mac[2], mac[3], mac[4], mac[5], (unsigned long)first_tick);

        char start_iso[32], end_iso[32];
        epoch_to_iso(GPS_TickToEpochMs(first_tick), start_iso);
        epoch_to_iso(GPS_TickToEpochMs(last_tick), end_iso);

        snprintf(body_buf, sizeof(body_buf),
                 "{\"id\":\"%s\","
                 "\"device_token\":\"%02x:%02x:%02x:%02x:%02x:%02x\","
                 "\"local_trip_id\":\"trip-%lu\","
                 "\"start_time\":\"%s\",\"end_time\":\"%s\"}",
                 trip_id, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
                 (unsigned long)first_tick, start_iso, end_iso);

        int code = http_request("POST", "/rest/v1/trips", body_buf);
        if (code < 200 || code >= 300) {
            DEBUG_PRINTF("[SB] trip create failed (HTTP %d)\n", code);
            DEBUG_PRINTF("[SB] response: %.200s\n", resp_buf);
            goto out;
        }
    }

    /* pass 2: stream events in JSON-array batches */
    f_lseek(&rd, 0);
    {
        int off = 1, batched = 0, sent_events = 0;
        body_buf[0] = '[';
        const char *pending = NULL;

        for (;;) {
            if (!pending && !f_gets(line, sizeof(line), &rd))
                break;
            const char *src = pending ? pending : line;
            int r = append_event(src, &off);
            pending = NULL;
            if (r == -2) {
                /* batch full: flush, then retry this line */
                body_buf[off] = ']';
                body_buf[off + 1] = '\0';
                int code = http_request("POST", "/rest/v1/trip_events",
                                        body_buf);
                if (code < 200 || code >= 300) {
                    DEBUG_PRINTF("[SB] batch failed (HTTP %d)\n", code);
                    goto out;
                }
                sent_events += batched;
                DEBUG_PRINTF("[SB] %d/%d sent\n", sent_events, total);
                off = 1;
                batched = 0;
                pending = line;
            } else if (r == 0) {
                batched++;
            } /* r == -1: not a log line, skip */
        }

        if (batched > 0) {
            body_buf[off] = ']';
            body_buf[off + 1] = '\0';
            int code = http_request("POST", "/rest/v1/trip_events", body_buf);
            if (code < 200 || code >= 300) {
                DEBUG_PRINTF("[SB] final batch failed (HTTP %d)\n", code);
                goto out;
            }
            sent_events += batched;
        }
        SUCCESS_PRINTF("[SB] upload complete: %d events\n", sent_events);
    }

    rc = 0;

out:
    f_close(&rd);
    /* clear the log only on full success */
    SD_Log_ResumeAfterUpload(rc == 0);
    return rc;
}

/* ── public API ─────────────────────────────────────────────────────── */

int Supabase_UploadNow(void) {
    if (!GPS_TimeKnown()) {
        DEBUG_PRINTF("[SB] no GPS time yet — cannot timestamp upload\n");
        return -1;
    }
    return upload_log();
}

void Supabase_Process(const GPS_Fix_t *fix) {
    if (!fix->valid || !GPS_TimeKnown())
        return;

    /* equirectangular distance to home — fine for sub-km ranges */
    const float m_per_deg_lat = 111320.0f;
    float m_per_deg_lon = 111320.0f * cosf((float)HOME_LAT * 3.1415926f / 180.0f);
    float dy = (fix->lat - (float)HOME_LAT) * m_per_deg_lat;
    float dx = (fix->lon - (float)HOME_LON) * m_per_deg_lon;
    float dist = sqrtf(dx * dx + dy * dy);

    if (dist > HOME_RADIUS_M * 1.5f) {
        was_outside = 1; /* re-arm once clearly outside */
        return;
    }

    if (dist <= HOME_RADIUS_M && was_outside) {
        was_outside = 0;
        SUCCESS_PRINTF("[SB] home zone reached (%.0f m) — uploading\n",
                       (double)dist);
        upload_log();
    }
}
