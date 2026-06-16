/**
 * ThingSpeak batch uploader — plain HTTP, direct from the board.
 *
 * The SD log accumulates one JSON line per inference. On upload, lines
 * are converted to ThingSpeak bulk updates and POSTed to
 * /channels/<id>/bulk_update.json in batches. The log is cleared only
 * after every batch succeeded.
 *
 * Field mapping: field1=prediction class, field2=confidence %,
 *                field3=latitude, field4=longitude, field5=speed km/h,
 *                field6=peak |a| harshness (g),
 *                field7=Unix epoch seconds (device/GPS timestamp),
 *                field8=collision flag (1 = detected impact)
 *
 * Timestamps: SD log stores HAL_GetTick() ms; the GPS time anchor
 * converts ticks to UTC (ISO 8601 created_at). Without a GPS fix the
 * timestamps start at 1970 — fine for bench tests.
 *
 * Free-tier rate limit: one bulk request per 15 s — honored between
 * consecutive batches of a large backlog.
 */
#include "thingspeak.h"
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

#define TS_SOCKET      0
#define TS_TIMEOUT_MS  10000
#define TS_HDR_SIZE    512
#define TS_BODY_SIZE   3072 /* ~25 updates per bulk request */
#define TS_RESP_SIZE   1024 /* big enough for headers + the error JSON body */
#define TS_RATE_MS     15500 /* free tier: 1 bulk request / 15 s */

static uint8_t server_ip[4];
static uint8_t wifi_joined  = 0;
static uint8_t dns_resolved = 0;
static uint8_t was_outside  = 1; /* geofence hysteresis */
static uint32_t ts_last_sec = 0; /* last created_at second — kept strictly
                                    increasing so no two updates collide */

static char hdr_buf[TS_HDR_SIZE];
static char body_buf[TS_BODY_SIZE];
static char resp_buf[TS_RESP_SIZE];

/* ── time formatting (Howard Hinnant's civil_from_days) ─────────────── */

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

/* epoch ms → "2026-06-11T12:34:56Z" (out must hold 24 bytes) */
static void epoch_to_iso(uint64_t ms, char *out) {
    uint64_t sec = ms / 1000;
    int y, mo, d;
    civil_from_days((int64_t)(sec / 86400), &y, &mo, &d);
    int rem = (int)(sec % 86400);
    snprintf(out, 24, "%04d-%02d-%02dT%02d:%02d:%02dZ", y, mo, d, rem / 3600,
             (rem % 3600) / 60, rem % 60);
}

/* ── tiny JSON field extractors for our own SD log lines ────────────── */

static int json_num(const char *line, const char *key, float *out) {
    const char *p = strstr(line, key);
    if (!p) return -1;
    *out = strtof(p + strlen(key), NULL);
    return 0;
}

/* ── HTTP over a plain module TCP socket ────────────────────────────── */

/* POST `body` to `path`. Returns HTTP status or <0 on transport error. */
static int http_post(const char *path, const char *body) {
    ES_WIFI_Conn_t conn;
    memset(&conn, 0, sizeof(conn));
    conn.Number     = TS_SOCKET;
    conn.Type       = ES_WIFI_TCP_CONNECTION;
    conn.RemotePort = THINGSPEAK_PORT;
    memcpy(conn.RemoteIP, server_ip, 4);

    int body_len = (int)strlen(body);
    int hdr_len = snprintf(hdr_buf, sizeof(hdr_buf),
        "POST %s HTTP/1.1\r\n"
        "Host: " THINGSPEAK_HOST "\r\n"
        "Content-Type: application/json\r\n"
        "Content-Length: %d\r\n"
        "Connection: close\r\n"
        "\r\n",
        path, body_len);
    if (hdr_len <= 0 || hdr_len >= (int)sizeof(hdr_buf))
        return -1;

    if (ES_WIFI_StartClientConnection(&EsWifiObj, &conn) != ES_WIFI_STATUS_OK) {
        DEBUG_PRINTF("[TS] connect failed (module says: %.100s)\n",
                     (char *)EsWifiObj.CmdData);
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
            if (ES_WIFI_SendData(&EsWifiObj, TS_SOCKET,
                                 (uint8_t *)parts[p] + off, chunk, &sent,
                                 TS_TIMEOUT_MS) != ES_WIFI_STATUS_OK ||
                sent == 0)
                goto out;
            off += sent;
        }
    }

    {
        /* Drain the whole response (headers + body). The server uses
           "Connection: close", so reads return 0 once it's done. Looping lets
           us capture ThingSpeak's JSON error body, not just the headers. */
        int total = 0;
        while (total < (int)sizeof(resp_buf) - 1) {
            uint16_t rcv = 0;
            if (ES_WIFI_ReceiveData(&EsWifiObj, TS_SOCKET,
                                    (uint8_t *)resp_buf + total,
                                    sizeof(resp_buf) - 1 - total, &rcv,
                                    TS_TIMEOUT_MS) != ES_WIFI_STATUS_OK)
                break;
            if (rcv == 0)
                break;
            total += rcv;
        }
        if (total == 0)
            goto out;
        resp_buf[total] = '\0';

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
        static const struct {
            const char *ssid;
            const char *pass;
        } networks[] = {NET_WIFI_NETWORKS};
        const int n_networks = sizeof(networks) / sizeof(networks[0]);

        if (WIFI_Init() != WIFI_STATUS_OK) {
            DEBUG_PRINTF("[TS] WiFi module init failed\n");
            return -1;
        }

        for (int i = 0; i < n_networks && !wifi_joined; i++) {
            DEBUG_PRINTF("[TS] joining \"%s\"...\n", networks[i].ssid);
            if (WIFI_Connect(networks[i].ssid, networks[i].pass,
                             NET_WIFI_SECURITY) == WIFI_STATUS_OK) {
                wifi_joined = 1;
                uint8_t ip[4] = {0};
                if (WIFI_GetIP_Address(ip, 4) == WIFI_STATUS_OK)
                    SUCCESS_PRINTF("[TS] WiFi up on \"%s\", IP %d.%d.%d.%d\n",
                                   networks[i].ssid, ip[0], ip[1], ip[2],
                                   ip[3]);
            }
        }
        if (!wifi_joined) {
            DEBUG_PRINTF("[TS] all %d WiFi networks failed\n", n_networks);
            return -2;
        }
    }
    if (!dns_resolved) {
        /* The ISM43362 DNS resolver is flaky and often fails the first
           try — retry a few times before giving up. */
        int ok = 0;
        for (int attempt = 0; attempt < 4 && !ok; attempt++) {
            if (attempt > 0)
                HAL_Delay(500);
            if (WIFI_GetHostAddress(THINGSPEAK_HOST, server_ip, 4) ==
                WIFI_STATUS_OK)
                ok = 1;
            else
                DEBUG_PRINTF("[TS] DNS lookup failed (try %d/4)\n", attempt + 1);
        }
        if (!ok)
            return -3;
        DEBUG_PRINTF("[TS] " THINGSPEAK_HOST " -> %d.%d.%d.%d\n", server_ip[0],
                     server_ip[1], server_ip[2], server_ip[3]);
        dns_resolved = 1;
    }
    return 0;
}

/* ── upload ─────────────────────────────────────────────────────────── */

/* Append one SD log line as a bulk update at body offset *off.
   Returns 0 = appended, -1 = not a log line, -2 = batch full. */
static int append_update(const char *line, int *off) {
    float tick = 0, lat = 0, lon = 0, spd = 0, pred = 0, conf = 0, gmax = 0,
          collision = 0;
    if (json_num(line, "\"time\":", &tick) != 0)
        return -1;
    json_num(line, "\"lat\":", &lat);
    json_num(line, "\"lon\":", &lon);
    json_num(line, "\"spd\":", &spd);
    json_num(line, "\"pred\":", &pred);
    json_num(line, "\"conf\":", &conf);
    json_num(line, "\"gmax\":", &gmax);           /* peak |a| harshness */
    json_num(line, "\"collision\":", &collision); /* 1 = detected impact */

    uint64_t epoch_ms = GPS_TickToEpochMs((uint32_t)tick);
    uint32_t sec = (uint32_t)(epoch_ms / 1000ULL);
    /* Force strictly-increasing, unique timestamps. ThingSpeak rejects a bulk
       request that contains two identical created_at values, so if this record
       lands on the same (or an earlier) second as the previous one, bump it. */
    if (sec <= ts_last_sec)
        sec = ts_last_sec + 1;
    ts_last_sec = sec;

    char iso[24];
    epoch_to_iso((uint64_t)sec * 1000ULL, iso);

    /* field7 = Unix epoch seconds. Sent as a 32-bit integer with %lu:
       newlib-nano has no %llu, and epoch seconds fit in uint32 until 2106.
       (%.0f is avoided — a float can't hold a 10-digit epoch exactly.) */
    int n = snprintf(body_buf + *off, TS_BODY_SIZE - *off,
                     "%s{\"created_at\":\"%s\",\"field1\":%d,"
                     "\"field2\":%.1f,\"field3\":%.6f,\"field4\":%.6f,"
                     "\"field5\":%.1f,\"field6\":%.3f,\"field7\":%lu,"
                     "\"field8\":%d}",
                     body_buf[*off - 1] == '[' ? "" : ",", iso, (int)pred,
                     (double)conf, (double)lat, (double)lon, (double)spd,
                     (double)gmax, (unsigned long)sec, (int)collision);
    if (n <= 0 || *off + n >= TS_BODY_SIZE - 2) /* room for closing ]} */
        return -2;
    *off += n;
    return 0;
}

#define TS_BULK_PATH "/channels/" THINGSPEAK_CHANNEL_ID "/bulk_update.json"
#define TS_BODY_PREFIX \
    "{\"write_api_key\":\"" THINGSPEAK_WRITE_KEY "\",\"updates\":["

static int body_reset(void) {
    int off = snprintf(body_buf, sizeof(body_buf), TS_BODY_PREFIX);
    return off;
}

#define TS_SEND_TRIES 3

static int body_send(int off) {
    body_buf[off]     = ']';
    body_buf[off + 1] = '}';
    body_buf[off + 2] = '\0';

    for (int attempt = 1; attempt <= TS_SEND_TRIES; attempt++) {
        if (attempt > 1) {
            DEBUG_PRINTF("[TS] retry %d/%d (waiting 15 s for rate limit)...\n",
                         attempt, TS_SEND_TRIES);
            HAL_Delay(TS_RATE_MS);
        }
        int code = http_post(TS_BULK_PATH, body_buf);
        if (code >= 200 && code < 300)
            return 0;

        DEBUG_PRINTF("[TS] bulk POST failed (HTTP %d)\n", code);
        /* Print ThingSpeak's actual error message: the JSON body sits after the
           blank line that ends the HTTP headers. */
        const char *b = strstr(resp_buf, "\r\n\r\n");
        DEBUG_PRINTF("[TS] response body: %.300s\n", b ? b + 4 : resp_buf);
    }
    return -1;
}

static int upload_log(void) {
    ts_last_sec = 0; /* restart the monotonic timestamp sequence for this upload */
    if (net_ensure() != 0)
        return -1;

    if (SD_Log_PauseForUpload() != 0) {
        DEBUG_PRINTF("[TS] SD log not available\n");
        return -2;
    }

    int rc = -1;
    FIL rd;
    char line[256];

    if (f_open(&rd, SD_LOG_PATH, FA_READ) != FR_OK) {
        DEBUG_PRINTF("[TS] cannot open log for reading\n");
        SD_Log_ResumeAfterUpload(0);
        return -2;
    }

    {
        int off = body_reset();
        int batched = 0, sent = 0, batches_sent = 0;
        const char *pending = NULL;

        for (;;) {
            if (!pending && !f_gets(line, sizeof(line), &rd))
                break;
            const char *src = pending ? pending : line;
            int r = append_update(src, &off);
            pending = NULL;
            if (r == -2) {
                if (batched == 0)
                    goto out; /* single record larger than buffer — abort */
                if (batches_sent > 0) {
                    DEBUG_PRINTF("[TS] rate limit — waiting 15 s...\n");
                    HAL_Delay(TS_RATE_MS);
                }
                if (body_send(off) != 0)
                    goto out;
                batches_sent++;
                sent += batched;
                DEBUG_PRINTF("[TS] %d events sent\n", sent);
                off = body_reset();
                batched = 0;
                pending = line; /* retry the line in the fresh batch */
            } else if (r == 0) {
                batched++;
            } /* r == -1: not a log line, skip */
        }

        if (batched > 0) {
            if (batches_sent > 0) {
                DEBUG_PRINTF("[TS] rate limit — waiting 15 s...\n");
                HAL_Delay(TS_RATE_MS);
            }
            if (body_send(off) != 0)
                goto out;
            sent += batched;
        }

        if (sent == 0) {
            DEBUG_PRINTF("[TS] log empty, nothing to upload\n");
            rc = 0;
            goto out;
        }
        SUCCESS_PRINTF("[TS] upload complete: %d events\n", sent);
    }

    rc = 0;

out:
    f_close(&rd);
    /* clear the log only on full success */
    SD_Log_ResumeAfterUpload(rc == 0);
    return rc;
}

/* ── public API ─────────────────────────────────────────────────────── */

int ThingSpeak_UploadNow(void) {
    if (!GPS_TimeKnown())
        DEBUG_PRINTF("[TS] WARNING: no GPS time — timestamps start at 1970\n");
    return upload_log();
}

void ThingSpeak_Process(const GPS_Fix_t *fix) {
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
        SUCCESS_PRINTF("[TS] home zone reached (%.0f m) — uploading\n",
                       (double)dist);
        /* Disarm only on a fully successful upload. If it fails (e.g. a bad
           batch), keep was_outside set so the next pass retries while still
           parked, instead of giving up until we leave and return. */
        if (upload_log() == 0)
            was_outside = 0;
    }
}
