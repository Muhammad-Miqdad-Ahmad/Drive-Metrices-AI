#include "gps.h"
#include "main.h"
#include <string.h>
#include <stdlib.h>

extern UART_HandleTypeDef huart4;

static uint8_t   rx_byte;
static char      line_buf[128];
static uint8_t   line_pos;
static GPS_Fix_t fix;

/* UTC anchor: epoch ms at the moment anchor_tick was sampled */
static uint64_t anchor_epoch_ms;
static uint32_t anchor_tick;
static uint8_t  time_known;

/* diagnostics */
static uint32_t sentence_count;

/* "DDMM.mmmm" + hemisphere → decimal degrees */
static float parse_coord(const char *field, char hemi) {
    if (!field || !*field) return 0.0f;
    float raw  = strtof(field, NULL);
    int   deg  = (int)(raw / 100);
    float mins = raw - (float)(deg * 100);
    float val  = (float)deg + mins / 60.0f;
    if (hemi == 'S' || hemi == 'W') val = -val;
    return val;
}

/* Return pointer to the character after the next comma, or NULL */
static const char *next_field(const char *s) {
    s = strchr(s, ',');
    return s ? s + 1 : NULL;
}

static int two_digits(const char *s) {
    if (s[0] < '0' || s[0] > '9' || s[1] < '0' || s[1] > '9') return -1;
    return (s[0] - '0') * 10 + (s[1] - '0');
}

/* Days since 1970-01-01 (Howard Hinnant's days_from_civil) */
static int64_t days_from_civil(int y, int m, int d) {
    y -= m <= 2;
    int era = (y >= 0 ? y : y - 399) / 400;
    int yoe = y - era * 400;
    int doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    int doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return (int64_t)era * 146097 + doe - 719468;
}

/* "hhmmss.ss" + "ddmmyy" → UTC epoch ms; sets the time anchor */
static void update_time_anchor(const char *time_s, const char *date_s) {
    int hh = two_digits(time_s), mi = two_digits(time_s + 2),
        ss = two_digits(time_s + 4);
    int dd = two_digits(date_s), mo = two_digits(date_s + 2),
        yy = two_digits(date_s + 4);
    if (hh < 0 || mi < 0 || ss < 0 || dd < 0 || mo < 0 || yy < 0) return;

    uint32_t msec = 0;
    if (time_s[6] == '.') /* fractional seconds, up to 3 digits */
        msec = (uint32_t)(strtof(time_s + 6, NULL) * 1000.0f);

    anchor_epoch_ms = (uint64_t)days_from_civil(2000 + yy, mo, dd) * 86400000ULL +
                      (uint64_t)hh * 3600000ULL + (uint64_t)mi * 60000ULL +
                      (uint64_t)ss * 1000ULL + msec;
    anchor_tick = HAL_GetTick();
    time_known  = 1;
}

static void parse_gprmc(const char *s) {
    /* $GPRMC,hhmmss.ss,status,lat,N/S,lon,E/W,speed_kn,course,date,...*XX */
    const char *time_s = next_field(s); /* → time   */
    const char *f = next_field(time_s); /* → status */
    if (!f) return;
    char status = *f;

    f = next_field(f);               /* → lat    */
    const char *lat_s = f;
    f = next_field(f);               /* → N/S    */
    char ns = f ? *f : 'N';

    f = next_field(f);               /* → lon    */
    const char *lon_s = f;
    f = next_field(f);               /* → E/W    */
    char ew = f ? *f : 'E';

    f = next_field(f);               /* → speed (knots) */
    float speed_kn = f ? strtof(f, NULL) : 0.0f;

    f = next_field(f);               /* → course */
    f = next_field(f);               /* → date ddmmyy */
    const char *date_s = f;

    fix.valid     = (status == 'A') ? 1 : 0;
    fix.lat       = parse_coord(lat_s, ns);
    fix.lon       = parse_coord(lon_s, ew);
    fix.speed_kmh = speed_kn * 1.852f;

    if (fix.valid && time_s && date_s)
        update_time_anchor(time_s, date_s);
}

static void process_line(void) {
    if (line_buf[0] == '$')
        sentence_count++;
    /* NEO-7M outputs $GPRMC; newer modules may use $GNRMC */
    if (strncmp(line_buf, "$GPRMC", 6) == 0 ||
        strncmp(line_buf, "$GNRMC", 6) == 0)
        parse_gprmc(line_buf);
}

void GPS_Init(void) {
    memset(&fix, 0, sizeof(fix));
    line_pos = 0;
    /* The GPS has been streaming since power-on with nobody reading — the
       overrun flag is set and would instantly abort interrupt reception.
       Clear all stale errors before arming. */
    __HAL_UART_CLEAR_FLAG(&huart4, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                       UART_CLEAR_FEF | UART_CLEAR_PEF);
    /* UART4 IRQ is enabled by HAL_UART_MspInit (NVIC ticked in CubeMX) */
    HAL_UART_Receive_IT(&huart4, &rx_byte, 1);
}

/* Call from HAL_UART_ErrorCallback: on any UART error (overrun etc.) the
   HAL stops reception — clear the error and re-arm so GPS RX survives. */
void GPS_UART_ErrorCallback(void) {
    __HAL_UART_CLEAR_FLAG(&huart4, UART_CLEAR_OREF | UART_CLEAR_NEF |
                                       UART_CLEAR_FEF | UART_CLEAR_PEF);
    line_pos = 0;
    HAL_UART_Receive_IT(&huart4, &rx_byte, 1);
}

void GPS_UART_RxCpltCallback(void) {
    char c = (char)rx_byte;
    if (c == '\n') {
        line_buf[line_pos] = '\0';
        process_line();
        line_pos = 0;
    } else if (c != '\r' && line_pos < (uint8_t)(sizeof(line_buf) - 1)) {
        line_buf[line_pos++] = c;
    }
    HAL_UART_Receive_IT(&huart4, &rx_byte, 1);
}

const GPS_Fix_t *GPS_GetFix(void) {
    return &fix;
}

uint32_t GPS_SentenceCount(void) {
    return sentence_count;
}

uint8_t GPS_TimeKnown(void) {
    return time_known;
}

uint64_t GPS_TickToEpochMs(uint32_t tick) {
    /* signed delta handles ticks taken before the anchor */
    return anchor_epoch_ms + (int64_t)(int32_t)(tick - anchor_tick);
}
