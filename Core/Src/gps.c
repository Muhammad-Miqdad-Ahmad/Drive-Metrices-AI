#include "gps.h"
#include "main.h"
#include <string.h>
#include <stdlib.h>

extern UART_HandleTypeDef huart4;

static uint8_t   rx_byte;
static char      line_buf[128];
static uint8_t   line_pos;
static GPS_Fix_t fix;

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

static void parse_gprmc(const char *s) {
    /* $GPRMC,hhmmss.ss,status,lat,N/S,lon,E/W,speed_kn,course,date,...*XX */
    const char *f = next_field(s);   /* → time   */
    f = next_field(f);               /* → status */
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

    fix.valid     = (status == 'A') ? 1 : 0;
    fix.lat       = parse_coord(lat_s, ns);
    fix.lon       = parse_coord(lon_s, ew);
    fix.speed_kmh = speed_kn * 1.852f;
}

static void process_line(void) {
    if (strncmp(line_buf, "$GPRMC", 6) == 0)
        parse_gprmc(line_buf);
}

void GPS_Init(void) {
    memset(&fix, 0, sizeof(fix));
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
