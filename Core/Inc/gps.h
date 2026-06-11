#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float   lat;        /* decimal degrees, positive = North */
    float   lon;        /* decimal degrees, positive = East  */
    float   speed_kmh;
    uint8_t valid;      /* 1 = active fix, 0 = void          */
} GPS_Fix_t;

void             GPS_Init(void);
void             GPS_UART_RxCpltCallback(void);
void             GPS_UART_ErrorCallback(void);
const GPS_Fix_t *GPS_GetFix(void);

/* Diagnostics: NMEA sentences received since boot (any type).
   0 after >2 s of run time = UART/wiring problem. */
uint32_t GPS_SentenceCount(void);

/* UTC time anchor — valid once a GPRMC with active fix has arrived */
uint8_t  GPS_TimeKnown(void);
/* Convert a HAL_GetTick() value to UTC epoch milliseconds.
   Works for ticks before or after the anchor was taken. */
uint64_t GPS_TickToEpochMs(uint32_t tick);

#ifdef __cplusplus
}
#endif
