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
const GPS_Fix_t *GPS_GetFix(void);

#ifdef __cplusplus
}
#endif
