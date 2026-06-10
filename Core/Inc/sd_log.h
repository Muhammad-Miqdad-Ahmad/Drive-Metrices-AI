#pragma once
#include "gps.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int  SD_Log_Init(void);
void SD_Log_Write(uint32_t timestamp_ms, const GPS_Fix_t *fix,
                  int pred_class, const char *pred_label, float confidence);

#ifdef __cplusplus
}
#endif
