#pragma once
#include "gps.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SD_LOG_PATH "0:/data.log"

int  SD_Log_Init(void);
/* window = n_samples * 6 floats, oldest-first, laid out per sample as
   [gx,gy,gz,ax,ay,az] (raw values, from Classifier_CopyWindow). */
void SD_Log_Write(uint32_t timestamp_ms, const GPS_Fix_t *fix,
                  int pred_class, const char *pred_label, float confidence,
                  const float *window, int n_samples);

/* Close the append handle so SD_LOG_PATH can be opened for reading.
   Returns 0 on success. */
int  SD_Log_PauseForUpload(void);
/* Reopen for logging. clear=1 truncates the file (after a successful
   upload), clear=0 resumes appending. Returns 0 on success. */
int  SD_Log_ResumeAfterUpload(int clear);

#ifdef __cplusplus
}
#endif
