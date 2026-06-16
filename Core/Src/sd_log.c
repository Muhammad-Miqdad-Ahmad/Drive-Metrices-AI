#include "sd_log.h"
#include "fatfs.h"
#include <math.h>
#include <stdio.h>

extern FATFS USERFatFS;
extern char USERPath[4];

static FIL log_file;
static int initialized = 0;
static int write_count = 0;

int SD_Log_Init(void) {
  FRESULT res = f_mount(&USERFatFS, USERPath, 1);
  printf("[SD] f_mount=%d\n", res);
  if (res != FR_OK) {
    printf("[SD] formatting...\n");
    BYTE work[512];
    FRESULT mkres = f_mkfs(USERPath, FM_FAT32, 0, work, sizeof(work));
    printf("[SD] f_mkfs=%d\n", mkres);
    if (mkres != FR_OK)
      return -1;
    res = f_mount(&USERFatFS, USERPath, 1);
    printf("[SD] f_mount after fmt=%d\n", res);
    if (res != FR_OK)
      return -1;
  }
  /* append — existing data on the card is preserved */
  res = f_open(&log_file, SD_LOG_PATH, FA_OPEN_ALWAYS | FA_WRITE);
  printf("[SD] f_open=%d\n", res);
  if (res != FR_OK)
    return -2;
  f_lseek(&log_file, f_size(&log_file));
  initialized = 1;
  return 0;
}

void SD_Log_Write(uint32_t timestamp_ms, const GPS_Fix_t *fix, int pred_class,
                  const char *pred_label, float confidence, const float *window,
                  int n_samples, int collision, float gpeak) {
  if (!initialized)
    return;

  /* Harshness = worst (peak) total acceleration magnitude in the window,
     |a| = sqrt(ax^2+ay^2+az^2). ~1.0 g at rest (gravity); spikes higher on
     hard brake/turn. Accel channels are 3,4,5 of each [gx,gy,gz,ax,ay,az]. */
  float gmax = 0.0f;
  for (int t = 0; t < n_samples; t++) {
    float ax = window[t * 6 + 3], ay = window[t * 6 + 4], az = window[t * 6 + 5];
    float mag = sqrtf(ax * ax + ay * ay + az * az);
    if (mag > gmax)
      gmax = mag;
  }

  /* Big enough for the prediction/GPS header + 6 arrays of n_samples
     floats (~10 chars each). 28 samples -> ~1.8 KB. */
  static char buf[2400];
  int o = snprintf(buf, sizeof(buf),
                   "{\"time\":%lu,\"pred\":%d,\"label\":\"%s\",\"conf\":%.1f,"
                   "\"lat\":%.6f,\"lon\":%.6f,\"spd\":%.1f,\"fix\":%d,"
                   "\"gmax\":%.3f,\"collision\":%d,\"gpeak\":%.2f",
                   (unsigned long)timestamp_ms, pred_class, pred_label,
                   (double)confidence, (double)fix->lat, (double)fix->lon,
                   (double)fix->speed_kmh, (int)fix->valid, (double)gmax,
                   collision, (double)gpeak);

  static const char *keys[6] = {"gx", "gy", "gz", "ax", "ay", "az"};
  for (int ch = 0; ch < 6 && o < (int)sizeof(buf); ch++) {
    o += snprintf(buf + o, sizeof(buf) - o, ",\"%s\":[", keys[ch]);
    for (int t = 0; t < n_samples && o < (int)sizeof(buf); t++)
      o += snprintf(buf + o, sizeof(buf) - o, "%s%.4f", t ? "," : "",
                    (double)window[t * 6 + ch]);
    o += snprintf(buf + o, sizeof(buf) - o, "]");
  }
  o += snprintf(buf + o, sizeof(buf) - o, "}\n");

  UINT bw;
  f_write(&log_file, buf, (UINT)o, &bw);

  if (++write_count % 10 == 0)
    f_sync(&log_file);
}

int SD_Log_PauseForUpload(void) {
  if (!initialized)
    return -1;
  f_sync(&log_file);
  f_close(&log_file);
  initialized = 0;
  return 0;
}

int SD_Log_ResumeAfterUpload(int clear) {
  FRESULT res = f_open(&log_file, SD_LOG_PATH,
                       clear ? (FA_CREATE_ALWAYS | FA_WRITE)
                             : (FA_OPEN_ALWAYS | FA_WRITE));
  if (res != FR_OK)
    return -1;
  if (!clear)
    f_lseek(&log_file, f_size(&log_file));
  write_count = 0;
  initialized = 1;
  return 0;
}
