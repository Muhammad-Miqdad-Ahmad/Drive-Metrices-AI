#include "sd_log.h"
#include "fatfs.h"
#include <stdio.h>

extern FATFS USERFatFS;
extern char  USERPath[4];

static FIL  log_file;
static int  initialized  = 0;
static int  write_count  = 0;

int SD_Log_Init(void) {
    FRESULT res = f_mount(&USERFatFS, USERPath, 1);
    printf("[SD] f_mount=%d\n", res);
    if (res != FR_OK) {
        printf("[SD] formatting...\n");
        BYTE work[512];
        FRESULT mkres = f_mkfs(USERPath, FM_FAT32, 0, work, sizeof(work));
        printf("[SD] f_mkfs=%d\n", mkres);
        if (mkres != FR_OK) return -1;
        res = f_mount(&USERFatFS, USERPath, 1);
        printf("[SD] f_mount after fmt=%d\n", res);
        if (res != FR_OK) return -1;
    }
    res = f_open(&log_file, "0:/data.log", FA_OPEN_ALWAYS | FA_WRITE);
    printf("[SD] f_open=%d\n", res);
    if (res != FR_OK) return -2;
    f_lseek(&log_file, f_size(&log_file));
    initialized = 1;
    return 0;
}

void SD_Log_Write(uint32_t timestamp_ms, const GPS_Fix_t *fix,
                  int pred_class, const char *pred_label, float confidence) {
    if (!initialized) return;

    char buf[256];
    int len = snprintf(buf, sizeof(buf),
        "{\"ts\":%lu,\"lat\":%.6f,\"lon\":%.6f,\"spd\":%.1f,"
        "\"fix\":%d,\"pred\":%d,\"label\":\"%s\",\"conf\":%.1f}\n",
        (unsigned long)timestamp_ms,
        (double)fix->lat, (double)fix->lon, (double)fix->speed_kmh,
        (int)fix->valid,
        pred_class, pred_label,
        (double)confidence);

    UINT bw;
    f_write(&log_file, buf, (UINT)len, &bw);

    if (++write_count % 10 == 0)
        f_sync(&log_file);
}
