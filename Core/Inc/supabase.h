#pragma once
#include "gps.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Call once per main-loop pass. When the GPS fix enters the home zone
   (HOME_LAT/HOME_LON/HOME_RADIUS_M in net_config.h) and the SD log has
   data, this joins WiFi and uploads the whole log to Supabase, then
   clears the log. Re-arms after the board leaves the zone. */
void Supabase_Process(const GPS_Fix_t *fix);

/* Force an upload attempt right now (e.g. for bench testing without GPS).
   Returns 0 on success. */
int Supabase_UploadNow(void);

#ifdef __cplusplus
}
#endif
