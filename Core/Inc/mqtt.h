#pragma once
#include "gps.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Drain the whole SD log to the MQTT broker (one PUBLISH per record),
   then clear the log on full success. Joins WiFi on first use.
   Returns 0 on success. */
int Mqtt_UploadNow(void);

/* Geofenced variant: call once per main-loop pass; uploads when the GPS
   fix enters the home zone (HOME_LAT/LON/RADIUS in net_config.h). */
void Mqtt_Process(const GPS_Fix_t *fix);

#ifdef __cplusplus
}
#endif
