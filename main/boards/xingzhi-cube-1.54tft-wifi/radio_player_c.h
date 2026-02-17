/*
 * Radio Player - C interface only
 * Use this header from C files (webserver.c)
 * For C++ code, use esp32_radio.h instead
 */
#ifndef RADIO_PLAYER_C_H
#define RADIO_PLAYER_C_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Playback control
bool Radio_PlayStation(const char* station_name);
bool Radio_PlayUrl(const char* url, const char* name);
bool Radio_Stop(void);

// State queries
bool Radio_IsPlaying(void);
const char* Radio_GetCurrentStation(void);

// Station list (returns JSON string with array of stations)
const char* Radio_GetStationListJson(void);

#ifdef __cplusplus
}
#endif

#endif // RADIO_PLAYER_C_H
