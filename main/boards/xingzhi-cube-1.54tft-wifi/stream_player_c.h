/*
 * Stream Music Player - C interface only
 * Use this header from C files (webserver.c)
 * For C++ code, use stream_player.h instead
 */
#ifndef STREAM_PLAYER_C_H
#define STREAM_PLAYER_C_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Init/destroy
void StreamPlayer_Init(void* audio_codec_ptr);
void StreamPlayer_Destroy(void);

// Playback control
int StreamPlayer_SearchAndPlay(const char* song_name, const char* artist_name);
void StreamPlayer_Stop(void);

// State queries
int StreamPlayer_GetState(void);  // 0=STOPPED,1=SEARCHING,2=BUFFERING,3=PLAYING,4=ERROR
const char* StreamPlayer_GetTitle(void);
const char* StreamPlayer_GetArtist(void);
int StreamPlayer_GetBitrate(void);
int StreamPlayer_GetSampleRate(void);
const char* StreamPlayer_GetStatusText(void);

// Lyrics
int StreamPlayer_GetLyricCount(void);
int StreamPlayer_GetCurrentLyricIndex(void);
const char* StreamPlayer_GetCurrentLyricText(void);
int64_t StreamPlayer_GetPlayTimeMs(void);

// Server URL
void StreamPlayer_SetServerUrl(const char* url);
const char* StreamPlayer_GetServerUrl(void);

#ifdef __cplusplus
}
#endif

#endif // STREAM_PLAYER_C_H
