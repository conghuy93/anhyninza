/*
 * MP3 Player (SD Card) - C interface only
 * Use this header from C files (webserver.c)
 * For C++ code, use mp3_player.h instead
 */
#ifndef MP3_PLAYER_C_H
#define MP3_PLAYER_C_H

#ifdef __cplusplus
extern "C" {
#endif

// SD Card file listing
typedef struct {
    char name[128];     // filename without path
    char path[256];     // full path
    int is_dir;         // 1=directory, 0=file
    long size;          // file size in bytes
} sd_file_entry_t;

// Initialize SD player (called once from board init)
void SdPlayer_Init(void);

// Playback control
int  SdPlayer_Play(const char* filepath);
void SdPlayer_Stop(void);
void SdPlayer_Pause(void);
void SdPlayer_Resume(void);
void SdPlayer_Next(void);
void SdPlayer_Previous(void);

// State: 0=STOPPED, 1=PLAYING, 2=PAUSED
int  SdPlayer_GetState(void);
const char* SdPlayer_GetCurrentTrack(void);
int  SdPlayer_GetCurrentIndex(void);
int  SdPlayer_GetPlaylistSize(void);

// SD Card file listing
// Returns number of entries written to out_entries (max max_entries)
int  SdPlayer_ListDir(const char* path, sd_file_entry_t* out_entries, int max_entries);

// Check if SD card is mounted
int  SdPlayer_IsSdMounted(void);

// Get playlist entry by index (returns filepath, NULL if out of range)
const char* SdPlayer_GetPlaylistEntry(int index);

// Scan directory and build playlist
int  SdPlayer_ScanAndBuildPlaylist(const char* path);

// Check if ANY music is currently playing (SD or streaming)
int  SdPlayer_IsAnyMusicPlaying(void);

// Repeat mode: 0=OFF, 1=REPEAT_ONE, 2=REPEAT_ALL
void SdPlayer_SetRepeatMode(int mode);
int  SdPlayer_GetRepeatMode(void);

#ifdef __cplusplus
}
#endif

#endif // MP3_PLAYER_C_H
