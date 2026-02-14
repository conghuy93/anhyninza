/*
 * Stream Music Player - HTTP streaming from music server
 * Downloads MP3 from xiaozhishop.xyz, decodes with minimp3, plays via AudioCodec
 */
#ifndef STREAM_PLAYER_H
#define STREAM_PLAYER_H

#include <string>
#include <vector>
#include <functional>
#include <cstdint>
#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_ae_rate_cvt.h>
#include <esp_http_client.h>

class AudioCodec;

enum class StreamPlayerState {
    STOPPED,
    SEARCHING,
    BUFFERING,
    PLAYING,
    ERROR
};

// Lyric line: timestamp_ms + text
struct LyricLine {
    int timestamp_ms;
    std::string text;
};

// Song info returned from server
struct SongInfo {
    std::string title;
    std::string artist;
    std::string thumbnail;  // Thumbnail image URL
    int bitrate_kbps;
    int sample_rate;
    int channels;
    std::string status_text; // "Đang phát...", "Lỗi", etc.
};

class StreamPlayer {
public:
    StreamPlayer(AudioCodec* codec);
    ~StreamPlayer();

    // Search and play a song by name
    bool SearchAndPlay(const std::string& song_name, const std::string& artist_name = "");
    
    // Stop playback
    void Stop();
    
    // State
    StreamPlayerState GetState() const { return state_; }
    const SongInfo& GetSongInfo() const { return song_info_; }
    
    // Lyrics
    const std::vector<LyricLine>& GetLyrics() const { return lyrics_; }
    int GetCurrentLyricIndex() const { return current_lyric_index_; }
    std::string GetCurrentLyricText() const;
    int64_t GetPlayTimeMs() const { return play_time_ms_; }
    
    // Server URL config
    void SetServerUrl(const std::string& url) { server_url_ = url; }
    const std::string& GetServerUrl() const { return server_url_; }
    
    // Thumbnail download (returns binary data, caller must free)
    bool DownloadThumbnail(const std::string& url, uint8_t** data, size_t* size);

private:
    AudioCodec* codec_;
    volatile StreamPlayerState state_ = StreamPlayerState::STOPPED;
    SongInfo song_info_;
    
    // Server
    std::string server_url_;
    std::string audio_url_;
    std::string lyric_url_;
    
    // Lyrics
    std::vector<LyricLine> lyrics_;
    volatile int current_lyric_index_ = -1;
    volatile int64_t play_time_ms_ = 0;
    
    // Threads 
    TaskHandle_t stream_task_ = nullptr;
    SemaphoreHandle_t mutex_ = nullptr;
    volatile bool stop_requested_ = false;
    
    // Static task memory for PSRAM stack allocation (decode task only)
    // Download task uses internal SRAM to avoid WiFi cache contention on Core 0
    StaticTask_t* stream_task_tcb_ = nullptr;
    StackType_t* stream_task_stack_ = nullptr;
    
    // Download task (separate from decode for smooth playback)
    TaskHandle_t download_task_ = nullptr;
    volatile bool download_done_ = false;
    volatile bool download_error_ = false;
    volatile size_t total_downloaded_ = 0;
    
    // Task completion flags (set BEFORE vTaskDelete to avoid race)
    std::atomic<bool> download_task_exited_{true};
    std::atomic<bool> stream_task_exited_{true};
    
    // HTTP client handle for instant abort on Stop()
    esp_http_client_handle_t active_http_ = nullptr;
    SemaphoreHandle_t http_mutex_ = nullptr;  // Use FreeRTOS semaphore (works with PSRAM stack)
    
    // Ring buffer for streaming (SPSC lock-free design)
    uint8_t* ring_buffer_ = nullptr;
    std::atomic<size_t> ring_write_pos_{0};  // Only written by download task
    std::atomic<size_t> ring_read_pos_{0};   // Only written by decode task
    // Buffer size must be power of 2 for fast modulo with bitmask
    static const size_t RING_BUFFER_SIZE = 256 * 1024; // 256KB ring buffer (power of 2)
    static const size_t RING_BUFFER_MASK = RING_BUFFER_SIZE - 1;
    static const size_t MIN_BUFFER_BEFORE_PLAY = 64 * 1024; // 64KB before starting decode
    
    // Internal methods
    bool FetchSongInfo(const std::string& song_name, const std::string& artist_name);
    bool FetchLyrics(const std::string& lyric_url);
    void ParseLyrics(const std::string& content);
    void UpdateCurrentLyric();
    
    static void DownloadTask(void* param);
    void DownloadLoop();
    static void StreamTask(void* param);
    void StreamAndDecode();
    
    // Ring buffer ops (lock-free SPSC)
    size_t RingWrite(const uint8_t* data, size_t len);
    size_t RingRead(uint8_t* data, size_t len);
    size_t RingAvailable() const {
        // Read positions atomically - lock-free
        size_t w = ring_write_pos_.load(std::memory_order_acquire);
        size_t r = ring_read_pos_.load(std::memory_order_acquire);
        return (w >= r) ? (w - r) : (RING_BUFFER_SIZE - r + w);
    }
    size_t RingSpace() const {
        return RING_BUFFER_SIZE - RingAvailable() - 1; // -1 to avoid full=empty ambiguity
    }
    
    // ESP Audio Elements resampler (optimized DSP)
    esp_ae_rate_cvt_handle_t resampler_ = nullptr;
    int resampler_src_rate_ = 0;
};

// C interface for webserver.c
#ifdef __cplusplus
extern "C" {
#endif

typedef void StreamPlayer_t;

// Init/destroy
void StreamPlayer_Init(void* audio_codec_ptr);
void StreamPlayer_Destroy(void);

// Playback control
int StreamPlayer_SearchAndPlay(const char* song_name, const char* artist_name);
void StreamPlayer_Stop(void);

// State queries (returns JSON string, caller must free)
int StreamPlayer_GetState(void);  // 0=STOPPED,1=SEARCHING,2=BUFFERING,3=PLAYING,4=ERROR
const char* StreamPlayer_GetTitle(void);
const char* StreamPlayer_GetArtist(void);
const char* StreamPlayer_GetThumbnail(void);  // Thumbnail image URL
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

// Thumbnail download (returns 1 on success, caller must free *data)
int StreamPlayer_DownloadThumbnail(const char* url, uint8_t** data, size_t* size);

#ifdef __cplusplus
}
#endif

#endif // STREAM_PLAYER_H
