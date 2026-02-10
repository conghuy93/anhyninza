#ifndef _MP3_PLAYER_H_
#define _MP3_PLAYER_H_

#include <string>
#include <vector>
#include <functional>
#include <cstdio>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

class AudioCodec;

enum class Mp3PlayerState {
    STOPPED,
    PLAYING,
    PAUSED
};

class Mp3Player {
public:
    Mp3Player(AudioCodec* codec);
    ~Mp3Player();

    // Playback control
    bool Play(const std::string& filepath);
    void Pause();
    void Resume();
    void Stop();
    void Next();
    void Previous();

    // Playlist management
    void SetPlaylist(const std::vector<std::string>& files);
    void ClearPlaylist();
    void AddToPlaylist(const std::string& filepath);
    
    // Scan SD card for MP3 files
    int ScanDirectory(const std::string& path, bool recursive = true);

    // State
    Mp3PlayerState GetState() const { return state_; }
    const std::string& GetCurrentTrack() const { return current_track_; }
    int GetCurrentIndex() const { return current_index_; }
    int GetPlaylistSize() const { return playlist_.size(); }

    // Callbacks
    void OnTrackChanged(std::function<void(const std::string&)> callback) { on_track_changed_ = callback; }
    void OnPlaybackFinished(std::function<void()> callback) { on_playback_finished_ = callback; }
    void OnError(std::function<void(const std::string&)> callback) { on_error_ = callback; }

private:
    AudioCodec* codec_;
    Mp3PlayerState state_ = Mp3PlayerState::STOPPED;
    
    std::vector<std::string> playlist_;
    std::string current_track_;
    int current_index_ = -1;
    
    TaskHandle_t player_task_ = nullptr;
    SemaphoreHandle_t mutex_ = nullptr;
    volatile bool stop_requested_ = false;
    volatile bool pause_requested_ = false;
    volatile bool playback_error_ = false;
    int consecutive_failures_ = 0;

    // Resources owned by decode task — stored as members so Stop() can clean up
    // if the task is force-killed
    FILE* current_file_ = nullptr;
    void* mp3_decoder_ = nullptr;   // mp3dec_t*
    void* pcm_buffer_ = nullptr;    // int16_t*
    void* input_buffer_ = nullptr;  // uint8_t*
    
    std::function<void(const std::string&)> on_track_changed_;
    std::function<void()> on_playback_finished_;
    std::function<void(const std::string&)> on_error_;

    static void PlayerTask(void* param);
    void DecodeAndPlay(const std::string& filepath);
    void ScanDirectoryRecursive(const std::string& path, bool recursive);
    void CleanupDecodeResources();
};

#endif // _MP3_PLAYER_H_
