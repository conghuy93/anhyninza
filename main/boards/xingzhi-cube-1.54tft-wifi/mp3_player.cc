// minimp3 - single-header MP3 decoder
// Put minimp3.h in this folder or include from component
// Download from: https://github.com/lieff/minimp3

#define MINIMP3_IMPLEMENTATION
#define MINIMP3_ONLY_MP3
#define MINIMP3_NO_SIMD
#include "minimp3.h"

#include "mp3_player.h"
#include "../../audio/audio_codec.h"

#include <esp_log.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <dirent.h>
#include <sys/stat.h>
#include <cstring>
#include <algorithm>
#include <cmath>

extern "C" {
    #include "robot_control.h"
}

static const char* TAG = "Mp3Player";

// Maximum consecutive playback failures before stopping
static const int MAX_CONSECUTIVE_FAILURES = 3;

Mp3Player::Mp3Player(AudioCodec* codec) : codec_(codec) {
    mutex_ = xSemaphoreCreateMutex();
}

Mp3Player::~Mp3Player() {
    Stop();
    if (mutex_) {
        vSemaphoreDelete(mutex_);
    }
}

void Mp3Player::CleanupDecodeResources() {
    if (current_file_) {
        fclose(current_file_);
        current_file_ = nullptr;
    }
    if (mp3_decoder_) {
        free(mp3_decoder_);
        mp3_decoder_ = nullptr;
    }
    if (pcm_buffer_) {
        free(pcm_buffer_);
        pcm_buffer_ = nullptr;
    }
    if (input_buffer_) {
        free(input_buffer_);
        input_buffer_ = nullptr;
    }
    // Close resampler
    if (resampler_) {
        esp_ae_rate_cvt_close(resampler_);
        resampler_ = nullptr;
    }
    resampler_src_rate_ = 0;
}

bool Mp3Player::Play(const std::string& filepath) {
    Stop();
    
    xSemaphoreTake(mutex_, portMAX_DELAY);
    current_track_ = filepath;
    state_ = Mp3PlayerState::PLAYING;
    stop_requested_ = false;
    pause_requested_ = false;
    playback_error_ = false;
    xSemaphoreGive(mutex_);
    
    if (on_track_changed_) {
        on_track_changed_(filepath);
    }
    
    BaseType_t ret = xTaskCreatePinnedToCore(PlayerTask, "mp3_player", 32768, this, 3, &player_task_, 1);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create player task");
        state_ = Mp3PlayerState::STOPPED;
        playback_error_ = true;
        return false;
    }
    
    return true;
}

void Mp3Player::Pause() {
    if (state_ == Mp3PlayerState::PLAYING) {
        pause_requested_ = true;
        state_ = Mp3PlayerState::PAUSED;
    }
}

void Mp3Player::Resume() {
    if (state_ == Mp3PlayerState::PAUSED) {
        pause_requested_ = false;
        state_ = Mp3PlayerState::PLAYING;
    }
}

void Mp3Player::Stop() {
    if (player_task_) {
        stop_requested_ = true;
        pause_requested_ = false;
        
        // Wait for task to finish (up to 3 seconds)
        int timeout = 300;
        while (player_task_ && timeout-- > 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        
        if (player_task_) {
            ESP_LOGW(TAG, "Force stopping player task (timed out)");
            vTaskDelete(player_task_);
            player_task_ = nullptr;
            // Clean up resources that the killed task left behind
            CleanupDecodeResources();
        }
    }
    
    state_ = Mp3PlayerState::STOPPED;
    stop_requested_ = false;
}

void Mp3Player::Next() {
    if (playlist_.empty()) return;
    
    xSemaphoreTake(mutex_, portMAX_DELAY);
    current_index_++;
    if (current_index_ >= (int)playlist_.size()) {
        current_index_ = 0;
    }
    std::string next_track = playlist_[current_index_];
    xSemaphoreGive(mutex_);
    
    Play(next_track);
}

void Mp3Player::Previous() {
    if (playlist_.empty()) return;
    
    xSemaphoreTake(mutex_, portMAX_DELAY);
    current_index_--;
    if (current_index_ < 0) {
        current_index_ = playlist_.size() - 1;
    }
    std::string prev_track = playlist_[current_index_];
    xSemaphoreGive(mutex_);
    
    Play(prev_track);
}

void Mp3Player::SetPlaylist(const std::vector<std::string>& files) {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    playlist_ = files;
    current_index_ = -1;
    xSemaphoreGive(mutex_);
}

void Mp3Player::ClearPlaylist() {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    playlist_.clear();
    current_index_ = -1;
    xSemaphoreGive(mutex_);
}

void Mp3Player::AddToPlaylist(const std::string& filepath) {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    playlist_.push_back(filepath);
    xSemaphoreGive(mutex_);
}

int Mp3Player::ScanDirectory(const std::string& path, bool recursive) {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    playlist_.clear();
    current_index_ = -1;
    xSemaphoreGive(mutex_);
    
    // Check if path exists
    struct stat st;
    if (stat(path.c_str(), &st) != 0) {
        ESP_LOGE(TAG, "Path does not exist: %s (SD card not mounted?)", path.c_str());
        return 0;
    }
    
    ESP_LOGI(TAG, "Scanning for MP3 files in: %s", path.c_str());
    ScanDirectoryRecursive(path, recursive);
    
    // Sort playlist alphabetically
    xSemaphoreTake(mutex_, portMAX_DELAY);
    std::sort(playlist_.begin(), playlist_.end());
    int count = playlist_.size();
    xSemaphoreGive(mutex_);
    
    ESP_LOGI(TAG, "Found %d MP3 files in %s", count, path.c_str());
    return count;
}

void Mp3Player::ScanDirectoryRecursive(const std::string& path, bool recursive) {
    ESP_LOGI(TAG, "Scanning directory: %s", path.c_str());
    DIR* dir = opendir(path.c_str());
    if (!dir) {
        ESP_LOGW(TAG, "Failed to open directory: %s", path.c_str());
        return;
    }
    
    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        if (entry->d_name[0] == '.') continue;
        
        std::string full_path = path + "/" + entry->d_name;
        
        struct stat st;
        if (stat(full_path.c_str(), &st) != 0) {
            ESP_LOGW(TAG, "Failed to stat: %s", full_path.c_str());
            continue;
        }
        
        if (S_ISDIR(st.st_mode)) {
            if (recursive) {
                ScanDirectoryRecursive(full_path, recursive);
            }
        } else {
            // Check if it's an MP3 file
            std::string name = entry->d_name;
            size_t len = name.length();
            if (len > 4) {
                std::string ext = name.substr(len - 4);
                for (auto& c : ext) c = tolower(c);
                if (ext == ".mp3") {
                    ESP_LOGI(TAG, "Found MP3: %s (size: %ld bytes)", full_path.c_str(), (long)st.st_size);
                    xSemaphoreTake(mutex_, portMAX_DELAY);
                    playlist_.push_back(full_path);
                    xSemaphoreGive(mutex_);
                }
            }
        }
    }
    
    closedir(dir);
}

void Mp3Player::PlayerTask(void* param) {
    Mp3Player* player = static_cast<Mp3Player*>(param);
    player->DecodeAndPlay(player->current_track_);
    
    player->player_task_ = nullptr;
    player->state_ = Mp3PlayerState::STOPPED;
    
    // Only auto-advance if playback finished normally (not stopped, not error)
    if (!player->stop_requested_ && !player->playback_error_) {
        player->consecutive_failures_ = 0;  // Reset on success
        if (player->on_playback_finished_) {
            player->on_playback_finished_();
        }
    } else if (player->playback_error_) {
        player->consecutive_failures_++;
        ESP_LOGW(TAG, "Playback error (consecutive failures: %d/%d)", 
                 player->consecutive_failures_, MAX_CONSECUTIVE_FAILURES);
        
        if (player->consecutive_failures_ < MAX_CONSECUTIVE_FAILURES) {
            // Try next track after a delay
            vTaskDelay(pdMS_TO_TICKS(1000));
            if (!player->stop_requested_ && player->on_playback_finished_) {
                player->on_playback_finished_();
            }
        } else {
            ESP_LOGE(TAG, "Too many consecutive failures, stopping playback");
            if (player->on_error_) {
                player->on_error_("Too many failures");
            }
        }
    }
    
    vTaskDelete(nullptr);
}

void Mp3Player::DecodeAndPlay(const std::string& filepath) {
    ESP_LOGI(TAG, "Playing: %s", filepath.c_str());
    
    current_file_ = fopen(filepath.c_str(), "rb");
    if (!current_file_) {
        ESP_LOGE(TAG, "Failed to open file: %s", filepath.c_str());
        playback_error_ = true;
        if (on_error_) on_error_("Failed to open file");
        return;
    }
    
    // Get file size for logging
    fseek(current_file_, 0, SEEK_END);
    size_t file_size = ftell(current_file_);
    fseek(current_file_, 0, SEEK_SET);
    ESP_LOGI(TAG, "File size: %u bytes", file_size);
    
    // Allocate input buffer
    static const size_t BUFFER_SIZE = 8 * 1024;
    input_buffer_ = heap_caps_malloc(BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!input_buffer_) {
        input_buffer_ = malloc(BUFFER_SIZE);
    }
    if (!input_buffer_) {
        ESP_LOGE(TAG, "Failed to allocate input buffer");
        playback_error_ = true;
        CleanupDecodeResources();
        if (on_error_) on_error_("Memory allocation failed");
        return;
    }
    
    // Allocate decoder on heap (mp3dec_t is ~10KB)
    mp3_decoder_ = heap_caps_malloc(sizeof(mp3dec_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!mp3_decoder_) {
        mp3_decoder_ = malloc(sizeof(mp3dec_t));
    }
    if (!mp3_decoder_) {
        ESP_LOGE(TAG, "Failed to allocate mp3 decoder");
        playback_error_ = true;
        CleanupDecodeResources();
        if (on_error_) on_error_("Decoder alloc failed");
        return;
    }
    mp3dec_init((mp3dec_t*)mp3_decoder_);
    
    // PCM buffer on heap (~4.5KB)
    pcm_buffer_ = heap_caps_malloc(MINIMP3_MAX_SAMPLES_PER_FRAME * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!pcm_buffer_) {
        pcm_buffer_ = malloc(MINIMP3_MAX_SAMPLES_PER_FRAME * sizeof(int16_t));
    }
    if (!pcm_buffer_) {
        ESP_LOGE(TAG, "Failed to allocate PCM buffer");
        playback_error_ = true;
        CleanupDecodeResources();
        if (on_error_) on_error_("PCM alloc failed");
        return;
    }
    
    mp3dec_t* mp3d = (mp3dec_t*)mp3_decoder_;
    int16_t* pcm = (int16_t*)pcm_buffer_;
    uint8_t* input_buf = (uint8_t*)input_buffer_;
    mp3dec_frame_info_t info;

    size_t bytes_left = 0;
    size_t input_pos = 0;
    size_t total_bytes_read = 0;
    int total_output_samples = 0;
    
    std::vector<int16_t> output_buffer;
    output_buffer.reserve(2048);
    
    int target_sample_rate = codec_->output_sample_rate();
    bool first_frame = true;
    int64_t start_time = esp_timer_get_time();
    
    // Ensure audio output is enabled
    codec_->EnableOutput(true);
    
    while (!stop_requested_) {
        // Handle pause
        while (pause_requested_ && !stop_requested_) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        
        if (stop_requested_) break;
        
        // Read more data if needed
        if (bytes_left < MINIMP3_MAX_SAMPLES_PER_FRAME * 2) {
            if (bytes_left > 0 && input_pos > 0) {
                memmove(input_buf, input_buf + input_pos, bytes_left);
            }
            input_pos = 0;
            
            size_t to_read = BUFFER_SIZE - bytes_left;
            size_t read = fread(input_buf + bytes_left, 1, to_read, current_file_);
            bytes_left += read;
            total_bytes_read += read;
            
            if (bytes_left == 0) {
                ESP_LOGI(TAG, "End of file reached (total read: %u bytes)", total_bytes_read);
                break;
            }
        }
        
        // Decode frame
        int samples = mp3dec_decode_frame(mp3d, input_buf + input_pos, bytes_left, pcm, &info);
        
        if (info.frame_bytes > 0) {
            input_pos += info.frame_bytes;
            bytes_left -= info.frame_bytes;
        }
        
        if (samples > 0) {
            if (first_frame) {
                ESP_LOGI(TAG, "First frame decoded: %d Hz, %d channels, %d samples/frame, bitrate: %d kbps",
                         info.hz, info.channels, samples, info.bitrate_kbps);
                first_frame = false;
            }
            
            // Compute audio energy for music-reactive LED
            {
                int total_samples = samples * info.channels;
                int64_t sum_sq = 0;
                for (int i = 0; i < total_samples; i++) {
                    int32_t s = pcm[i];
                    sum_sq += s * s;
                }
                float rms = sqrtf((float)sum_sq / total_samples);
                // Normalize: 32768 is max int16, map RMS to 0.0-1.0
                // Typical music RMS ~3000-8000, so scale with headroom
                float energy = rms / 12000.0f;
                if (energy > 1.0f) energy = 1.0f;
                ninja_led_set_audio_energy(energy);
            }
            
            // === Stereo to Mono conversion (if stereo) ===
            std::vector<int16_t> mono_pcm;
            int16_t* mono_data = pcm;
            int mono_samples = samples;
            
            if (info.channels == 2) {
                mono_pcm.resize(samples);
                for (int i = 0; i < samples; i++) {
                    // Use int32_t to avoid overflow/clipping
                    int32_t left = pcm[i * 2];
                    int32_t right = pcm[i * 2 + 1];
                    mono_pcm[i] = (int16_t)((left + right) / 2);
                }
                mono_data = mono_pcm.data();
            }
            
            // === Resample using ESP optimized rate converter ===
            if (info.hz != target_sample_rate && info.hz > 0) {
                // Create/recreate resampler if source rate changed
                if (resampler_src_rate_ != info.hz || resampler_ == nullptr) {
                    if (resampler_) {
                        esp_ae_rate_cvt_close(resampler_);
                        resampler_ = nullptr;
                    }
                    esp_ae_rate_cvt_cfg_t cfg = {
                        .src_rate = (uint32_t)info.hz,
                        .dest_rate = (uint32_t)target_sample_rate,
                        .channel = 1, // ESP_AUDIO_MONO
                        .bits_per_sample = ESP_AE_BIT16,
                        .complexity = 2,
                        .perf_type = ESP_AE_RATE_CVT_PERF_TYPE_SPEED,
                    };
                    esp_err_t ret = esp_ae_rate_cvt_open(&cfg, &resampler_);
                    if (ret != ESP_OK || !resampler_) {
                        ESP_LOGE(TAG, "Failed to create resampler: %d", ret);
                        // Fallback: direct copy
                        for (int i = 0; i < mono_samples; i++) {
                            output_buffer.push_back(mono_data[i]);
                        }
                        total_output_samples += mono_samples;
                    } else {
                        resampler_src_rate_ = info.hz;
                        ESP_LOGI(TAG, "Created resampler: %d -> %d Hz", info.hz, target_sample_rate);
                    }
                }
                
                if (resampler_) {
                    // Get max output samples
                    uint32_t max_out_samples = 0;
                    esp_ae_rate_cvt_get_max_out_sample_num(resampler_, mono_samples, &max_out_samples);
                    
                    std::vector<int16_t> resampled(max_out_samples);
                    uint32_t actual_output = max_out_samples;
                    esp_ae_rate_cvt_process(resampler_, (esp_ae_sample_t)mono_data, mono_samples,
                                            (esp_ae_sample_t)resampled.data(), &actual_output);
                    
                    // Append to output buffer
                    for (uint32_t i = 0; i < actual_output; i++) {
                        output_buffer.push_back(resampled[i]);
                    }
                    total_output_samples += actual_output;
                }
            } else {
                // No resample needed - direct copy
                for (int i = 0; i < mono_samples; i++) {
                    output_buffer.push_back(mono_data[i]);
                }
                total_output_samples += mono_samples;
            }
            
            // Output when buffer is full enough
            // The I2S write is blocking (portMAX_DELAY), providing natural pacing
            if (output_buffer.size() >= 1024) {
                codec_->OutputData(output_buffer);
                output_buffer.clear();
            }
        } else if (info.frame_bytes == 0) {
            if (bytes_left == 0) break;
            // Skip invalid byte
            input_pos++;
            bytes_left--;
        }
        
        // Yield to other tasks (I2S write already provides pacing)
        taskYIELD();
    }
    
    // Output remaining samples
    if (!output_buffer.empty() && !stop_requested_) {
        codec_->OutputData(output_buffer);
    }
    
    // Reset LED audio energy when playback ends
    ninja_led_set_audio_energy(0.0f);
    
    int64_t elapsed_ms = (esp_timer_get_time() - start_time) / 1000;
    float expected_duration = (float)total_output_samples / target_sample_rate;
    ESP_LOGI(TAG, "Playback finished: %s (elapsed: %lld ms, output: %.1f sec of audio, %d samples at %d Hz)",
             filepath.c_str(), elapsed_ms, expected_duration, total_output_samples, target_sample_rate);
    
    CleanupDecodeResources();
}
