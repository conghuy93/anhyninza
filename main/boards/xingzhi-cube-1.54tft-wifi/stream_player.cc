/*
 * Stream Music Player Implementation
 * HTTP streaming from huy.minizjp.com music server
 * Uses minimp3 for decoding, AudioCodec for I2S output
 */

#include "minimp3.h"
#include "stream_player.h"
#include "../../audio/audio_codec.h"

#include <esp_log.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <esp_http_client.h>
#include <esp_tls_errors.h>
#include <cJSON.h>
#include <esp_crt_bundle.h>
#include <cstring>
#include <algorithm>
#include <sstream>
#include <cmath>
#include <esp_ae_rate_cvt.h>

extern "C" {
    #include "robot_control.h"
}

#include "application.h"
#include "board.h"
#include <nvs_flash.h>
#include <nvs.h>

static const char* TAG = "StreamPlayer";

#define DEFAULT_MUSIC_SERVER "http://www.xiaozhishop.xyz:5005"

// ==================== URL Encoding/Decoding ====================
static int hex_to_int(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

static std::string url_decode(const std::string& str) {
    std::string decoded;
    for (size_t i = 0; i < str.length(); i++) {
        if (str[i] == '%' && i + 2 < str.length()) {
            int h1 = hex_to_int(str[i + 1]);
            int h2 = hex_to_int(str[i + 2]);
            if (h1 >= 0 && h2 >= 0) {
                decoded += (char)(h1 * 16 + h2);
                i += 2;
            } else {
                decoded += str[i];
            }
        } else if (str[i] == '+') {
            decoded += ' ';
        } else {
            decoded += str[i];
        }
    }
    return decoded;
}

static std::string url_encode(const std::string& str) {
    std::string encoded;
    char hex[4];
    for (size_t i = 0; i < str.length(); i++) {
        unsigned char c = str[i];
        if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
            (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.' || c == '~') {
            encoded += c;
        } else if (c == ' ') {
            encoded += '+';
        } else {
            snprintf(hex, sizeof(hex), "%%%02X", c);
            encoded += hex;
        }
    }
    return encoded;
}

// ==================== HTTP Helper ====================
// Simple HTTP GET that returns body as string
static std::string http_get(const std::string& url, int timeout_ms = 10000) {
    std::string result;
    
    esp_http_client_config_t config = {};
    config.url = url.c_str();
    config.timeout_ms = timeout_ms;
    config.buffer_size = 4096;
    config.buffer_size_tx = 1024;
    config.skip_cert_common_name_check = true;
    config.crt_bundle_attach = esp_crt_bundle_attach;
    
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "Failed to init HTTP client");
        return result;
    }
    
    esp_http_client_set_header(client, "User-Agent", "ESP32-Music-Player/1.0");
    esp_http_client_set_header(client, "Accept", "*/*");
    
    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "HTTP open failed: %s (0x%x)", esp_err_to_name(err), err);
        if (err == ESP_ERR_ESP_TLS_TCP_CLOSED_FIN) {
            ESP_LOGE(TAG, "TLS connection closed by server");
        } else if (err == ESP_ERR_ESP_TLS_FAILED_CONNECT_TO_HOST) {
            ESP_LOGE(TAG, "Failed to connect to HTTPS host");
        } else if (err == ESP_ERR_ESP_TLS_CONNECTION_TIMEOUT) {
            ESP_LOGE(TAG, "HTTPS connection timeout");
        }
        esp_http_client_cleanup(client);
        return result;
    }
    
    int content_length = esp_http_client_fetch_headers(client);
    int status = esp_http_client_get_status_code(client);
    
    if (status != 200) {
        ESP_LOGE(TAG, "HTTP GET %s => status %d", url.c_str(), status);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return result;
    }
    
    // Read response body
    char buf[1024];
    int total = 0;
    while (true) {
        int read_len = esp_http_client_read(client, buf, sizeof(buf) - 1);
        if (read_len <= 0) break;
        buf[read_len] = '\0';
        result.append(buf, read_len);
        total += read_len;
        if (content_length > 0 && total >= content_length) break;
    }
    
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    return result;
}

// HTTP GET for binary data (for thumbnails/images)
// Returns false if not valid baseline JPEG
static bool http_get_binary(const std::string& url, uint8_t** data, size_t* size, int timeout_ms = 10000) {
    *data = nullptr;
    *size = 0;
    
    esp_http_client_config_t config = {};
    config.url = url.c_str();
    config.timeout_ms = timeout_ms;
    config.buffer_size = 4096;
    config.buffer_size_tx = 1024;
    config.skip_cert_common_name_check = true;
    config.crt_bundle_attach = esp_crt_bundle_attach;
    
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "Failed to init HTTP client for binary");
        return false;
    }
    
    esp_http_client_set_header(client, "User-Agent", "ESP32-Music-Player/1.0");
    esp_http_client_set_header(client, "Accept", "image/jpeg");  // Prefer JPEG
    
    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "HTTP open failed for binary: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return false;
    }
    
    int content_length = esp_http_client_fetch_headers(client);
    int status = esp_http_client_get_status_code(client);
    
    // Log Content-Type for debugging
    char* content_type = nullptr;
    if (esp_http_client_get_header(client, "Content-Type", &content_type) == ESP_OK && content_type) {
        ESP_LOGI(TAG, "Thumbnail Content-Type: %s", content_type);
        // Reject non-JPEG content types
        if (strstr(content_type, "webp") != nullptr) {
            ESP_LOGW(TAG, "WebP images not supported, skipping thumbnail");
            esp_http_client_close(client);
            esp_http_client_cleanup(client);
            return false;
        }
    }
    
    if (status != 200) {
        ESP_LOGE(TAG, "HTTP GET binary %s => status %d", url.c_str(), status);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return false;
    }
    
    // Limit size: thumbnails shouldn't be too large (max 500KB)
    const size_t MAX_THUMB_SIZE = 500 * 1024;
    size_t alloc_size = (content_length > 0 && content_length < (int)MAX_THUMB_SIZE) 
                      ? content_length : MAX_THUMB_SIZE;
    
    uint8_t* buf = (uint8_t*)heap_caps_malloc(alloc_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf) {
        buf = (uint8_t*)heap_caps_malloc(alloc_size, MALLOC_CAP_8BIT);
    }
    if (!buf) {
        ESP_LOGE(TAG, "Failed to allocate %d bytes for thumbnail", (int)alloc_size);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return false;
    }
    
    // Read response body
    size_t total = 0;
    while (total < alloc_size) {
        int read_len = esp_http_client_read(client, (char*)(buf + total), alloc_size - total);
        if (read_len <= 0) break;
        total += read_len;
        if (content_length > 0 && (int)total >= content_length) break;
    }
    
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    
    if (total < 10) {
        ESP_LOGW(TAG, "Thumbnail too small: %d bytes", (int)total);
        heap_caps_free(buf);
        return false;
    }
    
    // Validate JPEG magic bytes: SOI marker (0xFF 0xD8) and APP0/APP1 (0xFF 0xE0/E1)
    if (buf[0] != 0xFF || buf[1] != 0xD8) {
        ESP_LOGW(TAG, "Not a valid JPEG: magic=%02X %02X (expected FF D8)", buf[0], buf[1]);
        // Check if it's WebP (RIFF....WEBP)
        if (total >= 12 && memcmp(buf, "RIFF", 4) == 0 && memcmp(buf + 8, "WEBP", 4) == 0) {
            ESP_LOGW(TAG, "Detected WebP image, not supported");
        }
        heap_caps_free(buf);
        return false;
    }
    
    // Check for progressive JPEG (SOF2 marker = 0xFF 0xC2)
    // Scan first 512 bytes for SOF markers
    bool is_progressive = false;
    for (size_t i = 2; i < total - 1 && i < 512; i++) {
        if (buf[i] == 0xFF) {
            if (buf[i+1] == 0xC2) {  // SOF2 = Progressive DCT
                is_progressive = true;
                ESP_LOGW(TAG, "Progressive JPEG detected at offset %d", (int)i);
                break;
            } else if (buf[i+1] == 0xC0) {  // SOF0 = Baseline DCT (OK)
                break;
            }
        }
    }
    
    if (is_progressive) {
        ESP_LOGW(TAG, "Progressive JPEG not supported by hardware decoder");
        // Still return the data, software decoder might handle it
    }
    
    *data = buf;
    *size = total;
    ESP_LOGI(TAG, "Downloaded valid JPEG: %d bytes from %s", (int)total, url.c_str());
    return true;
}

// ==================== StreamPlayer Implementation ====================

StreamPlayer::StreamPlayer(AudioCodec* codec) : codec_(codec) {
    server_url_ = DEFAULT_MUSIC_SERVER;
    // Strip trailing slash from default
    while (!server_url_.empty() && server_url_.back() == '/') server_url_.pop_back();
    // Load server URL from NVS if saved
    nvs_handle_t nvs;
    if (nvs_open("music", NVS_READONLY, &nvs) == ESP_OK) {
        size_t len = 0;
        if (nvs_get_str(nvs, "server_url", NULL, &len) == ESP_OK && len > 0 && len < 256) {
            char* buf = (char*)malloc(len);
            if (buf && nvs_get_str(nvs, "server_url", buf, &len) == ESP_OK) {
                server_url_ = buf;
                // Strip trailing slash
                while (!server_url_.empty() && server_url_.back() == '/') server_url_.pop_back();
                ESP_LOGI(TAG, "Loaded server URL from NVS: %s", server_url_.c_str());
            }
            free(buf);
        }
        nvs_close(nvs);
    }
    mutex_ = xSemaphoreCreateMutex();
    http_mutex_ = xSemaphoreCreateMutex();  // For HTTP handle protection
    // Note: ring buffer uses lock-free SPSC design, no mutex needed
}

StreamPlayer::~StreamPlayer() {
    Stop();
    if (resampler_) {
        esp_ae_rate_cvt_close(resampler_);
        resampler_ = nullptr;
    }
    if (http_mutex_) vSemaphoreDelete(http_mutex_);
    if (mutex_) vSemaphoreDelete(mutex_);
}

bool StreamPlayer::SearchAndPlay(const std::string& song_name, const std::string& artist_name) {
    Stop();
    
    xSemaphoreTake(mutex_, portMAX_DELAY);
    state_ = StreamPlayerState::SEARCHING;
    song_info_ = {};
    song_info_.title = song_name;
    song_info_.artist = artist_name;
    song_info_.status_text = "Dang tim...";
    lyrics_.clear();
    current_lyric_index_ = -1;
    play_time_ms_ = 0;
    stop_requested_ = false;
    xSemaphoreGive(mutex_);
    
    ESP_LOGI(TAG, "Searching for: %s - %s", song_name.c_str(), artist_name.c_str());
    
    // Step 1: Fetch song info from server
    if (!FetchSongInfo(song_name, artist_name)) {
        state_ = StreamPlayerState::ERROR;
        song_info_.status_text = "Khong tim thay!";
        ESP_LOGE(TAG, "Song not found: %s", song_name.c_str());
        return false;
    }
    
    // Step 2: Fetch lyrics in background (optional)
    if (!lyric_url_.empty()) {
        FetchLyrics(lyric_url_);
    }
    
    // Step 3: Start streaming task
    state_ = StreamPlayerState::BUFFERING;
    song_info_.status_text = "Dang tai...";
    
    // Allocate ring buffer from PSRAM
    ring_buffer_ = (uint8_t*)heap_caps_malloc(RING_BUFFER_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!ring_buffer_) {
        ring_buffer_ = (uint8_t*)malloc(RING_BUFFER_SIZE);
    }
    if (!ring_buffer_) {
        ESP_LOGE(TAG, "Failed to allocate ring buffer");
        state_ = StreamPlayerState::ERROR;
        song_info_.status_text = "Loi bo nho!";
        return false;
    }
    // Reset atomic positions for lock-free SPSC
    ring_write_pos_.store(0, std::memory_order_release);
    ring_read_pos_.store(0, std::memory_order_release);
    download_done_ = false;
    download_error_ = false;
    total_downloaded_ = 0;
    
    // Reset task exit flags before starting new tasks
    download_task_exited_.store(false, std::memory_order_release);
    stream_task_exited_.store(false, std::memory_order_release);
    
    // Create download task (core 0, priority 4) — fills ring buffer from HTTP
    // Use INTERNAL SRAM stack — PSRAM stack on Core 0 causes WiFi MMU cache errors
    const size_t DL_STACK_SIZE = 8192;
    BaseType_t ret = xTaskCreatePinnedToCore(DownloadTask, "stream_dl", DL_STACK_SIZE,
                                             this, 4, &download_task_, 0);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create download task");
        heap_caps_free(ring_buffer_);
        ring_buffer_ = nullptr;
        state_ = StreamPlayerState::ERROR;
        return false;
    }
    
    // Create decode task (core 1, priority 6) — reads ring buffer, decodes MP3, outputs I2S
    // Use PSRAM for stack (Core 1, no WiFi conflict)
    const size_t DEC_STACK_SIZE = 32768;  // minimp3 + resampler needs ~20KB
    stream_task_stack_ = (StackType_t*)heap_caps_malloc(DEC_STACK_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    stream_task_tcb_ = (StaticTask_t*)heap_caps_malloc(sizeof(StaticTask_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!stream_task_stack_ || !stream_task_tcb_) {
        ESP_LOGE(TAG, "Failed to allocate decode task stack");
        stop_requested_ = true;
        vTaskDelay(pdMS_TO_TICKS(500));
        download_task_ = nullptr;
        if (stream_task_stack_) { heap_caps_free(stream_task_stack_); stream_task_stack_ = nullptr; }
        if (stream_task_tcb_) { heap_caps_free(stream_task_tcb_); stream_task_tcb_ = nullptr; }
        heap_caps_free(ring_buffer_);
        ring_buffer_ = nullptr;
        state_ = StreamPlayerState::ERROR;
        return false;
    }
    stream_task_ = xTaskCreateStaticPinnedToCore(StreamTask, "stream_dec", DEC_STACK_SIZE,
                                                 this, 6, stream_task_stack_, stream_task_tcb_, 1);
    
    return true;
}

void StreamPlayer::Stop() {
    if (!stream_task_ && !download_task_) return;
    
    stop_requested_ = true;
    
    // ⚡ Close HTTP connection IMMEDIATELY to abort download
    // This makes download task exit fast instead of waiting for timeout
    xSemaphoreTake(http_mutex_, portMAX_DELAY);
    if (active_http_) {
        ESP_LOGI(TAG, "Closing HTTP connection immediately");
        esp_http_client_close(active_http_);
        esp_http_client_cleanup(active_http_);
        active_http_ = nullptr;
    }
    xSemaphoreGive(http_mutex_);
    
    // Wait for download task to exit (using atomic flag, not handle)
    if (download_task_) {
        int timeout = 300;
        while (!download_task_exited_.load(std::memory_order_acquire) && timeout-- > 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        if (!download_task_exited_.load(std::memory_order_acquire)) {
            ESP_LOGW(TAG, "Force stopping download task");
            vTaskDelete(download_task_);
        }
        // Wait a bit for vTaskDelete to complete
        vTaskDelay(pdMS_TO_TICKS(20));
        download_task_ = nullptr;
    }
    
    // Wait for decode task to exit (using atomic flag, not handle)
    if (stream_task_) {
        int timeout = 300;
        while (!stream_task_exited_.load(std::memory_order_acquire) && timeout-- > 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        if (!stream_task_exited_.load(std::memory_order_acquire)) {
            ESP_LOGW(TAG, "Force stopping decode task");
            vTaskDelete(stream_task_);
        }
        // Wait a bit for vTaskDelete to complete
        vTaskDelay(pdMS_TO_TICKS(20));
        stream_task_ = nullptr;
    }
    
    // Free PSRAM task stacks (decode task only; download task uses internal SRAM)
    if (stream_task_stack_) { heap_caps_free(stream_task_stack_); stream_task_stack_ = nullptr; }
    if (stream_task_tcb_) { heap_caps_free(stream_task_tcb_); stream_task_tcb_ = nullptr; }
    
    if (ring_buffer_) {
        heap_caps_free(ring_buffer_);
        ring_buffer_ = nullptr;
    }
    
    // Close resampler
    if (resampler_) {
        esp_ae_rate_cvt_close(resampler_);
        resampler_ = nullptr;
    }
    resampler_src_rate_ = 0;
    
    state_ = StreamPlayerState::STOPPED;
    stop_requested_ = false;
    song_info_.status_text = "";
}

std::string StreamPlayer::GetCurrentLyricText() const {
    if (current_lyric_index_ >= 0 && current_lyric_index_ < (int)lyrics_.size()) {
        return lyrics_[current_lyric_index_].text;
    }
    return "";
}

bool StreamPlayer::DownloadThumbnail(const std::string& url, uint8_t** data, size_t* size) {
    if (url.empty()) {
        ESP_LOGW(TAG, "Empty thumbnail URL");
        return false;
    }
    return http_get_binary(url, data, size, 15000);
}

// ==================== Server API ====================

bool StreamPlayer::FetchSongInfo(const std::string& song_name, const std::string& artist_name) {
    // URL decode first in case it's already encoded from web UI
    std::string decoded_song = url_decode(song_name);
    std::string decoded_artist = url_decode(artist_name);
    
    std::string url = server_url_ + "/stream_pcm?song=" + url_encode(decoded_song);
    if (!decoded_artist.empty()) {
        url += "&artist=" + url_encode(decoded_artist);
    }
    
    ESP_LOGI(TAG, "Fetching song info: %s", url.c_str());
    
    std::string response = http_get(url, 15000);
    if (response.empty()) {
        ESP_LOGE(TAG, "Empty response from server");
        return false;
    }
    
    cJSON* root = cJSON_Parse(response.c_str());
    if (!root) {
        ESP_LOGE(TAG, "Failed to parse JSON response");
        return false;
    }
    
    cJSON* title = cJSON_GetObjectItem(root, "title");
    cJSON* artist = cJSON_GetObjectItem(root, "artist");
    cJSON* thumbnail = cJSON_GetObjectItem(root, "thumbnail");
    cJSON* audio_url_item = cJSON_GetObjectItem(root, "audio_url");
    cJSON* lyric_url_item = cJSON_GetObjectItem(root, "lyric_url");
    
    if (cJSON_IsString(title)) {
        song_info_.title = title->valuestring;
    }
    if (cJSON_IsString(artist)) {
        song_info_.artist = artist->valuestring;
    }
    if (cJSON_IsString(thumbnail) && thumbnail->valuestring && strlen(thumbnail->valuestring) > 0) {
        song_info_.thumbnail = thumbnail->valuestring;
        ESP_LOGI(TAG, "Thumbnail URL: %s", song_info_.thumbnail.c_str());
    } else {
        song_info_.thumbnail.clear();
    }
    
    if (!cJSON_IsString(audio_url_item) || !audio_url_item->valuestring || 
        strlen(audio_url_item->valuestring) == 0) {
        ESP_LOGE(TAG, "No audio_url in response");
        cJSON_Delete(root);
        return false;
    }
    
    // Build full audio URL
    std::string audio_path = audio_url_item->valuestring;
    if (audio_path.find("http") == 0) {
        audio_url_ = audio_path; // Already absolute
    } else {
        audio_url_ = server_url_ + audio_path;
    }
    
    // Lyric URL
    if (cJSON_IsString(lyric_url_item) && lyric_url_item->valuestring &&
        strlen(lyric_url_item->valuestring) > 0) {
        std::string lyric_path = lyric_url_item->valuestring;
        if (lyric_path.find("http") == 0) {
            lyric_url_ = lyric_path;
        } else {
            lyric_url_ = server_url_ + lyric_path;
        }
    }
    
    cJSON_Delete(root);
    ESP_LOGI(TAG, "Song: %s - %s, Audio: %s", 
             song_info_.title.c_str(), song_info_.artist.c_str(), audio_url_.c_str());
    return true;
}

bool StreamPlayer::FetchLyrics(const std::string& url) {
    ESP_LOGI(TAG, "Fetching lyrics: %s", url.c_str());
    
    std::string content = http_get(url, 10000);
    if (content.empty()) {
        ESP_LOGW(TAG, "No lyrics found");
        return false;
    }
    
    ParseLyrics(content);
    ESP_LOGI(TAG, "Parsed %d lyric lines", lyrics_.size());
    return !lyrics_.empty();
}

void StreamPlayer::ParseLyrics(const std::string& content) {
    lyrics_.clear();
    
    std::istringstream stream(content);
    std::string line;
    
    while (std::getline(stream, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty() || line[0] != '[') continue;
        
        size_t close = line.find(']');
        if (close == std::string::npos) continue;
        
        std::string tag = line.substr(1, close - 1);
        std::string text = line.substr(close + 1);
        
        // Check if it's a timestamp [mm:ss.xx] 
        size_t colon = tag.find(':');
        if (colon == std::string::npos) continue;
        
        std::string left = tag.substr(0, colon);
        // Is it a number (timestamp) or metadata tag?
        bool is_time = true;
        for (char c : left) {
            if (!isdigit(c)) { is_time = false; break; }
        }
        if (!is_time) continue;
        
        try {
            int minutes = std::stoi(left);
            float seconds = std::stof(tag.substr(colon + 1));
            int ms = minutes * 60 * 1000 + (int)(seconds * 1000);
            lyrics_.push_back({ms, text});
        } catch (...) {
            continue;
        }
    }
    
    // Sort by timestamp
    std::sort(lyrics_.begin(), lyrics_.end(), 
              [](const LyricLine& a, const LyricLine& b) { return a.timestamp_ms < b.timestamp_ms; });
}

void StreamPlayer::UpdateCurrentLyric() {
    if (lyrics_.empty()) return;
    
    int64_t t = play_time_ms_;
    int new_idx = -1;
    
    for (int i = 0; i < (int)lyrics_.size(); i++) {
        if (lyrics_[i].timestamp_ms <= t) {
            new_idx = i;
        } else {
            break;
        }
    }
    
    current_lyric_index_ = new_idx;
}

// ==================== Ring Buffer (Lock-free SPSC) ====================

size_t StreamPlayer::RingWrite(const uint8_t* data, size_t len) {
    // Lock-free: only producer (download task) writes to write_pos
    size_t w = ring_write_pos_.load(std::memory_order_relaxed);
    size_t r = ring_read_pos_.load(std::memory_order_acquire);
    
    // Calculate available space (leave 1 byte to distinguish full from empty)
    size_t available;
    if (w >= r) {
        available = RING_BUFFER_SIZE - (w - r) - 1;
    } else {
        available = (r - w) - 1;
    }
    
    size_t to_write = (len < available) ? len : available;
    if (to_write == 0) return 0;
    
    // Copy with wrap-around using memcpy for performance
    size_t first = RING_BUFFER_SIZE - w;
    if (first > to_write) first = to_write;
    memcpy(ring_buffer_ + w, data, first);
    if (to_write > first) {
        memcpy(ring_buffer_, data + first, to_write - first);
    }
    
    // Update write position atomically (use bitmask for fast modulo)
    ring_write_pos_.store((w + to_write) & RING_BUFFER_MASK, std::memory_order_release);
    return to_write;
}

size_t StreamPlayer::RingRead(uint8_t* data, size_t len) {
    // Lock-free: only consumer (decode task) writes to read_pos
    size_t r = ring_read_pos_.load(std::memory_order_relaxed);
    size_t w = ring_write_pos_.load(std::memory_order_acquire);
    
    // Calculate available data
    size_t available;
    if (w >= r) {
        available = w - r;
    } else {
        available = RING_BUFFER_SIZE - r + w;
    }
    
    size_t to_read = (len < available) ? len : available;
    if (to_read == 0) return 0;
    
    // Copy with wrap-around using memcpy for performance
    size_t first = RING_BUFFER_SIZE - r;
    if (first > to_read) first = to_read;
    memcpy(data, ring_buffer_ + r, first);
    if (to_read > first) {
        memcpy(data + first, ring_buffer_, to_read - first);
    }
    
    // Update read position atomically (use bitmask for fast modulo)
    ring_read_pos_.store((r + to_read) & RING_BUFFER_MASK, std::memory_order_release);
    return to_read;
}

// ==================== Download Task (HTTP → Ring Buffer) ====================

void StreamPlayer::DownloadTask(void* param) {
    StreamPlayer* player = static_cast<StreamPlayer*>(param);
    player->DownloadLoop();
    // Signal completion BEFORE vTaskDelete to avoid race with Stop()
    player->download_task_exited_.store(true, std::memory_order_release);
    vTaskDelete(nullptr);
}

void StreamPlayer::DownloadLoop() {
    ESP_LOGI(TAG, "Download task started: %s", audio_url_.c_str());
    
    esp_http_client_config_t config = {};
    config.url = audio_url_.c_str();
    config.timeout_ms = 15000;
    config.buffer_size = 4096;
    config.buffer_size_tx = 512;
    config.crt_bundle_attach = esp_crt_bundle_attach;
    
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "Failed to init HTTP client for audio");
        download_error_ = true;
        download_done_ = true;
        return;
    }
    
    // ⚡ Store HTTP handle for instant abort from Stop()
    xSemaphoreTake(http_mutex_, portMAX_DELAY);
    active_http_ = client;
    xSemaphoreGive(http_mutex_);
    
    esp_http_client_set_header(client, "User-Agent", "ESP32-Music-Player/1.0");
    esp_http_client_set_header(client, "Accept", "*/*");
    esp_http_client_set_header(client, "Connection", "keep-alive");
    
    esp_err_t open_err = esp_http_client_open(client, 0);
    if (open_err != ESP_OK) {
        ESP_LOGE(TAG, "HTTP open audio failed: %s", esp_err_to_name(open_err));
        xSemaphoreTake(http_mutex_, portMAX_DELAY);
        if (active_http_) {
            esp_http_client_cleanup(active_http_);
            active_http_ = nullptr;
        }
        xSemaphoreGive(http_mutex_);
        download_error_ = true;
        download_done_ = true;
        state_ = StreamPlayerState::ERROR;
        song_info_.status_text = "Loi tai nhac!";
        return;
    }
    
    esp_http_client_fetch_headers(client);
    int status = esp_http_client_get_status_code(client);
    if (status != 200 && status != 206) {
        ESP_LOGE(TAG, "Audio HTTP status: %d", status);
        xSemaphoreTake(http_mutex_, portMAX_DELAY);
        if (active_http_) {
            esp_http_client_close(active_http_);
            esp_http_client_cleanup(active_http_);
            active_http_ = nullptr;
        }
        xSemaphoreGive(http_mutex_);
        download_error_ = true;
        download_done_ = true;
        state_ = StreamPlayerState::ERROR;
        song_info_.status_text = "Loi HTTP!";
        return;
    }
    
    char tmp[4096];
    while (!stop_requested_) {
        // Wait if ring buffer is nearly full (use lock-free space check)
        if (RingSpace() < sizeof(tmp)) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        
        // Check if HTTP was closed by Stop()
        int read_len = 0;
        xSemaphoreTake(http_mutex_, portMAX_DELAY);
        if (!active_http_) {
            xSemaphoreGive(http_mutex_);
            break; // HTTP closed by Stop()
        }
        read_len = esp_http_client_read(active_http_, tmp, sizeof(tmp));
        xSemaphoreGive(http_mutex_);
        
        if (read_len > 0) {
            size_t written = 0;
            while (written < (size_t)read_len && !stop_requested_) {
                size_t w = RingWrite((uint8_t*)tmp + written, read_len - written);
                written += w;
                if (w == 0) vTaskDelay(pdMS_TO_TICKS(5));
            }
            total_downloaded_ += read_len;
        } else if (read_len == 0) {
            ESP_LOGI(TAG, "Download complete: %u bytes", (unsigned)total_downloaded_);
            break;
        } else {
            size_t buffered = RingAvailable();
            ESP_LOGE(TAG, "HTTP read error, buffered: %u bytes", (unsigned)buffered);
            if (buffered == 0) download_error_ = true;
            break;
        }
    }
    
    download_done_ = true;
    
    // Cleanup HTTP handle
    xSemaphoreTake(http_mutex_, portMAX_DELAY);
    if (active_http_) {
        esp_http_client_close(active_http_);
        esp_http_client_cleanup(active_http_);
        active_http_ = nullptr;
    }
    xSemaphoreGive(http_mutex_);
}

// ==================== Decode Task (Ring Buffer → MP3 → I2S) ====================

void StreamPlayer::StreamTask(void* param) {
    StreamPlayer* player = static_cast<StreamPlayer*>(param);
    player->StreamAndDecode();
    
    bool was_stop = player->stop_requested_;
    player->stop_requested_ = true; // Signal download task to stop too
    
    // Reset LED audio energy
    ninja_led_set_audio_energy(0.0f);
    
    if (!was_stop) {
        player->state_ = StreamPlayerState::STOPPED;
        player->song_info_.status_text = "Ket thuc";
    }
    // Signal completion BEFORE vTaskDelete to avoid race with Stop()
    player->stream_task_exited_.store(true, std::memory_order_release);
    vTaskDelete(nullptr);
}

void StreamPlayer::StreamAndDecode() {
    ESP_LOGI(TAG, "Decode task started, waiting for buffer...");
    
    // Wait for pre-buffer from download task
    while (!stop_requested_ && !download_error_) {
        size_t buffered = RingAvailable();
        if (buffered >= MIN_BUFFER_BEFORE_PLAY) break;
        // If download finished but has some data, proceed anyway
        if (download_done_ && buffered > 0) break;
        if (download_done_ && buffered == 0) {
            ESP_LOGE(TAG, "Download failed before buffering");
            state_ = StreamPlayerState::ERROR;
            song_info_.status_text = "Loi tai nhac!";
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    
    if (stop_requested_) return;
    if (download_error_ && RingAvailable() == 0) {
        state_ = StreamPlayerState::ERROR;
        song_info_.status_text = "Loi tai nhac!";
        return;
    }
    
    // ===== MP3 Decoder Setup =====
    mp3dec_t* mp3d = (mp3dec_t*)heap_caps_malloc(sizeof(mp3dec_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!mp3d) mp3d = (mp3dec_t*)malloc(sizeof(mp3dec_t));
    if (!mp3d) {
        ESP_LOGE(TAG, "Failed to alloc mp3 decoder");
        state_ = StreamPlayerState::ERROR;
        return;
    }
    mp3dec_init(mp3d);
    
    int16_t* pcm = (int16_t*)heap_caps_malloc(MINIMP3_MAX_SAMPLES_PER_FRAME * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!pcm) pcm = (int16_t*)malloc(MINIMP3_MAX_SAMPLES_PER_FRAME * sizeof(int16_t));
    if (!pcm) {
        free(mp3d);
        state_ = StreamPlayerState::ERROR;
        return;
    }
    
    static const size_t INPUT_BUF_SIZE = 8 * 1024;
    uint8_t* input_buf = (uint8_t*)heap_caps_malloc(INPUT_BUF_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!input_buf) input_buf = (uint8_t*)malloc(INPUT_BUF_SIZE);
    if (!input_buf) {
        free(pcm);
        free(mp3d);
        state_ = StreamPlayerState::ERROR;
        return;
    }
    
    // ===== Decode + Output Loop =====
    codec_->EnableOutput(true);
    int target_sr = codec_->output_sample_rate();
    
    std::vector<int16_t> output_buffer;
    output_buffer.reserve(8192); // Reserve 8K samples for smooth output
    
    size_t bytes_left = 0;
    size_t input_pos = 0;
    bool first_frame = true;
    mp3dec_frame_info_t info;
    int stall_count = 0;
    
    state_ = StreamPlayerState::PLAYING;
    song_info_.status_text = "Dang phat";
    ESP_LOGI(TAG, "Buffered %u bytes, starting decode. Target SR=%d",
             (unsigned)RingAvailable(), target_sr);
    
    while (!stop_requested_) {
        // === Fill input buffer from ring buffer ===
        if (bytes_left < INPUT_BUF_SIZE / 2) {
            if (bytes_left > 0 && input_pos > 0) {
                memmove(input_buf, input_buf + input_pos, bytes_left);
            }
            input_pos = 0;
            
            size_t space = INPUT_BUF_SIZE - bytes_left;
            size_t got = RingRead(input_buf + bytes_left, space);
            bytes_left += got;
            
            if (got == 0) {
                if (download_done_ && RingAvailable() == 0 && bytes_left == 0) {
                    break; // All data consumed
                }
                if (!download_done_ || RingAvailable() > 0) {
                    stall_count++;
                    if (stall_count > 20) {
                        song_info_.status_text = "Dang tai them...";
                    }
                    vTaskDelay(pdMS_TO_TICKS(10));
                    continue;
                }
            } else {
                stall_count = 0;
                if (state_ == StreamPlayerState::PLAYING) {
                    song_info_.status_text = "Dang phat";
                }
            }
        }
        
        if (bytes_left == 0) {
            if (download_done_) break;
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        
        // === Decode MP3 frame ===
        int samples = mp3dec_decode_frame(mp3d, input_buf + input_pos, bytes_left, pcm, &info);
        
        if (info.frame_bytes > 0) {
            input_pos += info.frame_bytes;
            bytes_left -= info.frame_bytes;
        }
        
        if (samples > 0) {
            if (first_frame) {
                ESP_LOGI(TAG, "First frame: %d Hz, %d ch, %d kbps", info.hz, info.channels, info.bitrate_kbps);
                song_info_.bitrate_kbps = info.bitrate_kbps;
                song_info_.sample_rate = info.hz;
                song_info_.channels = info.channels;
                song_info_.status_text = "Dang phat";
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
                    mono_pcm[i] = (int16_t)((pcm[i * 2] + pcm[i * 2 + 1]) / 2);
                }
                mono_data = mono_pcm.data();
                mono_samples = samples;
            }
            
            // === Resample using ESP optimized rate converter ===
            if (info.hz != target_sr && info.hz > 0) {
                // Create/recreate resampler if source rate changed
                if (resampler_src_rate_ != info.hz || resampler_ == nullptr) {
                    if (resampler_) {
                        esp_ae_rate_cvt_close(resampler_);
                        resampler_ = nullptr;
                    }
                    esp_ae_rate_cvt_cfg_t cfg = {
                        .src_rate = (uint32_t)info.hz,
                        .dest_rate = (uint32_t)target_sr,
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
                    } else {
                        resampler_src_rate_ = info.hz;
                        ESP_LOGI(TAG, "Created resampler: %d -> %d Hz", info.hz, target_sr);
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
                }
            } else {
                // No resample needed - direct copy
                for (int i = 0; i < mono_samples; i++) {
                    output_buffer.push_back(mono_data[i]);
                }
            }
            
            // Update play time
            int frame_samples = (info.hz != target_sr && info.hz > 0) ? 
                                (int)(samples / ((float)info.hz / target_sr)) : samples;
            play_time_ms_ += (int64_t)frame_samples * 1000 / target_sr;
            
            // Update lyrics
            UpdateCurrentLyric();
            
            // Re-enable output if AudioService power timer disabled it
            if (!codec_->output_enabled()) {
                codec_->EnableOutput(true);
            }
            
            // Output to I2S when buffer is full enough
            // Threshold 70ms = target_sr * 7 / 100 (1680 samples @ 24kHz)
            // Balances latency vs overhead - copied from esp32_music
            int threshold_samples = target_sr * 7 / 100;
            if (threshold_samples < 500) threshold_samples = 500; // Minimum batch size
            if ((int)output_buffer.size() >= threshold_samples) {
                codec_->OutputData(output_buffer);
                output_buffer.clear();
                
                // Yield CPU to prevent watchdog timeout during intensive decode
                taskYIELD();
            }
        } else if (info.frame_bytes == 0) {
            if (bytes_left == 0 && download_done_ && RingAvailable() == 0) break;
            // Skip invalid byte
            if (bytes_left > 0) {
                input_pos++;
                bytes_left--;
            }
        }
        
        taskYIELD();
    }
    
    // Flush remaining audio
    if (!output_buffer.empty() && !stop_requested_) {
        if (!codec_->output_enabled()) codec_->EnableOutput(true);
        codec_->OutputData(output_buffer);
    }
    
    // Cleanup
    ESP_LOGI(TAG, "Stream finished. Downloaded: %u bytes, played: %u ms", 
             (unsigned)total_downloaded_, (unsigned)(play_time_ms_ & 0xFFFFFFFF));
    
    free(input_buf);
    free(pcm);
    free(mp3d);
}

// ==================== C Interface (Global Instance) ====================

static StreamPlayer* g_stream_player = nullptr;

extern "C" {

void StreamPlayer_Init(void* audio_codec_ptr) {
    if (g_stream_player) return;
    g_stream_player = new StreamPlayer(static_cast<AudioCodec*>(audio_codec_ptr));
    ESP_LOGI(TAG, "StreamPlayer initialized");
}

void StreamPlayer_Destroy(void) {
    if (g_stream_player) {
        delete g_stream_player;
        g_stream_player = nullptr;
    }
}

int StreamPlayer_SearchAndPlay(const char* song_name, const char* artist_name) {
    if (!g_stream_player) return 0;
    // Abort any ongoing TTS before streaming
    auto& app = Application::GetInstance();
    if (app.GetDeviceState() == kDeviceStateSpeaking) {
        app.AbortSpeaking(kAbortReasonNone);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    std::string sn = song_name ? song_name : "";
    std::string an = artist_name ? artist_name : "";
    return g_stream_player->SearchAndPlay(sn, an) ? 1 : 0;
}

void StreamPlayer_Stop(void) {
    if (g_stream_player) g_stream_player->Stop();
}

int StreamPlayer_GetState(void) {
    if (!g_stream_player) return 0;
    return (int)g_stream_player->GetState();
}

// Static buffers for C string returns (avoid malloc/free per call)
static char s_title_buf[128] = {0};
static char s_artist_buf[128] = {0};
static char s_thumbnail_buf[512] = {0};
static char s_status_buf[256] = {0};
static char s_lyric_buf[256] = {0};

const char* StreamPlayer_GetTitle(void) {
    if (!g_stream_player) return "";
    const auto& info = g_stream_player->GetSongInfo();
    strncpy(s_title_buf, info.title.c_str(), sizeof(s_title_buf) - 1);
    return s_title_buf;
}

const char* StreamPlayer_GetArtist(void) {
    if (!g_stream_player) return "";
    const auto& info = g_stream_player->GetSongInfo();
    strncpy(s_artist_buf, info.artist.c_str(), sizeof(s_artist_buf) - 1);
    return s_artist_buf;
}

const char* StreamPlayer_GetThumbnail(void) {
    if (!g_stream_player) return "";
    const auto& info = g_stream_player->GetSongInfo();
    strncpy(s_thumbnail_buf, info.thumbnail.c_str(), sizeof(s_thumbnail_buf) - 1);
    return s_thumbnail_buf;
}

int StreamPlayer_GetBitrate(void) {
    if (!g_stream_player) return 0;
    return g_stream_player->GetSongInfo().bitrate_kbps;
}

int StreamPlayer_GetSampleRate(void) {
    if (!g_stream_player) return 0;
    return g_stream_player->GetSongInfo().sample_rate;
}

const char* StreamPlayer_GetStatusText(void) {
    if (!g_stream_player) return "";
    const auto& info = g_stream_player->GetSongInfo();
    strncpy(s_status_buf, info.status_text.c_str(), sizeof(s_status_buf) - 1);
    return s_status_buf;
}

int StreamPlayer_GetLyricCount(void) {
    if (!g_stream_player) return 0;
    return g_stream_player->GetLyrics().size();
}

int StreamPlayer_GetCurrentLyricIndex(void) {
    if (!g_stream_player) return -1;
    return g_stream_player->GetCurrentLyricIndex();
}

const char* StreamPlayer_GetCurrentLyricText(void) {
    if (!g_stream_player) return "";
    std::string text = g_stream_player->GetCurrentLyricText();
    strncpy(s_lyric_buf, text.c_str(), sizeof(s_lyric_buf) - 1);
    return s_lyric_buf;
}

int64_t StreamPlayer_GetPlayTimeMs(void) {
    if (!g_stream_player) return 0;
    return g_stream_player->GetPlayTimeMs();
}

void StreamPlayer_SetServerUrl(const char* url) {
    if (g_stream_player && url) {
        std::string s(url);
        // Strip trailing slash to avoid double-slash in URLs
        while (!s.empty() && s.back() == '/') s.pop_back();
        g_stream_player->SetServerUrl(s);
        // Save to NVS for persistence (save cleaned URL without trailing slash)
        nvs_handle_t nvs;
        if (nvs_open("music", NVS_READWRITE, &nvs) == ESP_OK) {
            nvs_set_str(nvs, "server_url", s.c_str());
            nvs_commit(nvs);
            nvs_close(nvs);
            ESP_LOGI(TAG, "Server URL saved to NVS: %s", s.c_str());
        }
    }
}

static char s_server_url_buf[256] = {0};

const char* StreamPlayer_GetServerUrl(void) {
    if (!g_stream_player) return DEFAULT_MUSIC_SERVER;
    const auto& url = g_stream_player->GetServerUrl();
    strncpy(s_server_url_buf, url.c_str(), sizeof(s_server_url_buf) - 1);
    return s_server_url_buf;
}

int StreamPlayer_DownloadThumbnail(const char* url, uint8_t** data, size_t* size) {
    if (!g_stream_player || !url || !data || !size) return 0;
    return g_stream_player->DownloadThumbnail(url, data, size) ? 1 : 0;
}

} // extern "C"

// ============================================================================
// Radio Player C Interface
// ============================================================================

#include "../../application.h"
#include "../../features/music/esp32_radio.h"

extern "C" {

bool Radio_PlayStation(const char* station_name) {
    if (!station_name) return false;
    auto& app = Application::GetInstance();
    auto* radio = app.GetRadio();
    if (!radio) return false;
    return radio->PlayStation(station_name);
}

bool Radio_PlayUrl(const char* url, const char* name) {
    if (!url) return false;
    auto& app = Application::GetInstance();
    auto* radio = app.GetRadio();
    if (!radio) return false;
    std::string station_name = name ? name : "Custom Stream";
    return radio->PlayUrl(url, station_name);
}

bool Radio_Stop(void) {
    auto& app = Application::GetInstance();
    auto* radio = app.GetRadio();
    if (!radio) return false;
    return radio->Stop();
}

bool Radio_IsPlaying(void) {
    auto& app = Application::GetInstance();
    auto* radio = app.GetRadio();
    if (!radio) return false;
    return radio->IsPlaying();
}

static char s_current_station_buf[128] = {0};
const char* Radio_GetCurrentStation(void) {
    auto& app = Application::GetInstance();
    auto* radio = app.GetRadio();
    if (!radio) return "";
    const auto& name = radio->GetCurrentStation();
    strncpy(s_current_station_buf, name.c_str(), sizeof(s_current_station_buf) - 1);
    return s_current_station_buf;
}

static char s_station_list_json_buf[4096] = {0};
const char* Radio_GetStationListJson(void) {
    auto& app = Application::GetInstance();
    auto* radio = app.GetRadio();
    if (!radio) return "[]";
    
    const auto& stations = radio->GetStationList();
    std::string json = "[";
    bool first = true;
    for (const auto& station_name : stations) {
        if (!first) json += ",";
        json += "\"" + station_name + "\"";
        first = false;
    }
    json += "]";
    
    strncpy(s_station_list_json_buf, json.c_str(), sizeof(s_station_list_json_buf) - 1);
    return s_station_list_json_buf;
}

} // extern "C"
