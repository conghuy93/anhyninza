#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/lcd_display.h"
#include "display/lvgl_display/lvgl_theme.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "power_save_timer.h"
#include "led/single_led.h"
#include "assets/lang_config.h"
#include "power_manager.h"

#include <esp_log.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lvgl_port.h>
#include <esp_timer.h>
#include <esp_heap_caps.h>
#include <nvs_flash.h>
#include <atomic>
#include <vector>
#include <string>

#include "mcp_server.h"
#include <driver/rtc_io.h>
#include <esp_sleep.h>
#include <esp_vfs_fat.h>
#include <sdmmc_cmd.h>
#include <driver/sdspi_host.h>
#include "mp3_player.h"
#include "mp3_player_c.h"
#include "stream_player.h"
#include "radio_player_c.h"

// Robot control
#include "robot_control.h"
#include "webserver.h"
#include "robot_mcp_controller.h"
#include <esp_event.h>
#include <esp_wifi.h>
#include <esp_netif.h>
#include <dirent.h>
#include <sys/stat.h>

// Forward declaration for C function in robot_control.c
extern "C" void set_music_player_ptr(void* mp3_player_ptr);

#define TAG "XINGZHI_CUBE_1_54TFT_WIFI"

// Forward declaration
class XINGZHI_CUBE_1_54TFT_WIFI;

// ====== Global pointers for C interface (must be before class methods) ======
static XINGZHI_CUBE_1_54TFT_WIFI* g_board_instance = nullptr;
static Mp3Player* g_sd_player = nullptr;
static bool g_sd_mounted = false;
static std::vector<std::string>* g_playlist_cache = nullptr;

// ====== Sleep config & music power save (C-accessible from webserver.c) ======
static PowerSaveTimer* g_pst_ptr = nullptr;
static std::atomic<bool> g_music_power_save{false};
static std::atomic<bool> g_music_power_save_applied{false};

extern "C" void set_sleep_config(int sleep_sec, int shutdown_sec) {
    if (g_pst_ptr) {
        g_pst_ptr->SetSleepSeconds(sleep_sec);
        g_pst_ptr->SetShutdownSeconds(shutdown_sec);
        ESP_LOGI(TAG, "Sleep config updated: sleep=%ds, shutdown=%ds", sleep_sec, shutdown_sec);
    }
    // Save to NVS
    nvs_handle_t handle;
    if (nvs_open("sleep_cfg", NVS_READWRITE, &handle) == ESP_OK) {
        nvs_set_i32(handle, "sleep_s", sleep_sec);
        nvs_set_i32(handle, "shut_s", shutdown_sec);
        nvs_commit(handle);
        nvs_close(handle);
    }
}

extern "C" void get_sleep_config(int* sleep_sec, int* shutdown_sec) {
    if (g_pst_ptr) {
        *sleep_sec = g_pst_ptr->GetSleepSeconds();
        *shutdown_sec = g_pst_ptr->GetShutdownSeconds();
    } else {
        *sleep_sec = 60;
        *shutdown_sec = 300;
    }
}

extern "C" void set_music_power_save(int enable) {
    g_music_power_save.store(enable != 0);
    ESP_LOGI(TAG, "Music power save: %s", enable ? "ON" : "OFF");
}

extern "C" int get_music_power_save(void) {
    return g_music_power_save.load() ? 1 : 0;
}

class XINGZHI_CUBE_1_54TFT_WIFI : public WifiBoard {
private:
    Button boot_button_;
    Button volume_up_button_;
    Button volume_down_button_;
    SpiLcdDisplay* display_;
    PowerSaveTimer* power_save_timer_;
    PowerManager* power_manager_;
    esp_lcd_panel_io_handle_t panel_io_ = nullptr;
    esp_lcd_panel_handle_t panel_ = nullptr;
    Mp3Player* mp3_player_ = nullptr;
    bool music_mode_ = false;
    bool radio_mode_ = false;
    int radio_station_index_ = 0;
    bool sd_card_mounted_ = false;
    httpd_handle_t robot_webserver_ = nullptr;
    bool robot_initialized_ = false;

    // Music display overlay (LVGL)
    lv_obj_t* music_panel_ = nullptr;
    lv_obj_t* music_icon_label_ = nullptr;
    lv_obj_t* music_icon_bg_ = nullptr;  // Background circle for icon
    lv_obj_t* music_title_label_ = nullptr;
    lv_obj_t* music_artist_label_ = nullptr;
    lv_obj_t* music_state_label_ = nullptr;
    lv_obj_t* music_track_label_ = nullptr;
    lv_obj_t* music_progress_bar_ = nullptr;
    esp_timer_handle_t music_display_timer_ = nullptr;
    bool music_display_visible_ = false;
    
    // Spectrum bars for music visualization
    static constexpr int SPECTRUM_BAR_COUNT = 12;
    lv_obj_t* spectrum_bars_[SPECTRUM_BAR_COUNT] = {nullptr};
    int spectrum_heights_[SPECTRUM_BAR_COUNT] = {0};
    uint32_t spectrum_seed_ = 12345;  // For pseudo-random animation

    // Auto LED music mode tracking
    bool music_led_active_ = false;        // True when we switched LEDs for music
    led_state_t led_saved_before_music_;    // LED state before music auto-switch

    // Volume popup overlay (LVGL)
    lv_obj_t* volume_popup_ = nullptr;
    lv_obj_t* volume_bar_ = nullptr;
    lv_obj_t* volume_icon_label_ = nullptr;
    lv_obj_t* volume_value_label_ = nullptr;
    esp_timer_handle_t volume_popup_timer_ = nullptr;

    // Playlist selection mode
    bool playlist_selection_mode_ = false;
    int playlist_selected_index_ = 0;
    static constexpr int PLAYLIST_VISIBLE_ITEMS = 5;  // Number of items visible on screen
    lv_obj_t* playlist_panel_ = nullptr;
    lv_obj_t* playlist_title_label_ = nullptr;
    lv_obj_t* playlist_items_[7] = {nullptr};  // Max visible items + buffer
    esp_timer_handle_t playlist_timeout_timer_ = nullptr;

    // Radio station selection mode
    bool radio_station_selection_mode_ = false;
    int radio_station_selected_index_ = 0;
    static constexpr int RADIO_VISIBLE_ITEMS = 5;
    lv_obj_t* radio_station_panel_ = nullptr;
    lv_obj_t* radio_station_title_label_ = nullptr;
    lv_obj_t* radio_station_items_[7] = {nullptr};
    esp_timer_handle_t radio_station_timeout_timer_ = nullptr;
    std::vector<std::string> radio_stations_cache_;

    // Get text font from theme (supports Vietnamese/Chinese), fallback to montserrat
    const lv_font_t* GetTextFont() {
        if (display_ && display_->GetTheme()) {
            auto lvgl_theme = static_cast<LvglTheme*>(display_->GetTheme());
            if (lvgl_theme && lvgl_theme->text_font()) {
                return lvgl_theme->text_font()->font();
            }
        }
        return &lv_font_montserrat_14;  // Fallback for icons/ASCII only
    }
    
    // Simple pseudo-random for spectrum animation variation
    uint32_t NextRandom() {
        spectrum_seed_ = spectrum_seed_ * 1103515245 + 12345;
        return (spectrum_seed_ / 65536) % 32768;
    }
    
    // Update spectrum bars animation based on actual audio energy
    void UpdateSpectrumBars(bool playing) {
        if (!music_panel_) return;
        
        // Get actual audio energy (0.0 - 1.0)
        float energy = ninja_led_get_audio_energy();
        
        for (int i = 0; i < SPECTRUM_BAR_COUNT; i++) {
            if (!spectrum_bars_[i]) continue;
            
            int target_height;
            if (playing && energy > 0.01f) {
                // Use actual audio energy with some random variation for visual interest
                // Base height from energy (5-50 pixels based on energy level)
                int base_height = (int)(5 + energy * 45);
                // Add random variation (-10 to +10 pixels) for different bars
                int variation = (int)((NextRandom() % 21) - 10);
                // Apply position-based multiplier (center bars higher)
                float position_factor = 1.0f - 0.3f * abs(i - SPECTRUM_BAR_COUNT/2) / (float)(SPECTRUM_BAR_COUNT/2);
                target_height = (int)((base_height + variation) * position_factor);
                // Clamp to valid range
                if (target_height < 5) target_height = 5;
                if (target_height > 55) target_height = 55;
            } else if (playing) {
                // Playing but very low energy - show minimal movement
                target_height = 5 + (NextRandom() % 8);
            } else {
                // Paused/stopped - low idle heights
                target_height = 5 + (NextRandom() % 8);
            }
            
            // Smooth interpolation for fluid animation
            spectrum_heights_[i] = (spectrum_heights_[i] * 2 + target_height) / 3;
            
            lv_obj_set_height(spectrum_bars_[i], spectrum_heights_[i]);
            lv_obj_align(spectrum_bars_[i], LV_ALIGN_BOTTOM_MID, 
                        (i - SPECTRUM_BAR_COUNT/2) * 16 + 8, -10);
        }
    }

    // ===== Playlist Selection UI =====
    void CreatePlaylistSelection() {
        if (playlist_panel_) return;  // Already created
        
        lvgl_port_lock(0);
        auto screen = lv_screen_active();
        
        // Full-screen panel for playlist selection
        playlist_panel_ = lv_obj_create(screen);
        lv_obj_set_size(playlist_panel_, 240, 240);
        lv_obj_align(playlist_panel_, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_style_bg_color(playlist_panel_, lv_color_hex(0x1a1a2e), 0);
        lv_obj_set_style_bg_grad_color(playlist_panel_, lv_color_hex(0x16213e), 0);
        lv_obj_set_style_bg_grad_dir(playlist_panel_, LV_GRAD_DIR_VER, 0);
        lv_obj_set_style_border_width(playlist_panel_, 0, 0);
        lv_obj_set_style_radius(playlist_panel_, 0, 0);
        lv_obj_set_style_pad_all(playlist_panel_, 0, 0);
        lv_obj_add_flag(playlist_panel_, LV_OBJ_FLAG_HIDDEN);
        
        // Title label
        playlist_title_label_ = lv_label_create(playlist_panel_);
        lv_obj_set_style_text_font(playlist_title_label_, GetTextFont(), 0);
        lv_obj_set_style_text_color(playlist_title_label_, lv_color_hex(0x3498db), 0);
        lv_label_set_text(playlist_title_label_, LV_SYMBOL_LIST " Chon bai hat");
        lv_obj_align(playlist_title_label_, LV_ALIGN_TOP_MID, 0, 10);
        
        // Create playlist item labels (5 visible items)
        for (int i = 0; i < PLAYLIST_VISIBLE_ITEMS; i++) {
            playlist_items_[i] = lv_label_create(playlist_panel_);
            lv_obj_set_style_text_font(playlist_items_[i], GetTextFont(), 0);
            lv_obj_set_style_text_color(playlist_items_[i], lv_color_hex(0xecf0f1), 0);
            lv_obj_set_width(playlist_items_[i], 220);
            lv_obj_set_style_pad_left(playlist_items_[i], 10, 0);
            lv_obj_set_style_pad_ver(playlist_items_[i], 5, 0);
            lv_label_set_long_mode(playlist_items_[i], LV_LABEL_LONG_SCROLL_CIRCULAR);
            lv_label_set_text(playlist_items_[i], "");
            lv_obj_align(playlist_items_[i], LV_ALIGN_TOP_LEFT, 5, 40 + i * 38);
        }
        
        // Auto-hide timer (10 seconds)
        esp_timer_create_args_t timer_args = {};
        timer_args.callback = [](void* arg) {
            auto* self = static_cast<XINGZHI_CUBE_1_54TFT_WIFI*>(arg);
            self->HidePlaylistSelection();
        };
        timer_args.arg = this;
        timer_args.name = "playlist_timeout";
        esp_timer_create(&timer_args, &playlist_timeout_timer_);
        
        lvgl_port_unlock();
    }

    void ShowPlaylistSelection() {
        if (!playlist_panel_) {
            CreatePlaylistSelection();
        }
        if (!playlist_panel_ || !mp3_player_) return;
        
        playlist_selection_mode_ = true;
        // Start at current playing index
        playlist_selected_index_ = mp3_player_->GetCurrentIndex();
        if (playlist_selected_index_ < 0) playlist_selected_index_ = 0;
        
        UpdatePlaylistSelection();
        
        lvgl_port_lock(0);
        lv_obj_remove_flag(playlist_panel_, LV_OBJ_FLAG_HIDDEN);
        // Hide music panel while selecting
        if (music_panel_) {
            lv_obj_add_flag(music_panel_, LV_OBJ_FLAG_HIDDEN);
        }
        lvgl_port_unlock();
        
        // Reset timeout timer
        esp_timer_stop(playlist_timeout_timer_);
        esp_timer_start_once(playlist_timeout_timer_, 10000000);  // 10 seconds
    }

    void UpdatePlaylistSelection() {
        if (!playlist_panel_ || !mp3_player_) return;
        
        int playlist_size = mp3_player_->GetPlaylistSize();
        if (playlist_size == 0) return;
        
        // Calculate visible range (center selected item)
        int start_index = playlist_selected_index_ - PLAYLIST_VISIBLE_ITEMS / 2;
        if (start_index < 0) start_index = 0;
        if (start_index + PLAYLIST_VISIBLE_ITEMS > playlist_size) {
            start_index = playlist_size - PLAYLIST_VISIBLE_ITEMS;
            if (start_index < 0) start_index = 0;
        }
        
        lvgl_port_lock(0);
        
        // Update title with count
        char title[64];
        snprintf(title, sizeof(title), LV_SYMBOL_LIST " Chon bai (%d/%d)", 
                 playlist_selected_index_ + 1, playlist_size);
        lv_label_set_text(playlist_title_label_, title);
        
        // Update visible items
        for (int i = 0; i < PLAYLIST_VISIBLE_ITEMS; i++) {
            int item_index = start_index + i;
            
            if (item_index >= playlist_size) {
                lv_label_set_text(playlist_items_[i], "");
                lv_obj_set_style_bg_opa(playlist_items_[i], LV_OPA_0, 0);
                continue;
            }
            
            // Get track name (extract filename without path and extension)
            std::string track = mp3_player_->GetPlaylistEntry(item_index);
            size_t pos = track.find_last_of("/");
            std::string name = (pos != std::string::npos) ? track.substr(pos + 1) : track;
            size_t ext = name.find_last_of(".");
            if (ext != std::string::npos) name = name.substr(0, ext);
            
            // Format: index. name
            char item_text[128];
            snprintf(item_text, sizeof(item_text), "%d. %s", item_index + 1, name.c_str());
            lv_label_set_text(playlist_items_[i], item_text);
            
            // Highlight selected item
            if (item_index == playlist_selected_index_) {
                lv_obj_set_style_bg_color(playlist_items_[i], lv_color_hex(0x3498db), 0);
                lv_obj_set_style_bg_opa(playlist_items_[i], LV_OPA_50, 0);
                lv_obj_set_style_text_color(playlist_items_[i], lv_color_hex(0xffffff), 0);
                lv_obj_set_style_radius(playlist_items_[i], 5, 0);
            } else {
                lv_obj_set_style_bg_opa(playlist_items_[i], LV_OPA_0, 0);
                lv_obj_set_style_text_color(playlist_items_[i], lv_color_hex(0xbdc3c7), 0);
            }
        }
        
        lvgl_port_unlock();
        
        // Reset timeout
        esp_timer_stop(playlist_timeout_timer_);
        esp_timer_start_once(playlist_timeout_timer_, 10000000);
    }

    void HidePlaylistSelection() {
        playlist_selection_mode_ = false;
        
        if (playlist_panel_) {
            lvgl_port_lock(0);
            lv_obj_add_flag(playlist_panel_, LV_OBJ_FLAG_HIDDEN);
            // Show music panel again
            if (music_panel_ && music_mode_) {
                lv_obj_remove_flag(music_panel_, LV_OBJ_FLAG_HIDDEN);
            }
            lvgl_port_unlock();
        }
        
        esp_timer_stop(playlist_timeout_timer_);
    }

    void PlaylistMoveUp() {
        if (!playlist_selection_mode_ || !mp3_player_) return;
        
        playlist_selected_index_--;
        if (playlist_selected_index_ < 0) {
            playlist_selected_index_ = mp3_player_->GetPlaylistSize() - 1;
        }
        UpdatePlaylistSelection();
    }

    void PlaylistMoveDown() {
        if (!playlist_selection_mode_ || !mp3_player_) return;
        
        playlist_selected_index_++;
        if (playlist_selected_index_ >= mp3_player_->GetPlaylistSize()) {
            playlist_selected_index_ = 0;
        }
        UpdatePlaylistSelection();
    }

    void PlaylistSelectCurrent() {
        if (!playlist_selection_mode_ || !mp3_player_) return;
        
        int idx = playlist_selected_index_;
        HidePlaylistSelection();
        
        // Play the selected track
        mp3_player_->PlayAt(idx);
        GetDisplay()->ShowNotification("Dang phat...");
    }

    void CreateMusicDisplay() {
        if (music_panel_) return;  // Already created
        
        lvgl_port_lock(0);
        auto screen = lv_screen_active();
        
        // ===== Full-screen overlay with gradient background =====
        music_panel_ = lv_obj_create(screen);
        lv_obj_set_size(music_panel_, 240, 240);
        lv_obj_align(music_panel_, LV_ALIGN_CENTER, 0, 0);
        // Dark gradient background: deep navy to dark purple
        lv_obj_set_style_bg_color(music_panel_, lv_color_hex(0x0a0a1a), 0);
        lv_obj_set_style_bg_grad_color(music_panel_, lv_color_hex(0x1a0a2e), 0);
        lv_obj_set_style_bg_grad_dir(music_panel_, LV_GRAD_DIR_VER, 0);
        lv_obj_set_style_bg_opa(music_panel_, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(music_panel_, 0, 0);
        lv_obj_set_style_radius(music_panel_, 0, 0);
        lv_obj_set_style_pad_all(music_panel_, 0, 0);
        lv_obj_set_scrollbar_mode(music_panel_, LV_SCROLLBAR_MODE_OFF);
        
        // ===== Decorative top accent line (neon pink) =====
        lv_obj_t* accent_line = lv_obj_create(music_panel_);
        lv_obj_set_size(accent_line, 200, 3);
        lv_obj_align(accent_line, LV_ALIGN_TOP_MID, 0, 12);
        lv_obj_set_style_bg_color(accent_line, lv_color_hex(0xff1493), 0);
        lv_obj_set_style_bg_opa(accent_line, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(accent_line, 0, 0);
        lv_obj_set_style_radius(accent_line, 2, 0);
        lv_obj_set_style_shadow_color(accent_line, lv_color_hex(0xff1493), 0);
        lv_obj_set_style_shadow_width(accent_line, 15, 0);
        lv_obj_set_style_shadow_spread(accent_line, 3, 0);
        lv_obj_set_style_shadow_opa(accent_line, LV_OPA_60, 0);
        
        // ===== Music icon with glow circle background =====
        music_icon_bg_ = lv_obj_create(music_panel_);
        lv_obj_set_size(music_icon_bg_, 50, 50);
        lv_obj_align(music_icon_bg_, LV_ALIGN_TOP_MID, 0, 24);
        lv_obj_set_style_bg_color(music_icon_bg_, lv_color_hex(0x1e0a3c), 0);
        lv_obj_set_style_bg_opa(music_icon_bg_, LV_OPA_COVER, 0);
        lv_obj_set_style_border_color(music_icon_bg_, lv_color_hex(0xff1493), 0);
        lv_obj_set_style_border_width(music_icon_bg_, 2, 0);
        lv_obj_set_style_border_opa(music_icon_bg_, LV_OPA_70, 0);
        lv_obj_set_style_radius(music_icon_bg_, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_shadow_color(music_icon_bg_, lv_color_hex(0xff1493), 0);
        lv_obj_set_style_shadow_width(music_icon_bg_, 20, 0);
        lv_obj_set_style_shadow_spread(music_icon_bg_, 5, 0);
        lv_obj_set_style_shadow_opa(music_icon_bg_, LV_OPA_40, 0);
        
        music_icon_label_ = lv_label_create(music_icon_bg_);
        lv_label_set_text(music_icon_label_, LV_SYMBOL_AUDIO);
        lv_obj_set_style_text_color(music_icon_label_, lv_color_hex(0xff69b4), 0);
        lv_obj_set_style_text_font(music_icon_label_, &lv_font_montserrat_14, 0);
        lv_obj_center(music_icon_label_);
        
        // ===== State label (Playing / Paused / Buffering) =====
        music_state_label_ = lv_label_create(music_panel_);
        lv_label_set_text(music_state_label_, "");
        lv_obj_align(music_state_label_, LV_ALIGN_TOP_MID, 0, 82);
        lv_obj_set_style_text_color(music_state_label_, lv_color_hex(0x00ff88), 0);
        lv_obj_set_style_text_font(music_state_label_, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_letter_space(music_state_label_, 2, 0);
        
        // ===== Track Title (large, scrolling, bright white) =====
        // Use theme font for Vietnamese/Chinese support
        music_title_label_ = lv_label_create(music_panel_);
        lv_label_set_text(music_title_label_, "---");
        lv_obj_set_width(music_title_label_, 210);
        lv_obj_align(music_title_label_, LV_ALIGN_TOP_MID, 0, 106);
        lv_label_set_long_mode(music_title_label_, LV_LABEL_LONG_SCROLL_CIRCULAR);
        lv_obj_set_style_text_align(music_title_label_, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_color(music_title_label_, lv_color_hex(0xffffff), 0);
        lv_obj_set_style_text_font(music_title_label_, GetTextFont(), 0);
        lv_obj_set_style_text_letter_space(music_title_label_, 1, 0);
        
        // ===== Artist / folder label (muted blue-white) =====
        // Use theme font for Vietnamese/Chinese support
        music_artist_label_ = lv_label_create(music_panel_);
        lv_label_set_text(music_artist_label_, "");
        lv_obj_set_width(music_artist_label_, 210);
        lv_obj_align(music_artist_label_, LV_ALIGN_TOP_MID, 0, 128);
        lv_label_set_long_mode(music_artist_label_, LV_LABEL_LONG_SCROLL_CIRCULAR);
        lv_obj_set_style_text_align(music_artist_label_, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_color(music_artist_label_, lv_color_hex(0x8899bb), 0);
        lv_obj_set_style_text_font(music_artist_label_, GetTextFont(), 0);
        
        // ===== Progress bar (wide, thick, neon glow) =====
        music_progress_bar_ = lv_bar_create(music_panel_);
        lv_obj_set_size(music_progress_bar_, 200, 10);
        lv_obj_align(music_progress_bar_, LV_ALIGN_TOP_MID, 0, 156);
        lv_bar_set_range(music_progress_bar_, 0, 100);
        lv_bar_set_value(music_progress_bar_, 0, LV_ANIM_OFF);
        // Track background
        lv_obj_set_style_bg_color(music_progress_bar_, lv_color_hex(0x1a1a3e), 0);
        lv_obj_set_style_bg_opa(music_progress_bar_, LV_OPA_COVER, 0);
        lv_obj_set_style_radius(music_progress_bar_, 5, 0);
        // Indicator (neon pink with glow)
        lv_obj_set_style_bg_color(music_progress_bar_, lv_color_hex(0xff1493), LV_PART_INDICATOR);
        lv_obj_set_style_bg_grad_color(music_progress_bar_, lv_color_hex(0xff69b4), LV_PART_INDICATOR);
        lv_obj_set_style_bg_grad_dir(music_progress_bar_, LV_GRAD_DIR_HOR, LV_PART_INDICATOR);
        lv_obj_set_style_radius(music_progress_bar_, 5, LV_PART_INDICATOR);
        lv_obj_set_style_shadow_color(music_progress_bar_, lv_color_hex(0xff1493), LV_PART_INDICATOR);
        lv_obj_set_style_shadow_width(music_progress_bar_, 8, LV_PART_INDICATOR);
        lv_obj_set_style_shadow_spread(music_progress_bar_, 2, LV_PART_INDICATOR);
        lv_obj_set_style_shadow_opa(music_progress_bar_, LV_OPA_50, LV_PART_INDICATOR);
        
        // ===== Track number / lyrics label =====
        // Use theme font for Vietnamese/Chinese support in lyrics
        music_track_label_ = lv_label_create(music_panel_);
        lv_label_set_text(music_track_label_, "");
        lv_obj_set_width(music_track_label_, 210);
        lv_obj_align(music_track_label_, LV_ALIGN_TOP_MID, 0, 176);
        lv_label_set_long_mode(music_track_label_, LV_LABEL_LONG_SCROLL_CIRCULAR);
        lv_obj_set_style_text_align(music_track_label_, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_color(music_track_label_, lv_color_hex(0x6677aa), 0);
        lv_obj_set_style_text_font(music_track_label_, GetTextFont(), 0);
        
        // ===== Spectrum bars (music visualizer) =====
        // Colors cycling through neon spectrum
        const uint32_t bar_colors[] = {
            0xFF1493, 0xFF69B4, 0x00FF88, 0x00FFFF, 
            0x7B68EE, 0xFF6B6B, 0xFFD700, 0x00CED1,
            0xFF1493, 0xFF69B4, 0x00FF88, 0x00FFFF
        };
        for (int i = 0; i < SPECTRUM_BAR_COUNT; i++) {
            spectrum_bars_[i] = lv_obj_create(music_panel_);
            lv_obj_set_size(spectrum_bars_[i], 10, 20);  // Initial height
            lv_obj_align(spectrum_bars_[i], LV_ALIGN_BOTTOM_MID, 
                        (i - SPECTRUM_BAR_COUNT/2) * 16 + 8, -10);
            lv_obj_set_style_bg_color(spectrum_bars_[i], lv_color_hex(bar_colors[i]), 0);
            lv_obj_set_style_bg_opa(spectrum_bars_[i], LV_OPA_90, 0);
            lv_obj_set_style_border_width(spectrum_bars_[i], 0, 0);
            lv_obj_set_style_radius(spectrum_bars_[i], 3, 0);
            lv_obj_set_style_shadow_color(spectrum_bars_[i], lv_color_hex(bar_colors[i]), 0);
            lv_obj_set_style_shadow_width(spectrum_bars_[i], 8, 0);
            lv_obj_set_style_shadow_spread(spectrum_bars_[i], 2, 0);
            lv_obj_set_style_shadow_opa(spectrum_bars_[i], LV_OPA_60, 0);
            spectrum_heights_[i] = 20;
        }
        
        // Initially hidden
        lv_obj_add_flag(music_panel_, LV_OBJ_FLAG_HIDDEN);
        music_display_visible_ = false;
        
        lvgl_port_unlock();
    }

    void CreateVolumePopup() {
        if (volume_popup_) return;  // Already created
        
        lvgl_port_lock(0);
        auto screen = lv_screen_active();
        
        // ===== Volume popup container =====
        volume_popup_ = lv_obj_create(screen);
        lv_obj_set_size(volume_popup_, 180, 70);
        lv_obj_align(volume_popup_, LV_ALIGN_TOP_MID, 0, 40);
        // Semi-transparent dark background
        lv_obj_set_style_bg_color(volume_popup_, lv_color_hex(0x1a1a2e), 0);
        lv_obj_set_style_bg_opa(volume_popup_, LV_OPA_90, 0);
        lv_obj_set_style_border_color(volume_popup_, lv_color_hex(0x3498db), 0);
        lv_obj_set_style_border_width(volume_popup_, 2, 0);
        lv_obj_set_style_border_opa(volume_popup_, LV_OPA_70, 0);
        lv_obj_set_style_radius(volume_popup_, 12, 0);
        lv_obj_set_style_pad_all(volume_popup_, 8, 0);
        lv_obj_set_scrollbar_mode(volume_popup_, LV_SCROLLBAR_MODE_OFF);
        // Shadow for depth
        lv_obj_set_style_shadow_color(volume_popup_, lv_color_hex(0x000000), 0);
        lv_obj_set_style_shadow_width(volume_popup_, 15, 0);
        lv_obj_set_style_shadow_spread(volume_popup_, 2, 0);
        lv_obj_set_style_shadow_opa(volume_popup_, LV_OPA_50, 0);
        
        // ===== Volume icon =====
        volume_icon_label_ = lv_label_create(volume_popup_);
        lv_label_set_text(volume_icon_label_, LV_SYMBOL_VOLUME_MAX);
        lv_obj_align(volume_icon_label_, LV_ALIGN_TOP_LEFT, 4, 4);
        lv_obj_set_style_text_color(volume_icon_label_, lv_color_hex(0x3498db), 0);
        lv_obj_set_style_text_font(volume_icon_label_, &lv_font_montserrat_14, 0);
        
        // ===== Volume value label =====
        volume_value_label_ = lv_label_create(volume_popup_);
        lv_label_set_text(volume_value_label_, "70%");
        lv_obj_align(volume_value_label_, LV_ALIGN_TOP_RIGHT, -4, 4);
        lv_obj_set_style_text_color(volume_value_label_, lv_color_hex(0xecf0f1), 0);
        lv_obj_set_style_text_font(volume_value_label_, &lv_font_montserrat_14, 0);
        
        // ===== Volume progress bar =====
        volume_bar_ = lv_bar_create(volume_popup_);
        lv_obj_set_size(volume_bar_, 160, 14);
        lv_obj_align(volume_bar_, LV_ALIGN_BOTTOM_MID, 0, -4);
        lv_bar_set_range(volume_bar_, 0, 100);
        lv_bar_set_value(volume_bar_, 70, LV_ANIM_OFF);
        // Track background
        lv_obj_set_style_bg_color(volume_bar_, lv_color_hex(0x2c3e50), 0);
        lv_obj_set_style_bg_opa(volume_bar_, LV_OPA_COVER, 0);
        lv_obj_set_style_radius(volume_bar_, 7, 0);
        // Indicator
        lv_obj_set_style_bg_color(volume_bar_, lv_color_hex(0x3498db), LV_PART_INDICATOR);
        lv_obj_set_style_bg_grad_color(volume_bar_, lv_color_hex(0x2ecc71), LV_PART_INDICATOR);
        lv_obj_set_style_bg_grad_dir(volume_bar_, LV_GRAD_DIR_HOR, LV_PART_INDICATOR);
        lv_obj_set_style_radius(volume_bar_, 7, LV_PART_INDICATOR);
        
        // Initially hidden
        lv_obj_add_flag(volume_popup_, LV_OBJ_FLAG_HIDDEN);
        
        // Create timer for auto-hide
        esp_timer_create_args_t timer_args = {};
        timer_args.callback = [](void* arg) {
            auto* self = static_cast<XINGZHI_CUBE_1_54TFT_WIFI*>(arg);
            if (self->volume_popup_) {
                lvgl_port_lock(0);
                lv_obj_add_flag(self->volume_popup_, LV_OBJ_FLAG_HIDDEN);
                lvgl_port_unlock();
            }
        };
        timer_args.arg = this;
        timer_args.name = "vol_popup";
        esp_timer_create(&timer_args, &volume_popup_timer_);
        
        lvgl_port_unlock();
    }

    void ShowVolumePopup(int volume) {
        if (!volume_popup_) {
            CreateVolumePopup();
        }
        if (!volume_popup_) return;
        
        lvgl_port_lock(0);
        
        // Update icon based on volume level
        if (volume == 0) {
            lv_label_set_text(volume_icon_label_, LV_SYMBOL_MUTE);
            lv_obj_set_style_text_color(volume_icon_label_, lv_color_hex(0xe74c3c), 0);
        } else if (volume < 30) {
            lv_label_set_text(volume_icon_label_, LV_SYMBOL_VOLUME_MID);
            lv_obj_set_style_text_color(volume_icon_label_, lv_color_hex(0xf39c12), 0);
        } else {
            lv_label_set_text(volume_icon_label_, LV_SYMBOL_VOLUME_MAX);
            lv_obj_set_style_text_color(volume_icon_label_, lv_color_hex(0x3498db), 0);
        }
        
        // Update value label
        char vol_str[8];
        snprintf(vol_str, sizeof(vol_str), "%d%%", volume);
        lv_label_set_text(volume_value_label_, vol_str);
        
        // Update progress bar with animation
        lv_bar_set_value(volume_bar_, volume, LV_ANIM_ON);
        
        // Update bar color based on volume
        if (volume == 0) {
            lv_obj_set_style_bg_color(volume_bar_, lv_color_hex(0xe74c3c), LV_PART_INDICATOR);
            lv_obj_set_style_bg_grad_color(volume_bar_, lv_color_hex(0xc0392b), LV_PART_INDICATOR);
        } else if (volume >= 80) {
            lv_obj_set_style_bg_color(volume_bar_, lv_color_hex(0x2ecc71), LV_PART_INDICATOR);
            lv_obj_set_style_bg_grad_color(volume_bar_, lv_color_hex(0x27ae60), LV_PART_INDICATOR);
        } else {
            lv_obj_set_style_bg_color(volume_bar_, lv_color_hex(0x3498db), LV_PART_INDICATOR);
            lv_obj_set_style_bg_grad_color(volume_bar_, lv_color_hex(0x2ecc71), LV_PART_INDICATOR);
        }
        
        // Show popup
        lv_obj_remove_flag(volume_popup_, LV_OBJ_FLAG_HIDDEN);
        
        lvgl_port_unlock();
        
        // Reset and start auto-hide timer (2 seconds)
        esp_timer_stop(volume_popup_timer_);
        esp_timer_start_once(volume_popup_timer_, 2000000);  // 2 seconds
    }

    void ShowMusicDisplay(bool show) {
        if (!music_panel_) return;
        if (show == music_display_visible_) return;
        
        lvgl_port_lock(0);
        if (show) {
            lv_obj_remove_flag(music_panel_, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(music_panel_, LV_OBJ_FLAG_HIDDEN);
        }
        lvgl_port_unlock();
        music_display_visible_ = show;
    }

    void UpdateMusicDisplay() {
        if (!music_panel_) return;
        
        bool any_playing = false;
        
        // Check SD player
        if (mp3_player_ && mp3_player_->GetState() != Mp3PlayerState::STOPPED) {
            any_playing = true;
            lvgl_port_lock(0);
            
            // Title
            std::string track = mp3_player_->GetCurrentTrack();
            size_t pos = track.find_last_of("/");
            std::string name = (pos != std::string::npos) ? track.substr(pos + 1) : track;
            // Remove .mp3 extension
            size_t ext = name.find_last_of(".");
            if (ext != std::string::npos) name = name.substr(0, ext);
            lv_label_set_text(music_title_label_, name.c_str());
            
            // State
            if (mp3_player_->GetState() == Mp3PlayerState::PLAYING) {
                lv_label_set_text(music_state_label_, LV_SYMBOL_PLAY " SD Card");
                lv_obj_set_style_text_color(music_state_label_, lv_color_hex(0x2ecc71), 0);
            } else if (mp3_player_->GetState() == Mp3PlayerState::PAUSED) {
                lv_label_set_text(music_state_label_, LV_SYMBOL_PAUSE " Paused");
                lv_obj_set_style_text_color(music_state_label_, lv_color_hex(0xf39c12), 0);
            }
            
            // Artist = folder name
            size_t folder_end = track.find_last_of("/");
            std::string folder = "";
            if (folder_end != std::string::npos && folder_end > 0) {
                size_t folder_start = track.find_last_of("/", folder_end - 1);
                if (folder_start != std::string::npos) {
                    folder = track.substr(folder_start + 1, folder_end - folder_start - 1);
                }
            }
            lv_label_set_text(music_artist_label_, folder.c_str());
            
            // Track number
            char track_info[32];
            snprintf(track_info, sizeof(track_info), "%d / %d", 
                     mp3_player_->GetCurrentIndex() + 1, mp3_player_->GetPlaylistSize());
            lv_label_set_text(music_track_label_, track_info);
            
            // Progress bar (estimated)
            if (mp3_player_->GetPlaylistSize() > 0) {
                int progress = ((mp3_player_->GetCurrentIndex()) * 100) / mp3_player_->GetPlaylistSize();
                lv_bar_set_value(music_progress_bar_, progress, LV_ANIM_ON);
            }
            
            // Animate spectrum bars
            UpdateSpectrumBars(mp3_player_->GetState() == Mp3PlayerState::PLAYING);
            
            lvgl_port_unlock();
        }
        
        // Check stream player
        int stream_state = StreamPlayer_GetState();
        if (stream_state >= 1 && stream_state <= 3) {  // SEARCHING, BUFFERING, PLAYING
            any_playing = true;
            lvgl_port_lock(0);
            
            const char* title = StreamPlayer_GetTitle();
            const char* artist = StreamPlayer_GetArtist();
            
            if (title && strlen(title) > 0) {
                lv_label_set_text(music_title_label_, title);
            } else {
                lv_label_set_text(music_title_label_, "Searching...");
            }
            
            if (artist && strlen(artist) > 0) {
                lv_label_set_text(music_artist_label_, artist);
            } else {
                lv_label_set_text(music_artist_label_, "Online Stream");
            }
            
            // State
            if (stream_state == 1) {
                lv_label_set_text(music_state_label_, LV_SYMBOL_REFRESH " Searching...");
                lv_obj_set_style_text_color(music_state_label_, lv_color_hex(0x3498db), 0);
            } else if (stream_state == 2) {
                lv_label_set_text(music_state_label_, LV_SYMBOL_DOWNLOAD " Buffering...");
                lv_obj_set_style_text_color(music_state_label_, lv_color_hex(0xf39c12), 0);
            } else if (stream_state == 3) {
                lv_label_set_text(music_state_label_, LV_SYMBOL_PLAY " Streaming");
                lv_obj_set_style_text_color(music_state_label_, lv_color_hex(0x2ecc71), 0);
            }
            
            // Lyrics as track info
            const char* lyric = StreamPlayer_GetCurrentLyricText();
            if (lyric && strlen(lyric) > 0) {
                lv_label_set_text(music_track_label_, lyric);
            } else {
                lv_label_set_text(music_track_label_, "");
            }
            
            // Progress based on play time (estimate 4 min song)
            int64_t play_ms = StreamPlayer_GetPlayTimeMs();
            int progress = (int)((play_ms % 240000) * 100 / 240000);
            lv_bar_set_value(music_progress_bar_, progress, LV_ANIM_ON);
            
            // Animate spectrum bars
            UpdateSpectrumBars(stream_state == 3);  // PLAYING state
            
            lvgl_port_unlock();
        }
        
        // Check radio player (show UI when radio_mode_ is active OR Radio_IsPlaying)
        if (radio_mode_ || Radio_IsPlaying()) {
            any_playing = true;
            lvgl_port_lock(0);
            
            const char* station = Radio_GetCurrentStation();
            if (station && strlen(station) > 0) {
                lv_label_set_text(music_title_label_, station);
            } else {
                lv_label_set_text(music_title_label_, "FM Radio");
            }
            
            if (Radio_IsPlaying()) {
                lv_label_set_text(music_artist_label_, "VOV Radio");
                lv_label_set_text(music_state_label_, LV_SYMBOL_AUDIO " Dang phat");
                lv_label_set_text(music_track_label_, "Dang phat dai FM...");
            } else {
                lv_label_set_text(music_artist_label_, "Dang ket noi...");
                lv_label_set_text(music_state_label_, LV_SYMBOL_REFRESH " Dang tai...");
                lv_label_set_text(music_track_label_, "Vui long cho...");
            }
            lv_obj_set_style_text_color(music_state_label_, lv_color_hex(0xFF8C00), 0);
            
            // Change icon to radio style - warm orange theme
            lv_label_set_text(music_icon_label_, LV_SYMBOL_AUDIO);
            lv_obj_set_style_text_color(music_icon_label_, lv_color_hex(0xFFA500), 0);
            lv_obj_set_style_border_color(music_icon_bg_, lv_color_hex(0xFF6B00), 0);
            lv_obj_set_style_shadow_color(music_icon_bg_, lv_color_hex(0xFF6B00), 0);
            
            // Progress bar cycles for radio - orange theme
            static int radio_progress = 0;
            radio_progress = (radio_progress + 1) % 100;
            lv_bar_set_value(music_progress_bar_, radio_progress, LV_ANIM_ON);
            lv_obj_set_style_bg_color(music_progress_bar_, lv_color_hex(0xFF6B00), LV_PART_INDICATOR);
            lv_obj_set_style_bg_grad_color(music_progress_bar_, lv_color_hex(0xFFA500), LV_PART_INDICATOR);
            lv_obj_set_style_shadow_color(music_progress_bar_, lv_color_hex(0xFF6B00), LV_PART_INDICATOR);
            
            UpdateSpectrumBars(Radio_IsPlaying());
            
            lvgl_port_unlock();
        } else if (!radio_mode_ && !Radio_IsPlaying()) {
            // Restore music style when not in radio mode
            if (music_panel_ && (mp3_player_ && mp3_player_->GetState() != Mp3PlayerState::STOPPED)) {
                lvgl_port_lock(0);
                lv_label_set_text(music_icon_label_, LV_SYMBOL_AUDIO);
                lv_obj_set_style_text_color(music_icon_label_, lv_color_hex(0xff69b4), 0);
                lv_obj_set_style_border_color(music_icon_bg_, lv_color_hex(0xff1493), 0);
                lv_obj_set_style_shadow_color(music_icon_bg_, lv_color_hex(0xff1493), 0);
                lv_obj_set_style_bg_color(music_progress_bar_, lv_color_hex(0xff1493), LV_PART_INDICATOR);
                lv_obj_set_style_bg_grad_color(music_progress_bar_, lv_color_hex(0xff69b4), LV_PART_INDICATOR);
                lv_obj_set_style_shadow_color(music_progress_bar_, lv_color_hex(0xff1493), LV_PART_INDICATOR);
                lvgl_port_unlock();
            }
        }
        
        ShowMusicDisplay(any_playing);
        
        // ====== Auto LED music mode & Power save ======
        if (any_playing && !music_led_active_) {
            // Save current LED state before switching
            led_state_t* led = get_led_state();
            led_saved_before_music_ = *led;
            music_led_active_ = true;
            
            if (g_music_power_save.load()) {
                // Power save: turn off LED, dim display
                ninja_led_off();
                GetBacklight()->SetBrightness(1);
                g_music_power_save_applied.store(true);
                ESP_LOGI(TAG, "Music power save: LED off, display dimmed");
            } else {
                // Auto switch to MUSIC_REACTIVE mode
                ninja_led_set_mode(LED_MODE_MUSIC_REACTIVE);
                ESP_LOGI(TAG, "Auto LED: switched to MUSIC_REACTIVE");
            }
        } else if (any_playing && music_led_active_) {
            // Check if power save mode toggled while playing
            bool ps = g_music_power_save.load();
            bool ps_applied = g_music_power_save_applied.load();
            if (ps && !ps_applied) {
                // Just enabled power save -> turn off LED, dim display
                ninja_led_off();
                GetBacklight()->SetBrightness(1);
                g_music_power_save_applied.store(true);
            } else if (!ps && ps_applied) {
                // Just disabled power save -> switch to MUSIC_REACTIVE
                ninja_led_set_mode(LED_MODE_MUSIC_REACTIVE);
                GetBacklight()->RestoreBrightness();
                g_music_power_save_applied.store(false);
            }
        } else if (!any_playing && music_led_active_) {
            // Music stopped -> restore previous LED state
            led_state_t* led = get_led_state();
            *led = led_saved_before_music_;
            ninja_led_update();
            music_led_active_ = false;
            if (g_music_power_save_applied.load()) {
                GetBacklight()->RestoreBrightness();
                g_music_power_save_applied.store(false);
            }
            ESP_LOGI(TAG, "Auto LED: restored previous state");
        }
    }

    static void MusicDisplayTimerCallback(void* arg) {
        auto* self = static_cast<XINGZHI_CUBE_1_54TFT_WIFI*>(arg);
        self->UpdateMusicDisplay();
    }

    void StartMusicDisplayTimer() {
        if (music_display_timer_) return;
        
        esp_timer_create_args_t timer_args = {};
        timer_args.callback = MusicDisplayTimerCallback;
        timer_args.arg = this;
        timer_args.name = "music_disp";
        esp_timer_create(&timer_args, &music_display_timer_);
        esp_timer_start_periodic(music_display_timer_, 500000);  // 500ms interval
    }

    void InitializePowerManager() {
        power_manager_ = new PowerManager(GPIO_NUM_38);
        power_manager_->OnChargingStatusChanged([this](bool is_charging) {
            if (is_charging) {
                power_save_timer_->SetEnabled(false);
            } else {
                power_save_timer_->SetEnabled(true);
            }
        });
    }

    void InitializePowerSaveTimer() {
        // GPIO 21 now used for LED strip - power control disabled
        // rtc_gpio_init(GPIO_NUM_21);
        // rtc_gpio_set_direction(GPIO_NUM_21, RTC_GPIO_MODE_OUTPUT_ONLY);
        // rtc_gpio_set_level(GPIO_NUM_21, 1);

        // Load saved sleep config from NVS (defaults: sleep=60s, shutdown=300s)
        int sleep_sec = 60, shutdown_sec = 300;
        {
            nvs_handle_t handle;
            if (nvs_open("sleep_cfg", NVS_READONLY, &handle) == ESP_OK) {
                int32_t val;
                if (nvs_get_i32(handle, "sleep_s", &val) == ESP_OK) sleep_sec = val;
                if (nvs_get_i32(handle, "shut_s", &val) == ESP_OK) shutdown_sec = val;
                nvs_close(handle);
                ESP_LOGI(TAG, "Loaded sleep config: sleep=%ds, shutdown=%ds", sleep_sec, shutdown_sec);
            }
        }

        power_save_timer_ = new PowerSaveTimer(-1, sleep_sec, shutdown_sec);
        g_pst_ptr = power_save_timer_;  // Expose to C interface
        power_save_timer_->OnEnterSleepMode([this]() {
            GetDisplay()->SetPowerSaveMode(true);
            GetBacklight()->SetBrightness(1);
            // Turn off LED strip and detach all servos before sleep
            robot_prepare_sleep();
        });
        power_save_timer_->OnExitSleepMode([this]() {
            GetDisplay()->SetPowerSaveMode(false);
            GetBacklight()->RestoreBrightness();
        });
        power_save_timer_->OnShutdownRequest([this]() {
            ESP_LOGI(TAG, "Shutting down");
            // Ensure LED and servos are off before deep sleep
            robot_prepare_sleep();
            esp_lcd_panel_disp_on_off(panel_, false); //关闭显示
            esp_deep_sleep_start();
        });
        power_save_timer_->SetEnabled(true);
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_SDA;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = DISPLAY_SCL;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            power_save_timer_->WakeUp();
            
            // If in playlist selection mode - select and play
            if (playlist_selection_mode_) {
                PlaylistSelectCurrent();
                return;
            }
            
            // If in radio station selection mode - select and play
            if (radio_station_selection_mode_) {
                RadioStationSelectCurrent();
                return;
            }
            
            // Check if streaming music - single click stops it
            int stream_state = StreamPlayer_GetState();
            if (stream_state == 2 || stream_state == 3) {  // BUFFERING=2 or PLAYING=3
                ESP_LOGI(TAG, "Boot button: stopping streaming music");
                StreamPlayer_Stop();
                GetDisplay()->ShowNotification("Da tat nhac");
                return;
            }
            
            // In radio mode - single click stops radio
            if (radio_mode_) {
                StopRadioMode();
                return;
            }
            
            // Block AI when in music mode
            if (music_mode_) {
                if (mp3_player_) {
                    // Single click = Pause/Resume when in music mode
                    if (mp3_player_->GetState() == Mp3PlayerState::PLAYING) {
                        mp3_player_->Pause();
                        GetDisplay()->ShowNotification("Paused");
                    } else if (mp3_player_->GetState() == Mp3PlayerState::PAUSED) {
                        mp3_player_->Resume();
                        GetDisplay()->ShowNotification("Playing");
                    }
                }
                return;
            }
            
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });

        boot_button_.OnLongPress([this]() {
            power_save_timer_->WakeUp();
            // Long press BOOT in radio mode = show radio station selection
            if (radio_mode_) {
                if (radio_station_selection_mode_) {
                    HideRadioStationSelection();
                } else {
                    ShowRadioStationSelection();
                }
                return;
            }
            // Long press BOOT in music mode = show playlist selection
            if (music_mode_ && mp3_player_ && mp3_player_->GetPlaylistSize() > 0) {
                if (playlist_selection_mode_) {
                    HidePlaylistSelection();
                } else {
                    ShowPlaylistSelection();
                }
            }
        });

        boot_button_.OnDoubleClick([this]() {
            power_save_timer_->WakeUp();
            ToggleMusicMode();
        });

        volume_up_button_.OnClick([this]() {
            power_save_timer_->WakeUp();
            
            // In playlist selection mode - scroll up
            if (playlist_selection_mode_) {
                PlaylistMoveUp();
                return;
            }
            
            // In radio station selection mode - scroll up
            if (radio_station_selection_mode_) {
                RadioStationMoveUp();
                return;
            }
            
            // All modes (idle, music, radio): adjust volume
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() + 10;
            if (volume > 100) {
                volume = 100;
            }
            codec->SetOutputVolume(volume);
            ShowVolumePopup(volume);
        });

        volume_up_button_.OnDoubleClick([this]() {
            power_save_timer_->WakeUp();
            // In music mode - double click to next track
            if (music_mode_ && mp3_player_) {
                mp3_player_->Next();
                return;
            }
            // In radio mode - double click to next station
            if (radio_mode_) {
                RadioNextStation();
            }
        });

        volume_up_button_.OnLongPress([this]() {
            power_save_timer_->WakeUp();
            // Long press = gradual volume increase
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() + 5;
            if (volume > 100) volume = 100;
            codec->SetOutputVolume(volume);
            ShowVolumePopup(volume);
        });

        volume_up_button_.OnLongPressRepeat([this]() {
            // Continue increasing volume slowly
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() + 5;
            if (volume > 100) volume = 100;
            codec->SetOutputVolume(volume);
            ShowVolumePopup(volume);
        });

        volume_down_button_.OnClick([this]() {
            power_save_timer_->WakeUp();
            
            // In playlist selection mode - scroll down
            if (playlist_selection_mode_) {
                PlaylistMoveDown();
                return;
            }
            
            // In radio station selection mode - scroll down
            if (radio_station_selection_mode_) {
                RadioStationMoveDown();
                return;
            }
            
            // All modes (idle, music, radio): adjust volume
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() - 10;
            if (volume < 0) {
                volume = 0;
            }
            codec->SetOutputVolume(volume);
            ShowVolumePopup(volume);
        });

        volume_down_button_.OnDoubleClick([this]() {
            power_save_timer_->WakeUp();
            // In music mode - double click to previous track
            if (music_mode_ && mp3_player_) {
                mp3_player_->Previous();
                return;
            }
            // In radio mode - double click to previous station
            if (radio_mode_) {
                RadioPrevStation();
            }
        });

        volume_down_button_.OnLongPress([this]() {
            power_save_timer_->WakeUp();
            // Long press = gradual volume decrease
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() - 5;
            if (volume < 0) volume = 0;
            codec->SetOutputVolume(volume);
            ShowVolumePopup(volume);
        });

        volume_down_button_.OnLongPressRepeat([this]() {
            // Continue decreasing volume slowly
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() - 5;
            if (volume < 0) volume = 0;
            codec->SetOutputVolume(volume);
            ShowVolumePopup(volume);
        });
    }

    void StartMusicMode(bool from_llm = false) {
        if (music_mode_) return;
        if (!sd_card_mounted_) {
            GetDisplay()->ShowNotification("SD card not found!");
            ESP_LOGW(TAG, "Cannot enter music mode - SD card not mounted");
            return;
        }
        music_mode_ = true;
        {
            // Only stop AI chat if NOT called from LLM tool (to avoid conflict)
            if (!from_llm) {
                auto& app = Application::GetInstance();
                if (app.GetDeviceState() == kDeviceStateListening || 
                    app.GetDeviceState() == kDeviceStateSpeaking) {
                    ESP_LOGI(TAG, "Stopping AI chat before starting music...");
                    app.ToggleChatState();
                    vTaskDelay(pdMS_TO_TICKS(500));
                }
            }
            
            if (!mp3_player_) {
                mp3_player_ = new Mp3Player(GetAudioCodec());
                g_sd_player = mp3_player_;
                
                // Register with robot control for dance music
                set_music_player_ptr(mp3_player_);
                
                mp3_player_->OnTrackChanged([this](const std::string& track) {
                    size_t pos = track.find_last_of("/");
                    std::string name = (pos != std::string::npos) ? track.substr(pos + 1) : track;
                    GetDisplay()->ShowNotification(name);
                });
                mp3_player_->OnPlaybackFinished([this]() {
                    if (music_mode_) {
                        vTaskDelay(pdMS_TO_TICKS(300));
                        mp3_player_->Next();
                    }
                });
                mp3_player_->OnError([this](const std::string& error) {
                    ESP_LOGE(TAG, "Music error: %s", error.c_str());
                    if (error == "Too many failures") {
                        music_mode_ = false;
                        GetDisplay()->ShowNotification("Music Error");
                    }
                });
            }
            ESP_LOGI(TAG, "Scanning for MP3 files...");
            int count = mp3_player_->ScanDirectory(SDCARD_MOUNT_POINT);
            if (count > 0) {
                GetDisplay()->ShowNotification("Music: " + std::to_string(count) + " files");
                // Don't auto-play if called from LLM (delayed playback handled separately)
                if (!from_llm) {
                    vTaskDelay(pdMS_TO_TICKS(100));
                    mp3_player_->Next();
                }
            } else {
                GetDisplay()->ShowNotification("No MP3 files");
                ESP_LOGW(TAG, "No MP3 files found in %s", SDCARD_MOUNT_POINT);
                music_mode_ = false;
            }
        }
    }

    void StopMusicMode() {
        if (!music_mode_) return;
        music_mode_ = false;
        if (mp3_player_) {
            mp3_player_->Stop();
        }
        GetDisplay()->ShowNotification("Music OFF");
    }

    void StartRadioMode() {
        if (radio_mode_) return;
        // Stop music if playing
        if (music_mode_) {
            StopMusicMode();
        }
        // Stop streaming if active
        StreamPlayer_Stop();
        
        radio_mode_ = true;
        
        // Stop AI chat if active
        auto& app = Application::GetInstance();
        if (app.GetDeviceState() == kDeviceStateListening || 
            app.GetDeviceState() == kDeviceStateSpeaking) {
            app.ToggleChatState();
            vTaskDelay(pdMS_TO_TICKS(300));
        }
        
        // Play first station (or last selected)
        const char* json = Radio_GetStationListJson();
        ESP_LOGI(TAG, "Starting FM Radio mode, stations: %s", json);
        
        // Parse first station name from JSON array like ["VOV1 - VOV 1 - Thời sự", ...]
        std::string first_station;
        if (json && strlen(json) > 2) {
            std::string json_str(json);
            size_t start = json_str.find('"');
            size_t end = json_str.find('"', start + 1);
            if (start != std::string::npos && end != std::string::npos) {
                first_station = json_str.substr(start + 1, end - start - 1);
            }
        }
        
        if (!first_station.empty()) {
            Radio_PlayStation(first_station.c_str());
            GetDisplay()->ShowNotification("FM: " + first_station);
        } else {
            GetDisplay()->ShowNotification("FM Radio ON");
            Radio_PlayStation("VOV1");
        }
    }

    void StopRadioMode() {
        if (!radio_mode_) return;
        if (radio_station_selection_mode_) {
            HideRadioStationSelection();
        }
        radio_mode_ = false;
        radio_station_index_ = 0;
        Radio_Stop();
        GetDisplay()->ShowNotification("FM OFF");
    }

    void RadioNextStation() {
        if (!radio_mode_) return;
        const char* json = Radio_GetStationListJson();
        if (!json || strlen(json) < 3) return;
        
        // Parse station list
        std::vector<std::string> stations;
        std::string json_str(json);
        size_t pos = 0;
        while ((pos = json_str.find('"', pos)) != std::string::npos) {
            size_t end = json_str.find('"', pos + 1);
            if (end == std::string::npos) break;
            stations.push_back(json_str.substr(pos + 1, end - pos - 1));
            pos = end + 1;
        }
        
        if (stations.empty()) return;
        radio_station_index_ = (radio_station_index_ + 1) % (int)stations.size();
        Radio_PlayStation(stations[radio_station_index_].c_str());
        GetDisplay()->ShowNotification("FM: " + stations[radio_station_index_]);
    }

    void RadioPrevStation() {
        if (!radio_mode_) return;
        const char* json = Radio_GetStationListJson();
        if (!json || strlen(json) < 3) return;
        
        std::vector<std::string> stations;
        std::string json_str(json);
        size_t pos = 0;
        while ((pos = json_str.find('"', pos)) != std::string::npos) {
            size_t end = json_str.find('"', pos + 1);
            if (end == std::string::npos) break;
            stations.push_back(json_str.substr(pos + 1, end - pos - 1));
            pos = end + 1;
        }
        
        if (stations.empty()) return;
        radio_station_index_--;
        if (radio_station_index_ < 0) radio_station_index_ = (int)stations.size() - 1;
        Radio_PlayStation(stations[radio_station_index_].c_str());
        GetDisplay()->ShowNotification("FM: " + stations[radio_station_index_]);
    }

    // ===== Radio Station Selection UI =====
    void ParseRadioStations() {
        radio_stations_cache_.clear();
        const char* json = Radio_GetStationListJson();
        if (!json || strlen(json) < 3) return;
        
        std::string json_str(json);
        size_t pos = 0;
        while ((pos = json_str.find('"', pos)) != std::string::npos) {
            size_t end = json_str.find('"', pos + 1);
            if (end == std::string::npos) break;
            radio_stations_cache_.push_back(json_str.substr(pos + 1, end - pos - 1));
            pos = end + 1;
        }
    }

    void CreateRadioStationSelection() {
        if (radio_station_panel_) return;
        
        lvgl_port_lock(0);
        auto screen = lv_screen_active();
        
        // Full-screen panel for radio station selection - dark orange theme
        radio_station_panel_ = lv_obj_create(screen);
        lv_obj_set_size(radio_station_panel_, 240, 240);
        lv_obj_align(radio_station_panel_, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_style_bg_color(radio_station_panel_, lv_color_hex(0x1a1000), 0);
        lv_obj_set_style_bg_grad_color(radio_station_panel_, lv_color_hex(0x2e1a00), 0);
        lv_obj_set_style_bg_grad_dir(radio_station_panel_, LV_GRAD_DIR_VER, 0);
        lv_obj_set_style_border_width(radio_station_panel_, 0, 0);
        lv_obj_set_style_radius(radio_station_panel_, 0, 0);
        lv_obj_set_style_pad_all(radio_station_panel_, 0, 0);
        lv_obj_add_flag(radio_station_panel_, LV_OBJ_FLAG_HIDDEN);
        
        // Title label
        radio_station_title_label_ = lv_label_create(radio_station_panel_);
        lv_obj_set_style_text_font(radio_station_title_label_, GetTextFont(), 0);
        lv_obj_set_style_text_color(radio_station_title_label_, lv_color_hex(0xFF8C00), 0);
        lv_label_set_text(radio_station_title_label_, LV_SYMBOL_AUDIO " Chon kenh FM");
        lv_obj_align(radio_station_title_label_, LV_ALIGN_TOP_MID, 0, 10);
        
        // Create station item labels
        for (int i = 0; i < RADIO_VISIBLE_ITEMS; i++) {
            radio_station_items_[i] = lv_label_create(radio_station_panel_);
            lv_obj_set_style_text_font(radio_station_items_[i], GetTextFont(), 0);
            lv_obj_set_style_text_color(radio_station_items_[i], lv_color_hex(0xecf0f1), 0);
            lv_obj_set_width(radio_station_items_[i], 220);
            lv_obj_set_style_pad_left(radio_station_items_[i], 10, 0);
            lv_obj_set_style_pad_ver(radio_station_items_[i], 5, 0);
            lv_label_set_long_mode(radio_station_items_[i], LV_LABEL_LONG_SCROLL_CIRCULAR);
            lv_label_set_text(radio_station_items_[i], "");
            lv_obj_align(radio_station_items_[i], LV_ALIGN_TOP_LEFT, 5, 40 + i * 38);
        }
        
        // Auto-hide timer (10 seconds)
        esp_timer_create_args_t timer_args = {};
        timer_args.callback = [](void* arg) {
            auto* self = static_cast<XINGZHI_CUBE_1_54TFT_WIFI*>(arg);
            self->HideRadioStationSelection();
        };
        timer_args.arg = this;
        timer_args.name = "radio_sel_timeout";
        esp_timer_create(&timer_args, &radio_station_timeout_timer_);
        
        lvgl_port_unlock();
    }

    void ShowRadioStationSelection() {
        if (!radio_station_panel_) {
            CreateRadioStationSelection();
        }
        if (!radio_station_panel_) return;
        
        // Parse stations
        ParseRadioStations();
        if (radio_stations_cache_.empty()) {
            GetDisplay()->ShowNotification("Khong co kenh FM");
            return;
        }
        
        radio_station_selection_mode_ = true;
        radio_station_selected_index_ = radio_station_index_;
        if (radio_station_selected_index_ < 0 || radio_station_selected_index_ >= (int)radio_stations_cache_.size()) {
            radio_station_selected_index_ = 0;
        }
        
        UpdateRadioStationSelection();
        
        lvgl_port_lock(0);
        lv_obj_remove_flag(radio_station_panel_, LV_OBJ_FLAG_HIDDEN);
        // Hide music panel while selecting
        if (music_panel_) {
            lv_obj_add_flag(music_panel_, LV_OBJ_FLAG_HIDDEN);
        }
        lvgl_port_unlock();
        
        // Reset timeout timer
        esp_timer_stop(radio_station_timeout_timer_);
        esp_timer_start_once(radio_station_timeout_timer_, 10000000);  // 10 seconds
    }

    void UpdateRadioStationSelection() {
        if (!radio_station_panel_) return;
        
        int station_count = (int)radio_stations_cache_.size();
        if (station_count == 0) return;
        
        // Calculate visible range (center selected item)
        int start_index = radio_station_selected_index_ - RADIO_VISIBLE_ITEMS / 2;
        if (start_index < 0) start_index = 0;
        if (start_index + RADIO_VISIBLE_ITEMS > station_count) {
            start_index = station_count - RADIO_VISIBLE_ITEMS;
            if (start_index < 0) start_index = 0;
        }
        
        lvgl_port_lock(0);
        
        // Update title with count
        char title[64];
        snprintf(title, sizeof(title), LV_SYMBOL_AUDIO " Kenh FM (%d/%d)", 
                 radio_station_selected_index_ + 1, station_count);
        lv_label_set_text(radio_station_title_label_, title);
        
        // Update visible items
        for (int i = 0; i < RADIO_VISIBLE_ITEMS; i++) {
            int item_index = start_index + i;
            
            if (item_index >= station_count) {
                lv_label_set_text(radio_station_items_[i], "");
                lv_obj_set_style_bg_opa(radio_station_items_[i], LV_OPA_0, 0);
                continue;
            }
            
            // Extract display name from "KEY - Display Name" format
            std::string display_name = radio_stations_cache_[item_index];
            size_t dash_pos = display_name.find(" - ");
            if (dash_pos != std::string::npos && dash_pos + 3 < display_name.size()) {
                display_name = display_name.substr(dash_pos + 3);
            }
            
            // Format: index. name
            char item_text[128];
            snprintf(item_text, sizeof(item_text), "%d. %s", item_index + 1, display_name.c_str());
            lv_label_set_text(radio_station_items_[i], item_text);
            
            // Highlight selected item with orange
            if (item_index == radio_station_selected_index_) {
                lv_obj_set_style_bg_color(radio_station_items_[i], lv_color_hex(0xFF6B00), 0);
                lv_obj_set_style_bg_opa(radio_station_items_[i], LV_OPA_50, 0);
                lv_obj_set_style_text_color(radio_station_items_[i], lv_color_hex(0xffffff), 0);
                lv_obj_set_style_radius(radio_station_items_[i], 5, 0);
            } else {
                lv_obj_set_style_bg_opa(radio_station_items_[i], LV_OPA_0, 0);
                lv_obj_set_style_text_color(radio_station_items_[i], lv_color_hex(0xbdc3c7), 0);
            }
        }
        
        lvgl_port_unlock();
        
        // Reset timeout
        esp_timer_stop(radio_station_timeout_timer_);
        esp_timer_start_once(radio_station_timeout_timer_, 10000000);
    }

    void HideRadioStationSelection() {
        radio_station_selection_mode_ = false;
        
        if (radio_station_panel_) {
            lvgl_port_lock(0);
            lv_obj_add_flag(radio_station_panel_, LV_OBJ_FLAG_HIDDEN);
            // Show music panel again if radio is playing
            if (music_panel_ && radio_mode_) {
                lv_obj_remove_flag(music_panel_, LV_OBJ_FLAG_HIDDEN);
            }
            lvgl_port_unlock();
        }
        
        if (radio_station_timeout_timer_) {
            esp_timer_stop(radio_station_timeout_timer_);
        }
    }

    void RadioStationMoveUp() {
        if (!radio_station_selection_mode_) return;
        
        radio_station_selected_index_--;
        if (radio_station_selected_index_ < 0) {
            radio_station_selected_index_ = (int)radio_stations_cache_.size() - 1;
        }
        UpdateRadioStationSelection();
    }

    void RadioStationMoveDown() {
        if (!radio_station_selection_mode_) return;
        
        radio_station_selected_index_++;
        if (radio_station_selected_index_ >= (int)radio_stations_cache_.size()) {
            radio_station_selected_index_ = 0;
        }
        UpdateRadioStationSelection();
    }

    void RadioStationSelectCurrent() {
        if (!radio_station_selection_mode_) return;
        
        int idx = radio_station_selected_index_;
        if (idx < 0 || idx >= (int)radio_stations_cache_.size()) return;
        
        std::string station = radio_stations_cache_[idx];
        radio_station_index_ = idx;
        
        HideRadioStationSelection();
        
        // Play the selected station
        Radio_PlayStation(station.c_str());
        
        // Extract display name for notification
        std::string display_name = station;
        size_t dash_pos = display_name.find(" - ");
        if (dash_pos != std::string::npos && dash_pos + 3 < display_name.size()) {
            display_name = display_name.substr(dash_pos + 3);
        }
        GetDisplay()->ShowNotification("FM: " + display_name);
    }

    void ToggleMusicMode() {
        // Cycle: Idle → Music (SD) → FM Radio → Idle
        if (!music_mode_ && !radio_mode_) {
            // Idle → Music
            StartMusicMode();
        } else if (music_mode_ && !radio_mode_) {
            // Music → FM Radio
            StopMusicMode();
            StartRadioMode();
        } else if (radio_mode_) {
            // FM Radio → Idle
            StopRadioMode();
        }
    }

    void InitializeSt7789Display() {
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS;
        io_config.dc_gpio_num = DISPLAY_DC;
        io_config.spi_mode = 3;
        io_config.pclk_hz = 80 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io_));

        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RES;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io_, &panel_config, &panel_));
        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_));
        ESP_ERROR_CHECK(esp_lcd_panel_init(panel_));
        ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_, DISPLAY_SWAP_XY));
        ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y));
        ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_, true));

        display_ = new SpiLcdDisplay(panel_io_, panel_, DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, 
            DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }

    void InitializeMusicTools() {
        auto& mcp_server = McpServer::GetInstance();
        mcp_server.AddTool(
            "self.music.control",
            "Điều khiển phát nhạc MP3 từ thẻ nhớ SD. Hành động: play (bắt đầu phát nhạc), stop (dừng nhạc), next (bài tiếp theo), previous (bài trước), pause (tạm dừng), resume (tiếp tục phát)",
            PropertyList({
                Property("action", kPropertyTypeString),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                std::string action = properties["action"].value<std::string>();
                ESP_LOGI(TAG, "Music tool action: %s", action.c_str());
                
                if (action == "play") {
                    if (music_mode_) {
                        return std::string("Đang phát nhạc rồi");
                    }
                    // Called from LLM - delay music start until after LLM finishes speaking
                    StartMusicMode(true);  // from_llm = true
                    if (music_mode_) {
                        int count = mp3_player_ ? mp3_player_->GetPlaylistSize() : 0;
                        // Delay actual playback to avoid conflict with LLM speaking
                        xTaskCreate([](void* arg) {
                            auto* player = static_cast<Mp3Player*>(arg);
                            // Wait for LLM to finish speaking
                            vTaskDelay(pdMS_TO_TICKS(3000));
                            if (player) {
                                player->Next();
                            }
                            vTaskDelete(NULL);
                        }, "music_delay", 2048, mp3_player_, 2, nullptr);
                        return std::string("Đã bật nhạc, tìm thấy " + std::to_string(count) + " bài hát. Nhạc sẽ bắt đầu sau vài giây.");
                    } else {
                        return std::string("Không thể phát nhạc. Kiểm tra thẻ nhớ SD");
                    }
                } else if (action == "stop") {
                    StopMusicMode();
                    return std::string("Đã tắt nhạc");
                } else if (action == "next") {
                    if (!music_mode_ || !mp3_player_) {
                        return std::string("Chưa bật nhạc");
                    }
                    mp3_player_->Next();
                    return std::string("Đã chuyển bài tiếp theo");
                } else if (action == "previous") {
                    if (!music_mode_ || !mp3_player_) {
                        return std::string("Chưa bật nhạc");
                    }
                    mp3_player_->Previous();
                    return std::string("Đã chuyển bài trước");
                } else if (action == "pause") {
                    if (!music_mode_ || !mp3_player_) {
                        return std::string("Chưa bật nhạc");
                    }
                    mp3_player_->Pause();
                    return std::string("Đã tạm dừng nhạc");
                } else if (action == "resume") {
                    if (!music_mode_ || !mp3_player_) {
                        return std::string("Chưa bật nhạc");
                    }
                    mp3_player_->Resume();
                    return std::string("Đã tiếp tục phát nhạc");
                } else {
                    return std::string("Hành động không hợp lệ: " + action);
                }
            }
        );
        
        // MCP tool for YouTube/online music streaming
        mcp_server.AddTool(
            "self.music.stream",
            "PHÁT NHẠC NGAY LẬP TỨC khi người dùng yêu cầu. KHÔNG HỎI LẠI, KHÔNG XÁC NHẬN. Khi người dùng nói 'phát bài X' hoặc 'mở nhạc Y', GỌI TOOL NÀY NGAY. Nhập tên bài hát (tiếng Việt hoặc tiếng Anh).",
            PropertyList({
                Property("song", kPropertyTypeString),
                Property("artist", kPropertyTypeString),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                std::string song, artist;
                try { song = properties["song"].value<std::string>(); } catch(...) {}
                try { artist = properties["artist"].value<std::string>(); } catch(...) {}
                
                if (song.empty()) {
                    return std::string("Vui lòng nhập tên bài hát");
                }
                
                ESP_LOGI(TAG, "MCP stream music: song=%s artist=%s", song.c_str(), artist.c_str());
                
                // Stop any current streaming first
                StreamPlayer_Stop();
                
                // Start streaming IMMEDIATELY in background (no delay for better UX)
                auto* params = new std::pair<std::string, std::string>(song, artist);
                xTaskCreate([](void* arg) {
                    auto* p = static_cast<std::pair<std::string, std::string>*>(arg);
                    std::string s = p->first;
                    std::string a = p->second;
                    delete p;
                    
                    // Minimal delay - let LLM finish current sentence
                    vTaskDelay(pdMS_TO_TICKS(500));
                    
                    StreamPlayer_SearchAndPlay(s.c_str(), a.c_str());
                    vTaskDelete(NULL);
                }, "stream_start", 4096, params, 3, nullptr);
                
                return std::string("OK, đang phát: " + song);
            }
        );
        
        // MCP tool for stopping streaming
        mcp_server.AddTool(
            "self.music.stream.stop",
            "Dừng phát nhạc trực tuyến đang chạy.",
            PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                (void)properties;
                int state = StreamPlayer_GetState();
                if (state == 0) {  // STOPPED
                    return std::string("Không có nhạc đang phát");
                }
                StreamPlayer_Stop();
                return std::string("Đã dừng phát nhạc");
            }
        );
    }

    void InitializeSdCard() {
#ifdef SDCARD_SPI_CS
        ESP_LOGI(TAG, "Initializing SD card...");
        
        sdmmc_host_t host = SDSPI_HOST_DEFAULT();
        host.slot = SDCARD_SPI_HOST;
        
        spi_bus_config_t bus_cfg = {
            .mosi_io_num = SDCARD_SPI_MOSI,
            .miso_io_num = SDCARD_SPI_MISO,
            .sclk_io_num = SDCARD_SPI_SCLK,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 4000,
        };
        esp_err_t ret = spi_bus_initialize((spi_host_device_t)SDCARD_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to initialize SPI bus for SD card: %s", esp_err_to_name(ret));
            return;
        }

        sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
        slot_config.gpio_cs = SDCARD_SPI_CS;
        slot_config.host_id = (spi_host_device_t)SDCARD_SPI_HOST;

        esp_vfs_fat_sdmmc_mount_config_t mount_config = {
            .format_if_mount_failed = false,
            .max_files = 10,
            .allocation_unit_size = 0,
            .disk_status_check_enable = true,
        };
        sdmmc_card_t* card;
        ret = esp_vfs_fat_sdspi_mount(SDCARD_MOUNT_POINT, &host, &slot_config, &mount_config, &card);
        if (ret == ESP_OK) {
            sdmmc_card_print_info(stdout, card);
            ESP_LOGI(TAG, "SD card mounted at %s", SDCARD_MOUNT_POINT);
            sd_card_mounted_ = true;
            g_sd_mounted = true;
        } else {
            ESP_LOGW(TAG, "Failed to mount SD card: %s", esp_err_to_name(ret));
            sd_card_mounted_ = false;
            g_sd_mounted = false;
        }
#else
        ESP_LOGI(TAG, "SD card disabled");
        sd_card_mounted_ = false;
#endif
    }

public:
    XINGZHI_CUBE_1_54TFT_WIFI() :
        boot_button_(BOOT_BUTTON_GPIO),
        volume_up_button_(VOLUME_UP_BUTTON_GPIO),
        volume_down_button_(VOLUME_DOWN_BUTTON_GPIO) {
        InitializePowerManager();
        InitializePowerSaveTimer();
        InitializeSpi();
        InitializeButtons();
        InitializeSt7789Display();
        InitializeSdCard();
        InitializeMusicTools();
        InitializeRobotControl();
        RegisterWifiEventHandler();
        SetupNetworkCallback();
        GetBacklight()->RestoreBrightness();
        // Create music display overlay and start periodic update timer
        CreateMusicDisplay();
        StartMusicDisplayTimer();
    }

    void InitializeRobotControl() {
        ESP_LOGI(TAG, "Initializing robot control...");
        
        // Initialize servos
        robot_control_init();
        robot_initialized_ = true;
        
        // Start robot control task
        xTaskCreate([](void* arg) {
            robot_control_task(arg);
        }, "robot_task", 8192, nullptr, 5, nullptr);
        
        // Register MCP tools for LLM control
        register_robot_mcp_tools();
        
        ESP_LOGI(TAG, "Robot control initialized");
    }

    void StartRobotWebserver() {
        if (!robot_initialized_) {
            ESP_LOGW(TAG, "Robot not initialized, skipping webserver");
            return;
        }
        if (robot_webserver_) {
            ESP_LOGW(TAG, "Robot webserver already running");
            return;
        }
        robot_webserver_ = webserver_start();
        if (robot_webserver_) {
            ESP_LOGI(TAG, "Robot webserver started on port 80");
            // Initialize stream player for music streaming
            StreamPlayer_Init(GetAudioCodec());
            // Initialize SD player for web UI access
            if (sd_card_mounted_ && !mp3_player_) {
                mp3_player_ = new Mp3Player(GetAudioCodec());
                g_sd_player = mp3_player_;
                set_music_player_ptr(mp3_player_);
                mp3_player_->OnTrackChanged([this](const std::string& track) {
                    size_t pos = track.find_last_of("/");
                    std::string name = (pos != std::string::npos) ? track.substr(pos + 1) : track;
                    GetDisplay()->ShowNotification(name);
                });
                mp3_player_->OnPlaybackFinished([this]() {
                    // Auto-play next when finished
                    vTaskDelay(pdMS_TO_TICKS(300));
                    if (mp3_player_) mp3_player_->Next();
                });
                mp3_player_->OnError([this](const std::string& error) {
                    ESP_LOGE(TAG, "SD Music error: %s", error.c_str());
                });
                // Pre-scan playlist
                mp3_player_->ScanDirectory(SDCARD_MOUNT_POINT);
                ESP_LOGI(TAG, "SD player ready: %d files", mp3_player_->GetPlaylistSize());
            }
        }
    }

    static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                   int32_t event_id, void* event_data) {
        auto* self = static_cast<XINGZHI_CUBE_1_54TFT_WIFI*>(arg);
        if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
            ESP_LOGI(TAG, "WiFi connected, starting robot webserver...");
            self->StartRobotWebserver();
        }
    }

    void RegisterWifiEventHandler() {
        esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, this);
    }

    void SetupNetworkCallback() {
        // Get the existing callback if any
        auto parent_callback = network_event_callback_;
        
        // Set our callback that wraps the parent
        SetNetworkEventCallback([this, parent_callback](NetworkEvent event, const std::string& data) {
            // Call parent callback if it exists
            if (parent_callback) {
                parent_callback(event, data);
            }
            
            // Start webserver when network connected
            if (event == NetworkEvent::Connected) {
                ESP_LOGI(TAG, "Network callback: Connected event received, starting webserver...");
                StartRobotWebserver();
            }
        });
        
        // Also create a delayed task to ensure webserver starts even if callback missed
        xTaskCreate([](void* arg) {
            auto* self = static_cast<XINGZHI_CUBE_1_54TFT_WIFI*>(arg);
            // Wait up to 30 seconds for WiFi to get IP, polling every 2s
            for (int i = 0; i < 15; i++) {
                vTaskDelay(pdMS_TO_TICKS(2000));
                if (self->robot_webserver_) {
                    ESP_LOGI(TAG, "Webserver already started by event callback");
                    vTaskDelete(NULL);
                    return;
                }
                // Check if WiFi station has an IP address
                esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
                if (netif) {
                    esp_netif_ip_info_t ip_info;
                    if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK && ip_info.ip.addr != 0) {
                        ESP_LOGI(TAG, "Delayed check: WiFi has IP, starting webserver...");
                        self->StartRobotWebserver();
                        vTaskDelete(NULL);
                        return;
                    }
                }
            }
            ESP_LOGW(TAG, "Delayed webserver check: WiFi not connected after 30s, giving up");
            vTaskDelete(NULL);
        }, "web_start", 3072, this, 1, nullptr);
    }

    virtual AudioCodec* GetAudioCodec() override {
        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT, AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
        return &audio_codec;
    }

    virtual bool IsMusicPlaying() override {
        // Check SD card player
        if (mp3_player_ && mp3_player_->GetState() != Mp3PlayerState::STOPPED) return true;
        // Check streaming player  
        int stream_state = StreamPlayer_GetState();
        if (stream_state == 2 || stream_state == 3) return true;  // BUFFERING or PLAYING
        return false;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }
    
    virtual Backlight* GetBacklight() override {
        static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        return &backlight;
    }

    virtual bool GetBatteryLevel(int& level, bool& charging, bool& discharging) override {
        static bool last_discharging = false;
        charging = power_manager_->IsCharging();
        discharging = power_manager_->IsDischarging();
        if (discharging != last_discharging) {
            power_save_timer_->SetEnabled(discharging);
            last_discharging = discharging;
        }
        level = power_manager_->GetBatteryLevel();
        return true;
    }

    virtual void SetPowerSaveLevel(PowerSaveLevel level) override {
        if (level != PowerSaveLevel::LOW_POWER) {
            power_save_timer_->WakeUp();
        }
        WifiBoard::SetPowerSaveLevel(level);
    }
};

// C wrapper functions for Mp3Player (used by robot_control.c)
extern "C" {
    bool Mp3Player_Play(void* player, const char* path) {
        auto* mp3_player = static_cast<Mp3Player*>(player);
        return mp3_player->Play(std::string(path));
    }
    
    int Mp3Player_GetState(void* player) {
        auto* mp3_player = static_cast<Mp3Player*>(player);
        return static_cast<int>(mp3_player->GetState());
    }
    
    void Mp3Player_Stop(void* player) {
        auto* mp3_player = static_cast<Mp3Player*>(player);
        mp3_player->Stop();
    }
}

// ====== SD Player C interface for webserver ======

extern "C" {

void SdPlayer_Init(void) {
    // Called from board constructor after SD init
    // g_sd_player set by board code
}

int SdPlayer_Play(const char* filepath) {
    if (!g_sd_player) return 0;
    
    // Abort any ongoing TTS before playing music
    auto& app = Application::GetInstance();
    auto state = app.GetDeviceState();
    if (state == kDeviceStateSpeaking) {
        app.AbortSpeaking(kAbortReasonNone);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    // Also stop streaming if active
    StreamPlayer_Stop();
    
    return g_sd_player->Play(std::string(filepath)) ? 1 : 0;
}

void SdPlayer_Stop(void) {
    if (g_sd_player) g_sd_player->Stop();
}

void SdPlayer_Pause(void) {
    if (g_sd_player) g_sd_player->Pause();
}

void SdPlayer_Resume(void) {
    if (g_sd_player) g_sd_player->Resume();
}

void SdPlayer_Next(void) {
    if (!g_sd_player) return;
    // Abort TTS
    auto& app = Application::GetInstance();
    if (app.GetDeviceState() == kDeviceStateSpeaking) {
        app.AbortSpeaking(kAbortReasonNone);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    StreamPlayer_Stop();
    g_sd_player->Next();
}

void SdPlayer_Previous(void) {
    if (!g_sd_player) return;
    auto& app = Application::GetInstance();
    if (app.GetDeviceState() == kDeviceStateSpeaking) {
        app.AbortSpeaking(kAbortReasonNone);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    StreamPlayer_Stop();
    g_sd_player->Previous();
}

int SdPlayer_GetState(void) {
    if (!g_sd_player) return 0;
    return static_cast<int>(g_sd_player->GetState());
}

const char* SdPlayer_GetCurrentTrack(void) {
    if (!g_sd_player) return "";
    static std::string s_track;
    s_track = g_sd_player->GetCurrentTrack();
    return s_track.c_str();
}

int SdPlayer_GetCurrentIndex(void) {
    if (!g_sd_player) return -1;
    return g_sd_player->GetCurrentIndex();
}

int SdPlayer_GetPlaylistSize(void) {
    if (!g_sd_player) return 0;
    return g_sd_player->GetPlaylistSize();
}

int SdPlayer_ListDir(const char* path, sd_file_entry_t* out_entries, int max_entries) {
    if (!g_sd_mounted || !out_entries || max_entries <= 0) return 0;
    
    DIR* dir = opendir(path);
    if (!dir) return 0;
    
    int count = 0;
    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr && count < max_entries) {
        if (entry->d_name[0] == '.') continue;
        
        char full_path[512];
        snprintf(full_path, sizeof(full_path), "%s/%s", path, entry->d_name);
        
        struct stat st;
        if (stat(full_path, &st) != 0) continue;
        
        strncpy(out_entries[count].name, entry->d_name, sizeof(out_entries[count].name) - 1);
        out_entries[count].name[sizeof(out_entries[count].name) - 1] = '\0';
        strncpy(out_entries[count].path, full_path, sizeof(out_entries[count].path) - 1);
        out_entries[count].path[sizeof(out_entries[count].path) - 1] = '\0';
        out_entries[count].is_dir = S_ISDIR(st.st_mode) ? 1 : 0;
        out_entries[count].size = (long)st.st_size;
        count++;
    }
    closedir(dir);
    return count;
}

int SdPlayer_IsSdMounted(void) {
    return g_sd_mounted ? 1 : 0;
}

const char* SdPlayer_GetPlaylistEntry(int index) {
    if (!g_sd_player || index < 0 || index >= g_sd_player->GetPlaylistSize()) return NULL;
    if (!g_playlist_cache) return NULL;
    if (index >= (int)g_playlist_cache->size()) return NULL;
    return (*g_playlist_cache)[index].c_str();
}

int SdPlayer_ScanAndBuildPlaylist(const char* path) {
    if (!g_sd_player || !g_sd_mounted) return 0;
    int count = g_sd_player->ScanDirectory(std::string(path));
    return count;
}

int SdPlayer_IsAnyMusicPlaying(void) {
    // Check SD player
    if (g_sd_player && g_sd_player->GetState() != Mp3PlayerState::STOPPED) return 1;
    // Check stream player
    int stream_state = StreamPlayer_GetState();
    if (stream_state == 2 || stream_state == 3) return 1;  // BUFFERING or PLAYING
    return 0;
}

void SdPlayer_SetRepeatMode(int mode) {
    if (g_sd_player) {
        g_sd_player->SetRepeatMode(static_cast<Mp3RepeatMode>(mode));
    }
}

int SdPlayer_GetRepeatMode(void) {
    if (g_sd_player) {
        return static_cast<int>(g_sd_player->GetRepeatMode());
    }
    return 0;
}

// ====== Chat AI Integration ======
#define MAX_CHAT_MESSAGES 20
#define MAX_MESSAGE_LENGTH 512

struct ChatMessage {
    char role[16];        // "user" or "assistant"
    char content[MAX_MESSAGE_LENGTH];
    int64_t timestamp;
};

static ChatMessage s_chat_history[MAX_CHAT_MESSAGES];
static int s_chat_count = 0;
static int s_chat_write_idx = 0;
static char s_last_ai_response[MAX_MESSAGE_LENGTH] = {0};
static char s_chat_history_json[8192] = {0};  // Large buffer for JSON array
static SemaphoreHandle_t s_chat_mutex = NULL;

static void init_chat_storage() {
    if (!s_chat_mutex) {
        s_chat_mutex = xSemaphoreCreateMutex();
    }
}

void add_chat_message(const char* role, const char* content) {
    init_chat_storage();
    if (xSemaphoreTake(s_chat_mutex, portMAX_DELAY)) {
        ChatMessage* msg = &s_chat_history[s_chat_write_idx];
        strncpy(msg->role, role, sizeof(msg->role) - 1);
        msg->role[sizeof(msg->role) - 1] = '\0';
        strncpy(msg->content, content, sizeof(msg->content) - 1);
        msg->content[sizeof(msg->content) - 1] = '\0';
        msg->timestamp = esp_timer_get_time() / 1000000;  // seconds
        
        s_chat_write_idx = (s_chat_write_idx + 1) % MAX_CHAT_MESSAGES;
        if (s_chat_count < MAX_CHAT_MESSAGES) {
            s_chat_count++;
        }
        
        xSemaphoreGive(s_chat_mutex);
        ESP_LOGI(TAG, "Chat message added: [%s] %s", role, content);
    }
}

void set_robot_emoji(const char* emotion) {
    if (!emotion || strlen(emotion) == 0) return;
    auto& board = Board::GetInstance();
    auto display = board.GetDisplay();
    if (display) {
        auto& app = Application::GetInstance();
        std::string emo(emotion);
        app.Schedule([display, emo]() {
            display->SetEmotion(emo.c_str());
            ESP_LOGI(TAG, "Emoji set: %s", emo.c_str());
        });
    }
}

void send_text_to_ai(const char* text) {
    if (!text || strlen(text) == 0) {
        ESP_LOGW(TAG, "send_text_to_ai: empty text");
        return;
    }
    
    // Add user message to history
    add_chat_message("user", text);
    
    // Get display and show user message on robot screen
    auto& board = Board::GetInstance();
    auto display = board.GetDisplay();
    auto& app = Application::GetInstance();
    
    // Play popup sound to acknowledge message received
    app.PlaySound(Lang::Sounds::OGG_POPUP);
    
    // Display user message on robot screen
    app.Schedule([display, user_msg = std::string(text)]() {
        display->SetChatMessage("user", user_msg.c_str());
        ESP_LOGI(TAG, "Displayed user message on robot: %s", user_msg.c_str());
    });
    
    // Try to send text to AI server using wake word trick
    // Server will treat text as STT result and process with LLM
    std::string text_str(text);
    if (app.SendChatText(text_str)) {
        ESP_LOGI(TAG, "Text sent to AI server via wake word trick, waiting for TTS response...");
        // Server response will be handled by OnIncomingJson callback in application.cc
        // which will update display with TTS text, play audio, and save to chat history
        return;
    }
    
    // Fall back to demo mode if server is not connected
    ESP_LOGW(TAG, "Server not connected, using demo mode response");
    
    // Generate demo response after 1.5 seconds
    app.Schedule([display, user_msg = std::string(text)]() {
        vTaskDelay(pdMS_TO_TICKS(1500));
        
        // Convert to lowercase for matching
        std::string lower_msg = user_msg;
        for (auto& c : lower_msg) {
            c = tolower(c);
        }
        
        // Generate demo response based on user input (with length limit)
        std::string ai_response;
        
        // Date/Time related
        if (lower_msg.find("ngày") != std::string::npos || lower_msg.find("date") != std::string::npos ||
            lower_msg.find("hôm nay") != std::string::npos || lower_msg.find("nay") != std::string::npos) {
            // Get current date
            time_t now;
            struct tm timeinfo;
            time(&now);
            localtime_r(&now, &timeinfo);
            char date_str[64];
            strftime(date_str, sizeof(date_str), "%d/%m/%Y", &timeinfo);
            ai_response = std::string("Hôm nay là ") + date_str;
        } else if (lower_msg.find("giờ") != std::string::npos || lower_msg.find("time") != std::string::npos ||
                   lower_msg.find("mấy giờ") != std::string::npos) {
            // Get current time
            time_t now;
            struct tm timeinfo;
            time(&now);
            localtime_r(&now, &timeinfo);
            char time_str[32];
            strftime(time_str, sizeof(time_str), "%H:%M:%S", &timeinfo);
            ai_response = std::string("Bây giờ là ") + time_str;
        }
        // Greetings
        else if (lower_msg.find("hi") != std::string::npos || lower_msg.find("hello") != std::string::npos || 
                 lower_msg.find("xin chào") != std::string::npos || lower_msg.find("chào") != std::string::npos) {
            ai_response = "Xin chào! Tôi có thể giúp gì?";
        } 
        // Name
        else if (lower_msg.find("name") != std::string::npos || lower_msg.find("tên") != std::string::npos) {
            ai_response = "Tôi là Xiaozhi Robot!";
        } 
        // Help
        else if (lower_msg.find("help") != std::string::npos || lower_msg.find("giúp") != std::string::npos) {
            ai_response = "Tôi có thể: trả lời ngày giờ, trò chuyện...";
        } 
        // Dance
        else if (lower_msg.find("dance") != std::string::npos || lower_msg.find("nhảy") != std::string::npos) {
            ai_response = "OK! Để tôi nhảy cho bạn xem!";
        }
        // Weather
        else if (lower_msg.find("thời tiết") != std::string::npos || lower_msg.find("weather") != std::string::npos) {
            ai_response = "Tính năng thời tiết cần server AI.";
        }
        // Default - Note: Demo mode, connect to AI server for full features
        else {
            // Keep response short to avoid buffer issues
            if (user_msg.length() > 30) {
                ai_response = "Demo: Kết nối server AI để trò chuyện!";
            } else {
                ai_response = "Demo: \"" + user_msg + "\" - Kết nối server để AI trả lời!";
            }
        }
        
        // Ensure response is not too long (max 100 chars for safety)
        if (ai_response.length() > 100) {
            ai_response = ai_response.substr(0, 97) + "...";
        }
        
        ESP_LOGI(TAG, "Demo AI response: %s", ai_response.c_str());
        
        // Play notification sound
        Application::GetInstance().PlaySound(Lang::Sounds::OGG_SUCCESS);
        
        // Display AI response on robot screen
        display->SetChatMessage("assistant", ai_response.c_str());
        
        // Add to chat history
        add_chat_message("assistant", ai_response.c_str());
        
        // Update last response for web UI
        strncpy(s_last_ai_response, ai_response.c_str(), sizeof(s_last_ai_response) - 1);
        s_last_ai_response[sizeof(s_last_ai_response) - 1] = '\0';
        
        ESP_LOGI(TAG, "Demo AI response displayed successfully");
    });
}

const char* get_chat_history_json(void) {
    init_chat_storage();
    if (xSemaphoreTake(s_chat_mutex, pdMS_TO_TICKS(100))) {
        s_chat_history_json[0] = '\0';
        strcat(s_chat_history_json, "[");
        
        int start_idx = (s_chat_count < MAX_CHAT_MESSAGES) ? 0 : s_chat_write_idx;
        for (int i = 0; i < s_chat_count; i++) {
            int idx = (start_idx + i) % MAX_CHAT_MESSAGES;
            ChatMessage* msg = &s_chat_history[idx];
            
            if (i > 0) strcat(s_chat_history_json, ",");
            
            char entry[1536];
            // Escape quotes in content
            char escaped_content[MAX_MESSAGE_LENGTH * 2];
            const char* src = msg->content;
            char* dst = escaped_content;
            while (*src && (dst - escaped_content) < (int)sizeof(escaped_content) - 2) {
                if (*src == '"' || *src == '\\') {
                    *dst++ = '\\';
                }
                *dst++ = *src++;
            }
            *dst = '\0';
            
            snprintf(entry, sizeof(entry), "{\"role\":\"%s\",\"content\":\"%s\",\"timestamp\":%lld}",
                     msg->role, escaped_content, msg->timestamp);
            strcat(s_chat_history_json, entry);
        }
        strcat(s_chat_history_json, "]");
        
        xSemaphoreGive(s_chat_mutex);
    } else {
        strcpy(s_chat_history_json, "[]");
    }
    return s_chat_history_json;
}

const char* get_last_ai_response(void) {
    return s_last_ai_response;
}

} // extern "C"

DECLARE_BOARD(XINGZHI_CUBE_1_54TFT_WIFI);
