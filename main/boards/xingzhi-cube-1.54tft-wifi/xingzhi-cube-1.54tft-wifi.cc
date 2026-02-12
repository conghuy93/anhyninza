#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/lcd_display.h"
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

#include "mcp_server.h"
#include <driver/rtc_io.h>
#include <esp_sleep.h>
#include <esp_vfs_fat.h>
#include <sdmmc_cmd.h>
#include <driver/sdspi_host.h>
#include "mp3_player.h"

// Robot control
#include "robot_control.h"
#include "webserver.h"
#include "robot_mcp_controller.h"
#include <esp_event.h>
#include <esp_wifi.h>
#include <esp_netif.h>

// Forward declaration for C function in robot_control.c
extern "C" void set_music_player_ptr(void* mp3_player_ptr);

#define TAG "XINGZHI_CUBE_1_54TFT_WIFI"

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
    bool sd_card_mounted_ = false;
    httpd_handle_t robot_webserver_ = nullptr;
    bool robot_initialized_ = false;

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

        power_save_timer_ = new PowerSaveTimer(-1, 60, 300);
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

        boot_button_.OnDoubleClick([this]() {
            power_save_timer_->WakeUp();
            ToggleMusicMode();
        });

        volume_up_button_.OnClick([this]() {
            power_save_timer_->WakeUp();
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() + 10;
            if (volume > 100) {
                volume = 100;
            }
            codec->SetOutputVolume(volume);
            GetDisplay()->ShowNotification(Lang::Strings::VOLUME + std::to_string(volume));
        });

        volume_up_button_.OnLongPress([this]() {
            power_save_timer_->WakeUp();
            if (music_mode_ && mp3_player_) {
                mp3_player_->Next();
                return;
            }
            GetAudioCodec()->SetOutputVolume(100);
            GetDisplay()->ShowNotification(Lang::Strings::MAX_VOLUME);
        });

        volume_down_button_.OnClick([this]() {
            power_save_timer_->WakeUp();
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() - 10;
            if (volume < 0) {
                volume = 0;
            }
            codec->SetOutputVolume(volume);
            GetDisplay()->ShowNotification(Lang::Strings::VOLUME + std::to_string(volume));
        });

        volume_down_button_.OnLongPress([this]() {
            power_save_timer_->WakeUp();
            if (music_mode_ && mp3_player_) {
                mp3_player_->Previous();
                return;
            }
            GetAudioCodec()->SetOutputVolume(0);
            GetDisplay()->ShowNotification(Lang::Strings::MUTED);
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

    void ToggleMusicMode() {
        if (music_mode_) {
            StopMusicMode();
        } else {
            StartMusicMode();
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
        } else {
            ESP_LOGW(TAG, "Failed to mount SD card: %s", esp_err_to_name(ret));
            sd_card_mounted_ = false;
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

DECLARE_BOARD(XINGZHI_CUBE_1_54TFT_WIFI);
