/*
 * Robot MCP Controller - LLM Tools for Robot Control
 * Provides voice/LLM control of the 4-servo robot
 */

#ifndef ROBOT_MCP_CONTROLLER_H
#define ROBOT_MCP_CONTROLLER_H

#include "robot_control.h"
#include "mcp_server.h"
#include "webserver.h"
#include <esp_log.h>
#include <string>
#include <cmath>

#define TAG_MCP "robot_mcp"

// MCP movement task parameter structure
typedef struct {
    int speed;
    int turn;
    int duration;
    bool is_roll; // true=roll, false=walk
} move_task_params_t;

// Dance task parameter structure
typedef struct {
    int duration;
} dance_task_params_t;

// Background walk/roll task (non-blocking)
static void move_task_func(void* pvParameters) {
    move_task_params_t* params = (move_task_params_t*)pvParameters;
    
    // CRITICAL: Clear manual_mode so robot_control_task processes joystick values
    // (tilt/dance/servo_direct_write set manual_mode=true and never clear it)
    set_manual_mode(false);
    
    if (params->is_roll) {
        ninja_set_roll();
    } else {
        ninja_set_walk();
    }
    
    control_state_t* state = get_control_state();
    state->j_y = params->speed;
    state->j_x = params->turn;
    
    vTaskDelay(pdMS_TO_TICKS(params->duration));
    
    state->j_x = 0;
    state->j_y = 0;
    
    ESP_LOGI(TAG_MCP, "Move task done: %s speed=%d turn=%d dur=%dms",
             params->is_roll ? "roll" : "walk", params->speed, params->turn, params->duration);
    
    free(params);
    vTaskDelete(NULL);
}

// Dance task function (runs in separate FreeRTOS task to avoid blocking LLM)
static void dance_task_func(void* pvParameters) {
    dance_task_params_t* params = (dance_task_params_t*)pvParameters;
    int duration = params->duration;
    free(params);
    
    ESP_LOGI(TAG_MCP, "Dance task started: %dms", duration);
    
    // Start playing "em" song
    bool music_started = ninja_play_music("em");
    if (!music_started) {
        ESP_LOGW(TAG_MCP, "Failed to start music, dancing without it");
    }
    
    ninja_set_walk();
    
    int elapsed = 0;
    while (elapsed < duration) {
        ninja_tilt_left();
        vTaskDelay(pdMS_TO_TICKS(500));
        elapsed += 500;
        
        if (elapsed >= duration) break;
        
        ninja_tilt_right();
        vTaskDelay(pdMS_TO_TICKS(500));
        elapsed += 500;
        
        if (elapsed >= duration) break;
        
        control_state_t* state = get_control_state();
        state->j_y = 60;
        state->j_x = 50;
        vTaskDelay(pdMS_TO_TICKS(800));
        state->j_x = -50;
        vTaskDelay(pdMS_TO_TICKS(800));
        state->j_x = 0;
        state->j_y = 0;
        elapsed += 1600;
    }
    
    // Wait for music to finish (max 30 more seconds)
    if (music_started) {
        ESP_LOGI(TAG_MCP, "Dance complete, waiting for music to finish...");
        int music_wait = 0;
        while (ninja_is_music_playing() && music_wait < 30000) {
            vTaskDelay(pdMS_TO_TICKS(500));
            music_wait += 500;
        }
        ninja_stop_music();
    }
    
    // CRITICAL: Clear manual_mode before go_home so robot_control_task resumes
    set_manual_mode(false);
    go_home();
    ESP_LOGI(TAG_MCP, "Dance task finished");
    
    vTaskDelete(NULL);
}

// Helper function to register robot MCP tools
static void register_robot_mcp_tools() {
    auto& mcp_server = McpServer::GetInstance();
    
    // Walk command
    mcp_server.AddTool(
        "self.robot.walk",
        "Di chuyển robot đi bộ. Tốc độ từ -100 (lùi) đến 100 (tiến). Rẽ từ -100 (trái) đến 100 (phải). Thời gian tính bằng ms.",
        PropertyList({
            Property("speed", kPropertyTypeInteger),
            Property("turn", kPropertyTypeInteger),
            Property("duration", kPropertyTypeInteger),
        }),
        [](const PropertyList& properties) -> ReturnValue {
            int speed = 70;
            int turn = 0;
            int duration = 2000;
            
            try { speed = properties["speed"].value<int>(); } catch(...) {}
            try { turn = properties["turn"].value<int>(); } catch(...) {}
            try { duration = properties["duration"].value<int>(); } catch(...) {}
            
            if (speed < -100) speed = -100;
            if (speed > 100) speed = 100;
            if (turn < -100) turn = -100;
            if (turn > 100) turn = 100;
            if (duration < 100) duration = 100;
            if (duration > 10000) duration = 10000;
            
            move_task_params_t* params = (move_task_params_t*)malloc(sizeof(move_task_params_t));
            if (!params) return std::string("Lỗi bộ nhớ");
            params->speed = speed;
            params->turn = turn;
            params->duration = duration;
            params->is_roll = false;
            
            xTaskCreate(move_task_func, "walk_task", 3072, params, 5, NULL);
            
            ESP_LOGI(TAG_MCP, "Walk: speed=%d turn=%d duration=%dms (non-blocking)", speed, turn, duration);
            return std::string("Robot đang đi tốc độ " + std::to_string(speed) + " trong " + std::to_string(duration) + "ms");
        }
    );
    
    // Roll command
    mcp_server.AddTool(
        "self.robot.roll",
        "Chuyển robot sang chế độ bánh xe và lăn. Tốc độ từ -100 đến 100. Thời gian tính bằng ms.",
        PropertyList({
            Property("speed", kPropertyTypeInteger),
            Property("turn", kPropertyTypeInteger),
            Property("duration", kPropertyTypeInteger),
        }),
        [](const PropertyList& properties) -> ReturnValue {
            int speed = 50;
            int turn = 0;
            int duration = 2000;
            
            try { speed = properties["speed"].value<int>(); } catch(...) {}
            try { turn = properties["turn"].value<int>(); } catch(...) {}
            try { duration = properties["duration"].value<int>(); } catch(...) {}
            
            if (speed < -100) speed = -100;
            if (speed > 100) speed = 100;
            if (turn < -100) turn = -100;
            if (turn > 100) turn = 100;
            if (duration < 100) duration = 100;
            if (duration > 10000) duration = 10000;
            
            move_task_params_t* params = (move_task_params_t*)malloc(sizeof(move_task_params_t));
            if (!params) return std::string("Lỗi bộ nhớ");
            params->speed = speed;
            params->turn = turn;
            params->duration = duration;
            params->is_roll = true;
            
            xTaskCreate(move_task_func, "roll_task", 3072, params, 5, NULL);
            
            ESP_LOGI(TAG_MCP, "Roll: speed=%d turn=%d duration=%dms (non-blocking)", speed, turn, duration);
            return std::string("Robot đang lăn tốc độ " + std::to_string(speed) + " trong " + std::to_string(duration) + "ms");
        }
    );
    
    // Home command
    mcp_server.AddTool(
        "self.robot.home",
        "Đưa robot về vị trí đứng thẳng ban đầu.",
        PropertyList(),
        [](const PropertyList& properties) -> ReturnValue {
            (void)properties;
            set_manual_mode(false);
            go_home();
            ESP_LOGI(TAG_MCP, "Robot returned home");
            return std::string("Robot đã về vị trí ban đầu");
        }
    );
    
    // Tilt command
    mcp_server.AddTool(
        "self.robot.tilt",
        "Nghiêng robot sang trái hoặc phải. direction: left hoặc right.",
        PropertyList({
            Property("direction", kPropertyTypeString),
        }),
        [](const PropertyList& properties) -> ReturnValue {
            std::string direction = "left";
            try { direction = properties["direction"].value<std::string>(); } catch(...) {}
            
            if (direction == "left") {
                ninja_tilt_left();
                ESP_LOGI(TAG_MCP, "Tilted left");
            } else {
                ninja_tilt_right();
                ESP_LOGI(TAG_MCP, "Tilted right");
            }
            
            // Hold tilt for 1 second then clear manual_mode so subsequent MCP commands work
            vTaskDelay(pdMS_TO_TICKS(1000));
            set_manual_mode(false);
            
            return std::string(direction == "left" ? "Robot đã nghiêng trái" : "Robot đã nghiêng phải");
        }
    );
    
    // Mode command
    mcp_server.AddTool(
        "self.robot.mode",
        "Chuyển chế độ robot. mode: walk (đi bộ) hoặc roll (lăn).",
        PropertyList({
            Property("mode", kPropertyTypeString),
        }),
        [](const PropertyList& properties) -> ReturnValue {
            std::string mode = "walk";
            try { mode = properties["mode"].value<std::string>(); } catch(...) {}
            
            if (mode == "roll") {
                ninja_set_roll();
                ESP_LOGI(TAG_MCP, "Mode: roll");
                return std::string("Robot chuyển sang chế độ lăn");
            } else {
                ninja_set_walk();
                ESP_LOGI(TAG_MCP, "Mode: walk");
                return std::string("Robot chuyển sang chế độ đi bộ");
            }
        }
    );
    
    // Play recorded action
    mcp_server.AddTool(
        "self.robot.play",
        "Phát động tác đã ghi. slot: 0, 1 hoặc 2.",
        PropertyList({
            Property("slot", kPropertyTypeInteger),
        }),
        [](const PropertyList& properties) -> ReturnValue {
            int slot = 0;
            try { slot = properties["slot"].value<int>(); } catch(...) {}
            
            if (slot < 0) slot = 0;
            if (slot > 2) slot = 2;
            
            action_slot_t* actions = get_action_slot(slot);
            if (actions && actions->count > 0) {
                play_action(slot);
                ESP_LOGI(TAG_MCP, "Playing action slot %d (%d actions)", slot + 1, actions->count);
                return std::string("Đã phát " + std::to_string(actions->count) + " động tác từ slot " + std::to_string(slot + 1));
            } else {
                ESP_LOGW(TAG_MCP, "Action slot %d is empty", slot + 1);
                return std::string("Slot " + std::to_string(slot + 1) + " trống");
            }
        }
    );
    
    // Dance command
    mcp_server.AddTool(
        "self.robot.dance",
        "Robot nhảy múa! Sẽ phát nhạc 'em' trong khi nhảy. Lệnh trả về ngay, robot nhảy nền.",
        PropertyList({
            Property("duration", kPropertyTypeInteger),
        }),
        [](const PropertyList& properties) -> ReturnValue {
            int duration = 5000;
            try { duration = properties["duration"].value<int>(); } catch(...) {}
            
            if (duration < 1000) duration = 1000;
            if (duration > 30000) duration = 30000;
            
            // Allocate params for the task
            dance_task_params_t* params = (dance_task_params_t*)malloc(sizeof(dance_task_params_t));
            if (!params) {
                return std::string("Lỗi: không đủ bộ nhớ");
            }
            params->duration = duration;
            
            // Create background task so LLM doesn't block
            BaseType_t ret = xTaskCreate(dance_task_func, "dance_task", 4096, params, 5, NULL);
            if (ret != pdPASS) {
                free(params);
                return std::string("Lỗi: không tạo được task nhảy");
            }
            
            ESP_LOGI(TAG_MCP, "Dance task created for %dms (non-blocking)", duration);
            return std::string("Robot đang nhảy " + std::to_string(duration) + "ms với nhạc 'em'. Tôi sẵn sàng nhận lệnh tiếp.");
        }
    );
    
    // Status command
    mcp_server.AddTool(
        "self.robot.status",
        "Lấy trạng thái hiện tại của robot.",
        PropertyList(),
        [](const PropertyList& properties) -> ReturnValue {
            (void)properties;
            
            robot_mode_t mode = get_robot_mode();
            
            std::string status = "Robot: ";
            status += (mode == MODE_WALK) ? "Đi bộ" : "Lăn";
            status += ", Servo: 4";
            
            int total_actions = 0;
            for (int i = 0; i < 3; i++) {
                action_slot_t* slot = get_action_slot(i);
                if (slot) total_actions += slot->count;
            }
            status += ", Động tác: " + std::to_string(total_actions);
            
            return status;
        }
    );
    
    // LED color control
    mcp_server.AddTool(
        "self.robot.led.color",
        "Đặt màu LED RGB. r: 0-255, g: 0-255, b: 0-255. Màu phổ biến: đỏ(255,0,0), xanh lá(0,255,0), xanh dương(0,0,255), vàng(255,255,0), tím(255,0,255), trắng(255,255,255).",
        PropertyList({
            Property("r", kPropertyTypeInteger),
            Property("g", kPropertyTypeInteger),
            Property("b", kPropertyTypeInteger),
        }),
        [](const PropertyList& properties) -> ReturnValue {
            int r = 255, g = 255, b = 255;
            
            try { r = properties["r"].value<int>(); } catch(...) {}
            try { g = properties["g"].value<int>(); } catch(...) {}
            try { b = properties["b"].value<int>(); } catch(...) {}
            
            if (r < 0) r = 0;
            if (r > 255) r = 255;
            if (g < 0) g = 0;
            if (g > 255) g = 255;
            if (b < 0) b = 0;
            if (b > 255) b = 255;
            
            ninja_led_set_color(r, g, b);
            ESP_LOGI(TAG_MCP, "LED color: R=%d G=%d B=%d", r, g, b);
            return std::string("Đã đặt LED màu RGB(" + std::to_string(r) + "," + std::to_string(g) + "," + std::to_string(b) + ")");
        }
    );
    
    // LED brightness control
    mcp_server.AddTool(
        "self.robot.led.brightness",
        "Đặt độ sáng LED. brightness: 0-100 (0=tắt, 100=sáng nhất).",
        PropertyList({
            Property("brightness", kPropertyTypeInteger),
        }),
        [](const PropertyList& properties) -> ReturnValue {
            int brightness = 50;
            
            try { brightness = properties["brightness"].value<int>(); } catch(...) {}
            
            if (brightness < 0) brightness = 0;
            if (brightness > 100) brightness = 100;
            
            ninja_led_set_brightness(brightness);
            ESP_LOGI(TAG_MCP, "LED brightness: %d%%", brightness);
            return std::string("Đã đặt độ sáng LED " + std::to_string(brightness) + "%");
        }
    );
    
    // LED mode control
    mcp_server.AddTool(
        "self.robot.led.mode",
        "Đặt chế độ hiệu ứng LED. mode: off, solid, rainbow, breathing, chase, blink, strobe, fade, comet, sparkle, theater, music (nhảy theo nhạc).",
        PropertyList({
            Property("mode", kPropertyTypeString),
        }),
        [](const PropertyList& properties) -> ReturnValue {
            std::string mode_str = "solid";
            try { mode_str = properties["mode"].value<std::string>(); } catch(...) {}
            
            led_mode_t mode = LED_MODE_SOLID;
            if (mode_str == "off") mode = LED_MODE_OFF;
            else if (mode_str == "solid") mode = LED_MODE_SOLID;
            else if (mode_str == "rainbow") mode = LED_MODE_RAINBOW;
            else if (mode_str == "breathing") mode = LED_MODE_BREATHING;
            else if (mode_str == "chase") mode = LED_MODE_CHASE;
            else if (mode_str == "blink") mode = LED_MODE_BLINK;
            else if (mode_str == "strobe") mode = LED_MODE_STROBE;
            else if (mode_str == "fade") mode = LED_MODE_FADE;
            else if (mode_str == "comet") mode = LED_MODE_COMET;
            else if (mode_str == "sparkle") mode = LED_MODE_SPARKLE;
            else if (mode_str == "theater") mode = LED_MODE_THEATER_CHASE;
            else if (mode_str == "music") mode = LED_MODE_MUSIC_REACTIVE;
            
            ninja_led_set_mode(mode);
            ESP_LOGI(TAG_MCP, "LED mode: %s", mode_str.c_str());
            return std::string("Đã đặt LED chế độ " + mode_str);
        }
    );
    
    // LED off
    mcp_server.AddTool(
        "self.robot.led.off",
        "Tắt toàn bộ LED.",
        PropertyList(),
        [](const PropertyList& properties) -> ReturnValue {
            (void)properties;
            ninja_led_off();
            ESP_LOGI(TAG_MCP, "LED off");
            return std::string("Đã tắt LED");
        }
    );
    
    // ==================== ALARM SCHEDULING ====================
    
    // Set alarm or schedule
    mcp_server.AddTool(
        "self.alarm.set",
        "Đặt báo thức hoặc lịch nhắc nhở.\n"
        "Args:\n"
        "  type: 'alarm' (phát nhạc) hoặc 'schedule' (gửi tin nhắn AI)\n"
        "  hour: 0-23\n"
        "  minute: 0-59\n"
        "  repeat: 'once' (1 lần), 'daily' (hàng ngày), 'weekday' (thứ 2-6), 'weekend' (thứ 7, CN)\n"
        "  message: nội dung nhắc nhở (cho schedule)\n"
        "  music: đường dẫn nhạc SD (cho alarm, vd: /sdcard/music/song.mp3)\n"
        "  music_name: tên bài hát (cho alarm)\n"
        "Return: Thông báo kết quả",
        PropertyList({
            Property("type", kPropertyTypeString),
            Property("hour", kPropertyTypeInteger),
            Property("minute", kPropertyTypeInteger),
            Property("repeat", kPropertyTypeString, std::string("once")),
            Property("message", kPropertyTypeString, std::string("")),
            Property("music", kPropertyTypeString, std::string("")),
            Property("music_name", kPropertyTypeString, std::string(""))
        }),
        [](const PropertyList& properties) -> ReturnValue {
            std::string type = "schedule";
            int hour = 0;
            int minute = 0;
            std::string repeat = "once";
            std::string message = "";
            std::string music = "";
            std::string music_name = "";
            
            try { type = properties["type"].value<std::string>(); } catch(...) {}
            try { hour = properties["hour"].value<int>(); } catch(...) {}
            try { minute = properties["minute"].value<int>(); } catch(...) {}
            try { repeat = properties["repeat"].value<std::string>(); } catch(...) {}
            try { message = properties["message"].value<std::string>(); } catch(...) {}
            try { music = properties["music"].value<std::string>(); } catch(...) {}
            try { music_name = properties["music_name"].value<std::string>(); } catch(...) {}
            
            if (type != "alarm" && type != "schedule") {
                return std::string("Lỗi: type phải là 'alarm' hoặc 'schedule'");
            }
            
            int result = alarm_add_from_mcp(
                type.c_str(),
                hour,
                minute,
                repeat.c_str(),
                message.empty() ? nullptr : message.c_str(),
                music.empty() ? nullptr : music.c_str(),
                music_name.empty() ? nullptr : music_name.c_str()
            );
            
            if (result >= 0) {
                ESP_LOGI(TAG_MCP, "Alarm set: %s %02d:%02d %s", type.c_str(), hour, minute, repeat.c_str());
                char buf[128];
                snprintf(buf, sizeof(buf), "Đã đặt %s lúc %02d:%02d (%s)",
                         type == "alarm" ? "báo thức" : "lịch nhắc",
                         hour, minute, repeat.c_str());
                return std::string(buf);
            } else if (result == -2) {
                return std::string("Lỗi: giờ/phút không hợp lệ (0-23, 0-59)");
            } else if (result == -3) {
                return std::string("Lỗi: hệ thống bận");
            } else if (result == -4) {
                return std::string("Lỗi: đã đầy (tối đa 10 báo thức)");
            } else {
                return std::string("Lỗi: không thể đặt báo thức");
            }
        }
    );
    
    // Play saved action (already exists, but add alias for "dance from slot")
    mcp_server.AddTool(
        "self.robot.play_saved",
        "Nhảy động tác đã lưu trong slot. Đây là alias của self.robot.play.\n"
        "slot: 0, 1 hoặc 2 (tương ứng slot 1, 2, 3)",
        PropertyList({
            Property("slot", kPropertyTypeInteger)
        }),
        [](const PropertyList& properties) -> ReturnValue {
            int slot = 0;
            try { slot = properties["slot"].value<int>(); } catch(...) {}
            
            if (slot < 0) slot = 0;
            if (slot > 2) slot = 2;
            
            action_slot_t* actions = get_action_slot(slot);
            if (actions && actions->count > 0) {
                play_action(slot);
                ESP_LOGI(TAG_MCP, "Playing saved action slot %d (%d steps)", slot, actions->count);
                return std::string("Đang phát " + std::to_string(actions->count) + " động tác từ slot " + std::to_string(slot + 1));
            } else {
                ESP_LOGW(TAG_MCP, "Saved action slot %d is empty", slot);
                return std::string("Slot " + std::to_string(slot + 1) + " chưa có động tác");
            }
        }
    );
    
    // Play music from SD card
    mcp_server.AddTool(
        "self.music.play",
        "Phát nhạc MP3 từ thẻ nhớ SD card. LLM có thể mở bài hát theo yêu cầu của người dùng.\n"
        "song_name: Tên file nhạc (có thể có hoặc không có .mp3). VD: 'happy', 'birthday.mp3', 'music/pop/song1'\n"
        "File phải nằm trong /sdcard/. VD: /sdcard/happy.mp3, /sdcard/music/song.mp3\n"
        "⚠️ NÊN GỌI self.music.search TRƯỚC để tìm file chính xác!\n"
        "Dùng khi người dùng nói 'mở bài...', 'phát nhạc...', 'play song...', 'bật nhạc...'",
        PropertyList({
            Property("song_name", kPropertyTypeString)
        }),
        [](const PropertyList& properties) -> ReturnValue {
            std::string song_name;
            try { 
                song_name = properties["song_name"].value<std::string>(); 
            } catch(...) {
                ESP_LOGW(TAG_MCP, "Music play: missing song_name parameter");
                return std::string("❌ Thiếu tên bài hát! Vui lòng cung cấp song_name.");
            }
            
            if (song_name.empty()) {
                ESP_LOGW(TAG_MCP, "Music play: empty song_name");
                return std::string("❌ Tên bài hát không được rỗng!");
            }
            
            ESP_LOGI(TAG_MCP, "Playing music: %s", song_name.c_str());
            bool success = ninja_play_music(song_name.c_str());
            
            if (success) {
                return std::string("🎵 Đang phát: " + song_name);
            } else {
                return std::string("❌ Không tìm thấy bài hát: " + song_name + ". Hãy dùng self.music.search trước!");
            }
        }
    );
    
    // Search music files in SD card
    mcp_server.AddTool(
        "self.music.search",
        "Tìm kiếm file nhạc MP3 trong thẻ nhớ SD card theo từ khóa (không phân biệt hoa thường).\n"
        "keyword: Từ khóa tìm kiếm (VD: 'Xuân', 'happy', 'birthday')\n"
        "Trả về danh sách file phù hợp. LLM nên gọi hàm này TRƯỚC self.music.play để tìm tên file chính xác.\n"
        "Dùng khi người dùng muốn tìm hoặc phát nhạc nhưng chưa biết tên file chính xác.",
        PropertyList({
            Property("keyword", kPropertyTypeString)
        }),
        [](const PropertyList& properties) -> ReturnValue {
            std::string keyword;
            try { 
                keyword = properties["keyword"].value<std::string>(); 
            } catch(...) {
                ESP_LOGW(TAG_MCP, "Music search: missing keyword parameter");
                return std::string("❌ Thiếu từ khóa! Vui lòng cung cấp keyword.");
            }
            
            if (keyword.empty()) {
                ESP_LOGW(TAG_MCP, "Music search: empty keyword");
                return std::string("❌ Từ khóa không được rỗng!");
            }
            
            // Search with buffer size 2048, max 10 results
            char result_buffer[2048];
            int found_count = search_music_files_in_sdcard(keyword.c_str(), result_buffer, sizeof(result_buffer), 10);
            
            if (found_count == 0) {
                ESP_LOGI(TAG_MCP, "Music search '%s': no results", keyword.c_str());
                return std::string("🔍 Không tìm thấy file nào chứa từ khóa: " + keyword);
            }
            
            ESP_LOGI(TAG_MCP, "Music search '%s': found %d files", keyword.c_str(), found_count);
            
            // Parse results (newline-separated paths)
            std::string response = "🔍 Tìm thấy " + std::to_string(found_count) + " file:\n";
            char* line = strtok(result_buffer, "\n");
            int idx = 1;
            while (line != NULL && idx <= found_count) {
                response += std::to_string(idx) + ". " + std::string(line) + "\n";
                line = strtok(NULL, "\n");
                idx++;
            }
            response += "\nDùng self.music.play với đường dẫn đầy đủ từ kết quả trên.";
            
            return response;
        }
    );
    
    // Stop music playback
    mcp_server.AddTool(
        "self.music.stop",
        "Dừng phát nhạc hiện tại. Dùng khi người dùng nói 'dừng nhạc', 'tắt nhạc', 'stop music'.",
        PropertyList(),
        [](const PropertyList& properties) -> ReturnValue {
            (void)properties;
            
            bool was_playing = ninja_is_music_playing();
            ninja_stop_music();
            
            ESP_LOGI(TAG_MCP, "Music stopped");
            if (was_playing) {
                return std::string("🔇 Đã dừng phát nhạc");
            } else {
                return std::string("⏹️ Không có nhạc nào đang phát");
            }
        }
    );
    
    // Play dead command
    mcp_server.AddTool(
        "self.robot.play_dead",
        "Robot giả chết! Hiện emoji shock 😱, lùi 2 bước, nghiêng trái + xoay chân LF 360° theo 1 chiều trong 2s, nằm xuống (LL=155°) rồi về home. Dùng khi người dùng nói 'giả chết', 'chết đi', 'play dead', 'ngã xuống'.",
        PropertyList(),
        [](const PropertyList& properties) -> ReturnValue {
            (void)properties;
            
            // play_dead_task is defined in webserver.c, we call it via the HTTP endpoint pattern
            // But since we can access servo/emoji directly, create the task inline
            xTaskCreate([](void* arg) {
                (void)arg;
                control_state_t *state = get_control_state();
                
                // Step 1: Shocked emoji
                set_robot_emoji("shocked");
                vTaskDelay(pdMS_TO_TICKS(500));
                
                // Step 2: Walk backward
                set_manual_mode(false);
                ninja_set_walk();
                state->j_y = -80;
                state->j_x = 0;
                vTaskDelay(pdMS_TO_TICKS(2000));
                state->j_y = 0;
                state->j_x = 0;
                vTaskDelay(pdMS_TO_TICKS(300));
                
                // Step 3: Tilt left, rotate LF to 60 deg over 2s, then go home
                set_manual_mode(true);
                servo_attach(SERVO_CH_LEFT_LEG);
                servo_attach(SERVO_CH_RIGHT_LEG);
                servo_attach(SERVO_CH_LEFT_FOOT);
                servo_write(SERVO_CH_LEFT_LEG, 100);
                servo_write(SERVO_CH_RIGHT_LEG, 175);
                vTaskDelay(pdMS_TO_TICKS(300));
                
                // Smoothly rotate LF to 60 deg over 2s
                calibration_t *cal = get_calibration();
                int lf_start = cal->lf_neutral;
                int lf_target = 60;
                int rotation_steps = 20;
                int step_delay = 2000 / rotation_steps;
                for (int i = 1; i <= rotation_steps; i++) {
                    int angle = lf_start + (lf_target - lf_start) * i / rotation_steps;
                    servo_write(SERVO_CH_LEFT_FOOT, angle);
                    vTaskDelay(pdMS_TO_TICKS(step_delay));
                }
                set_manual_mode(false);
                go_home();
                vTaskDelay(pdMS_TO_TICKS(500));
                
                // Step 4: Lie down (tilt left with LL=155)
                set_manual_mode(true);
                servo_attach(SERVO_CH_LEFT_LEG);
                servo_attach(SERVO_CH_RIGHT_LEG);
                servo_write(SERVO_CH_LEFT_LEG, 155);
                servo_write(SERVO_CH_RIGHT_LEG, 175);
                vTaskDelay(pdMS_TO_TICKS(2000));
                
                set_manual_mode(false);
                go_home();
                vTaskDelay(pdMS_TO_TICKS(500));
                set_robot_emoji("neutral");
                
                vTaskDelete(NULL);
            }, "mcp_playdead", 3072, NULL, 5, NULL);
            
            ESP_LOGI(TAG_MCP, "Play Dead triggered");
            return std::string("Robot dang gia chet! Shock -> Lui 2 buoc -> Nghieng trai + LF 60 (2s) -> Home -> Nam xuong -> Home");
        }
    );
    
    // QR Code display - show control/settings page QR
    mcp_server.AddTool(
        "self.qr_code",
        "Hien thi ma QR len man hinh de nguoi dung quet truy cap trang web. "
        "page='control': trang dieu khien robot. "
        "page='settings': trang cai dat/calibration. "
        "Dung khi nguoi dung noi 'hien thi QR', 'vao trang dieu khien', 'show QR code', 'trang cai dat'.",
        PropertyList({Property("page", kPropertyTypeString)}),
        [](const PropertyList& properties) -> ReturnValue {
            std::string page = properties["page"].value<std::string>();
            show_qr_for_page(page.c_str());
            
            ESP_LOGI(TAG_MCP, "QR code shown for page: %s", page.c_str());
            return std::string("Da hien thi ma QR trang ") + page + " tren man hinh (30 giay). Nguoi dung co the quet bang dien thoai.";
        }
    );
    
    ESP_LOGI(TAG_MCP, "Robot MCP tools registered (19 tools: 12 robot + 3 alarm/slot/playdead + 3 music + 1 qr)");
}

#endif // ROBOT_MCP_CONTROLLER_H
