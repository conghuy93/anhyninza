/*
 * Robot MCP Controller - LLM Tools for Robot Control
 * Provides voice/LLM control of the 4-servo robot
 */

#ifndef ROBOT_MCP_CONTROLLER_H
#define ROBOT_MCP_CONTROLLER_H

#include "robot_control.h"
#include "mcp_server.h"
#include <esp_log.h>
#include <string>

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
                return std::string("Robot đã nghiêng trái");
            } else {
                ninja_tilt_right();
                ESP_LOGI(TAG_MCP, "Tilted right");
                return std::string("Robot đã nghiêng phải");
            }
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
    
    ESP_LOGI(TAG_MCP, "Robot MCP tools registered (8 tools)");
}

#endif // ROBOT_MCP_CONTROLLER_H
