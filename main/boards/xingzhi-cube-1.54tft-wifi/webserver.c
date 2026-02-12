/*
 * Otto Ninja Robot - ESP-IDF Version
 * Web Server Implementation
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_http_server.h"
#include "webserver.h"
#include "robot_control.h"
#include <cJSON.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "webserver";

// Background task types for non-blocking operations
typedef enum {
    WEB_TASK_TEST_BOTH_FEET = 0,
    WEB_TASK_COMBO1 = 1,
    WEB_TASK_COMBO2 = 2,
    WEB_TASK_TURN_LEFT = 3,
    WEB_TASK_TURN_RIGHT = 4,
    WEB_TASK_RHYTHM_LEFT = 5,
    WEB_TASK_RHYTHM_RIGHT = 6,
    WEB_TASK_WALK_PHASE = 7,
    WEB_TASK_WAVE_RIGHT_LEG = 8,
    WEB_TASK_WAVE_LEFT_LEG = 9
} web_task_type_t;

typedef struct {
    web_task_type_t task_type;
    int param1;  // For speed values
    int param2;  // For additional params
} web_task_params_t;

// Background task for long-running web operations
static void web_action_task(void *pvParameters) {
    web_task_params_t *params = (web_task_params_t *)pvParameters;
    calibration_t *cal = get_calibration();
    
    switch (params->task_type) {
        case WEB_TASK_TEST_BOTH_FEET:
            ESP_LOGI(TAG, "🔧 Background task: TEST_BOTH_FEET");
            test_both_feet();
            break;
            
        case WEB_TASK_COMBO1:
            ESP_LOGI(TAG, "🔧 Background task: COMBO1 (LF speed=%dms)", params->param1);
            cal->combo_lf_speed = params->param1;
            ninja_combo1();
            break;
            
        case WEB_TASK_COMBO2:
            ESP_LOGI(TAG, "🔧 Background task: COMBO2 (RF speed=%dms)", params->param1);
            cal->combo_rf_speed = params->param1;
            ninja_combo2();
            break;
            
        case WEB_TASK_TURN_LEFT: {
            int speed_ms = params->param1;
            robot_mode_t mode = get_robot_mode();
            control_state_t *state = get_control_state();
            cal->turn_left_speed = speed_ms;
            
            ESP_LOGI(TAG, "🔧 Background task: TURN_LEFT (Mode=%s, Speed=%dms)", 
                     mode == MODE_WALK ? "WALK" : "ROLL", speed_ms);
            
            if (mode == MODE_WALK) {
                ninja_tilt_left();
                vTaskDelay(pdMS_TO_TICKS(speed_ms));
                
                int lf_angle = cal->lf_neutral + cal->lffwrs;
                servo_direct_write(SERVO_CH_LEFT_FOOT, lf_angle);
                vTaskDelay(pdMS_TO_TICKS(speed_ms));
                servo_direct_write(SERVO_CH_LEFT_FOOT, cal->lf_neutral);
                
                state->manual_mode = false;
                go_home();
            } else {
                state->manual_mode = false;  // Ensure control task processes joystick
                state->j_x = -75;
                state->j_y = -64;
                vTaskDelay(pdMS_TO_TICKS(speed_ms));
                state->j_x = 0;
                state->j_y = 0;
            }
            break;
        }
            
        case WEB_TASK_TURN_RIGHT: {
            int speed_ms = params->param1;
            robot_mode_t mode = get_robot_mode();
            control_state_t *state = get_control_state();
            cal->turn_right_speed = speed_ms;
            
            ESP_LOGI(TAG, "🔧 Background task: TURN_RIGHT (Mode=%s, Speed=%dms)", 
                     mode == MODE_WALK ? "WALK" : "ROLL", speed_ms);
            
            if (mode == MODE_WALK) {
                ninja_tilt_right();
                vTaskDelay(pdMS_TO_TICKS(speed_ms));
                
                int rf_angle = cal->rf_neutral - cal->rffwrs;
                servo_direct_write(SERVO_CH_RIGHT_FOOT, rf_angle);
                vTaskDelay(pdMS_TO_TICKS(speed_ms));
                servo_direct_write(SERVO_CH_RIGHT_FOOT, cal->rf_neutral);
                
                state->manual_mode = false;
                go_home();
            } else {
                state->manual_mode = false;  // Ensure control task processes joystick
                state->j_x = 51;
                state->j_y = -81;
                vTaskDelay(pdMS_TO_TICKS(speed_ms));
                state->j_x = 0;
                state->j_y = 0;
            }
            break;
        }
        
        case WEB_TASK_RHYTHM_LEFT:
            ESP_LOGI(TAG, "🔧 Background task: LEFT LEG RHYTHM (34-45-65 x3)");
            left_leg_rhythm();
            break;
            
        case WEB_TASK_RHYTHM_RIGHT:
            ESP_LOGI(TAG, "🔧 Background task: RIGHT LEG RHYTHM (140-150-170 x3)");
            right_leg_rhythm();
            break;
            
        case WEB_TASK_WALK_PHASE: {
            int combo = params->param1;
            ESP_LOGI(TAG, "🔧 Background task: WALK COMBO %d", combo);
            if (combo == 1) {
                ninja_walk_combo_123();  // Phases 1+2+3
            } else if (combo == 2) {
                ninja_walk_combo_345();  // Phases 3+4+5
            }
            break;
        }
            
        case WEB_TASK_WAVE_RIGHT_LEG: {
            ESP_LOGI(TAG, "🦵 Background task: WAVE RIGHT LEG");
            control_state_t *st = get_control_state();
            st->manual_mode = true;
            servo_attach(SERVO_CH_RIGHT_LEG);
            for (int i = 0; i < 3; i++) {
                servo_write(SERVO_CH_RIGHT_LEG, 135);
                vTaskDelay(pdMS_TO_TICKS(200));
                servo_write(SERVO_CH_RIGHT_LEG, 155);
                vTaskDelay(pdMS_TO_TICKS(200));
                servo_write(SERVO_CH_RIGHT_LEG, 180);
                vTaskDelay(pdMS_TO_TICKS(200));
            }
            break;
        }

        case WEB_TASK_WAVE_LEFT_LEG: {
            ESP_LOGI(TAG, "🦵 Background task: WAVE LEFT LEG");
            control_state_t *st2 = get_control_state();
            st2->manual_mode = true;
            servo_attach(SERVO_CH_LEFT_LEG);
            for (int i = 0; i < 3; i++) {
                servo_write(SERVO_CH_LEFT_LEG, 10);
                vTaskDelay(pdMS_TO_TICKS(200));
                servo_write(SERVO_CH_LEFT_LEG, 30);
                vTaskDelay(pdMS_TO_TICKS(200));
                servo_write(SERVO_CH_LEFT_LEG, 75);
                vTaskDelay(pdMS_TO_TICKS(200));
            }
            break;
        }

        default:
            ESP_LOGW(TAG, "Unknown web task type: %d", params->task_type);
            break;
    }
    
    free(params);
    vTaskDelete(NULL);
}

// Timing for auto-save
static uint32_t last_setting_change = 0;
static bool settings_changed = false;
#define SAVE_DELAY_MS 1000

// HTML Web UI (embedded)
static const char html_content[] = 
"<!DOCTYPE html>"
"<html>"
"<head>"
"<meta charset=\"UTF-8\">"
"<meta name=\"viewport\" content=\"width=device-width,initial-scale=1.0,maximum-scale=1.0,user-scalable=no\">"
"<title>Otto Ninja Control</title>"
"<style>"
"*{margin:0;padding:0;box-sizing:border-box;-webkit-tap-highlight-color:transparent}"
"body{font-family:Arial,sans-serif;background:#2c3e50;color:#ecf0f1;padding:10px;text-align:center;min-height:100vh;display:flex;flex-direction:column;justify-content:flex-start;overflow-x:hidden}"
"h1{margin:8px 0 8px;font-size:1.3em}"
".tabs{display:flex;gap:5px;max-width:400px;margin:0 auto 8px auto}"
".tab{flex:1;padding:10px;font-size:0.95em;font-weight:bold;border:none;border-radius:8px 8px 0 0;cursor:pointer;color:white;background:#34495e}"
".tab.active{background:#3498db}"
".tab-content{display:none}"
".tab-content.active{display:block}"
".joystick-container{width:min(200px,50vw);height:min(200px,50vw);margin:5px auto;position:relative;background:#34495e;border-radius:50%;touch-action:none;box-shadow:0 4px 8px rgba(0,0,0,0.3)}"
".joystick{width:25%;height:25%;background:#3498db;border-radius:50%;position:absolute;top:50%;left:50%;transform:translate(-50%,-50%);cursor:pointer;box-shadow:0 2px 6px rgba(0,0,0,0.4)}"
".buttons{display:grid;grid-template-columns:1fr 1fr;gap:6px;max-width:400px;margin:8px auto;padding:0 10px}"
".btn{padding:10px 8px;font-size:0.85em;font-weight:bold;border:none;border-radius:8px;cursor:pointer;color:white;touch-action:manipulation;min-height:40px}"
".btn-walk{background:#27ae60}"
".btn-roll{background:#e67e22}"
".btn-left{background:#3498db}"
".btn-right{background:#9b59b6}"
".values{margin:5px;font-family:monospace;font-size:0.8em;padding:5px;background:#34495e;border-radius:6px;display:inline-block}"
".calibration{max-width:500px;margin:10px auto;background:#34495e;padding:10px;border-radius:8px;max-height:60vh;overflow-y:auto}"
".calibration h2{font-size:0.95em;margin-bottom:6px;color:#3498db;border-bottom:2px solid #2c3e50;padding-bottom:5px}"
".cal-section{margin-bottom:10px}"
".cal-section h3{font-size:0.85em;margin-bottom:5px;color:#e67e22}"
".cal-item{margin-bottom:6px}"
".cal-item label{display:block;margin-bottom:2px;font-size:0.7em;color:#bdc3c7}"
".cal-row{display:flex;gap:6px;align-items:center}"
".cal-row input[type=\"range\"]{flex:1;height:5px;background:#2c3e50;border-radius:3px;outline:none}"
".cal-row input[type=\"range\"]::-webkit-slider-thumb{-webkit-appearance:none;width:16px;height:16px;background:#3498db;border-radius:50%;cursor:pointer}"
".cal-value{min-width:30px;text-align:center;font-family:monospace;background:#2c3e50;padding:3px;border-radius:4px;font-size:0.7em}"
".btn-apply{width:100%;padding:10px;font-size:0.9em;font-weight:bold;border:none;border-radius:6px;cursor:pointer;background:#27ae60;color:white;margin-top:8px}"
".btn-apply:active{background:#229954}"
".btn.clicked{transform:scale(0.95);opacity:0.7}"
".btn.success{background:#27ae60!important;transition:background 0.2s}"
"@keyframes pulse{0%{transform:scale(1)}50%{transform:scale(0.95)}100%{transform:scale(1)}}"
".btn.running{animation:pulse 0.5s infinite;opacity:0.8}"
".led-mode-btn.active{box-shadow:0 0 10px #fff,inset 0 0 5px rgba(255,255,255,0.3);transform:scale(1.05)}"
".switch{position:relative;display:inline-block;width:50px;height:24px}"
".switch input{opacity:0;width:0;height:0}"
".slider{position:absolute;cursor:pointer;top:0;left:0;right:0;bottom:0;background-color:#c0392b;transition:0.3s;border-radius:24px}"
".slider:before{position:absolute;content:'';height:18px;width:18px;left:3px;bottom:3px;background-color:white;transition:0.3s;border-radius:50%}"
"input:checked+.slider{background-color:#27ae60}"
"input:checked+.slider:before{transform:translateX(26px)}"
"@media(max-width:480px){h1{font-size:1.1em}.btn{font-size:0.8em;padding:8px 6px;min-height:35px}}"
"</style>"
"</head>"
"<body>"
"<h1>🤖 miniZninja</h1>"
"<div class=\"tabs\">"
"<button class=\"tab active\" onclick=\"switchTab(1)\">🎮 Điều Khiển</button>"
"<button class=\"tab\" onclick=\"switchTab(2)\">⚙️ Cài Đặt</button>"
"</div>"
"<div id=\"tab1\" class=\"tab-content active\">"
"<div style=\"background:#1a252f;padding:8px;border-radius:6px;margin:5px 0 10px 0;font-size:11px;line-height:1.5;\">"
"<div style=\"color:#3498db;font-weight:bold;margin-bottom:4px;\">📱 Hướng dẫn điều khiển:</div>"
"<div style=\"color:#bdc3c7;margin-bottom:2px;\">• <b>Joystick giữa</b>: Kéo để di chuyển tiến/lùi/rẽ trái/phải</div>"
"<div style=\"color:#bdc3c7;margin-bottom:2px;\">• <b>↺/↻ (bên cạnh)</b>: Quay trái/phải tại chỗ (kéo thanh dọc để điều tốc độ)</div>"
"<div style=\"color:#bdc3c7;margin-bottom:2px;\">• <b>🚶 WALK / ⚙️ ROLL</b>: Chuyển chế độ đi bộ/lăn xe</div>"
"<div style=\"color:#bdc3c7;margin-bottom:2px;\">• <b>👈 L.Arm / 👉 R.Arm</b>: Vẫy tay trái/phải (giữ để vẫy liên tục)</div>"
"<div style=\"color:#bdc3c7;margin-bottom:2px;\">• <b>🏠 HOME</b>: Về tư thế ban đầu, <b>❌ Off</b>: Tắt chế độ thủ công</div>"
"<div style=\"color:#e67e22;font-size:10px;margin-top:4px;\">💡 Combo 1/2 thực hiện động tác nghiêng + vẫy chân + xoay bánh xe</div>"
"</div>"
"<div style=\"display:flex;align-items:flex-start;justify-content:center;gap:8px;margin:5px auto\">"  
"<div style=\"text-align:center\">"  
"<button class=\"btn\" style=\"background:#e91e63;width:45px;height:45px;border-radius:50%;font-size:1.2em;margin-bottom:5px\" id=\"btnTurnLeft\">↺</button>"  
"<div style=\"background:#2c3e50;padding:5px;border-radius:6px;width:45px\">"  
"<label style=\"color:#e91e63;font-size:0.6em;display:block;margin-bottom:2px\">⏱️<span id=\"turnLeftSpeedDisp\">500</span></label>"  
"<input type=\"range\" min=\"100\" max=\"1500\" value=\"500\" id=\"turnLeftSpeed\" style=\"width:100%;height:55px;writing-mode:vertical-lr;direction:rtl\">"  
"</div>"  
"</div>"  
"<div class=\"joystick-container\" id=\"joystickZone\">"  
"<div class=\"joystick\" id=\"joystick\"></div>"  
"</div>"  
"<div style=\"text-align:center\">"  
"<button class=\"btn\" style=\"background:#9c27b0;width:45px;height:45px;border-radius:50%;font-size:1.2em;margin-bottom:5px\" id=\"btnTurnRight\">↻</button>"  
"<div style=\"background:#2c3e50;padding:5px;border-radius:6px;width:45px\">"  
"<label style=\"color:#9c27b0;font-size:0.6em;display:block;margin-bottom:2px\">⏱️<span id=\"turnRightSpeedDisp\">500</span></label>"  
"<input type=\"range\" min=\"100\" max=\"1500\" value=\"500\" id=\"turnRightSpeed\" style=\"width:100%;height:55px;writing-mode:vertical-lr;direction:rtl\">"  
"</div>"  
"</div>"  
"</div>"  
"<div class=\"values\">X:<span id=\"xVal\">0</span> Y:<span id=\"yVal\">0</span></div>"
"<div style=\"background:#2c3e50;padding:8px;border-radius:8px;margin:8px auto;max-width:400px\">"
"<div style=\"display:flex;align-items:center;gap:8px;margin-bottom:6px\">"
"<label style=\"color:#1abc9c;font-size:0.7em;white-space:nowrap\">⏱️ Duration:</label>"
"<input type=\"range\" id=\"moveDuration\" min=\"0\" max=\"30000\" step=\"500\" value=\"3000\" style=\"flex:1\">"
"<span style=\"color:#1abc9c;font-size:0.75em;min-width:50px\" id=\"moveDurationDisp\">3.0s</span>"
"</div>"
"<div style=\"font-size:0.6em;color:#7f8c8d\">0=Không giới hạn (giữ nút) | 0.5s-30s = Tự động dừng sau thời gian</div>"
"</div>"
"<div class=\"buttons\">"
"<button class=\"btn btn-walk\" id=\"btnY\">🚶 WALK</button>"
"<button class=\"btn btn-roll\" id=\"btnX\">⚙️ ROLL</button>"
"<button class=\"btn btn-left\" id=\"btnA\">👈 L.Arm</button>"
"<button class=\"btn btn-right\" id=\"btnB\">👉 R.Arm</button>"
"</div>"
"<div class=\"buttons\">"
"<button class=\"btn\" style=\"background:#16a085\" id=\"btnTestFwd\">⬆️ Forward</button>"
"<button class=\"btn\" style=\"background:#c0392b\" id=\"btnTestBack\">⬇️ Back</button>"
"<button class=\"btn\" style=\"background:#3498db\" id=\"btnHome\">🏠 HOME</button>"
"<button class=\"btn\" style=\"background:#e74c3c\" id=\"btnManualOff\">❌ Off</button>"
"</div>"
"<div class=\"buttons\">"
"<button class=\"btn\" style=\"background:#e91e63\" id=\"btnTiltLeft\">⬅️ Tilt L</button>"
"<button class=\"btn\" style=\"background:#9c27b0\" id=\"btnTiltRight\">➡️ Tilt R</button>"
"</div>"
"<div class=\"calibration\" style=\"margin-top:10px;background:#1a2530;border:2px solid #9c27b0;padding:10px\">"
"<h2 style=\"color:#9c27b0;font-size:0.95em;margin-bottom:8px\">🔄 Quay Tại Chỗ (RF+LF Spin)</h2>"
"<div style=\"background:#2c3e50;padding:10px;border-radius:8px\">"
"<div style=\"display:flex;align-items:center;gap:10px;margin-bottom:8px\">"
"<label style=\"color:#e91e63;font-size:0.8em;min-width:50px\">↺ Trái</label>"
"<input type=\"range\" min=\"-100\" max=\"100\" value=\"0\" id=\"spinSpeed\" style=\"flex:1\">"
"<label style=\"color:#9c27b0;font-size:0.8em;min-width:50px;text-align:right\">Phải ↻</label>"
"</div>"
"<div style=\"text-align:center;color:#3498db;font-size:0.85em;margin-bottom:8px\">Tốc độ: <span id=\"spinSpeedDisp\" style=\"font-weight:bold\">0</span></div>"
"<button class=\"btn\" style=\"background:#9c27b0;width:100%;font-size:0.9em\" id=\"btnSpin\">🔄 SPIN</button>"
"<div style=\"font-size:0.6em;color:#7f8c8d;margin-top:5px;text-align:center\">Kéo thanh trượt sang trái (âm) để quay trái, sang phải (dương) để quay phải. Nhấn SPIN để dừng.</div>"
"</div>"
"</div>"
"<div class=\"buttons\">"
"<button class=\"btn\" style=\"background:#8e44ad\" id=\"btnRhythmL\">💃 L.Leg (34-45-65)</button>"
"<button class=\"btn\" style=\"background:#2980b9\" id=\"btnRhythmR\">💃 R.Leg (140-150-170)</button>"
"</div>"
"<div class=\"calibration\" style=\"margin-top:10px;background:#1a2530;border:2px solid #3498db;padding:10px\">"
"<h2 style=\"color:#3498db;font-size:0.95em;margin-bottom:8px\">🚶 Walk Combos (Combo bước đi)</h2>"
"<div class=\"buttons\">"
"<button class=\"btn\" style=\"background:#1abc9c;font-size:0.9em;padding:12px\" id=\"btnCombo123\">🎯 Combo 1-2-3<br><small style=\"font-size:0.7em;opacity:0.8\">(Tilt R → RF Fwd → Tilt L)</small></button>"
"<button class=\"btn\" style=\"background:#2ecc71;font-size:0.9em;padding:12px\" id=\"btnCombo345\">🎯 Combo 3-4-5<br><small style=\"font-size:0.7em;opacity:0.8\">(Tilt L → LF Fwd → Neutral)</small></button>"
"</div>"
"</div>"
"<div class=\"calibration\" style=\"margin-top:10px;background:#1a2530;border:2px solid #ff5722;padding:10px\">"
"<h2 style=\"color:#ff5722;font-size:0.95em;margin-bottom:8px\">🎯 COMBO (Tốc độ xoay chân)</h2>"
"<div style=\"display:flex;gap:10px\">"
"<div style=\"flex:1;background:#2c3e50;padding:8px;border-radius:8px;text-align:center\">"
"<label style=\"color:#ff5722;font-size:0.7em;display:block;margin-bottom:3px\">LF: <span id=\"comboLfSpeedDisp\">1000</span>ms</label>"
"<input type=\"range\" min=\"200\" max=\"3000\" value=\"1000\" id=\"comboLfSpeed\" style=\"width:100%\">"
"<button class=\"btn\" style=\"background:#ff5722;font-size:0.8em;padding:10px;margin-top:5px;width:100%\" id=\"btnCombo1\">🎯 COMBO 1</button>"
"</div>"
"<div style=\"flex:1;background:#2c3e50;padding:8px;border-radius:8px;text-align:center\">"
"<label style=\"color:#00bcd4;font-size:0.7em;display:block;margin-bottom:3px\">RF: <span id=\"comboRfSpeedDisp\">1000</span>ms</label>"
"<input type=\"range\" min=\"200\" max=\"3000\" value=\"1000\" id=\"comboRfSpeed\" style=\"width:100%\">"
"<button class=\"btn\" style=\"background:#00bcd4;font-size:0.8em;padding:10px;margin-top:5px;width:100%\" id=\"btnCombo2\">🎯 COMBO 2</button>"
"</div>"
"</div>"
"</div>"
"<div class=\"calibration\" style=\"margin-top:8px;background:#1a2530;border:2px solid #e91e63;padding:10px\">"
"<h2 style=\"color:#e91e63;font-size:0.95em;margin-bottom:8px\">🦵 Wave Legs (Vẫy chân)</h2>"
"<div class=\"buttons\">"
"<button class=\"btn\" style=\"background:#e91e63;font-size:0.9em;padding:12px\" id=\"btnWaveRight\">🦵 Vẫy chân phải<br><small style=\"font-size:0.7em;opacity:0.8\">(Nghiêng trái → Vẫy RL x3)</small></button>"
"<button class=\"btn\" style=\"background:#9c27b0;font-size:0.9em;padding:12px\" id=\"btnWaveLeft\">🦵 Vẫy chân trái<br><small style=\"font-size:0.7em;opacity:0.8\">(Nghiêng phải → Vẫy LL x3)</small></button>"
"</div>"
"</div>"
"<div class=\"calibration\" style=\"margin-top:8px;background:#1a2530;border:2px solid #e67e22;padding:10px\">"
"<h2 style=\"color:#e67e22;font-size:1em;margin-bottom:8px\">🎬 Recording</h2>"
"<div style=\"display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-bottom:8px\">"
"<div style=\"background:#2c3e50;padding:8px;border-radius:8px;text-align:center\">"
"<div style=\"font-weight:bold;color:#3498db;font-size:0.9em;margin-bottom:4px\">Slot 1</div>"
"<div style=\"font-size:0.75em;color:#7f8c8d;margin-bottom:6px\" id=\"slot1Info\">-</div>"
"<button class=\"btn\" style=\"background:#e74c3c;font-size:1.2em;padding:10px 6px;width:100%;margin-bottom:4px;min-height:44px;border-radius:8px\" id=\"btnRec1\">🔴 REC</button>"
"<button class=\"btn\" style=\"background:#7f8c8d;font-size:1.2em;padding:10px 6px;width:100%;margin-bottom:4px;min-height:44px;border-radius:8px\" id=\"btnStop1\">⏹ STOP</button>"
"<button class=\"btn\" style=\"background:#27ae60;font-size:1.2em;padding:10px 6px;width:100%;min-height:44px;border-radius:8px\" id=\"btnPlay1\">▶️ PLAY</button>"
"</div>"
"<div style=\"background:#2c3e50;padding:8px;border-radius:8px;text-align:center\">"
"<div style=\"font-weight:bold;color:#e74c3c;font-size:0.9em;margin-bottom:4px\">Slot 2</div>"
"<div style=\"font-size:0.75em;color:#7f8c8d;margin-bottom:6px\" id=\"slot2Info\">-</div>"
"<button class=\"btn\" style=\"background:#e74c3c;font-size:1.2em;padding:10px 6px;width:100%;margin-bottom:4px;min-height:44px;border-radius:8px\" id=\"btnRec2\">🔴 REC</button>"
"<button class=\"btn\" style=\"background:#7f8c8d;font-size:1.2em;padding:10px 6px;width:100%;margin-bottom:4px;min-height:44px;border-radius:8px\" id=\"btnStop2\">⏹ STOP</button>"
"<button class=\"btn\" style=\"background:#27ae60;font-size:1.2em;padding:10px 6px;width:100%;min-height:44px;border-radius:8px\" id=\"btnPlay2\">▶️ PLAY</button>"
"</div>"
"<div style=\"background:#2c3e50;padding:8px;border-radius:8px;text-align:center\">"
"<div style=\"font-weight:bold;color:#9b59b6;font-size:0.9em;margin-bottom:4px\">Slot 3</div>"
"<div style=\"font-size:0.75em;color:#7f8c8d;margin-bottom:6px\" id=\"slot3Info\">-</div>"
"<button class=\"btn\" style=\"background:#e74c3c;font-size:1.2em;padding:10px 6px;width:100%;margin-bottom:4px;min-height:44px;border-radius:8px\" id=\"btnRec3\">🔴 REC</button>"
"<button class=\"btn\" style=\"background:#7f8c8d;font-size:1.2em;padding:10px 6px;width:100%;margin-bottom:4px;min-height:44px;border-radius:8px\" id=\"btnStop3\">⏹ STOP</button>"
"<button class=\"btn\" style=\"background:#27ae60;font-size:1.2em;padding:10px 6px;width:100%;min-height:44px;border-radius:8px\" id=\"btnPlay3\">▶️ PLAY</button>"
"</div>"
"</div>"
"<div id=\"recStatus\" style=\"padding:8px;background:#34495e;border-radius:6px;font-size:0.85em;color:#27ae60;font-weight:bold\">⏸️ Ready</div>"
"</div>"
"<div class=\"calibration\" style=\"margin-top:8px;background:#1a2530;border:2px solid #1abc9c;padding:10px\">"
"<h2 style=\"color:#1abc9c;font-size:0.95em;margin-bottom:8px\">📦 Export / Import Actions (JSON)</h2>"
"<div class=\"buttons\" style=\"margin-bottom:8px\">"
"<button class=\"btn\" style=\"background:#16a085;font-size:0.85em;padding:10px\" id=\"btnExport1\">📤 Export Slot 1</button>"
"<button class=\"btn\" style=\"background:#1abc9c;font-size:0.85em;padding:10px\" id=\"btnExport2\">📤 Export Slot 2</button>"
"<button class=\"btn\" style=\"background:#2ecc71;font-size:0.85em;padding:10px\" id=\"btnExport3\">📤 Export Slot 3</button>"
"<button class=\"btn\" style=\"background:#27ae60;font-size:0.85em;padding:10px\" id=\"btnImport\">📥 Import JSON</button>"
"</div>"
"<div style=\"background:#2c3e50;padding:8px;border-radius:6px\">"
"<label style=\"color:#1abc9c;font-size:0.7em;display:block;margin-bottom:4px\">📝 JSON Data:</label>"
"<textarea id=\"jsonArea\" rows=\"4\" style=\"width:100%;background:#1a252f;color:#ecf0f1;border:1px solid #1abc9c;border-radius:4px;padding:6px;font-family:monospace;font-size:0.7em;resize:vertical\" placeholder=\"Export sẽ hiện JSON ở đây. Hoặc paste JSON vào đây rồi nhấn Import...\"></textarea>"
"<div style=\"display:flex;gap:6px;margin-top:6px\">"
"<label style=\"color:#95a5a6;font-size:0.65em;white-space:nowrap;align-self:center\">Import vào Slot:</label>"
"<select id=\"importSlot\" style=\"background:#34495e;color:#ecf0f1;border:1px solid #1abc9c;border-radius:4px;padding:4px;font-size:0.75em\">"
"<option value=\"1\">Slot 1</option>"
"<option value=\"2\">Slot 2</option>"
"<option value=\"3\">Slot 3</option>"
"</select>"
"</div>"
"</div>"
"<div id=\"jsonStatus\" style=\"padding:6px;font-size:0.75em;color:#1abc9c;margin-top:6px\"></div>"
"</div>"
"<div class=\"calibration\" style=\"margin-top:6px;background:#1a2530;border:2px solid #f39c12;padding:8px\">"
"<h2 style=\"color:#f39c12;font-size:0.85em;margin-bottom:6px\">🔋 Battery Alert (Cảnh báo pin)</h2>"
"<div style=\"display:flex;align-items:center;justify-content:space-between;background:#2c3e50;padding:8px;border-radius:6px\">"
"<div style=\"flex:1\">"
"<div style=\"color:#ecf0f1;font-size:0.8em;font-weight:bold\">Popup & Sound</div>"
"<div style=\"color:#95a5a6;font-size:0.65em;margin-top:2px\">Bật/tắt thông báo pin yếu</div>"
"</div>"
"<label class=\"switch\" style=\"margin-left:10px\">"
"<input type=\"checkbox\" id=\"batteryAlertToggle\" checked>"
"<span class=\"slider\"></span>"
"</label>"
"</div>"
"</div>"
"<div class=\"calibration\" style=\"margin-top:6px;background:#1a2530;border:2px solid #9b59b6;padding:8px\">"
"<h2 style=\"color:#9b59b6;font-size:0.85em;margin-bottom:6px\">🌈 LED Control (8 LED)</h2>"
"<div class=\"cal-item\">"
"<label>🎨 Màu sắc</label>"
"<div class=\"cal-row\" style=\"gap:8px\">"
"<input type=\"color\" id=\"ledColor\" value=\"#ffffff\" style=\"width:50px;height:30px;border:none;cursor:pointer\">"
"<span class=\"cal-value\" id=\"ledColorDisp\">#FFFFFF</span>"
"</div>"
"</div>"
"<div class=\"cal-item\">"
"<label>💡 Độ sáng</label>"
"<div class=\"cal-row\"><input type=\"range\" id=\"ledBrightness\" min=\"0\" max=\"255\" value=\"128\"><span class=\"cal-value\" id=\"ledBrightnessDisp\">128</span></div>"
"</div>"
"<div class=\"cal-item\">"
"<label>⚡ Tốc độ hiệu ứng (ms)</label>"
"<div class=\"cal-row\"><input type=\"range\" id=\"ledSpeed\" min=\"10\" max=\"500\" value=\"50\"><span class=\"cal-value\" id=\"ledSpeedDisp\">50</span></div>"
"</div>"
"<div class=\"cal-item\">"
"<label>🎯 Chế độ LED</label>"
"<div class=\"buttons\" style=\"flex-wrap:wrap;gap:4px\">"
"<button class=\"btn led-mode-btn\" data-mode=\"0\" style=\"background:#7f8c8d;font-size:0.65em;padding:4px 8px\">❌ Tắt</button>"
"<button class=\"btn led-mode-btn active\" data-mode=\"1\" style=\"background:#3498db;font-size:0.65em;padding:4px 8px\">💎 Solid</button>"
"<button class=\"btn led-mode-btn\" data-mode=\"2\" style=\"background:#e74c3c;font-size:0.65em;padding:4px 8px\">🌈 Rainbow</button>"
"<button class=\"btn led-mode-btn\" data-mode=\"3\" style=\"background:#27ae60;font-size:0.65em;padding:4px 8px\">💨 Breathing</button>"
"<button class=\"btn led-mode-btn\" data-mode=\"4\" style=\"background:#f39c12;font-size:0.65em;padding:4px 8px\">🏃 Chase</button>"
"<button class=\"btn led-mode-btn\" data-mode=\"5\" style=\"background:#9b59b6;font-size:0.65em;padding:4px 8px\">⚡ Blink</button>"
"</div>"
"</div>"
"<div class=\"buttons\" style=\"margin-top:8px\">"
"<button class=\"btn-apply\" id=\"btnSaveLed\" style=\"background:#9b59b6\">💾 Lưu LED</button>"
"</div>"
"<div class=\"buttons\" style=\"margin-top:6px;gap:4px\">"
"<button class=\"btn\" style=\"background:#e74c3c;font-size:0.7em;padding:5px\" data-color=\"#ff0000\">🔴</button>"
"<button class=\"btn\" style=\"background:#27ae60;font-size:0.7em;padding:5px\" data-color=\"#00ff00\">🟢</button>"
"<button class=\"btn\" style=\"background:#3498db;font-size:0.7em;padding:5px\" data-color=\"#0000ff\">🔵</button>"
"<button class=\"btn\" style=\"background:#f1c40f;font-size:0.7em;padding:5px\" data-color=\"#ffff00\">🟡</button>"
"<button class=\"btn\" style=\"background:#9b59b6;font-size:0.7em;padding:5px\" data-color=\"#ff00ff\">🟣</button>"
"<button class=\"btn\" style=\"background:#1abc9c;font-size:0.7em;padding:5px\" data-color=\"#00ffff\">🩵</button>"
"<button class=\"btn\" style=\"background:#ecf0f1;color:#2c3e50;font-size:0.7em;padding:5px\" data-color=\"#ffffff\">⚪</button>"
"</div>"
"</div>"
"</div>"
"<div id=\"tab2\" class=\"tab-content\">"
"<button class=\"btn-apply\" id=\"btnSaveTop\" style=\"max-width:400px;margin:5px auto 8px auto\">💾 Save All</button>"
"<div class=\"calibration\">"
"<h2>🎛️ Servo Realtime</h2>"
"<div class=\"buttons\" style=\"margin-bottom:6px\">"
"<button class=\"btn\" style=\"background:#9b59b6\" id=\"btnTestLF\">🦶 LF</button>"
"<button class=\"btn\" style=\"background:#9b59b6\" id=\"btnTestRF\">🦶 RF</button>"
"<button class=\"btn\" style=\"background:#27ae60\" id=\"btnTestBoth\">🦶🦶</button>"
"<button class=\"btn\" style=\"background:#e74c3c\" id=\"btnStopFoot\">⏹️</button>"
"</div>"
"<div class=\"cal-item\">"
"<label>🦶 Left Foot (CH0)</label>"
"<div class=\"cal-row\"><input type=\"range\" id=\"srv0\" min=\"0\" max=\"180\" value=\"90\"><span class=\"cal-value\" id=\"srv0Disp\">90</span></div>"
"</div>"
"<div class=\"cal-item\">"
"<label>🦵 Left Leg (CH1)</label>"
"<div class=\"cal-row\"><input type=\"range\" id=\"srv1\" min=\"0\" max=\"180\" value=\"90\"><span class=\"cal-value\" id=\"srv1Disp\">90</span></div>"
"</div>"
"<div class=\"cal-item\">"
"<label>🦶 Right Foot (CH2)</label>"
"<div class=\"cal-row\"><input type=\"range\" id=\"srv2\" min=\"0\" max=\"180\" value=\"90\"><span class=\"cal-value\" id=\"srv2Disp\">90</span></div>"
"</div>"
"<div class=\"cal-item\">"
"<label>🦵 Right Leg (CH3)</label>"
"<div class=\"cal-row\"><input type=\"range\" id=\"srv3\" min=\"0\" max=\"180\" value=\"90\"><span class=\"cal-value\" id=\"srv3Disp\">90</span></div>"
"</div>"
"</div>"
"<div class=\"calibration\">"
"<h2>🎛️ Calibration</h2>"
"<p style=\"color:#888;font-size:12px;margin:5px 0 8px 0;\">📝 Di chuột lên nhãn hoặc thanh trượt để xem hướng dẫn chi tiết. Hover over labels or sliders for detailed tooltips.</p>"
"<div style=\"background:#1a252f;padding:10px;border-radius:6px;margin:0 0 15px 0;font-size:11px;line-height:1.6;\">"
"<div style=\"color:#3498db;font-weight:bold;margin-bottom:5px;\">📱 Hướng dẫn sử dụng trên điện thoại:</div>"
"<div style=\"color:#bdc3c7;margin-bottom:3px;\">• <b>Foot Neutral (80-100)</b>: Vị trí đứng yên của chân trái/phải. Điều chỉnh nếu robot nghiêng khi đứng.</div>"
"<div style=\"color:#bdc3c7;margin-bottom:3px;\">• <b>Walk Speed (5-40)</b>: Tốc độ quay chân khi đi. Giá trị càng cao robot đi càng nhanh.</div>"
"<div style=\"color:#bdc3c7;margin-bottom:3px;\">• <b>Standing (0-180)</b>: Góc chân khi đứng ở chế độ đi bộ. Điều chỉnh độ cao robot.</div>"
"<div style=\"color:#bdc3c7;margin-bottom:3px;\">• <b>Tilt (0-180)</b>: Góc nghiêng trái/phải của cánh tay khi robot nghiêng người.</div>"
"<div style=\"color:#bdc3c7;margin-bottom:8px;\">• <b>Roll (0-180)</b>: Góc chân khi robot biến hình thành xe lăn.</div>"
"<div style=\"color:#e67e22;font-size:10px;\">⚠️ Sau khi điều chỉnh, nhấn nút <b>✅ Apply</b> để lưu thay đổi.</div>"
"</div>"
"<div class=\"cal-section\">"
"<h3>⚖️ Foot Neutral</h3>"
"<div class=\"cal-item\">"
"<label title=\"Góc trung tính chân trái khi đứng (Left Foot Neutral Position)\">LF Neutral</label>"
"<div class=\"cal-row\"><input type=\"range\" id=\"lfn\" min=\"80\" max=\"100\" value=\"90\" title=\"Điều chỉnh vị trí trung tính chân trái (80-100°)\"><span class=\"cal-value\" id=\"lfnDisp\">90</span></div>"
"</div>"
"<div class=\"cal-item\">"
"<label title=\"Góc trung tính chân phải khi đứng (Right Foot Neutral Position)\">RF Neutral</label>"
"<div class=\"cal-row\"><input type=\"range\" id=\"rfn\" min=\"80\" max=\"100\" value=\"90\" title=\"Điều chỉnh vị trí trung tính chân phải (80-100°)\"><span class=\"cal-value\" id=\"rfnDisp\">90</span></div>"
"</div>"
"</div>"
"<div class=\"cal-section\">"
"<h3>🚶 Walk Speed</h3>"
"<div class=\"cal-item\">"
"<label title=\"Tốc độ quay chân trái khi đi tiến (Left Foot Forward Rotation Speed)\">Left Fwd</label>"
"<div class=\"cal-row\"><input type=\"range\" id=\"lff\" min=\"5\" max=\"40\" value=\"18\" title=\"Điều chỉnh tốc độ chân trái đi tiến (5-40). Giá trị càng cao càng nhanh\"><span class=\"cal-value\" id=\"lffDisp\">18</span></div>"
"</div>"
"<div class=\"cal-item\">"
"<label title=\"Tốc độ quay chân phải khi đi tiến (Right Foot Forward Rotation Speed)\">Right Fwd</label>"
"<div class=\"cal-row\"><input type=\"range\" id=\"rff\" min=\"5\" max=\"40\" value=\"18\" title=\"Điều chỉnh tốc độ chân phải đi tiến (5-40). Giá trị càng cao càng nhanh\"><span class=\"cal-value\" id=\"rffDisp\">18</span></div>"
"</div>"
"<div class=\"cal-item\">"
"<label title=\"Tốc độ quay chân trái khi lùi (Left Foot Backward Rotation Speed)\">Left Back</label>"
"<div class=\"cal-row\"><input type=\"range\" id=\"lfb\" min=\"5\" max=\"40\" value=\"18\" title=\"Điều chỉnh tốc độ chân trái lùi (5-40). Giá trị càng cao càng nhanh\"><span class=\"cal-value\" id=\"lfbDisp\">18</span></div>"
"</div>"
"<div class=\"cal-item\">"
"<label title=\"Tốc độ quay chân phải khi lùi (Right Foot Backward Rotation Speed)\">Right Back</label>"
"<div class=\"cal-row\"><input type=\"range\" id=\"rfb\" min=\"5\" max=\"40\" value=\"18\" title=\"Điều chỉnh tốc độ chân phải lùi (5-40). Giá trị càng cao càng nhanh\"><span class=\"cal-value\" id=\"rfbDisp\">18</span></div>"
"</div>"
"</div>"
"<div class=\"cal-section\">"
"<h3>🦵 Standing ⚡</h3>"
"<div class=\"cal-item\">"
"<label title=\"Góc chân trái khi đứng ở chế độ đi bộ (Left Leg Standing Position)\">LL (LA0)</label>"
"<div class=\"cal-row\"><input type=\"range\" id=\"la0\" min=\"0\" max=\"180\" value=\"60\" title=\"Điều chỉnh góc chân trái khi đứng (0-180°). Quyết định độ cao và vị trí đứng\"><span class=\"cal-value\" id=\"la0Disp\">60</span></div>"
"</div>"
"<div class=\"cal-item\">"
"<label title=\"Góc chân phải khi đứng ở chế độ đi bộ (Right Leg Standing Position)\">RL (RA0)</label>"
"<div class=\"cal-row\"><input type=\"range\" id=\"ra0\" min=\"0\" max=\"180\" value=\"135\" title=\"Điều chỉnh góc chân phải khi đứng (0-180°). Quyết định độ cao và vị trí đứng\"><span class=\"cal-value\" id=\"ra0Disp\">135</span></div>"
"</div>"
"</div>"
"<div class=\"cal-section\">"
"<h3>↔️ Góc nghiêng chân (Tilt) ⚡</h3>"
"<div class=\"cal-item\">"
"<label title=\"Góc chân trái khi robot nghiêng sang TRÁI (Left Leg Tilt Left)\">LL Nghiêng Trái</label>"
"<div class=\"cal-row\"><input type=\"range\" id=\"latl\" min=\"0\" max=\"180\" value=\"100\" title=\"Điều chỉnh góc chân trái khi nghiêng trái (0-180°). Ảnh hưởng pha 3 đi bộ\"><span class=\"cal-value\" id=\"latlDisp\">100</span></div>"
"</div>"
"<div class=\"cal-item\">"
"<label title=\"Góc chân phải khi robot nghiêng sang TRÁI (Right Leg Tilt Left)\">RL Nghiêng Trái</label>"
"<div class=\"cal-row\"><input type=\"range\" id=\"ratl\" min=\"0\" max=\"180\" value=\"175\" title=\"Điều chỉnh góc chân phải khi nghiêng trái (0-180°). Ảnh hưởng pha 3 đi bộ\"><span class=\"cal-value\" id=\"ratlDisp\">175</span></div>"
"</div>"
"<div class=\"cal-item\">"
"<label title=\"Góc chân trái khi robot nghiêng sang PHẢI (Left Leg Tilt Right)\">LL Nghiêng Phải</label>"
"<div class=\"cal-row\"><input type=\"range\" id=\"latr\" min=\"0\" max=\"180\" value=\"5\" title=\"Điều chỉnh góc chân trái khi nghiêng phải (0-180°). Ảnh hưởng pha 1 đi bộ\"><span class=\"cal-value\" id=\"latrDisp\">5</span></div>"
"</div>"
"<div class=\"cal-item\">"
"<label title=\"Góc chân phải khi robot nghiêng sang PHẢI (Right Leg Tilt Right)\">RL Nghiêng Phải</label>"
"<div class=\"cal-row\"><input type=\"range\" id=\"ratr\" min=\"0\" max=\"180\" value=\"80\" title=\"Điều chỉnh góc chân phải khi nghiêng phải (0-180°). Ảnh hưởng pha 1 đi bộ\"><span class=\"cal-value\" id=\"ratrDisp\">80</span></div>"
"</div>"
"</div>"
"<div class=\"cal-section\">"
"<h3>⚙️ Roll ⚡</h3>"
"<div class=\"cal-item\">"
"<label title=\"Góc chân trái khi biến hình thành xe (Left Leg Roll/Wheel Position)\">LL (LA1)</label>"
"<div class=\"cal-row\"><input type=\"range\" id=\"la1\" min=\"0\" max=\"180\" value=\"160\" title=\"Điều chỉnh góc chân trái khi ở chế độ lăn xe (0-180°). Quyết định vị trí bánh xe\"><span class=\"cal-value\" id=\"la1Disp\">160</span></div>"
"</div>"
"<div class=\"cal-item\">"
"<label title=\"Góc chân phải khi biến hình thành xe (Right Leg Roll/Wheel Position)\">RL (RA1)</label>"
"<div class=\"cal-row\"><input type=\"range\" id=\"ra1\" min=\"0\" max=\"180\" value=\"25\" title=\"Điều chỉnh góc chân phải khi ở chế độ lăn xe (0-180°). Quyết định vị trí bánh xe\"><span class=\"cal-value\" id=\"ra1Disp\">25</span></div>"
"</div>"
"</div>"
"<div class=\"cal-section\">"
"<h3>🏎️ Roll Speed (Tốc độ lăn)</h3>"
"<div class=\"cal-item\">"
"<label title=\"Left Foot lăn tiến: góc quay từ 90°. Giá trị càng cao càng nhanh\">⬆️ LF Fwd</label>"
"<div class=\"cal-row\"><input type=\"range\" id=\"rlff\" min=\"10\" max=\"60\" value=\"45\" title=\"LF Roll Forward (10-60)\"><span class=\"cal-value\" id=\"rlffDisp\">45</span></div>"
"</div>"
"<div class=\"cal-item\">"
"<label title=\"Left Foot lăn lùi: góc quay từ 90°. Giá trị càng cao càng nhanh\">⬇️ LF Bwd</label>"
"<div class=\"cal-row\"><input type=\"range\" id=\"rlfb\" min=\"10\" max=\"60\" value=\"45\" title=\"LF Roll Backward (10-60)\"><span class=\"cal-value\" id=\"rlfbDisp\">45</span></div>"
"</div>"
"<div class=\"cal-item\">"
"<label title=\"Right Foot lăn tiến: góc quay từ 90°. Giá trị càng cao càng nhanh\">⬆️ RF Fwd</label>"
"<div class=\"cal-row\"><input type=\"range\" id=\"rrff\" min=\"10\" max=\"60\" value=\"45\" title=\"RF Roll Forward (10-60)\"><span class=\"cal-value\" id=\"rrffDisp\">45</span></div>"
"</div>"
"<div class=\"cal-item\">"
"<label title=\"Right Foot lăn lùi: góc quay từ 90°. Giá trị càng cao càng nhanh\">⬇️ RF Bwd</label>"
"<div class=\"cal-row\"><input type=\"range\" id=\"rrfb\" min=\"10\" max=\"60\" value=\"45\" title=\"RF Roll Backward (10-60)\"><span class=\"cal-value\" id=\"rrfbDisp\">45</span></div>"
"</div>"
"</div>"
"<div class=\"cal-section\">"
"<h3>🦿 Transform Speed (Tốc độ biến hình)</h3>"
"<div class=\"cal-item\">"
"<label title=\"Left Leg speed khi biến hình: ms mỗi bước. Càng thấp càng nhanh\">🦵 LL Speed</label>"
"<div class=\"cal-row\"><input type=\"range\" id=\"tll\" min=\"1\" max=\"20\" value=\"5\" title=\"LL Transform Speed (1-20ms)\"><span class=\"cal-value\" id=\"tllDisp\">5</span></div>"
"</div>"
"<div class=\"cal-item\">"
"<label title=\"Right Leg speed khi biến hình: ms mỗi bước. Càng thấp càng nhanh\">🦵 RL Speed</label>"
"<div class=\"cal-row\"><input type=\"range\" id=\"trl\" min=\"1\" max=\"20\" value=\"5\" title=\"RL Transform Speed (1-20ms)\"><span class=\"cal-value\" id=\"trlDisp\">5</span></div>"
"</div>"
"</div>"
"<div style=\"display:flex;gap:6px;\">"
"<button class=\"btn-apply\" id=\"btnApply\" style=\"flex:1;\">✅ Apply</button>"
"<button class=\"btn-apply\" id=\"btnReset\" style=\"flex:1;background:#e74c3c;\">🔄 Reset</button>"
"</div>"
"</div>"
"</div>"
"<script>"
"function switchTab(n){document.querySelectorAll('.tab-content').forEach(t=>t.classList.remove('active'));document.querySelectorAll('.tab').forEach(t=>t.classList.remove('active'));document.getElementById('tab'+n).classList.add('active');document.querySelectorAll('.tab')[n-1].classList.add('active');}"
"let jx=0,jy=0,bA=0,bB=0,bX=0,bY=0;"
"const js=document.getElementById('joystick'),jz=document.getElementById('joystickZone');"
"let drag=false;const md=70;"
"let lastSendTime=0;"
"let pendingUpdate=false;"
"function sendControl(){"  
"fetch('/control?x='+jx+'&y='+jy+'&a='+bA+'&b='+bB+'&x2='+bX+'&y2='+bY);"  
"lastSendTime=Date.now();"  
"pendingUpdate=false;"  
"}"  
"function fb(id){const b=document.getElementById(id);b.classList.add('clicked');setTimeout(()=>b.classList.remove('clicked'),150);}"
"function uj(e){if(!drag)return;"
"const r=jz.getBoundingClientRect(),cx=r.width/2,cy=r.height/2;"
"let x=(e.touches?e.touches[0].clientX:e.clientX)-r.left-cx,"
"y=(e.touches?e.touches[0].clientY:e.clientY)-r.top-cy;"
"const d=Math.sqrt(x*x+y*y);if(d>md){x=(x/d)*md;y=(y/d)*md;}"
"js.style.left=(cx+x)+'px';js.style.top=(cy+y)+'px';"
"let rawX=Math.round((x/md)*100);"
"let rawY=Math.round(-(y/md)*100);"
"jx=(Math.abs(rawX)<5)?0:rawX;"
"jy=(Math.abs(rawY)<5)?0:rawY;"
"document.getElementById('xVal').textContent=jx;"
"document.getElementById('yVal').textContent=jy;"
"if(Date.now()-lastSendTime>100){sendControl();}else{pendingUpdate=true;}"
"}"
"setInterval(()=>{if(pendingUpdate&&Date.now()-lastSendTime>100){sendControl();}},50);"
"function rj(){drag=false;js.style.left='50%';js.style.top='50%';"
"jx=0;jy=0;document.getElementById('xVal').textContent='0';"
"document.getElementById('yVal').textContent='0';"
"sendControl();}"
"js.addEventListener('mousedown',()=>drag=true);"
"js.addEventListener('touchstart',()=>drag=true);"
"document.addEventListener('mousemove',uj);"
"document.addEventListener('touchmove',uj);"
"document.addEventListener('mouseup',rj);"
"document.addEventListener('touchend',rj);"
"document.getElementById('btnY').onclick=()=>{bY=1;setTimeout(()=>bY=0,100);sendControl();};"
"document.getElementById('btnX').onclick=()=>{bX=1;setTimeout(()=>bX=0,100);sendControl();};"
"document.getElementById('btnA').onmousedown=()=>{bA=1;sendControl();};"
"document.getElementById('btnA').onmouseup=()=>{bA=0;sendControl();};"
"document.getElementById('btnB').onmousedown=()=>{bB=1;sendControl();};"
"document.getElementById('btnB').onmouseup=()=>{bB=0;sendControl();};"
"document.getElementById('moveDuration').oninput=()=>{const v=document.getElementById('moveDuration').value;document.getElementById('moveDurationDisp').textContent=v==0?'∞':(v/1000).toFixed(1)+'s';};"
"let fwdTimer=null,bwdTimer=null;"
"function startFwd(){const dur=parseInt(document.getElementById('moveDuration').value);const b=document.getElementById('btnTestFwd');b.classList.add('running');fetch('/move?dir=fwd&duration='+dur);if(dur>0){fwdTimer=setTimeout(()=>{b.classList.remove('running');fetch('/move?dir=stop');},dur);}}"
"function stopFwd(){if(fwdTimer){clearTimeout(fwdTimer);fwdTimer=null;}document.getElementById('btnTestFwd').classList.remove('running');fetch('/move?dir=stop');}"
"function startBwd(){const dur=parseInt(document.getElementById('moveDuration').value);const b=document.getElementById('btnTestBack');b.classList.add('running');fetch('/move?dir=bwd&duration='+dur);if(dur>0){bwdTimer=setTimeout(()=>{b.classList.remove('running');fetch('/move?dir=stop');},dur);}}"
"function stopBwd(){if(bwdTimer){clearTimeout(bwdTimer);bwdTimer=null;}document.getElementById('btnTestBack').classList.remove('running');fetch('/move?dir=stop');}"
"document.getElementById('btnTestFwd').onmousedown=startFwd;document.getElementById('btnTestFwd').ontouchstart=startFwd;"
"document.getElementById('btnTestFwd').onmouseup=stopFwd;document.getElementById('btnTestFwd').ontouchend=stopFwd;"
"document.getElementById('btnTestBack').onmousedown=startBwd;document.getElementById('btnTestBack').ontouchstart=startBwd;"
"document.getElementById('btnTestBack').onmouseup=stopBwd;document.getElementById('btnTestBack').ontouchend=stopBwd;"
"document.getElementById('btnHome').onclick=()=>{fb('btnHome');fetch('/home');};"  
"document.getElementById('btnTestLF').onclick=()=>{fb('btnTestLF');fetch('/testfoot?foot=left');};"  
"document.getElementById('btnTestRF').onclick=()=>{fb('btnTestRF');fetch('/testfoot?foot=right');};"  
"document.getElementById('btnTestBoth').onclick=()=>{fb('btnTestBoth');fetch('/testfoot?foot=both');};"  
"document.getElementById('btnStopFoot').onclick=()=>{fb('btnStopFoot');fetch('/testfoot?foot=stop');};"
"document.getElementById('btnTiltLeft').onclick=()=>{fb('btnTiltLeft');fetch('/tilt?dir=left');};"
"document.getElementById('btnTiltRight').onclick=()=>{fb('btnTiltRight');fetch('/tilt?dir=right');};"
"document.getElementById('spinSpeed').oninput=function(){const v=this.value;document.getElementById('spinSpeedDisp').textContent=v;if(v!=0){fetch('/spin?speed='+v);}};"
"document.getElementById('btnSpin').onclick=()=>{document.getElementById('spinSpeed').value=0;document.getElementById('spinSpeedDisp').textContent='0';fetch('/spin?speed=0');};"
"document.getElementById('btnRhythmL').onclick=()=>{const b=document.getElementById('btnRhythmL');b.classList.add('running');fetch('/rhythm?leg=left').finally(()=>b.classList.remove('running'));};"
"document.getElementById('btnRhythmR').onclick=()=>{const b=document.getElementById('btnRhythmR');b.classList.add('running');fetch('/rhythm?leg=right').finally(()=>b.classList.remove('running'));};"
"document.getElementById('btnTurnLeft').onclick=()=>{const b=document.getElementById('btnTurnLeft');b.classList.add('running');const spd=document.getElementById('turnLeftSpeed').value;fetch('/turn?dir=left&speed='+spd).finally(()=>b.classList.remove('running'));};"
"document.getElementById('btnTurnRight').onclick=()=>{const b=document.getElementById('btnTurnRight');b.classList.add('running');const spd=document.getElementById('turnRightSpeed').value;fetch('/turn?dir=right&speed='+spd).finally(()=>b.classList.remove('running'));};"
"document.getElementById('turnLeftSpeed').oninput=()=>{document.getElementById('turnLeftSpeedDisp').textContent=document.getElementById('turnLeftSpeed').value;};"
"document.getElementById('turnRightSpeed').oninput=()=>{document.getElementById('turnRightSpeedDisp').textContent=document.getElementById('turnRightSpeed').value;};"
"document.getElementById('turnRightSpeed').oninput=()=>{document.getElementById('turnRightSpeedDisp').textContent=document.getElementById('turnRightSpeed').value;};"
"document.getElementById('btnCombo1').onclick=()=>{const b=document.getElementById('btnCombo1');b.classList.add('running');const spd=document.getElementById('comboLfSpeed').value;fetch('/combo?id=1&lf_speed='+spd).finally(()=>b.classList.remove('running'));};"  
"document.getElementById('btnCombo2').onclick=()=>{const b=document.getElementById('btnCombo2');b.classList.add('running');const spd=document.getElementById('comboRfSpeed').value;fetch('/combo?id=2&rf_speed='+spd).finally(()=>b.classList.remove('running'));};"  
"document.getElementById('comboLfSpeed').oninput=()=>{document.getElementById('comboLfSpeedDisp').textContent=document.getElementById('comboLfSpeed').value;};"
"document.getElementById('comboRfSpeed').oninput=()=>{document.getElementById('comboRfSpeedDisp').textContent=document.getElementById('comboRfSpeed').value;};"
"document.getElementById('btnCombo123').onclick=()=>{const b=document.getElementById('btnCombo123');b.classList.add('running');fetch('/phase?id=1').finally(()=>b.classList.remove('running'));};"
"document.getElementById('btnCombo345').onclick=()=>{const b=document.getElementById('btnCombo345');b.classList.add('running');fetch('/phase?id=2').finally(()=>b.classList.remove('running'));};"
"document.getElementById('btnWaveRight').onclick=()=>{const b=document.getElementById('btnWaveRight');b.classList.add('running');fetch('/wave?leg=right').finally(()=>b.classList.remove('running'));};"
"document.getElementById('btnWaveLeft').onclick=()=>{const b=document.getElementById('btnWaveLeft');b.classList.add('running');fetch('/wave?leg=left').finally(()=>b.classList.remove('running'));};"
"document.getElementById('btnExport1').onclick=()=>{fetch('/action_json?action=export&slot=0').then(r=>r.text()).then(t=>{document.getElementById('jsonArea').value=t;document.getElementById('jsonStatus').textContent='\u2705 Exported Slot 1';});};"
"document.getElementById('btnExport2').onclick=()=>{fetch('/action_json?action=export&slot=1').then(r=>r.text()).then(t=>{document.getElementById('jsonArea').value=t;document.getElementById('jsonStatus').textContent='\u2705 Exported Slot 2';});};"
"document.getElementById('btnExport3').onclick=()=>{fetch('/action_json?action=export&slot=2').then(r=>r.text()).then(t=>{document.getElementById('jsonArea').value=t;document.getElementById('jsonStatus').textContent='\u2705 Exported Slot 3';});};"
"document.getElementById('btnImport').onclick=()=>{const json=document.getElementById('jsonArea').value;const slot=document.getElementById('importSlot').value;if(!json.trim()){document.getElementById('jsonStatus').textContent='\u274c Vui l\u00f2ng paste JSON v\u00e0o \u00f4 tr\u01b0\u1edbc!';return;}fetch('/action_json?action=import&slot='+(slot-1),{method:'POST',body:json}).then(r=>r.text()).then(t=>{document.getElementById('jsonStatus').textContent=t;updateSlotInfo();});};"
"document.getElementById('btnManualOff').onclick=()=>{fb('btnManualOff');fetch('/manualoff');};"
"let activeRecSlot=-1;"
"for(let s=1;s<=3;s++){"
"document.getElementById('btnRec'+s).onclick=()=>{fetch('/record?action=start&slot='+(s-1));document.getElementById('recStatus').innerHTML='🔴 Đang ghi Slot '+s+' (0/20) - Kéo servo sẽ tự ghi!';document.getElementById('recStatus').style.color='#e74c3c';activeRecSlot=s;document.querySelectorAll('[id^=btnRec]').forEach(b=>b.style.animation='');document.getElementById('btnRec'+s).style.animation='pulse 0.8s infinite';startRecordPoll();};"
"document.getElementById('btnStop'+s).onclick=()=>{fetch('/record?action=stop').then(r=>r.text()).then(t=>{document.getElementById('recStatus').innerHTML='⏸️ '+t;document.getElementById('recStatus').style.color='#27ae60';activeRecSlot=-1;document.querySelectorAll('[id^=btnRec]').forEach(b=>b.style.animation='');stopRecordPoll();updateSlotInfo();});};"
"document.getElementById('btnPlay'+s).onclick=()=>{const b=document.getElementById('btnPlay'+s);b.classList.add('running');b.textContent='⏳ ...';fetch('/record?action=play&slot='+(s-1)).then(()=>{b.classList.remove('running');b.textContent='▶️ PLAY';}).catch(()=>{b.classList.remove('running');b.textContent='▶️ PLAY';});};"
"}"
"let recordPollTimer=null;"
"function startRecordPoll(){recordPollTimer=setInterval(updateSlotInfo,500);}"
"function stopRecordPoll(){if(recordPollTimer){clearInterval(recordPollTimer);recordPollTimer=null;}}"
"function updateSlotInfo(){fetch('/record?action=info').then(r=>r.json()).then(d=>{"
"for(let i=0;i<3;i++){document.getElementById('slot'+(i+1)+'Info').textContent=d.slots[i]>0?d.slots[i]+'/20 actions':'Trống';}"
"if(d.recording){document.getElementById('recStatus').innerHTML='🔴 Đang ghi Slot '+(d.rec_slot+1)+' ('+d.rec_count+'/'+d.max+') - Kéo servo sẽ tự ghi!';}"
"});}"
"updateSlotInfo();"
"for(let i=0;i<4;i++){"
"const el=document.getElementById('srv'+i);"
"el.oninput=()=>{"  
"document.getElementById('srv'+i+'Disp').textContent=el.value;"  
"fetch('/servo?ch='+i+'&angle='+el.value);"  
"};"  
"}"
"const sliders=['lfn','rfn','lff','rff','lfb','rfb','rlff','rlfb','rrff','rrfb','tll','trl'];"
"sliders.forEach(id=>{"
"const el=document.getElementById(id);"
"el.oninput=()=>{document.getElementById(id+'Disp').textContent=el.value;};"
"});"
"const realtimeSliders=[{id:'la0',ch:1},{id:'ra0',ch:3},{id:'la1',ch:1},{id:'ra1',ch:3},{id:'latl',ch:1},{id:'ratl',ch:3},{id:'latr',ch:1},{id:'ratr',ch:3}];"
"realtimeSliders.forEach(s=>{"
"const el=document.getElementById(s.id);"
"el.oninput=()=>{"  
"document.getElementById(s.id+'Disp').textContent=el.value;"  
"fetch('/servo?ch='+s.ch+'&angle='+el.value);"  
"};"  
"});"
"document.getElementById('btnApply').onclick=()=>{fb('btnApply');"  
"const allSliders=['lfn','rfn','lff','rff','lfb','rfb','la0','ra0','latl','ratl','latr','ratr','la1','ra1','rlff','rlfb','rrff','rrfb','tll','trl'];"  
"const params=allSliders.map(id=>id+'='+document.getElementById(id).value).join('&');"
"const turnComboParams='&tls='+document.getElementById('turnLeftSpeed').value+'&trs='+document.getElementById('turnRightSpeed').value+'&clf='+document.getElementById('comboLfSpeed').value+'&crf='+document.getElementById('comboRfSpeed').value;"
"const batteryAlert='&balert='+(document.getElementById('batteryAlertToggle').checked?'1':'0');"
"fetch('/calibrate?'+params+turnComboParams+batteryAlert);};"
"document.getElementById('btnSaveTop').onclick=()=>{fb('btnSaveTop');"  
"const allSliders=['lfn','rfn','lff','rff','lfb','rfb','la0','ra0','latl','ratl','latr','ratr','la1','ra1','rlff','rlfb','rrff','rrfb','tll','trl'];"  
"const params=allSliders.map(id=>id+'='+document.getElementById(id).value).join('&');"
"const turnComboParams='&tls='+document.getElementById('turnLeftSpeed').value+'&trs='+document.getElementById('turnRightSpeed').value+'&clf='+document.getElementById('comboLfSpeed').value+'&crf='+document.getElementById('comboRfSpeed').value;"
"const batteryAlert='&balert='+(document.getElementById('batteryAlertToggle').checked?'1':'0');"
"fetch('/calibrate?'+params+turnComboParams+batteryAlert);};"
"document.getElementById('btnReset').onclick=()=>{if(confirm('Reset all calibration to factory defaults?')){fb('btnReset');fetch('/reset_cal').then(()=>{setTimeout(()=>location.reload(),500);});}};"
"fetch('/getCal').then(r=>r.json()).then(d=>{"
"const allSliders=['lfn','rfn','lff','rff','lfb','rfb','la0','ra0','latl','ratl','latr','ratr','la1','ra1','rlff','rlfb','rrff','rrfb','tll','trl'];"
"allSliders.forEach(id=>{"
"if(d[id]!==undefined){"
"document.getElementById(id).value=d[id];"
"document.getElementById(id+'Disp').textContent=d[id];"
"}"
"});"
"if(d.tls){document.getElementById('turnLeftSpeed').value=d.tls;document.getElementById('turnLeftSpeedDisp').textContent=d.tls;}"
"if(d.trs){document.getElementById('turnRightSpeed').value=d.trs;document.getElementById('turnRightSpeedDisp').textContent=d.trs;}"
"if(d.clf){document.getElementById('comboLfSpeed').value=d.clf;document.getElementById('comboLfSpeedDisp').textContent=d.clf;}"
"if(d.crf){document.getElementById('comboRfSpeed').value=d.crf;document.getElementById('comboRfSpeedDisp').textContent=d.crf;}"
"if(d.balert!==undefined){document.getElementById('batteryAlertToggle').checked=(d.balert===1);}"
"});"
"document.getElementById('batteryAlertToggle').onchange=()=>{"
"const isEnabled=document.getElementById('batteryAlertToggle').checked;"
"fetch('/calibrate?balert='+(isEnabled?'1':'0'));"
"console.log('Battery alert '+(isEnabled?'enabled':'disabled'));"
"};"
"/* LED Control */"
"let ledMode=1;"
"function updateLedColor(){"
"const c=document.getElementById('ledColor').value;"
"document.getElementById('ledColorDisp').textContent=c.toUpperCase();"
"const r=parseInt(c.substr(1,2),16);"
"const g=parseInt(c.substr(3,2),16);"
"const b=parseInt(c.substr(5,2),16);"
"const br=document.getElementById('ledBrightness').value;"
"const spd=document.getElementById('ledSpeed').value;"
"fetch('/led?r='+r+'&g='+g+'&b='+b+'&br='+br+'&mode='+ledMode+'&speed='+spd);"
"}"
"function setLedMode(m){"
"ledMode=m;"
"document.querySelectorAll('.led-mode-btn').forEach(b=>b.classList.remove('active'));"
"document.querySelector('.led-mode-btn[data-mode=\"'+m+'\"]').classList.add('active');"
"updateLedColor();"
"}"
"document.getElementById('ledColor').oninput=updateLedColor;"
"document.getElementById('ledBrightness').oninput=()=>{"
"document.getElementById('ledBrightnessDisp').textContent=document.getElementById('ledBrightness').value;"
"updateLedColor();"
"};"
"document.getElementById('ledSpeed').oninput=()=>{"
"document.getElementById('ledSpeedDisp').textContent=document.getElementById('ledSpeed').value;"
"updateLedColor();"
"};"
"document.querySelectorAll('.led-mode-btn').forEach(b=>{"
"b.onclick=()=>setLedMode(parseInt(b.dataset.mode));"
"});"
"document.querySelectorAll('[data-color]').forEach(b=>{"
"b.onclick=()=>{"
"document.getElementById('ledColor').value=b.dataset.color;"
"updateLedColor();"
"};"
"});"
"document.getElementById('btnSaveLed').onclick=()=>{"
"fetch('/led?save=1');"
"document.getElementById('btnSaveLed').textContent='✅ Đã lưu!';"
"setTimeout(()=>document.getElementById('btnSaveLed').textContent='💾 Lưu LED',1500);"
"};"
"/* Load LED state */"
"fetch('/get_led').then(r=>r.json()).then(d=>{"
"if(d.r!==undefined&&d.g!==undefined&&d.b!==undefined){"
"const hex='#'+d.r.toString(16).padStart(2,'0')+d.g.toString(16).padStart(2,'0')+d.b.toString(16).padStart(2,'0');"
"document.getElementById('ledColor').value=hex;"
"document.getElementById('ledColorDisp').textContent=hex.toUpperCase();"
"}"
"if(d.br!==undefined){document.getElementById('ledBrightness').value=d.br;document.getElementById('ledBrightnessDisp').textContent=d.br;}"
"if(d.speed!==undefined){document.getElementById('ledSpeed').value=d.speed;document.getElementById('ledSpeedDisp').textContent=d.speed;}"
"if(d.mode!==undefined){"
"ledMode=d.mode;"
"document.querySelectorAll('.led-mode-btn').forEach(b=>b.classList.remove('active'));"
"const activeBtn=document.querySelector('.led-mode-btn[data-mode=\"'+d.mode+'\"]');"
"if(activeBtn)activeBtn.classList.add('active');"
"}"
"});"
"</script>"
"</body></html>";

// Helper function to get query parameter value
static esp_err_t get_query_param(httpd_req_t *req, const char *key, char *value, size_t value_len) {
    size_t buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        char *buf = malloc(buf_len);
        if (!buf) return ESP_ERR_NO_MEM;
        
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            if (httpd_query_key_value(buf, key, value, value_len) == ESP_OK) {
                free(buf);
                return ESP_OK;
            }
        }
        free(buf);
    }
    return ESP_FAIL;
}

static int get_query_param_int(httpd_req_t *req, const char *key, int default_val) {
    char value[16];
    if (get_query_param(req, key, value, sizeof(value)) == ESP_OK) {
        return atoi(value);
    }
    return default_val;
}

// Handler for root page
static esp_err_t root_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html_content, strlen(html_content));
    return ESP_OK;
}

// Handler for control endpoint
static esp_err_t control_handler(httpd_req_t *req) {
    control_state_t *state = get_control_state();
    recording_state_t *rec = get_recording_state();
    
    char val[16];
    int new_x = state->j_x, new_y = state->j_y;
    int new_a = 0, new_b = 0, new_mode_x = 0, new_mode_y = 0;
    
    if (get_query_param(req, "x", val, sizeof(val)) == ESP_OK) {
        new_x = atoi(val);
        state->j_x = new_x;
    }
    if (get_query_param(req, "y", val, sizeof(val)) == ESP_OK) {
        new_y = atoi(val);
        state->j_y = new_y;
    }
    if (get_query_param(req, "a", val, sizeof(val)) == ESP_OK) {
        new_a = atoi(val);
        state->button_a = new_a;
    }
    if (get_query_param(req, "b", val, sizeof(val)) == ESP_OK) {
        new_b = atoi(val);
        state->button_b = new_b;
    }
    if (get_query_param(req, "x2", val, sizeof(val)) == ESP_OK) {
        new_mode_x = atoi(val);
        state->button_x = new_mode_x;
    }
    if (get_query_param(req, "y2", val, sizeof(val)) == ESP_OK) {
        new_mode_y = atoi(val);
        state->button_y = new_mode_y;
    }
    
    // Record actions if recording
    // Track last joystick values to avoid duplicate recordings
    static int last_rec_x = 0, last_rec_y = 0;
    
    if (rec->is_recording) {
        // Only record joystick if significant change (> 20 units) from last recorded value
        // Or if moving from non-zero to zero (stop)
        bool significant_change = (abs(new_x - last_rec_x) > 20 || abs(new_y - last_rec_y) > 20);
        bool new_movement = (new_x != 0 || new_y != 0) && (last_rec_x == 0 && last_rec_y == 0);
        
        if (significant_change || new_movement) {
            if (new_x != 0 || new_y != 0) {
                record_action(ACTION_JOYSTICK, new_x, new_y, 500);
                last_rec_x = new_x;
                last_rec_y = new_y;
                ESP_LOGI(TAG, "🎬 REC Joystick: x=%d y=%d", new_x, new_y);
            }
        }
        // Reset tracking when joystick returns to center
        if (new_x == 0 && new_y == 0) {
            last_rec_x = 0;
            last_rec_y = 0;
        }
        
        if (new_a == 1) {
            record_action(ACTION_BUTTON_A, 0, 0, 300);
            ESP_LOGI(TAG, "🎬 REC Button A");
        }
        if (new_b == 1) {
            record_action(ACTION_BUTTON_B, 0, 0, 300);
            ESP_LOGI(TAG, "🎬 REC Button B");
        }
        if (new_mode_x == 1) {
            record_action(ACTION_ROLL_MODE, 0, 0, 0);
            ESP_LOGI(TAG, "🎬 REC Roll Mode");
        }
        if (new_mode_y == 1) {
            record_action(ACTION_WALK_MODE, 0, 0, 0);
            ESP_LOGI(TAG, "🎬 REC Walk Mode");
        }
    }
    
    // Test mode
    if (get_query_param(req, "test", val, sizeof(val)) == ESP_OK && atoi(val) == 1) {
        state->test_mode_active = true;
        state->test_cycles_remaining = 5;
        ESP_LOGI(TAG, "[TEST MODE] Starting 5 cycles...");
    }
    
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

// Handler for calibration endpoint
static esp_err_t calibrate_handler(httpd_req_t *req) {
    calibration_t *cal = get_calibration();
    
    cal->lf_neutral = get_query_param_int(req, "lfn", cal->lf_neutral);
    cal->rf_neutral = get_query_param_int(req, "rfn", cal->rf_neutral);
    cal->lffwrs = get_query_param_int(req, "lff", cal->lffwrs);
    cal->rffwrs = get_query_param_int(req, "rff", cal->rffwrs);
    cal->lfbwrs = get_query_param_int(req, "lfb", cal->lfbwrs);
    cal->rfbwrs = get_query_param_int(req, "rfb", cal->rfbwrs);
    cal->la0 = get_query_param_int(req, "la0", cal->la0);
    cal->ra0 = get_query_param_int(req, "ra0", cal->ra0);
    cal->latl = get_query_param_int(req, "latl", cal->latl);
    cal->ratl = get_query_param_int(req, "ratl", cal->ratl);
    cal->latr = get_query_param_int(req, "latr", cal->latr);
    cal->ratr = get_query_param_int(req, "ratr", cal->ratr);
    cal->la1 = get_query_param_int(req, "la1", cal->la1);
    cal->ra1 = get_query_param_int(req, "ra1", cal->ra1);
    cal->roll_lf_fwd_speed = get_query_param_int(req, "rlff", cal->roll_lf_fwd_speed);
    cal->roll_lf_bwd_speed = get_query_param_int(req, "rlfb", cal->roll_lf_bwd_speed);
    cal->roll_rf_fwd_speed = get_query_param_int(req, "rrff", cal->roll_rf_fwd_speed);
    cal->roll_rf_bwd_speed = get_query_param_int(req, "rrfb", cal->roll_rf_bwd_speed);
    cal->transform_ll_speed = get_query_param_int(req, "tll", cal->transform_ll_speed);
    cal->transform_rl_speed = get_query_param_int(req, "trl", cal->transform_rl_speed);
    cal->turn_left_speed = get_query_param_int(req, "tls", cal->turn_left_speed);
    cal->turn_right_speed = get_query_param_int(req, "trs", cal->turn_right_speed);
    cal->combo_lf_speed = get_query_param_int(req, "clf", cal->combo_lf_speed);
    cal->combo_rf_speed = get_query_param_int(req, "crf", cal->combo_rf_speed);
    
    // Battery alert toggle
    char balert_str[8];
    if (get_query_param(req, "balert", balert_str, sizeof(balert_str)) == ESP_OK) {
        cal->battery_alert_enabled = (strcmp(balert_str, "1") == 0);
        ESP_LOGI(TAG, "🔋 Battery Alert set to: %s", cal->battery_alert_enabled ? "ENABLED" : "DISABLED");
        // Save immediately when battery alert changes
        save_calibration_to_nvs();
    }
    
    // Mark for auto-save
    settings_changed = true;
    last_setting_change = esp_log_timestamp();
    
    ESP_LOGI(TAG, "Cal: LFN=%d RFN=%d LFF=%d RFF=%d",
             cal->lf_neutral, cal->rf_neutral, cal->lffwrs, cal->rffwrs);
    
    // Save immediately for now
    save_calibration_to_nvs();
    
    httpd_resp_sendstr(req, "Settings applied and saved!");
    return ESP_OK;
}

// Handler for get calibration endpoint
static esp_err_t get_cal_handler(httpd_req_t *req) {
    calibration_t *cal = get_calibration();
    
    char json[896];
    snprintf(json, sizeof(json),
             "{\"lfn\":%d,\"rfn\":%d,\"lff\":%d,\"rff\":%d,"
             "\"lfb\":%d,\"rfb\":%d,\"la0\":%d,\"ra0\":%d,"
             "\"latl\":%d,\"ratl\":%d,\"latr\":%d,\"ratr\":%d,"
             "\"la1\":%d,\"ra1\":%d,\"rlff\":%d,\"rlfb\":%d,"
             "\"rrff\":%d,\"rrfb\":%d,\"tll\":%d,\"trl\":%d,"
             "\"tls\":%d,\"trs\":%d,\"clf\":%d,\"crf\":%d,\"balert\":%d}",
             cal->lf_neutral, cal->rf_neutral, cal->lffwrs, cal->rffwrs,
             cal->lfbwrs, cal->rfbwrs, cal->la0, cal->ra0,
             cal->latl, cal->ratl, cal->latr, cal->ratr,
             cal->la1, cal->ra1, 
             cal->roll_lf_fwd_speed, cal->roll_lf_bwd_speed,
             cal->roll_rf_fwd_speed, cal->roll_rf_bwd_speed,
             cal->transform_ll_speed, cal->transform_rl_speed,
             cal->turn_left_speed, cal->turn_right_speed,
             cal->combo_lf_speed, cal->combo_rf_speed,
             cal->battery_alert_enabled ? 1 : 0);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, json);
    return ESP_OK;
}

// Handler for LED control
static esp_err_t led_handler(httpd_req_t *req) {
    led_state_t *led = get_led_state();
    
    // Check if save request
    char val[16];
    if (get_query_param(req, "save", val, sizeof(val)) == ESP_OK && atoi(val) == 1) {
        save_led_state_to_nvs();
        httpd_resp_sendstr(req, "LED state saved!");
        return ESP_OK;
    }
    
    // Get color values
    if (get_query_param(req, "r", val, sizeof(val)) == ESP_OK) {
        led->r = (uint8_t)atoi(val);
    }
    if (get_query_param(req, "g", val, sizeof(val)) == ESP_OK) {
        led->g = (uint8_t)atoi(val);
    }
    if (get_query_param(req, "b", val, sizeof(val)) == ESP_OK) {
        led->b = (uint8_t)atoi(val);
    }
    if (get_query_param(req, "br", val, sizeof(val)) == ESP_OK) {
        led->brightness = (uint8_t)atoi(val);
    }
    if (get_query_param(req, "mode", val, sizeof(val)) == ESP_OK) {
        led->mode = (led_mode_t)atoi(val);
    }
    if (get_query_param(req, "speed", val, sizeof(val)) == ESP_OK) {
        led->speed = (uint16_t)atoi(val);
    }
    
    // Update LED strip
    ninja_led_update();
    
    ESP_LOGI(TAG, "🌈 LED: R=%d G=%d B=%d Br=%d Mode=%d Speed=%d", 
             led->r, led->g, led->b, led->brightness, led->mode, led->speed);
    
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

// Handler for getting LED state
static esp_err_t get_led_handler(httpd_req_t *req) {
    led_state_t *led = get_led_state();
    
    char json[128];
    snprintf(json, sizeof(json),
             "{\"r\":%d,\"g\":%d,\"b\":%d,\"br\":%d,\"mode\":%d,\"speed\":%d}",
             led->r, led->g, led->b, led->brightness, led->mode, led->speed);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, json);
    return ESP_OK;
}

// Handler for reset calibration to defaults
static esp_err_t reset_cal_handler(httpd_req_t *req) {
    reset_calibration_to_defaults();
    save_calibration_to_nvs();
    
    httpd_resp_sendstr(req, "Reset to factory defaults!");
    return ESP_OK;
}

// Handler for home endpoint
static esp_err_t home_handler(httpd_req_t *req) {
    // Record home action if recording
    recording_state_t *rec = get_recording_state();
    if (rec->is_recording) {
        record_action(ACTION_HOME, 0, 0, 0);
        ESP_LOGI(TAG, "🎬 REC Home");
    }
    
    set_manual_mode(false);  // Tắt manual mode khi về HOME
    go_home();
    httpd_resp_sendstr(req, "Robot returned to HOME position");
    return ESP_OK;
}

// Handler for free movement endpoint (Forward/Backward with custom duration)
static esp_err_t move_handler(httpd_req_t *req) {
    char dir[16];
    if (get_query_param(req, "dir", dir, sizeof(dir)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing dir param");
        return ESP_FAIL;
    }
    
    control_state_t *state = get_control_state();
    recording_state_t *rec = get_recording_state();
    int duration = get_query_param_int(req, "duration", 0);
    
    if (strcmp(dir, "fwd") == 0) {
        // Record if recording
        if (rec->is_recording) {
            record_action(ACTION_MOVE_FWD, duration > 0 ? duration : 1000, 0, 0);
            ESP_LOGI(TAG, "🎬 REC Move Forward (dur=%d)", duration);
        }
        // Forward movement
        state->j_y = 80;
        state->j_x = 0;
        state->move_duration_ms = duration;
        ESP_LOGI(TAG, "🚀 Move FORWARD (duration=%dms, 0=hold)", duration);
        httpd_resp_sendstr(req, "Moving forward");
    } else if (strcmp(dir, "bwd") == 0) {
        // Record if recording
        if (rec->is_recording) {
            record_action(ACTION_MOVE_BWD, duration > 0 ? duration : 1000, 0, 0);
            ESP_LOGI(TAG, "🎬 REC Move Backward (dur=%d)", duration);
        }
        // Backward movement
        state->j_y = -80;
        state->j_x = 0;
        state->move_duration_ms = duration;
        ESP_LOGI(TAG, "🚀 Move BACKWARD (duration=%dms, 0=hold)", duration);
        httpd_resp_sendstr(req, "Moving backward");
    } else if (strcmp(dir, "stop") == 0) {
        // Record if recording
        if (rec->is_recording) {
            record_action(ACTION_MOVE_STOP, 0, 0, 0);
            ESP_LOGI(TAG, "🎬 REC Move Stop");
        }
        // Stop movement
        state->j_y = 0;
        state->j_x = 0;
        state->move_duration_ms = 0;
        ESP_LOGI(TAG, "🛑 Move STOP");
        httpd_resp_sendstr(req, "Stopped");
    } else {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid dir param");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

// Handler for test foot endpoint
static esp_err_t testfoot_handler(httpd_req_t *req) {
    char foot[16];
    if (get_query_param(req, "foot", foot, sizeof(foot)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing foot param");
        return ESP_FAIL;
    }
    
    recording_state_t *rec = get_recording_state();
    
    if (strcmp(foot, "left") == 0) {
        if (rec->is_recording) {
            record_action(ACTION_TEST_LF, 0, 0, 1000);
            ESP_LOGI(TAG, "🎬 REC Test Left Foot");
        }
        test_left_foot();
        httpd_resp_sendstr(req, "Left foot running");
    } else if (strcmp(foot, "right") == 0) {
        if (rec->is_recording) {
            record_action(ACTION_TEST_RF, 0, 0, 1000);
            ESP_LOGI(TAG, "🎬 REC Test Right Foot");
        }
        test_right_foot();
        httpd_resp_sendstr(req, "Right foot running");
    } else if (strcmp(foot, "both") == 0) {
        if (rec->is_recording) {
            record_action(ACTION_TEST_BOTH, 0, 0, 3000);
            ESP_LOGI(TAG, "🎬 REC Test Both Feet");
        }
        // Run in background task to avoid blocking HTTP handler
        web_task_params_t *params = malloc(sizeof(web_task_params_t));
        params->task_type = WEB_TASK_TEST_BOTH_FEET;
        params->param1 = 0;
        params->param2 = 0;
        xTaskCreate(web_action_task, "web_both_feet", 4096, params, 5, NULL);
        httpd_resp_sendstr(req, "Both feet running for 3s (background)");
    } else if (strcmp(foot, "stop") == 0) {
        if (rec->is_recording) {
            record_action(ACTION_TEST_STOP, 0, 0, 0);
            ESP_LOGI(TAG, "🎬 REC Stop Feet Test");
        }
        stop_feet_test();
        httpd_resp_sendstr(req, "Both feet stopped");
    } else {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid foot param");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

// Handler for tilt endpoint
static esp_err_t tilt_handler(httpd_req_t *req) {
    char dir[16];
    if (get_query_param(req, "dir", dir, sizeof(dir)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing dir param");
        return ESP_FAIL;
    }
    
    recording_state_t *rec = get_recording_state();
    
    if (strcmp(dir, "left") == 0) {
        if (rec->is_recording) {
            record_action(ACTION_TILT_LEFT, 0, 0, 500);
            ESP_LOGI(TAG, "🎬 REC Tilt Left");
        }
        ninja_tilt_left();
        httpd_resp_sendstr(req, "Tilted LEFT");
    } else if (strcmp(dir, "right") == 0) {
        if (rec->is_recording) {
            record_action(ACTION_TILT_RIGHT, 0, 0, 500);
            ESP_LOGI(TAG, "🎬 REC Tilt Right");
        }
        ninja_tilt_right();
        httpd_resp_sendstr(req, "Tilted RIGHT");
    } else {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid dir param");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

// Handler for leg rhythm/dance
static esp_err_t rhythm_handler(httpd_req_t *req) {
    char leg[16];
    if (get_query_param(req, "leg", leg, sizeof(leg)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing leg param");
        return ESP_FAIL;
    }
    
    recording_state_t *rec = get_recording_state();
    
    if (strcmp(leg, "left") == 0) {
        // Record rhythm action if recording
        if (rec->is_recording) {
            record_action(ACTION_RHYTHM_LEFT, 0, 0, 0);
            ESP_LOGI(TAG, "🎬 REC Left Leg Rhythm");
        }
        
        // Run in background task to avoid blocking
        web_task_params_t *params = malloc(sizeof(web_task_params_t));
        params->task_type = WEB_TASK_RHYTHM_LEFT;
        params->param1 = 0;
        params->param2 = 0;
        xTaskCreate(web_action_task, "web_rhythm_l", 4096, params, 5, NULL);
        httpd_resp_sendstr(req, "Left leg rhythm started (34-45-65 x3)");
    } else if (strcmp(leg, "right") == 0) {
        // Record rhythm action if recording
        if (rec->is_recording) {
            record_action(ACTION_RHYTHM_RIGHT, 0, 0, 0);
            ESP_LOGI(TAG, "🎬 REC Right Leg Rhythm");
        }
        
        // Run in background task to avoid blocking
        web_task_params_t *params = malloc(sizeof(web_task_params_t));
        params->task_type = WEB_TASK_RHYTHM_RIGHT;
        params->param1 = 0;
        params->param2 = 0;
        xTaskCreate(web_action_task, "web_rhythm_r", 4096, params, 5, NULL);
        httpd_resp_sendstr(req, "Right leg rhythm started (140-150-170 x3)");
    } else {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid leg param");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

// Handler for turn left/right (like LLM turn action)
static esp_err_t turn_handler(httpd_req_t *req) {
    char dir[16];
    if (get_query_param(req, "dir", dir, sizeof(dir)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing dir param");
        return ESP_FAIL;
    }
    
    int speed_ms = get_query_param_int(req, "speed", 500);  // Default 500ms delay
    if (speed_ms < 100) speed_ms = 100;
    if (speed_ms > 1500) speed_ms = 1500;
    
    // Record turn action if recording
    recording_state_t *rec = get_recording_state();
    
    ESP_LOGI(TAG, "Turn %s - Speed: %dms (background task)", dir, speed_ms);
    
    // Run in background task to avoid blocking HTTP handler
    web_task_params_t *params = malloc(sizeof(web_task_params_t));
    params->param1 = speed_ms;
    params->param2 = 0;
    
    if (strcmp(dir, "left") == 0) {
        if (rec->is_recording) {
            record_action(ACTION_TURN_LEFT, speed_ms, 0, 0);
            ESP_LOGI(TAG, "🎬 REC Turn Left (speed=%d)", speed_ms);
        }
        params->task_type = WEB_TASK_TURN_LEFT;
        xTaskCreate(web_action_task, "web_turn_left", 4096, params, 5, NULL);
        httpd_resp_sendstr(req, "Turn LEFT started (background)");
    } else if (strcmp(dir, "right") == 0) {
        if (rec->is_recording) {
            record_action(ACTION_TURN_RIGHT, speed_ms, 0, 0);
            ESP_LOGI(TAG, "🎬 REC Turn Right (speed=%d)", speed_ms);
        }
        params->task_type = WEB_TASK_TURN_RIGHT;
        xTaskCreate(web_action_task, "web_turn_right", 4096, params, 5, NULL);
        httpd_resp_sendstr(req, "Turn RIGHT started (background)");
    } else {
        free(params);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid dir param");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

// Handler for direct servo control
static esp_err_t servo_handler(httpd_req_t *req) {
    int ch = get_query_param_int(req, "ch", -1);
    int angle = get_query_param_int(req, "angle", -1);
    
    if (ch < 0 || ch > 3 || angle < 0 || angle > 180) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid params");
        return ESP_FAIL;
    }
    
    // Record servo action if recording
    recording_state_t *rec = get_recording_state();
    if (rec->is_recording) {
        record_action(ACTION_SERVO, ch, angle, 200);
        ESP_LOGI(TAG, "🎬 REC Servo ch=%d angle=%d", ch, angle);
    }
    
    servo_direct_write(ch, angle);
    
    char resp[32];
    snprintf(resp, sizeof(resp), "Servo %d = %d°", ch, angle);
    httpd_resp_sendstr(req, resp);
    return ESP_OK;
}

// Handler for in-place spin rotation
static esp_err_t spin_handler(httpd_req_t *req) {
    int speed = get_query_param_int(req, "speed", 0);
    
    // Clamp speed to -100...100
    if (speed < -100) speed = -100;
    if (speed > 100) speed = 100;
    
    ESP_LOGI(TAG, "Spin: speed=%d (%s)", speed, 
             speed == 0 ? "stop" : (speed < 0 ? "left" : "right"));
    
    // Record action if recording
    recording_state_t *rec = get_recording_state();
    if (rec->is_recording && speed != 0) {
        record_action(ACTION_SPIN, speed, 0, 500);
        ESP_LOGI(TAG, "🎬 REC Spin speed=%d", speed);
    }
    
    ninja_spin_in_place(speed);
    
    char resp[64];
    if (speed == 0) {
        snprintf(resp, sizeof(resp), "Spin stopped");
    } else {
        snprintf(resp, sizeof(resp), "Spinning %s at speed %d", 
                 speed < 0 ? "left" : "right", speed < 0 ? -speed : speed);
    }
    httpd_resp_sendstr(req, resp);
    return ESP_OK;
}

// Handler to disable manual mode
static esp_err_t manualoff_handler(httpd_req_t *req) {
    // Record home action if recording
    recording_state_t *rec = get_recording_state();
    if (rec->is_recording) {
        record_action(ACTION_HOME, 0, 0, 0);
        ESP_LOGI(TAG, "🎬 REC Manual OFF (Home)");
    }
    
    set_manual_mode(false);
    go_home();
    httpd_resp_sendstr(req, "Manual mode OFF");
    return ESP_OK;
}

// Handler for combo actions
static esp_err_t combo_handler(httpd_req_t *req) {
    int combo_id = get_query_param_int(req, "id", 0);
    recording_state_t *rec = get_recording_state();
    
    if (combo_id == 1) {
        // Get LF speed from query (default 1000ms)
        int lf_speed = get_query_param_int(req, "lf_speed", 1000);
        
        // Record combo1 if recording
        if (rec->is_recording) {
            record_action(ACTION_COMBO1, 0, 0, 0);
            ESP_LOGI(TAG, "🎬 REC Combo 1");
        }
        
        // Run in background task
        web_task_params_t *params = malloc(sizeof(web_task_params_t));
        params->task_type = WEB_TASK_COMBO1;
        params->param1 = lf_speed;
        params->param2 = 0;
        xTaskCreate(web_action_task, "web_combo1", 4096, params, 5, NULL);
        
        httpd_resp_sendstr(req, "COMBO 1 started (background)");
    } else if (combo_id == 2) {
        // Get RF speed from query (default 1000ms)
        int rf_speed = get_query_param_int(req, "rf_speed", 1000);
        
        // Record combo2 if recording
        if (rec->is_recording) {
            record_action(ACTION_COMBO2, 0, 0, 0);
            ESP_LOGI(TAG, "🎬 REC Combo 2");
        }
        
        // Run in background task
        web_task_params_t *params = malloc(sizeof(web_task_params_t));
        params->task_type = WEB_TASK_COMBO2;
        params->param1 = rf_speed;
        params->param2 = 0;
        xTaskCreate(web_action_task, "web_combo2", 4096, params, 5, NULL);
        
        httpd_resp_sendstr(req, "COMBO 2 started (background)");
    } else {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid combo id");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

// Handler for action recording
static esp_err_t record_handler(httpd_req_t *req) {
    char action[16] = {0};
    get_query_param(req, "action", action, sizeof(action));
    int slot = get_query_param_int(req, "slot", 0);
    
    if (strcmp(action, "start") == 0) {
        start_recording(slot);
        char resp[64];
        snprintf(resp, sizeof(resp), "Recording slot %d started (max 20 actions)", slot + 1);
        httpd_resp_sendstr(req, resp);
    } else if (strcmp(action, "stop") == 0) {
        recording_state_t* rec = get_recording_state();
        int count = rec->step_count;
        stop_recording();
        char resp[64];
        snprintf(resp, sizeof(resp), "Saved %d actions to NVS", count);
        httpd_resp_sendstr(req, resp);
    } else if (strcmp(action, "play") == 0) {
        action_slot_t* actions = get_action_slot(slot);
        if (actions && actions->count > 0) {
            play_action(slot);
            char resp[64];
            snprintf(resp, sizeof(resp), "Played %d actions from slot %d", actions->count, slot + 1);
            httpd_resp_sendstr(req, resp);
        } else {
            httpd_resp_sendstr(req, "Slot empty");
        }
    } else if (strcmp(action, "info") == 0) {
        // Return JSON with action counts for all slots + recording state
        recording_state_t* rec = get_recording_state();
        char json[128];
        snprintf(json, sizeof(json), 
                 "{\"slots\":[%d,%d,%d],\"recording\":%s,\"rec_slot\":%d,\"rec_count\":%d,\"max\":%d}", 
                 get_action_slot(0)->count,
                 get_action_slot(1)->count,
                 get_action_slot(2)->count,
                 rec->is_recording ? "true" : "false",
                 rec->current_slot,
                 rec->step_count,
                 MAX_ACTIONS);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, json);
    } else {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid action");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

// ========== ACTION TYPE <-> STRING CONVERSION (for JSON export/import) ==========
static const char* web_action_type_to_string(action_type_t type) {
    switch (type) {
        case ACTION_NONE: return "none";
        case ACTION_JOYSTICK: return "joystick";
        case ACTION_BUTTON_A: return "button_a";
        case ACTION_BUTTON_B: return "button_b";
        case ACTION_WALK_MODE: return "walk_mode";
        case ACTION_ROLL_MODE: return "roll_mode";
        case ACTION_HOME: return "home";
        case ACTION_TILT_LEFT: return "tilt_left";
        case ACTION_TILT_RIGHT: return "tilt_right";
        case ACTION_SERVO: return "servo";
        case ACTION_DELAY: return "delay";
        case ACTION_COMBO1: return "combo1";
        case ACTION_COMBO2: return "combo2";
        case ACTION_RHYTHM_LEFT: return "rhythm_left";
        case ACTION_RHYTHM_RIGHT: return "rhythm_right";
        case ACTION_WALK_COMBO_123: return "walk_combo_123";
        case ACTION_WALK_COMBO_345: return "walk_combo_345";
        case ACTION_TEST_LF: return "test_lf";
        case ACTION_TEST_RF: return "test_rf";
        case ACTION_TEST_BOTH: return "test_both";
        case ACTION_TEST_STOP: return "test_stop";
        case ACTION_MOVE_FWD: return "move_fwd";
        case ACTION_MOVE_BWD: return "move_bwd";
        case ACTION_MOVE_STOP: return "move_stop";
        case ACTION_TURN_LEFT: return "turn_left";
        case ACTION_TURN_RIGHT: return "turn_right";
        case ACTION_SPIN: return "spin";
        default: return "unknown";
    }
}

static action_type_t web_string_to_action_type(const char* str) {
    if (!str) return ACTION_NONE;
    if (strcmp(str, "joystick") == 0) return ACTION_JOYSTICK;
    if (strcmp(str, "button_a") == 0) return ACTION_BUTTON_A;
    if (strcmp(str, "button_b") == 0) return ACTION_BUTTON_B;
    if (strcmp(str, "walk_mode") == 0) return ACTION_WALK_MODE;
    if (strcmp(str, "roll_mode") == 0) return ACTION_ROLL_MODE;
    if (strcmp(str, "home") == 0) return ACTION_HOME;
    if (strcmp(str, "tilt_left") == 0) return ACTION_TILT_LEFT;
    if (strcmp(str, "tilt_right") == 0) return ACTION_TILT_RIGHT;
    if (strcmp(str, "servo") == 0) return ACTION_SERVO;
    if (strcmp(str, "delay") == 0) return ACTION_DELAY;
    if (strcmp(str, "combo1") == 0) return ACTION_COMBO1;
    if (strcmp(str, "combo2") == 0) return ACTION_COMBO2;
    if (strcmp(str, "rhythm_left") == 0) return ACTION_RHYTHM_LEFT;
    if (strcmp(str, "rhythm_right") == 0) return ACTION_RHYTHM_RIGHT;
    if (strcmp(str, "walk_combo_123") == 0) return ACTION_WALK_COMBO_123;
    if (strcmp(str, "walk_combo_345") == 0) return ACTION_WALK_COMBO_345;
    if (strcmp(str, "test_lf") == 0) return ACTION_TEST_LF;
    if (strcmp(str, "test_rf") == 0) return ACTION_TEST_RF;
    if (strcmp(str, "test_both") == 0) return ACTION_TEST_BOTH;
    if (strcmp(str, "test_stop") == 0) return ACTION_TEST_STOP;
    if (strcmp(str, "move_fwd") == 0) return ACTION_MOVE_FWD;
    if (strcmp(str, "move_bwd") == 0) return ACTION_MOVE_BWD;
    if (strcmp(str, "move_stop") == 0) return ACTION_MOVE_STOP;
    if (strcmp(str, "turn_left") == 0) return ACTION_TURN_LEFT;
    if (strcmp(str, "turn_right") == 0) return ACTION_TURN_RIGHT;
    if (strcmp(str, "spin") == 0) return ACTION_SPIN;
    return ACTION_NONE;
}

// Handler for wave leg actions
static esp_err_t wave_handler(httpd_req_t *req) {
    char leg[16] = {0};
    get_query_param(req, "leg", leg, sizeof(leg));
    
    web_task_params_t *params = malloc(sizeof(web_task_params_t));
    if (strcmp(leg, "right") == 0) {
        params->task_type = WEB_TASK_WAVE_RIGHT_LEG;
        params->param1 = 0;
        params->param2 = 0;
        xTaskCreate(web_action_task, "web_wave_r", 4096, params, 5, NULL);
        httpd_resp_sendstr(req, "Wave right leg started");
    } else if (strcmp(leg, "left") == 0) {
        params->task_type = WEB_TASK_WAVE_LEFT_LEG;
        params->param1 = 0;
        params->param2 = 0;
        xTaskCreate(web_action_task, "web_wave_l", 4096, params, 5, NULL);
        httpd_resp_sendstr(req, "Wave left leg started");
    } else {
        free(params);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid leg (right/left)");
        return ESP_FAIL;
    }
    return ESP_OK;
}

// Handler for action JSON export/import
static esp_err_t action_json_handler(httpd_req_t *req) {
    char action[16] = {0};
    get_query_param(req, "action", action, sizeof(action));
    int slot = get_query_param_int(req, "slot", 0);
    
    if (strcmp(action, "export") == 0) {
        if (slot < 0 || slot >= 3) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid slot (0-2)");
            return ESP_FAIL;
        }
        
        action_slot_t* s = get_action_slot(slot);
        if (!s || s->count == 0) {
            httpd_resp_sendstr(req, "{\"error\": \"Slot empty\"}");
            return ESP_OK;
        }
        
        cJSON *root = cJSON_CreateObject();
        cJSON_AddStringToObject(root, "name", slot == 0 ? "Otto Ninja Slot 1" : slot == 1 ? "Otto Ninja Slot 2" : "Otto Ninja Slot 3");
        cJSON_AddNumberToObject(root, "slot", slot + 1);
        cJSON_AddNumberToObject(root, "count", s->count);
        
        cJSON *steps_arr = cJSON_CreateArray();
        for (int i = 0; i < s->count; i++) {
            cJSON *step_obj = cJSON_CreateObject();
            cJSON_AddStringToObject(step_obj, "type", web_action_type_to_string(s->steps[i].type));
            cJSON_AddNumberToObject(step_obj, "param1", s->steps[i].param1);
            cJSON_AddNumberToObject(step_obj, "param2", s->steps[i].param2);
            cJSON_AddNumberToObject(step_obj, "duration_ms", s->steps[i].duration_ms);
            cJSON_AddItemToArray(steps_arr, step_obj);
        }
        cJSON_AddItemToObject(root, "steps", steps_arr);
        
        char *json_str = cJSON_Print(root);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_sendstr(req, json_str);
        cJSON_free(json_str);
        cJSON_Delete(root);
        
        ESP_LOGI(TAG, "📤 Exported slot %d (%d actions)", slot + 1, s->count);
        
    } else if (strcmp(action, "import") == 0) {
        if (slot < 0 || slot >= 3) {
            httpd_resp_sendstr(req, "❌ Invalid slot!");
            return ESP_OK;
        }
        
        // Read POST body
        int total_len = req->content_len;
        if (total_len <= 0 || total_len > 4096) {
            httpd_resp_sendstr(req, "❌ Invalid JSON data length!");
            return ESP_OK;
        }
        
        char *buf = malloc(total_len + 1);
        int received = 0;
        while (received < total_len) {
            int ret = httpd_req_recv(req, buf + received, total_len - received);
            if (ret <= 0) {
                free(buf);
                httpd_resp_sendstr(req, "❌ Failed to receive data!");
                return ESP_OK;
            }
            received += ret;
        }
        buf[total_len] = '\0';
        
        cJSON *root = cJSON_Parse(buf);
        free(buf);
        
        if (!root) {
            httpd_resp_sendstr(req, "❌ JSON parse error!");
            return ESP_OK;
        }
        
        cJSON *steps_arr = cJSON_GetObjectItem(root, "steps");
        if (!steps_arr || !cJSON_IsArray(steps_arr)) {
            cJSON_Delete(root);
            httpd_resp_sendstr(req, "❌ Missing 'steps' array!");
            return ESP_OK;
        }
        
        int count = cJSON_GetArraySize(steps_arr);
        if (count > MAX_ACTIONS) count = MAX_ACTIONS;
        if (count <= 0) {
            cJSON_Delete(root);
            httpd_resp_sendstr(req, "❌ No actions in JSON!");
            return ESP_OK;
        }
        
        action_slot_t* s = get_action_slot(slot);
        memset(s, 0, sizeof(action_slot_t));
        s->count = count;
        
        for (int i = 0; i < count; i++) {
            cJSON *step_obj = cJSON_GetArrayItem(steps_arr, i);
            if (!step_obj) break;
            
            cJSON *type_item = cJSON_GetObjectItem(step_obj, "type");
            cJSON *p1_item = cJSON_GetObjectItem(step_obj, "param1");
            cJSON *p2_item = cJSON_GetObjectItem(step_obj, "param2");
            cJSON *dur_item = cJSON_GetObjectItem(step_obj, "duration_ms");
            
            if (type_item && cJSON_IsString(type_item)) {
                s->steps[i].type = web_string_to_action_type(type_item->valuestring);
            }
            if (p1_item && cJSON_IsNumber(p1_item)) {
                s->steps[i].param1 = (int16_t)p1_item->valueint;
            }
            if (p2_item && cJSON_IsNumber(p2_item)) {
                s->steps[i].param2 = (int16_t)p2_item->valueint;
            }
            if (dur_item && cJSON_IsNumber(dur_item)) {
                s->steps[i].duration_ms = (uint16_t)dur_item->valueint;
            }
        }
        
        cJSON_Delete(root);
        save_actions_to_nvs(slot);
        
        char resp[64];
        snprintf(resp, sizeof(resp), "✅ Imported %d actions into Slot %d!", count, slot + 1);
        httpd_resp_sendstr(req, resp);
        
        ESP_LOGI(TAG, "📥 Imported %d actions into slot %d", count, slot + 1);
        
    } else {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid action (export/import)");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

// Handler for OttoDIY dance functions
static esp_err_t dance_handler(httpd_req_t *req) {
    int dance_id = get_query_param_int(req, "id", 0);
    
    if (dance_id < 1 || dance_id > 8) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid dance id (1-8)");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Dance %d requested", dance_id);
    
    switch (dance_id) {
        case 1: ninja_moonwalk(4, 1500, 1); break;
        case 2: ninja_swing(4, 1000, 30); break;
        case 3: ninja_updown(4, 1000, 30); break;
        case 4: ninja_crusaito(4, 1500, 30, 1); break;
        case 5: ninja_flapping(4, 1000, 30, 1); break;
        case 6: ninja_jitter(4, 1000, 20); break;
        case 7: ninja_ascending_turn(4, 1500, 30); break;
        case 8: ninja_tiptoe_swing(4, 1000, 30); break;
    }
    
    char resp[48];
    snprintf(resp, sizeof(resp), "Dance %d executed", dance_id);
    httpd_resp_sendstr(req, resp);
    return ESP_OK;
}

// URI handlers
static const httpd_uri_t uri_root = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = root_handler,
    .user_ctx = NULL
};

static const httpd_uri_t uri_control = {
    .uri = "/control",
    .method = HTTP_GET,
    .handler = control_handler,
    .user_ctx = NULL
};

static const httpd_uri_t uri_calibrate = {
    .uri = "/calibrate",
    .method = HTTP_GET,
    .handler = calibrate_handler,
    .user_ctx = NULL
};

static const httpd_uri_t uri_get_cal = {
    .uri = "/getCal",
    .method = HTTP_GET,
    .handler = get_cal_handler,
    .user_ctx = NULL
};

static const httpd_uri_t uri_led = {
    .uri = "/led",
    .method = HTTP_GET,
    .handler = led_handler,
    .user_ctx = NULL
};

static const httpd_uri_t uri_get_led = {
    .uri = "/get_led",
    .method = HTTP_GET,
    .handler = get_led_handler,
    .user_ctx = NULL
};

static const httpd_uri_t uri_reset_cal = {
    .uri = "/reset_cal",
    .method = HTTP_GET,
    .handler = reset_cal_handler,
    .user_ctx = NULL
};

static const httpd_uri_t uri_home = {
    .uri = "/home",
    .method = HTTP_GET,
    .handler = home_handler,
    .user_ctx = NULL
};

static const httpd_uri_t uri_move = {
    .uri = "/move",
    .method = HTTP_GET,
    .handler = move_handler,
    .user_ctx = NULL
};

static const httpd_uri_t uri_testfoot = {
    .uri = "/testfoot",
    .method = HTTP_GET,
    .handler = testfoot_handler,
    .user_ctx = NULL
};

// Handler for walk combo testing
static esp_err_t phase_handler(httpd_req_t *req) {
    int combo = get_query_param_int(req, "id", 0);
    
    if (combo < 1 || combo > 2) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid combo (1-2)");
        return ESP_FAIL;
    }
    
    // Record walk combo action if recording
    recording_state_t *rec = get_recording_state();
    if (rec->is_recording) {
        if (combo == 1) {
            record_action(ACTION_WALK_COMBO_123, 0, 0, 0);
            ESP_LOGI(TAG, "🎬 REC Walk Combo 1-2-3");
        } else {
            record_action(ACTION_WALK_COMBO_345, 0, 0, 0);
            ESP_LOGI(TAG, "🎬 REC Walk Combo 3-4-5");
        }
    }
    
    ESP_LOGI(TAG, "Walk combo %d test", combo);
    
    // Run in background task
    web_task_params_t *params = malloc(sizeof(web_task_params_t));
    params->task_type = WEB_TASK_WALK_PHASE;
    params->param1 = combo;
    params->param2 = 0;
    xTaskCreate(web_action_task, "web_phase", 4096, params, 5, NULL);
    
    char resp[48];
    snprintf(resp, sizeof(resp), "Walk Combo %d executed", combo);
    httpd_resp_sendstr(req, resp);
    return ESP_OK;
}

static const httpd_uri_t uri_phase = {
    .uri = "/phase",
    .method = HTTP_GET,
    .handler = phase_handler,
    .user_ctx = NULL
};

static const httpd_uri_t uri_tilt = {
    .uri = "/tilt",
    .method = HTTP_GET,
    .handler = tilt_handler,
    .user_ctx = NULL
};

static const httpd_uri_t uri_rhythm = {
    .uri = "/rhythm",
    .method = HTTP_GET,
    .handler = rhythm_handler,
    .user_ctx = NULL
};

static const httpd_uri_t uri_turn = {
    .uri = "/turn",
    .method = HTTP_GET,
    .handler = turn_handler,
    .user_ctx = NULL
};

static const httpd_uri_t uri_servo = {
    .uri = "/servo",
    .method = HTTP_GET,
    .handler = servo_handler,
    .user_ctx = NULL
};

static const httpd_uri_t uri_spin = {
    .uri = "/spin",
    .method = HTTP_GET,
    .handler = spin_handler,
    .user_ctx = NULL
};

static const httpd_uri_t uri_manualoff = {
    .uri = "/manualoff",
    .method = HTTP_GET,
    .handler = manualoff_handler,
    .user_ctx = NULL
};

static const httpd_uri_t uri_combo = {
    .uri = "/combo",
    .method = HTTP_GET,
    .handler = combo_handler,
    .user_ctx = NULL
};

static const httpd_uri_t uri_record = {
    .uri = "/record",
    .method = HTTP_GET,
    .handler = record_handler,
    .user_ctx = NULL
};

static const httpd_uri_t uri_wave = {
    .uri = "/wave",
    .method = HTTP_GET,
    .handler = wave_handler,
    .user_ctx = NULL
};

static const httpd_uri_t uri_action_json_get = {
    .uri = "/action_json",
    .method = HTTP_GET,
    .handler = action_json_handler,
    .user_ctx = NULL
};

static const httpd_uri_t uri_action_json_post = {
    .uri = "/action_json",
    .method = HTTP_POST,
    .handler = action_json_handler,
    .user_ctx = NULL
};

static const httpd_uri_t uri_dance = {
    .uri = "/dance",
    .method = HTTP_GET,
    .handler = dance_handler,
    .user_ctx = NULL
};

httpd_handle_t webserver_start(void) {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;
    config.max_uri_handlers = 30;  // Increased for wave + JSON + dance endpoints
    
    // Load saved actions from NVS on startup
    for (int i = 0; i < 3; i++) {
        load_actions_from_nvs(i);
    }
    
    ESP_LOGI(TAG, "Starting web server on port %d", config.server_port);
    
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &uri_root);
        httpd_register_uri_handler(server, &uri_control);
        httpd_register_uri_handler(server, &uri_calibrate);
        httpd_register_uri_handler(server, &uri_get_cal);
        httpd_register_uri_handler(server, &uri_led);
        httpd_register_uri_handler(server, &uri_get_led);
        httpd_register_uri_handler(server, &uri_reset_cal);
        httpd_register_uri_handler(server, &uri_home);
        httpd_register_uri_handler(server, &uri_move);
        httpd_register_uri_handler(server, &uri_testfoot);
        httpd_register_uri_handler(server, &uri_phase);
        httpd_register_uri_handler(server, &uri_tilt);
        httpd_register_uri_handler(server, &uri_rhythm);
        httpd_register_uri_handler(server, &uri_turn);
        httpd_register_uri_handler(server, &uri_servo);
        httpd_register_uri_handler(server, &uri_spin);
        httpd_register_uri_handler(server, &uri_manualoff);
        httpd_register_uri_handler(server, &uri_combo);
        httpd_register_uri_handler(server, &uri_record);
        httpd_register_uri_handler(server, &uri_wave);
        httpd_register_uri_handler(server, &uri_action_json_get);
        httpd_register_uri_handler(server, &uri_action_json_post);
        httpd_register_uri_handler(server, &uri_dance);
        
        ESP_LOGI(TAG, "Web server started!");
        return server;
    }
    
    ESP_LOGE(TAG, "Failed to start web server!");
    return NULL;
}

void webserver_stop(httpd_handle_t server) {
    if (server) {
        httpd_stop(server);
    }
}
