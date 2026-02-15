/*
 * Xingzhi Cube Robot Control Implementation
 * Based on Otto Ninja - 4 Servo Version
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "led_strip.h"
#include "robot_control.h"
#include "config.h"

static const char *TAG = "robot_ctrl";

// Servo configuration
#define SERVO_MIN_PULSEWIDTH_US 544
#define SERVO_MAX_PULSEWIDTH_US 2400
#define SERVO_MIN_DEGREE        0
#define SERVO_MAX_DEGREE        180
#define SERVO_FREQ_HZ           50
#define LEDC_TIMER              LEDC_TIMER_1
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_14_BIT
#define LEDC_FULL_DUTY          ((1 << 14) - 1)

// GPIO pin assignments for each servo channel
static const int servo_gpio[SERVO_CH_MAX] = {
    [SERVO_CH_LEFT_FOOT]  = SERVO_LEFT_FOOT_PIN,
    [SERVO_CH_LEFT_LEG]   = SERVO_LEFT_LEG_PIN,
    [SERVO_CH_RIGHT_FOOT] = SERVO_RIGHT_FOOT_PIN,
    [SERVO_CH_RIGHT_LEG]  = SERVO_RIGHT_LEG_PIN,
};

// Track which servos are attached
static bool servo_is_attached[SERVO_CH_MAX] = {false};

// Current calibration settings
static calibration_t calibration = {
    .lf_neutral = CAL_DEFAULT_LF_NEUTRAL,
    .rf_neutral = CAL_DEFAULT_RF_NEUTRAL,
    .lffwrs = CAL_DEFAULT_LFFWRS,
    .rffwrs = CAL_DEFAULT_RFFWRS,
    .lfbwrs = CAL_DEFAULT_LFBWRS,
    .rfbwrs = CAL_DEFAULT_RFBWRS,
    .la0 = CAL_DEFAULT_LA0,
    .ra0 = CAL_DEFAULT_RA0,
    .latl = CAL_DEFAULT_LATL,
    .ratl = CAL_DEFAULT_RATL,
    .latr = CAL_DEFAULT_LATR,
    .ratr = CAL_DEFAULT_RATR,
    .la1 = CAL_DEFAULT_LA1,
    .ra1 = CAL_DEFAULT_RA1,
    .roll_lf_fwd_speed = CAL_DEFAULT_ROLL_LF_FWD,
    .roll_lf_bwd_speed = CAL_DEFAULT_ROLL_LF_BWD,
    .roll_rf_fwd_speed = CAL_DEFAULT_ROLL_RF_FWD,
    .roll_rf_bwd_speed = CAL_DEFAULT_ROLL_RF_BWD,
    .transform_ll_speed = CAL_DEFAULT_TRANSFORM_LL,
    .transform_rl_speed = CAL_DEFAULT_TRANSFORM_RL,
    .turn_left_speed = CAL_DEFAULT_TURN_L,
    .turn_right_speed = CAL_DEFAULT_TURN_R,
    .combo_lf_speed = CAL_DEFAULT_COMBO_LF,
    .combo_rf_speed = CAL_DEFAULT_COMBO_RF,
    .battery_alert_enabled = CAL_DEFAULT_BATTERY_ALERT,
};

// Current control state
static control_state_t control_state = {
    .j_x = 0,
    .j_y = 0,
    .button_x = 0,
    .button_y = 0,
    .manual_mode = false,
    .move_duration_ms = 0,
};

// Robot mode
static robot_mode_t current_mode = MODE_WALK;

// Walking state tracking
static bool was_moving = false;
static bool is_at_home = true;
static bool walk_cal_logged = false;
static bool walk_cycle_reset = false;
static uint32_t current_millis1 = 0;
static bool walk_cycle_active = false;
static bool walk_trigger_armed = true;

// Action recording
static action_slot_t action_slots[MAX_ACTION_SLOTS] = {0};
static recording_state_t recording_state = {
    .is_recording = false,
    .current_slot = 0,
    .step_count = 0
};

// Convert angle to PWM duty
static uint32_t angle_to_duty(int angle) {
    if (angle < SERVO_MIN_DEGREE) angle = SERVO_MIN_DEGREE;
    if (angle > SERVO_MAX_DEGREE) angle = SERVO_MAX_DEGREE;
    
    uint32_t pulse_width = SERVO_MIN_PULSEWIDTH_US + 
        (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) * angle / SERVO_MAX_DEGREE;
    uint32_t duty = (pulse_width * LEDC_FULL_DUTY) / 20000;
    
    return duty;
}

// Initialize servo PWM
void robot_control_init(void) {
    ESP_LOGI(TAG, "Initializing robot control with 4 servos...");
    
    // Configure LEDC timer
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = SERVO_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));
    
    // Configure LEDC channels for each servo (offset +1 to avoid backlight channel)
    for (int ch = 0; ch < SERVO_CH_MAX; ch++) {
        ledc_channel_config_t channel_conf = {
            .gpio_num = servo_gpio[ch],
            .speed_mode = LEDC_MODE,
            .channel = (ledc_channel_t)(ch + 1),
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER,
            .duty = 0,
            .hpoint = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&channel_conf));
        servo_is_attached[ch] = false;
    }
    
    // Load calibration from NVS
    load_calibration_from_nvs();
    
    // Load saved actions from NVS
    ESP_LOGI(TAG, "Loading saved actions from NVS...");
    for (int i = 0; i < MAX_ACTION_SLOTS; i++) {
        load_actions_from_nvs(i);
    }
    
    // Initialize LED strip
    ninja_led_init();
    
    ESP_LOGI(TAG, "Robot control initialized - GPIO: LF=%d, LL=%d, RF=%d, RL=%d",
             SERVO_LEFT_FOOT_PIN, SERVO_LEFT_LEG_PIN, 
             SERVO_RIGHT_FOOT_PIN, SERVO_RIGHT_LEG_PIN);
    
    // Always go to HOME position after initialization
    vTaskDelay(pdMS_TO_TICKS(500));  // Small delay to ensure hardware is ready
    go_home();
}

void servo_write(servo_channel_t channel, int angle) {
    if (channel >= SERVO_CH_MAX) return;
    
    uint32_t duty = angle_to_duty(angle);
    ledc_set_duty(LEDC_MODE, (ledc_channel_t)(channel + 1), duty);
    ledc_update_duty(LEDC_MODE, (ledc_channel_t)(channel + 1));
}

void servo_attach(servo_channel_t channel) {
    if (channel >= SERVO_CH_MAX) return;
    servo_is_attached[channel] = true;
}

void servo_detach(servo_channel_t channel) {
    if (channel >= SERVO_CH_MAX) return;
    
    ledc_set_duty(LEDC_MODE, (ledc_channel_t)(channel + 1), 0);
    ledc_update_duty(LEDC_MODE, (ledc_channel_t)(channel + 1));
    servo_is_attached[channel] = false;
}

bool servo_attached(servo_channel_t channel) {
    if (channel >= SERVO_CH_MAX) return false;
    return servo_is_attached[channel];
}

// ========== NVS FUNCTIONS ==========

void reset_calibration_to_defaults(void) {
    calibration.lf_neutral = CAL_DEFAULT_LF_NEUTRAL;
    calibration.rf_neutral = CAL_DEFAULT_RF_NEUTRAL;
    calibration.lffwrs = CAL_DEFAULT_LFFWRS;
    calibration.rffwrs = CAL_DEFAULT_RFFWRS;
    calibration.lfbwrs = CAL_DEFAULT_LFBWRS;
    calibration.rfbwrs = CAL_DEFAULT_RFBWRS;
    calibration.la0 = CAL_DEFAULT_LA0;
    calibration.ra0 = CAL_DEFAULT_RA0;
    calibration.latl = CAL_DEFAULT_LATL;
    calibration.ratl = CAL_DEFAULT_RATL;
    calibration.latr = CAL_DEFAULT_LATR;
    calibration.ratr = CAL_DEFAULT_RATR;
    calibration.la1 = CAL_DEFAULT_LA1;
    calibration.ra1 = CAL_DEFAULT_RA1;
    calibration.roll_lf_fwd_speed = CAL_DEFAULT_ROLL_LF_FWD;
    calibration.roll_lf_bwd_speed = CAL_DEFAULT_ROLL_LF_BWD;
    calibration.roll_rf_fwd_speed = CAL_DEFAULT_ROLL_RF_FWD;
    calibration.roll_rf_bwd_speed = CAL_DEFAULT_ROLL_RF_BWD;
    calibration.transform_ll_speed = CAL_DEFAULT_TRANSFORM_LL;
    calibration.transform_rl_speed = CAL_DEFAULT_TRANSFORM_RL;
    calibration.turn_left_speed = CAL_DEFAULT_TURN_L;
    calibration.turn_right_speed = CAL_DEFAULT_TURN_R;
    calibration.combo_lf_speed = CAL_DEFAULT_COMBO_LF;
    calibration.combo_rf_speed = CAL_DEFAULT_COMBO_RF;
    calibration.battery_alert_enabled = CAL_DEFAULT_BATTERY_ALERT;
    ESP_LOGI(TAG, "Calibration reset to defaults");
}

void save_calibration_to_nvs(void) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open("robot_cal", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS: %s", esp_err_to_name(err));
        return;
    }
    
    nvs_set_i32(handle, "lfn", calibration.lf_neutral);
    nvs_set_i32(handle, "rfn", calibration.rf_neutral);
    nvs_set_i32(handle, "lff", calibration.lffwrs);
    nvs_set_i32(handle, "rff", calibration.rffwrs);
    nvs_set_i32(handle, "lfb", calibration.lfbwrs);
    nvs_set_i32(handle, "rfb", calibration.rfbwrs);
    nvs_set_i32(handle, "la0", calibration.la0);
    nvs_set_i32(handle, "ra0", calibration.ra0);
    nvs_set_i32(handle, "latl", calibration.latl);
    nvs_set_i32(handle, "ratl", calibration.ratl);
    nvs_set_i32(handle, "latr", calibration.latr);
    nvs_set_i32(handle, "ratr", calibration.ratr);
    nvs_set_i32(handle, "la1", calibration.la1);
    nvs_set_i32(handle, "ra1", calibration.ra1);
    nvs_set_i32(handle, "rlff", calibration.roll_lf_fwd_speed);
    nvs_set_i32(handle, "rlfb", calibration.roll_lf_bwd_speed);
    nvs_set_i32(handle, "rrff", calibration.roll_rf_fwd_speed);
    nvs_set_i32(handle, "rrfb", calibration.roll_rf_bwd_speed);
    nvs_set_i32(handle, "tll", calibration.transform_ll_speed);
    nvs_set_i32(handle, "trl", calibration.transform_rl_speed);
    nvs_set_i32(handle, "tls", calibration.turn_left_speed);
    nvs_set_i32(handle, "trs", calibration.turn_right_speed);
    nvs_set_i32(handle, "clf", calibration.combo_lf_speed);
    nvs_set_i32(handle, "crf", calibration.combo_rf_speed);
    nvs_set_i32(handle, "balert", calibration.battery_alert_enabled ? 1 : 0);
    
    nvs_commit(handle);
    nvs_close(handle);
    ESP_LOGI(TAG, "Calibration saved to NVS");
}

void load_calibration_from_nvs(void) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open("robot_cal", NVS_READONLY, &handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS not found, using defaults");
        return;
    }
    
    int32_t val;
    if (nvs_get_i32(handle, "lfn", &val) == ESP_OK) calibration.lf_neutral = val;
    if (nvs_get_i32(handle, "rfn", &val) == ESP_OK) calibration.rf_neutral = val;
    if (nvs_get_i32(handle, "lff", &val) == ESP_OK) calibration.lffwrs = val;
    if (nvs_get_i32(handle, "rff", &val) == ESP_OK) calibration.rffwrs = val;
    if (nvs_get_i32(handle, "lfb", &val) == ESP_OK) calibration.lfbwrs = val;
    if (nvs_get_i32(handle, "rfb", &val) == ESP_OK) calibration.rfbwrs = val;
    if (nvs_get_i32(handle, "la0", &val) == ESP_OK) calibration.la0 = val;
    if (nvs_get_i32(handle, "ra0", &val) == ESP_OK) calibration.ra0 = val;
    if (nvs_get_i32(handle, "latl", &val) == ESP_OK) calibration.latl = val;
    if (nvs_get_i32(handle, "ratl", &val) == ESP_OK) calibration.ratl = val;
    if (nvs_get_i32(handle, "latr", &val) == ESP_OK) calibration.latr = val;
    if (nvs_get_i32(handle, "ratr", &val) == ESP_OK) calibration.ratr = val;
    if (nvs_get_i32(handle, "la1", &val) == ESP_OK) calibration.la1 = val;
    if (nvs_get_i32(handle, "ra1", &val) == ESP_OK) calibration.ra1 = val;
    if (nvs_get_i32(handle, "rlff", &val) == ESP_OK) calibration.roll_lf_fwd_speed = val;
    if (nvs_get_i32(handle, "rlfb", &val) == ESP_OK) calibration.roll_lf_bwd_speed = val;
    if (nvs_get_i32(handle, "rrff", &val) == ESP_OK) calibration.roll_rf_fwd_speed = val;
    if (nvs_get_i32(handle, "rrfb", &val) == ESP_OK) calibration.roll_rf_bwd_speed = val;
    if (nvs_get_i32(handle, "tll", &val) == ESP_OK) calibration.transform_ll_speed = val;
    if (nvs_get_i32(handle, "trl", &val) == ESP_OK) calibration.transform_rl_speed = val;
    if (nvs_get_i32(handle, "tls", &val) == ESP_OK) calibration.turn_left_speed = val;
    if (nvs_get_i32(handle, "trs", &val) == ESP_OK) calibration.turn_right_speed = val;
    if (nvs_get_i32(handle, "clf", &val) == ESP_OK) calibration.combo_lf_speed = val;
    if (nvs_get_i32(handle, "crf", &val) == ESP_OK) calibration.combo_rf_speed = val;
    if (nvs_get_i32(handle, "balert", &val) == ESP_OK) calibration.battery_alert_enabled = (val != 0);
    
    nvs_close(handle);
    ESP_LOGI(TAG, "Calibration loaded from NVS");
}

calibration_t* get_calibration(void) {
    return &calibration;
}

void set_calibration(const calibration_t* cal) {
    if (cal) {
        memcpy(&calibration, cal, sizeof(calibration_t));
    }
}

control_state_t* get_control_state(void) {
    return &control_state;
}

robot_mode_t get_robot_mode(void) {
    return current_mode;
}

// ========== MOVEMENT FUNCTIONS ==========

void go_home(void) {
    ESP_LOGI(TAG, "Going to HOME position...");
    
    servo_attach(SERVO_CH_LEFT_FOOT);
    servo_attach(SERVO_CH_RIGHT_FOOT);
    servo_write(SERVO_CH_LEFT_FOOT, calibration.lf_neutral);
    servo_write(SERVO_CH_RIGHT_FOOT, calibration.rf_neutral);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    servo_detach(SERVO_CH_LEFT_FOOT);
    servo_detach(SERVO_CH_RIGHT_FOOT);
    
    servo_attach(SERVO_CH_LEFT_LEG);
    servo_attach(SERVO_CH_RIGHT_LEG);
    servo_write(SERVO_CH_LEFT_LEG, calibration.la0);
    servo_write(SERVO_CH_RIGHT_LEG, calibration.ra0);
    vTaskDelay(pdMS_TO_TICKS(400));
    
    ESP_LOGI(TAG, "HOME position reached: LL=%d, RL=%d", calibration.la0, calibration.ra0);
    is_at_home = true;
}

static void smooth_transform_legs(int ll_target, int rl_target) {
    static int ll_current = -1;
    static int rl_current = -1;
    
    if (ll_current < 0) ll_current = calibration.la0;
    if (rl_current < 0) rl_current = calibration.ra0;
    
    int ll_dir = (ll_target > ll_current) ? 1 : -1;
    int rl_dir = (rl_target > rl_current) ? 1 : -1;
    
    int ll_steps_remaining = abs(ll_target - ll_current);
    int rl_steps_remaining = abs(rl_target - rl_current);
    
    ESP_LOGI(TAG, "Smooth transform: LL %d->%d, RL %d->%d", ll_current, ll_target, rl_current, rl_target);
    
    // At 100Hz tick rate, 1 tick = 10ms. Advance timer by 10 per tick
    // to match original 1ms-per-iteration timing.
    int ll_timer = 0, rl_timer = 0;
    int max_iterations = 500;
    
    while ((ll_steps_remaining > 0 || rl_steps_remaining > 0) && max_iterations > 0) {
        max_iterations--;
        
        if (ll_steps_remaining > 0) {
            ll_timer += 10;
            while (ll_timer >= calibration.transform_ll_speed && ll_steps_remaining > 0) {
                ll_current += ll_dir;
                ll_steps_remaining--;
                ll_timer -= calibration.transform_ll_speed;
            }
            servo_write(SERVO_CH_LEFT_LEG, ll_current);
        }
        
        if (rl_steps_remaining > 0) {
            rl_timer += 10;
            while (rl_timer >= calibration.transform_rl_speed && rl_steps_remaining > 0) {
                rl_current += rl_dir;
                rl_steps_remaining--;
                rl_timer -= calibration.transform_rl_speed;
            }
            servo_write(SERVO_CH_RIGHT_LEG, rl_current);
        }
        
        vTaskDelay(1);  // 1 tick = 10ms at 100Hz
    }
    
    servo_write(SERVO_CH_LEFT_LEG, ll_target);
    servo_write(SERVO_CH_RIGHT_LEG, rl_target);
    ll_current = ll_target;
    rl_current = rl_target;
}

void ninja_set_walk(void) {
    ESP_LOGI(TAG, "Setting WALK mode");
    
    // Reset feet to neutral first
    servo_attach(SERVO_CH_LEFT_FOOT);
    servo_attach(SERVO_CH_RIGHT_FOOT);
    servo_write(SERVO_CH_LEFT_FOOT, calibration.lf_neutral);
    servo_write(SERVO_CH_RIGHT_FOOT, calibration.rf_neutral);
    vTaskDelay(pdMS_TO_TICKS(100));
    servo_detach(SERVO_CH_LEFT_FOOT);
    servo_detach(SERVO_CH_RIGHT_FOOT);
    
    // Transform legs to walk position
    servo_attach(SERVO_CH_LEFT_LEG);
    servo_attach(SERVO_CH_RIGHT_LEG);
    
    smooth_transform_legs(calibration.la0, calibration.ra0);
    
    vTaskDelay(pdMS_TO_TICKS(200));
    servo_detach(SERVO_CH_LEFT_LEG);
    servo_detach(SERVO_CH_RIGHT_LEG);
    
    // Reset walk state
    was_moving = false;
    is_at_home = true;
    
    current_mode = MODE_WALK;
    ESP_LOGI(TAG, "WALK mode set (HOME position)");
}

void ninja_set_roll(void) {
    ESP_LOGI(TAG, "Setting ROLL mode");
    
    servo_attach(SERVO_CH_LEFT_LEG);
    servo_attach(SERVO_CH_RIGHT_LEG);
    
    smooth_transform_legs(calibration.la1, calibration.ra1);
    
    // Keep LL/RL attached to hold roll position (4-servo board has no arms to support)
    // DO NOT detach legs servos here!
    
    current_mode = MODE_ROLL;
    ESP_LOGI(TAG, "ROLL mode set (LL/RL holding at la1=%d, ra1=%d)", calibration.la1, calibration.ra1);
}

void ninja_walk_stop(void) {
    servo_write(SERVO_CH_LEFT_FOOT, calibration.lf_neutral);
    servo_write(SERVO_CH_RIGHT_FOOT, calibration.rf_neutral);
    servo_write(SERVO_CH_LEFT_LEG, calibration.la0);
    servo_write(SERVO_CH_RIGHT_LEG, calibration.ra0);
}

void ninja_roll_stop(void) {
    servo_write(SERVO_CH_LEFT_FOOT, calibration.lf_neutral);
    servo_write(SERVO_CH_RIGHT_FOOT, calibration.rf_neutral);
    servo_detach(SERVO_CH_LEFT_FOOT);
    servo_detach(SERVO_CH_RIGHT_FOOT);
}

static int map_value(int x, int in_min, int in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static int constrain_value(int x, int min_val, int max_val) {
    if (x < min_val) return min_val;
    if (x > max_val) return max_val;
    return x;
}

static uint32_t millis(void) {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

// ========== WALKING LOGIC ==========

void ninja_walk(void) {
    int8_t j_x = control_state.j_x;
    int8_t j_y = control_state.j_y;
    
    if (j_x == 0 && j_y == 0) {
        if (walk_cycle_active) {
            ninja_walk_stop();
            servo_detach(SERVO_CH_LEFT_FOOT);
            servo_detach(SERVO_CH_RIGHT_FOOT);
            servo_detach(SERVO_CH_LEFT_LEG);
            servo_detach(SERVO_CH_RIGHT_LEG);
            walk_cycle_active = false;
            walk_cal_logged = false;
            walk_cycle_reset = false;
        }
        walk_trigger_armed = true;
        
        if (was_moving && !is_at_home) {
            was_moving = false;
            is_at_home = true;
            go_home();
        }
        return;
    }
    
    if (!walk_cycle_active && walk_trigger_armed) {
        walk_cycle_active = true;
        walk_trigger_armed = false;
        walk_cycle_reset = false;
    }
    
    if (!walk_cycle_active) return;
    
    if (!walk_cycle_reset) {
        current_millis1 = millis();
        walk_cycle_reset = true;
    }
    
    was_moving = true;
    is_at_home = false;
    
    // Forward walking (j_y >= 0)
    if (j_y >= 0) {
        int lt = map_value(j_x, -100, 100, 200, 700);
        int rt = map_value(j_x, -100, 100, 700, 200);
        
        int interval1 = 250;
        int interval2 = 250 + rt;
        int interval3 = 250 + rt + 250;
        int interval4 = 250 + rt + 250 + lt;
        int interval5 = 250 + rt + 250 + lt + 50;
        
        static int last_phase = 0;
        int current_phase = 0;
        uint32_t elapsed = millis() - current_millis1;
        
        if (elapsed <= interval1) current_phase = 1;
        else if (elapsed <= interval2) current_phase = 2;
        else if (elapsed <= interval3) current_phase = 3;
        else if (elapsed <= interval4) current_phase = 4;
        else current_phase = 5;
        
        if (millis() > current_millis1 + interval5) {
            last_phase = 0;
            current_millis1 = millis();
            walk_cycle_reset = true;
        }
        
        elapsed = millis() - current_millis1;
        
        // Phase 1: Tilt right
        if (elapsed <= interval1) {
            if (current_phase != last_phase) {
                last_phase = current_phase;
            }
            servo_attach(SERVO_CH_LEFT_LEG);
            servo_attach(SERVO_CH_RIGHT_LEG);
            servo_write(SERVO_CH_LEFT_LEG, calibration.latr);
            servo_write(SERVO_CH_RIGHT_LEG, calibration.ratr);
        }
        
        // Phase 2: Right foot forward
        elapsed = millis() - current_millis1;
        if (elapsed >= interval1 && elapsed <= interval2) {
            int right_foot_angle = calibration.rf_neutral - calibration.rffwrs;
            if (current_phase != last_phase) {
                last_phase = current_phase;
                servo_attach(SERVO_CH_RIGHT_FOOT);
            }
            servo_write(SERVO_CH_RIGHT_FOOT, right_foot_angle);
        }
        
        // Phase 3: Tilt left
        elapsed = millis() - current_millis1;
        if (elapsed >= interval2 && elapsed <= interval3) {
            if (current_phase != last_phase) {
                last_phase = current_phase;
                servo_write(SERVO_CH_RIGHT_FOOT, calibration.rf_neutral);
            }
            servo_write(SERVO_CH_LEFT_LEG, calibration.latl);
            servo_write(SERVO_CH_RIGHT_LEG, calibration.ratl);
        }
        
        // Phase 4: Left foot forward
        elapsed = millis() - current_millis1;
        if (elapsed >= interval3 && elapsed <= interval4) {
            int left_foot_angle = calibration.lf_neutral + calibration.lffwrs;
            if (current_phase != last_phase) {
                last_phase = current_phase;
                servo_attach(SERVO_CH_LEFT_FOOT);
            }
            servo_write(SERVO_CH_LEFT_FOOT, left_foot_angle);
        }
        
        // Phase 5: Stop left foot
        elapsed = millis() - current_millis1;
        if (elapsed >= interval4 && elapsed <= interval5) {
            if (current_phase != last_phase) {
                last_phase = current_phase;
                servo_write(SERVO_CH_LEFT_FOOT, calibration.lf_neutral);
            }
        }
    }
    // Backward walking (j_y < 0)
    else {
        int lt = map_value(j_x, -100, 100, 200, 700);
        int rt = map_value(j_x, -100, 100, 700, 200);
        
        int interval1 = 250;
        int interval2 = 250 + rt;
        int interval3 = 250 + rt + 250;
        int interval4 = 250 + rt + 250 + lt;
        int interval5 = 250 + rt + 250 + lt + 50;
        
        static int last_back_phase = 0;
        int current_phase = 0;
        uint32_t elapsed = millis() - current_millis1;
        
        if (elapsed <= interval1) current_phase = 1;
        else if (elapsed <= interval2) current_phase = 2;
        else if (elapsed <= interval3) current_phase = 3;
        else if (elapsed <= interval4) current_phase = 4;
        else current_phase = 5;
        
        if (millis() > current_millis1 + interval5) {
            last_back_phase = 0;
            current_millis1 = millis();
            walk_cycle_reset = true;
        }
        
        elapsed = millis() - current_millis1;
        
        // Phase 1: Tilt right
        if (elapsed <= interval1) {
            if (current_phase != last_back_phase) last_back_phase = current_phase;
            servo_attach(SERVO_CH_LEFT_LEG);
            servo_attach(SERVO_CH_RIGHT_LEG);
            servo_write(SERVO_CH_LEFT_LEG, calibration.latr);
            servo_write(SERVO_CH_RIGHT_LEG, calibration.ratr);
        }
        
        // Phase 2: Right foot backward
        elapsed = millis() - current_millis1;
        if (elapsed >= interval1 && elapsed <= interval2) {
            int right_foot_angle = calibration.rf_neutral + calibration.rfbwrs;
            if (current_phase != last_back_phase) {
                last_back_phase = current_phase;
                servo_attach(SERVO_CH_RIGHT_FOOT);
            }
            servo_write(SERVO_CH_RIGHT_FOOT, right_foot_angle);
        }
        
        // Phase 3: Tilt left
        elapsed = millis() - current_millis1;
        if (elapsed >= interval2 && elapsed <= interval3) {
            if (current_phase != last_back_phase) {
                last_back_phase = current_phase;
                servo_write(SERVO_CH_RIGHT_FOOT, calibration.rf_neutral);
            }
            servo_write(SERVO_CH_LEFT_LEG, calibration.latl);
            servo_write(SERVO_CH_RIGHT_LEG, calibration.ratl);
        }
        
        // Phase 4: Left foot backward
        elapsed = millis() - current_millis1;
        if (elapsed >= interval3 && elapsed <= interval4) {
            int left_foot_angle = calibration.lf_neutral - calibration.lfbwrs;
            if (current_phase != last_back_phase) {
                last_back_phase = current_phase;
                servo_attach(SERVO_CH_LEFT_FOOT);
            }
            servo_write(SERVO_CH_LEFT_FOOT, left_foot_angle);
        }
        
        // Phase 5: Stop left foot
        elapsed = millis() - current_millis1;
        if (elapsed >= interval4 && elapsed <= interval5) {
            if (current_phase != last_back_phase) {
                last_back_phase = current_phase;
                servo_write(SERVO_CH_LEFT_FOOT, calibration.lf_neutral);
            }
        }
    }
}

// ========== ROLLING LOGIC ==========

void ninja_roll(void) {
    int8_t j_x = control_state.j_x;
    int8_t j_y = control_state.j_y;
    
    static bool roll_idle = false;
    
    if ((j_x >= -10) && (j_x <= 10) && (j_y >= -10) && (j_y <= 10)) {
        if (!roll_idle) {
            ninja_roll_stop();
            roll_idle = true;
        }
        return;
    }
    
    roll_idle = false;
    
    if (!servo_attached(SERVO_CH_LEFT_FOOT)) servo_attach(SERVO_CH_LEFT_FOOT);
    if (!servo_attached(SERVO_CH_RIGHT_FOOT)) servo_attach(SERVO_CH_RIGHT_FOOT);
    
    int lf_fwd_speed = calibration.roll_lf_fwd_speed;
    int lf_bwd_speed = calibration.roll_lf_bwd_speed;
    int rf_fwd_speed = calibration.roll_rf_fwd_speed;
    int rf_bwd_speed = calibration.roll_rf_bwd_speed;
    
    int lf_speed_offset = (j_y >= 0) ? lf_fwd_speed : lf_bwd_speed;
    int rf_speed_offset = (j_y >= 0) ? rf_fwd_speed : rf_bwd_speed;
    
    int lws = map_value(j_y, 100, -100, 90 + lf_speed_offset, 90 - lf_speed_offset);
    int rws = map_value(j_y, 100, -100, 90 - rf_speed_offset, 90 + rf_speed_offset);
    
    int avg_speed = (lf_speed_offset + rf_speed_offset) / 2;
    int lwd = map_value(j_x, 100, -100, avg_speed, 0);
    int rwd = map_value(j_x, 100, -100, 0, -avg_speed);
    
    int left_speed = constrain_value(lws + lwd, 90 - lf_speed_offset, 90 + lf_speed_offset);
    int right_speed = constrain_value(rws + rwd, 90 - rf_speed_offset, 90 + rf_speed_offset);
    
    servo_write(SERVO_CH_LEFT_FOOT, left_speed);
    servo_write(SERVO_CH_RIGHT_FOOT, right_speed);
}

// ========== TILT FUNCTIONS ==========

void ninja_tilt_left(void) {
    control_state.manual_mode = true;
    ESP_LOGI(TAG, "Tilt LEFT");
    servo_attach(SERVO_CH_LEFT_LEG);
    servo_attach(SERVO_CH_RIGHT_LEG);
    servo_write(SERVO_CH_LEFT_LEG, calibration.latl);
    servo_write(SERVO_CH_RIGHT_LEG, calibration.ratl);
}

void ninja_tilt_right(void) {
    control_state.manual_mode = true;
    ESP_LOGI(TAG, "Tilt RIGHT");
    servo_attach(SERVO_CH_LEFT_LEG);
    servo_attach(SERVO_CH_RIGHT_LEG);
    servo_write(SERVO_CH_LEFT_LEG, calibration.latr);
    servo_write(SERVO_CH_RIGHT_LEG, calibration.ratr);
}

// ========== ARM FUNCTIONS (no-ops on 4-servo board) ==========

void ninja_left_arm_up(void) {
    // No arm servos on 4-servo board
}

void ninja_left_arm_down(void) {
    // No arm servos on 4-servo board
}

void ninja_right_arm_up(void) {
    // No arm servos on 4-servo board
}

void ninja_right_arm_down(void) {
    // No arm servos on 4-servo board
}

// ========== TEST FUNCTIONS ==========

static bool feet_test_running = false;

void test_left_foot(void) {
    control_state.manual_mode = true;
    feet_test_running = true;
    servo_attach(SERVO_CH_LEFT_FOOT);
    // Rotate left foot forward
    servo_write(SERVO_CH_LEFT_FOOT, calibration.lf_neutral + calibration.lffwrs);
    ESP_LOGI(TAG, "Test LEFT foot forward");
}

void test_right_foot(void) {
    control_state.manual_mode = true;
    feet_test_running = true;
    servo_attach(SERVO_CH_RIGHT_FOOT);
    // Rotate right foot forward  
    servo_write(SERVO_CH_RIGHT_FOOT, calibration.rf_neutral - calibration.rffwrs);
    ESP_LOGI(TAG, "Test RIGHT foot forward");
}

void test_both_feet(void) {
    control_state.manual_mode = true;
    feet_test_running = true;
    servo_attach(SERVO_CH_LEFT_FOOT);
    servo_attach(SERVO_CH_RIGHT_FOOT);
    
    // Run for 3 seconds
    servo_write(SERVO_CH_LEFT_FOOT, calibration.lf_neutral + calibration.lffwrs);
    servo_write(SERVO_CH_RIGHT_FOOT, calibration.rf_neutral - calibration.rffwrs);
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    // Stop
    servo_write(SERVO_CH_LEFT_FOOT, calibration.lf_neutral);
    servo_write(SERVO_CH_RIGHT_FOOT, calibration.rf_neutral);
    servo_detach(SERVO_CH_LEFT_FOOT);
    servo_detach(SERVO_CH_RIGHT_FOOT);
    
    feet_test_running = false;
    control_state.manual_mode = false;
    ESP_LOGI(TAG, "Test BOTH feet completed");
}

void stop_feet_test(void) {
    servo_write(SERVO_CH_LEFT_FOOT, calibration.lf_neutral);
    servo_write(SERVO_CH_RIGHT_FOOT, calibration.rf_neutral);
    servo_detach(SERVO_CH_LEFT_FOOT);
    servo_detach(SERVO_CH_RIGHT_FOOT);
    feet_test_running = false;
    control_state.manual_mode = false;
    ESP_LOGI(TAG, "Feet test STOPPED");
}

// ========== DANCE/RHYTHM FUNCTIONS ==========

void left_leg_rhythm(void) {
    control_state.manual_mode = true;
    servo_attach(SERVO_CH_LEFT_LEG);
    
    // Rhythm pattern: 34 -> 45 -> 65, repeat 3 times
    for (int i = 0; i < 3; i++) {
        servo_write(SERVO_CH_LEFT_LEG, 34);
        vTaskDelay(pdMS_TO_TICKS(200));
        servo_write(SERVO_CH_LEFT_LEG, 45);
        vTaskDelay(pdMS_TO_TICKS(200));
        servo_write(SERVO_CH_LEFT_LEG, 65);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    
    go_home();
    ESP_LOGI(TAG, "Left leg rhythm completed");
}

void right_leg_rhythm(void) {
    control_state.manual_mode = true;
    servo_attach(SERVO_CH_RIGHT_LEG);
    
    // Rhythm pattern: 140 -> 150 -> 170, repeat 3 times
    for (int i = 0; i < 3; i++) {
        servo_write(SERVO_CH_RIGHT_LEG, 140);
        vTaskDelay(pdMS_TO_TICKS(200));
        servo_write(SERVO_CH_RIGHT_LEG, 150);
        vTaskDelay(pdMS_TO_TICKS(200));
        servo_write(SERVO_CH_RIGHT_LEG, 170);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    
    go_home();
    ESP_LOGI(TAG, "Right leg rhythm completed");
}

// ========== COMBO FUNCTIONS ==========

void ninja_combo1(void) {
    control_state.manual_mode = true;
    ESP_LOGI(TAG, "COMBO 1 START");
    
    // Get speed from calibration (default 1000 if not set)
    int lf_speed_ms = (calibration.combo_lf_speed > 0) ? calibration.combo_lf_speed : 1000;
    
    // Step 1: Tilt LEFT and wait 1s
    ESP_LOGI(TAG, "  Step 1: Tilt LEFT...");
    servo_attach(SERVO_CH_LEFT_LEG);
    servo_attach(SERVO_CH_RIGHT_LEG);
    servo_write(SERVO_CH_LEFT_LEG, calibration.latl);
    servo_write(SERVO_CH_RIGHT_LEG, calibration.ratl);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Step 2: Wave Right Leg (RL) 3 times: 135 -> 155 -> 180
    ESP_LOGI(TAG, "  Step 2: Wave Right Leg 3 times (135->155->180)");
    for (int i = 0; i < 3; i++) {
        servo_write(SERVO_CH_RIGHT_LEG, 135);
        vTaskDelay(pdMS_TO_TICKS(200));
        servo_write(SERVO_CH_RIGHT_LEG, 155);
        vTaskDelay(pdMS_TO_TICKS(200));
        servo_write(SERVO_CH_RIGHT_LEG, 180);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    
    // Step 3: Rotate Left Foot (LF) using configured speed
    ESP_LOGI(TAG, "  Step 3: Rotate Left Foot for %dms", lf_speed_ms);
    servo_attach(SERVO_CH_LEFT_FOOT);
    int lf_angle = calibration.lf_neutral + calibration.lffwrs;
    servo_write(SERVO_CH_LEFT_FOOT, lf_angle);
    vTaskDelay(pdMS_TO_TICKS(lf_speed_ms));
    servo_write(SERVO_CH_LEFT_FOOT, calibration.lf_neutral);
    
    // Step 4: Return to home
    ESP_LOGI(TAG, "  Step 4: Return to HOME");
    control_state.manual_mode = false;
    go_home();
    ESP_LOGI(TAG, "COMBO 1 COMPLETE");
}

void ninja_combo2(void) {
    control_state.manual_mode = true;
    ESP_LOGI(TAG, "COMBO 2 START");
    
    // Get speed from calibration (default 1000 if not set)
    int rf_speed_ms = (calibration.combo_rf_speed > 0) ? calibration.combo_rf_speed : 1000;
    
    // Step 1: Tilt RIGHT and wait 1s
    ESP_LOGI(TAG, "  Step 1: Tilt RIGHT...");
    servo_attach(SERVO_CH_LEFT_LEG);
    servo_attach(SERVO_CH_RIGHT_LEG);
    servo_write(SERVO_CH_LEFT_LEG, calibration.latr);
    servo_write(SERVO_CH_RIGHT_LEG, calibration.ratr);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Step 2: Wave Left Leg (LL) 3 times: 10 -> 30 -> 75
    ESP_LOGI(TAG, "  Step 2: Wave Left Leg 3 times (10->30->75)");
    for (int i = 0; i < 3; i++) {
        servo_write(SERVO_CH_LEFT_LEG, 10);
        vTaskDelay(pdMS_TO_TICKS(200));
        servo_write(SERVO_CH_LEFT_LEG, 30);
        vTaskDelay(pdMS_TO_TICKS(200));
        servo_write(SERVO_CH_LEFT_LEG, 75);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    
    // Step 3: Rotate Right Foot (RF) using configured speed
    ESP_LOGI(TAG, "  Step 3: Rotate Right Foot for %dms", rf_speed_ms);
    servo_attach(SERVO_CH_RIGHT_FOOT);
    int rf_angle = calibration.rf_neutral - calibration.rffwrs;
    servo_write(SERVO_CH_RIGHT_FOOT, rf_angle);
    vTaskDelay(pdMS_TO_TICKS(rf_speed_ms));
    servo_write(SERVO_CH_RIGHT_FOOT, calibration.rf_neutral);
    
    // Step 4: Return to home
    ESP_LOGI(TAG, "  Step 4: Return to HOME");
    control_state.manual_mode = false;
    go_home();
    ESP_LOGI(TAG, "COMBO 2 COMPLETE");
}

// ========== DIRECT SERVO CONTROL ==========

void servo_direct_write(int channel, int angle) {
    if (channel < 0 || channel >= SERVO_CH_MAX) return;
    control_state.manual_mode = true;
    servo_attach((servo_channel_t)channel);
    servo_write((servo_channel_t)channel, angle);
}

void set_manual_mode(bool enable) {
    control_state.manual_mode = enable;
    if (!enable) {
        ESP_LOGI(TAG, "Manual mode OFF");
    }
}

// ========== FOOT ROTATION FUNCTIONS ==========

void ninja_rotate_left_foot(int speed, int duration_ms) {
    if (speed == 0 || duration_ms <= 0) return;
    
    calibration_t* cal = get_calibration();
    int start_angle = cal->lf_neutral;
    int target_angle = (speed > 0) ? 180 : 0;
    int steps = duration_ms / 50;  // 50ms per step
    
    ESP_LOGI(TAG, "Rotating LF: speed=%d, duration=%dms", speed, duration_ms);
    
    for (int i = 0; i <= steps; i++) {
        float progress = (float)i / steps;
        int angle = start_angle + (target_angle - start_angle) * progress;
        servo_write(SERVO_CH_LEFT_FOOT, angle);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    // Return to neutral
    servo_write(SERVO_CH_LEFT_FOOT, start_angle);
}

void ninja_rotate_right_foot(int speed, int duration_ms) {
    if (speed == 0 || duration_ms <= 0) return;
    
    calibration_t* cal = get_calibration();
    int start_angle = cal->rf_neutral;
    int target_angle = (speed > 0) ? 180 : 0;
    int steps = duration_ms / 50;  // 50ms per step
    
    ESP_LOGI(TAG, "Rotating RF: speed=%d, duration=%dms", speed, duration_ms);
    
    for (int i = 0; i <= steps; i++) {
        float progress = (float)i / steps;
        int angle = start_angle + (target_angle - start_angle) * progress;
        servo_write(SERVO_CH_RIGHT_FOOT, angle);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    // Return to neutral
    servo_write(SERVO_CH_RIGHT_FOOT, start_angle);
}

// In-place rotation using both feet
// speed: -100 to 100 (negative = counterclockwise/left, positive = clockwise/right)
void ninja_spin_in_place(int speed) {
    if (speed == 0) {
        // Stop spinning - return to neutral
        calibration_t* cal = get_calibration();
        servo_write(SERVO_CH_LEFT_FOOT, cal->lf_neutral);
        servo_write(SERVO_CH_RIGHT_FOOT, cal->rf_neutral);
        ESP_LOGI(TAG, "Spin stopped - returned to neutral");
        return;
    }
    
    calibration_t* cal = get_calibration();
    
    // Map speed (-100 to 100) to servo angles
    // - For left turn (speed < 0): LF goes backward, RF goes forward
    // - For right turn (speed > 0): LF goes forward, RF goes backward
    
    int abs_speed = (speed < 0) ? -speed : speed;
    if (abs_speed > 100) abs_speed = 100;
    
    // Calculate target angles based on speed magnitude
    // Speed 100 = full rotation (0 or 180 degrees from neutral)
    // Speed 50 = half rotation
    float rotation_amount = (float)abs_speed / 100.0f;
    
    int lf_angle, rf_angle;
    
    if (speed < 0) {
        // Counterclockwise (left turn)
        lf_angle = cal->lf_neutral - (int)(90 * rotation_amount);  // LF backward
        rf_angle = cal->rf_neutral + (int)(90 * rotation_amount);  // RF forward
    } else {
        // Clockwise (right turn)
        lf_angle = cal->lf_neutral + (int)(90 * rotation_amount);  // LF forward
        rf_angle = cal->rf_neutral - (int)(90 * rotation_amount);  // RF backward
    }
    
    // Constrain angles to valid servo range
    if (lf_angle < 0) lf_angle = 0;
    if (lf_angle > 180) lf_angle = 180;
    if (rf_angle < 0) rf_angle = 0;
    if (rf_angle > 180) rf_angle = 180;
    
    servo_write(SERVO_CH_LEFT_FOOT, lf_angle);
    servo_write(SERVO_CH_RIGHT_FOOT, rf_angle);
    
    ESP_LOGI(TAG, "Spinning: speed=%d, LF=%d°, RF=%d°", speed, lf_angle, rf_angle);
}

// ========== SLEEP PREPARATION ==========

void robot_prepare_sleep(void) {
    ESP_LOGI(TAG, "Preparing for sleep: turning off LED and detaching all servos");
    
    // Turn off LED strip
    ninja_led_off();
    
    // Stop any movement
    control_state.j_x = 0;
    control_state.j_y = 0;
    control_state.manual_mode = true;
    
    // Detach all servos (stop sending PWM pulses)
    for (int ch = 0; ch < SERVO_CH_MAX; ch++) {
        servo_detach((servo_channel_t)ch);
    }
    
    ESP_LOGI(TAG, "All servos detached, LED off - ready to sleep");
}

// ========== MAIN CONTROL TASK ==========

void robot_control_task(void *pvParameters) {
    static int8_t last_j_x = 0;
    static int8_t last_j_y = 0;
    
    while (1) {
        // Skip normal control if in manual mode
        if (control_state.manual_mode) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        
        // Debug: Print joystick values when they change
        if (control_state.j_x != last_j_x || control_state.j_y != last_j_y) {
            const char *direction = "";
            
            if (control_state.j_x == 0 && control_state.j_y == 0) {
                direction = "STOP";
            } else if (control_state.j_y >= 0) {
                if (control_state.j_x > 20) direction = "FORWARD + TURN RIGHT";
                else if (control_state.j_x < -20) direction = "FORWARD + TURN LEFT";
                else direction = "FORWARD STRAIGHT";
            } else {
                if (control_state.j_x > 20) direction = "BACKWARD + TURN RIGHT";
                else if (control_state.j_x < -20) direction = "BACKWARD + TURN LEFT";
                else direction = "BACKWARD STRAIGHT";
            }
            
            ESP_LOGI(TAG, "Joystick: X=%d, Y=%d | Mode=%s | %s",
                     control_state.j_x, control_state.j_y,
                     current_mode == MODE_WALK ? "WALK" : "ROLL", direction);
            last_j_x = control_state.j_x;
            last_j_y = control_state.j_y;
        }
        
        // Handle mode buttons
        if (control_state.button_x == 1) {
            ninja_set_roll();
            control_state.button_x = 0;
        }
        if (control_state.button_y == 1) {
            ninja_set_walk();
            control_state.button_y = 0;
        }
        
        // Handle arm buttons (no-ops on 4-servo board, but keeps state consistent)
        if (control_state.button_a == 1) {
            ninja_left_arm_up();
        } else {
            ninja_left_arm_down();
        }
        
        if (control_state.button_b == 1) {
            ninja_right_arm_up();
        } else {
            ninja_right_arm_down();
        }
        
        // Process movement based on mode
        if (current_mode == MODE_WALK) {
            ninja_walk();
        } else {
            ninja_roll();
        }
        
        // Update LED strip animation
        ninja_led_update();
        
        // CONFIG_FREERTOS_HZ=100 means 1 tick = 10ms
        // pdMS_TO_TICKS(2) = 0 at 100Hz! Use minimum 1 tick.
        vTaskDelay(1);
    }
}

// ========== ACTION RECORDING ==========

recording_state_t* get_recording_state(void) {
    return &recording_state;
}

action_slot_t* get_action_slot(uint8_t slot) {
    if (slot >= MAX_ACTION_SLOTS) return NULL;
    return &action_slots[slot];
}

void start_recording(uint8_t slot) {
    if (slot >= MAX_ACTION_SLOTS) return;
    
    recording_state.is_recording = true;
    recording_state.current_slot = slot;
    recording_state.step_count = 0;
    memset(&action_slots[slot], 0, sizeof(action_slot_t));
    
    ESP_LOGI(TAG, "START RECORDING slot %d", slot + 1);
}

void stop_recording(void) {
    if (!recording_state.is_recording) return;
    
    uint8_t slot = recording_state.current_slot;
    action_slots[slot].count = recording_state.step_count;
    
    ESP_LOGI(TAG, "STOP RECORDING - Slot %d: %d actions", slot + 1, recording_state.step_count);
    
    save_actions_to_nvs(slot);
    recording_state.is_recording = false;
}

void record_action(action_type_t type, int16_t param1, int16_t param2, uint16_t duration) {
    if (!recording_state.is_recording) return;
    if (recording_state.step_count >= MAX_ACTIONS) return;
    
    uint8_t slot = recording_state.current_slot;
    uint8_t idx = recording_state.step_count;
    
    action_slots[slot].steps[idx].type = type;
    action_slots[slot].steps[idx].param1 = param1;
    action_slots[slot].steps[idx].param2 = param2;
    action_slots[slot].steps[idx].duration_ms = duration;
    
    recording_state.step_count++;
    action_slots[slot].count = recording_state.step_count;
    
    ESP_LOGI(TAG, "Recorded [%d/%d]: type=%d", recording_state.step_count, MAX_ACTIONS, type);
}

void play_action(uint8_t slot) {
    if (slot >= MAX_ACTION_SLOTS) return;
    
    action_slot_t* actions = &action_slots[slot];
    if (actions->count == 0) {
        ESP_LOGW(TAG, "Slot %d is empty", slot + 1);
        return;
    }
    
    ESP_LOGI(TAG, "PLAYING Slot %d (%d actions)", slot + 1, actions->count);
    
    control_state.manual_mode = true;
    
    for (int i = 0; i < actions->count; i++) {
        action_step_t* step = &actions->steps[i];
        
        switch (step->type) {
            case ACTION_JOYSTICK:
                control_state.j_x = step->param1;
                control_state.j_y = step->param2;
                control_state.manual_mode = false;
                vTaskDelay(pdMS_TO_TICKS(step->duration_ms));
                control_state.j_x = 0;
                control_state.j_y = 0;
                control_state.manual_mode = true;
                break;
            
            case ACTION_BUTTON_A:
                ninja_left_arm_up();
                vTaskDelay(pdMS_TO_TICKS(300));
                ninja_left_arm_down();
                break;
                
            case ACTION_BUTTON_B:
                ninja_right_arm_up();
                vTaskDelay(pdMS_TO_TICKS(300));
                ninja_right_arm_down();
                break;
                
            case ACTION_WALK_MODE:
                ninja_set_walk();
                break;
                
            case ACTION_ROLL_MODE:
                ninja_set_roll();
                break;
                
            case ACTION_HOME:
                control_state.manual_mode = false;
                go_home();
                control_state.manual_mode = true;
                break;
                
            case ACTION_TILT_LEFT:
                ninja_tilt_left();
                vTaskDelay(pdMS_TO_TICKS(step->duration_ms > 0 ? step->duration_ms : 500));
                break;
                
            case ACTION_TILT_RIGHT:
                ninja_tilt_right();
                vTaskDelay(pdMS_TO_TICKS(step->duration_ms > 0 ? step->duration_ms : 500));
                break;
                
            case ACTION_SERVO:
                servo_direct_write(step->param1, step->param2);
                vTaskDelay(pdMS_TO_TICKS(step->duration_ms > 0 ? step->duration_ms : 200));
                break;
                
            case ACTION_DELAY:
                vTaskDelay(pdMS_TO_TICKS(step->param1));
                break;
                
            case ACTION_MOVE_FWD:
                control_state.j_y = 80;
                control_state.j_x = 0;
                control_state.manual_mode = false;
                vTaskDelay(pdMS_TO_TICKS(step->param1 > 0 ? step->param1 : 1000));
                control_state.j_y = 0;
                control_state.j_x = 0;
                control_state.manual_mode = true;
                break;
                
            case ACTION_MOVE_BWD:
                control_state.j_y = -80;
                control_state.j_x = 0;
                control_state.manual_mode = false;
                vTaskDelay(pdMS_TO_TICKS(step->param1 > 0 ? step->param1 : 1000));
                control_state.j_y = 0;
                control_state.j_x = 0;
                control_state.manual_mode = true;
                break;
                
            case ACTION_MOVE_STOP:
                control_state.j_y = 0;
                control_state.j_x = 0;
                break;
                
            case ACTION_TURN_LEFT: {
                int spd = step->param1 > 0 ? step->param1 : 500;
                if (current_mode == MODE_WALK) {
                    ninja_tilt_left();
                    vTaskDelay(pdMS_TO_TICKS(spd));
                    int lf_angle = calibration.lf_neutral + calibration.lffwrs;
                    servo_direct_write(SERVO_CH_LEFT_FOOT, lf_angle);
                    vTaskDelay(pdMS_TO_TICKS(spd));
                    servo_direct_write(SERVO_CH_LEFT_FOOT, calibration.lf_neutral);
                    control_state.manual_mode = false;
                    go_home();
                    control_state.manual_mode = true;
                } else {
                    control_state.j_x = -75;
                    control_state.j_y = -64;
                    control_state.manual_mode = false;
                    vTaskDelay(pdMS_TO_TICKS(spd));
                    control_state.j_x = 0;
                    control_state.j_y = 0;
                    control_state.manual_mode = true;
                }
                break;
            }
                
            case ACTION_TURN_RIGHT: {
                int spd = step->param1 > 0 ? step->param1 : 500;
                if (current_mode == MODE_WALK) {
                    ninja_tilt_right();
                    vTaskDelay(pdMS_TO_TICKS(spd));
                    int rf_angle = calibration.rf_neutral - calibration.rffwrs;
                    servo_direct_write(SERVO_CH_RIGHT_FOOT, rf_angle);
                    vTaskDelay(pdMS_TO_TICKS(spd));
                    servo_direct_write(SERVO_CH_RIGHT_FOOT, calibration.rf_neutral);
                    control_state.manual_mode = false;
                    go_home();
                    control_state.manual_mode = true;
                } else {
                    control_state.j_x = 51;
                    control_state.j_y = -81;
                    control_state.manual_mode = false;
                    vTaskDelay(pdMS_TO_TICKS(spd));
                    control_state.j_x = 0;
                    control_state.j_y = 0;
                    control_state.manual_mode = true;
                }
                break;
            }
            
            case ACTION_SPIN: {
                int speed = step->param1;  // -100 to 100
                int duration = step->duration_ms > 0 ? step->duration_ms : 500;
                ninja_spin_in_place(speed);
                vTaskDelay(pdMS_TO_TICKS(duration));
                ninja_spin_in_place(0);  // Stop spinning
                break;
            }
                
            case ACTION_RHYTHM_LEFT:
                left_leg_rhythm();
                break;
                
            case ACTION_RHYTHM_RIGHT:
                right_leg_rhythm();
                break;
                
            case ACTION_COMBO1:
                ninja_combo1();
                break;
                
            case ACTION_COMBO2:
                ninja_combo2();
                break;
            
            case ACTION_WALK_COMBO_123:
                ninja_walk_combo_123();
                break;
            
            case ACTION_WALK_COMBO_345:
                ninja_walk_combo_345();
                break;
            
            case ACTION_TEST_LF:
                test_left_foot();
                vTaskDelay(pdMS_TO_TICKS(step->duration_ms > 0 ? step->duration_ms : 1000));
                stop_feet_test();
                break;
            
            case ACTION_TEST_RF:
                test_right_foot();
                vTaskDelay(pdMS_TO_TICKS(step->duration_ms > 0 ? step->duration_ms : 1000));
                stop_feet_test();
                break;
            
            case ACTION_TEST_BOTH:
                test_both_feet();
                break;
            
            case ACTION_TEST_STOP:
                stop_feet_test();
                break;
                
            default:
                break;
        }
    }
    
    control_state.manual_mode = false;
    go_home();
    ESP_LOGI(TAG, "PLAYBACK COMPLETE");
}

void save_actions_to_nvs(uint8_t slot) {
    if (slot >= MAX_ACTION_SLOTS) return;
    
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("robot_actions", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for actions");
        return;
    }
    
    char key[16];
    snprintf(key, sizeof(key), "action_%d", slot);
    
    err = nvs_set_blob(nvs_handle, key, &action_slots[slot], sizeof(action_slot_t));
    if (err == ESP_OK) {
        nvs_commit(nvs_handle);
        ESP_LOGI(TAG, "Saved %d actions to NVS slot %d", action_slots[slot].count, slot + 1);
    }
    
    nvs_close(nvs_handle);
}

void load_actions_from_nvs(uint8_t slot) {
    if (slot >= MAX_ACTION_SLOTS) return;
    
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("robot_actions", NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) return;
    
    char key[16];
    snprintf(key, sizeof(key), "action_%d", slot);
    
    size_t required_size = sizeof(action_slot_t);
    err = nvs_get_blob(nvs_handle, key, &action_slots[slot], &required_size);
    if (err == ESP_OK) {
        if (action_slots[slot].count > MAX_ACTIONS) {
            action_slots[slot].count = 0;
        }
        ESP_LOGI(TAG, "Loaded slot %d: %d actions", slot + 1, action_slots[slot].count);
    } else {
        memset(&action_slots[slot], 0, sizeof(action_slot_t));
    }
    
    nvs_close(nvs_handle);
}

// ========== WALK PHASE CONTROL FUNCTIONS ==========

void ninja_walk_phase1(void) {
    ESP_LOGI(TAG, "[PHASE 1] Tilt to right");
    servo_attach(SERVO_CH_LEFT_LEG);
    servo_attach(SERVO_CH_RIGHT_LEG);
    servo_write(SERVO_CH_LEFT_LEG, calibration.latr);
    servo_write(SERVO_CH_RIGHT_LEG, calibration.ratr);
    vTaskDelay(pdMS_TO_TICKS(300));
}

void ninja_walk_phase2(void) {
    ESP_LOGI(TAG, "[PHASE 2] Right foot forward");
    servo_attach(SERVO_CH_RIGHT_FOOT);
    int angle = calibration.rf_neutral - calibration.rffwrs;
    servo_write(SERVO_CH_RIGHT_FOOT, angle);
    vTaskDelay(pdMS_TO_TICKS(500));
}

void ninja_walk_phase3(void) {
    ESP_LOGI(TAG, "[PHASE 3] Tilt to left");
    servo_write(SERVO_CH_RIGHT_FOOT, calibration.rf_neutral);
    servo_write(SERVO_CH_LEFT_LEG, calibration.latl);
    servo_write(SERVO_CH_RIGHT_LEG, calibration.ratl);
    vTaskDelay(pdMS_TO_TICKS(300));
}

void ninja_walk_phase4(void) {
    ESP_LOGI(TAG, "[PHASE 4] Left foot forward");
    servo_attach(SERVO_CH_LEFT_FOOT);
    int angle = calibration.lf_neutral + calibration.lffwrs;
    servo_write(SERVO_CH_LEFT_FOOT, angle);
    vTaskDelay(pdMS_TO_TICKS(500));
}

void ninja_walk_phase5(void) {
    ESP_LOGI(TAG, "[PHASE 5] Return neutral");
    servo_write(SERVO_CH_LEFT_FOOT, calibration.lf_neutral);
    vTaskDelay(pdMS_TO_TICKS(100));
    servo_detach(SERVO_CH_LEFT_FOOT);
    servo_detach(SERVO_CH_RIGHT_FOOT);
    servo_detach(SERVO_CH_LEFT_LEG);
    servo_detach(SERVO_CH_RIGHT_LEG);
}

void ninja_walk_combo_123(void) {
    ESP_LOGI(TAG, "WALK COMBO 1-2-3");
    ninja_walk_phase1();
    ninja_walk_phase2();
    ninja_walk_phase3();
}

void ninja_walk_combo_345(void) {
    ESP_LOGI(TAG, "WALK COMBO 3-4-5");
    ninja_walk_phase3();
    ninja_walk_phase4();
    ninja_walk_phase5();
}

// ========== BATTERY ALERT ==========

bool is_battery_alert_enabled(void) {
    return calibration.battery_alert_enabled;
}

// ========== OSCILLATOR ENGINE (OttoDIY) ==========

#define DEG2RAD(g) ((g) * M_PI / 180.0)
#define OTTO_FORWARD     1
#define OTTO_BACKWARD   -1
#define OTTO_LEFT        1
#define OTTO_RIGHT      -1

// OttoDIY servo mapping: [0]=LeftLeg, [1]=RightLeg, [2]=LeftFoot, [3]=RightFoot
// Our channels:          CH0=LeftFoot, CH1=LeftLeg, CH2=RightFoot, CH3=RightLeg
static const servo_channel_t otto_to_our[4] = {
    SERVO_CH_LEFT_LEG,    // Otto [0] = Left Leg
    SERVO_CH_RIGHT_LEG,   // Otto [1] = Right Leg
    SERVO_CH_LEFT_FOOT,   // Otto [2] = Left Foot
    SERVO_CH_RIGHT_FOOT,  // Otto [3] = Right Foot
};

static void execute_oscillation(int A[4], int O[4], int T, double phase_diff[4], float steps) {
    const int sampling_period = 30;
    int num_samples = T / sampling_period;
    if (num_samples < 1) num_samples = 1;
    double inc = 2.0 * M_PI / (double)num_samples;
    int total_samples = (int)(steps * num_samples);

    for (int i = 0; i < 4; i++) {
        servo_attach(otto_to_our[i]);
    }

    for (int j = 0; j < total_samples; j++) {
        double phase = inc * (double)j;
        for (int i = 0; i < 4; i++) {
            int pos = (int)round(A[i] * sin(phase + phase_diff[i]) + O[i]) + 90;
            if (pos < 0) pos = 0;
            if (pos > 180) pos = 180;
            servo_write(otto_to_our[i], pos);
        }
        vTaskDelay(pdMS_TO_TICKS(sampling_period));
    }
}

void ninja_moonwalk(int steps, int T, int dir) {
    int h = 20;
    double phi = -dir * DEG2RAD(90);
    int A[4]  = {0, 0, h, h};
    int O[4]  = {0, 0, h/2 + 2, -h/2 - 2};
    double phase_diff[4] = {0, 0, phi, DEG2RAD(-60 * dir) + phi};
    control_state.manual_mode = true;
    execute_oscillation(A, O, T, phase_diff, (float)steps);
    go_home();
    control_state.manual_mode = false;
}

void ninja_swing(int steps, int T, int h) {
    int A[4]  = {0, 0, h, h};
    int O[4]  = {0, 0, h/2, -h/2};
    double phase_diff[4] = {0, 0, 0, 0};
    control_state.manual_mode = true;
    execute_oscillation(A, O, T, phase_diff, (float)steps);
    go_home();
    control_state.manual_mode = false;
}

void ninja_updown(int steps, int T, int h) {
    int A[4]  = {0, 0, h, h};
    int O[4]  = {0, 0, h, -h};
    double phase_diff[4] = {0, 0, DEG2RAD(-90), DEG2RAD(90)};
    control_state.manual_mode = true;
    execute_oscillation(A, O, T, phase_diff, (float)steps);
    go_home();
    control_state.manual_mode = false;
}

void ninja_crusaito(int steps, int T, int h, int dir) {
    int A[4]  = {25, 25, h, h};
    int O[4]  = {0, 0, h/2 + 4, -h/2 - 4};
    double phase_diff[4] = {DEG2RAD(90), DEG2RAD(90), 0, DEG2RAD(-60 * dir)};
    control_state.manual_mode = true;
    execute_oscillation(A, O, T, phase_diff, (float)steps);
    go_home();
    control_state.manual_mode = false;
}

void ninja_flapping(int steps, int T, int h, int dir) {
    int A[4]  = {12, 12, h, h};
    int O[4]  = {0, 0, h - 10, -h + 10};
    double phase_diff[4] = {0, DEG2RAD(180), DEG2RAD(-90 * dir), DEG2RAD(90 * dir)};
    control_state.manual_mode = true;
    execute_oscillation(A, O, T, phase_diff, (float)steps);
    go_home();
    control_state.manual_mode = false;
}

void ninja_jitter(int steps, int T, int h) {
    if (h > 25) h = 25;
    int A[4]  = {h, h, 0, 0};
    int O[4]  = {0, 0, 0, 0};
    double phase_diff[4] = {DEG2RAD(-90), DEG2RAD(90), 0, 0};
    control_state.manual_mode = true;
    execute_oscillation(A, O, T, phase_diff, (float)steps);
    go_home();
    control_state.manual_mode = false;
}

void ninja_ascending_turn(int steps, int T, int h) {
    if (h > 13) h = 13;
    int A[4]  = {h, h, h, h};
    int O[4]  = {0, 0, h + 4, -h + 4};
    double phase_diff[4] = {DEG2RAD(-90), DEG2RAD(90), DEG2RAD(-90), DEG2RAD(90)};
    control_state.manual_mode = true;
    execute_oscillation(A, O, T, phase_diff, (float)steps);
    go_home();
    control_state.manual_mode = false;
}

void ninja_tiptoe_swing(int steps, int T, int h) {
    int A[4]  = {0, 0, h, h};
    int O[4]  = {0, 0, h, -h};
    double phase_diff[4] = {0, 0, 0, 0};
    control_state.manual_mode = true;
    execute_oscillation(A, O, T, phase_diff, (float)steps);
    go_home();
    control_state.manual_mode = false;
}

// ========== MUSIC PLAYER INTERFACE ==========

static void* g_mp3_player = NULL;
static volatile bool g_music_playing = false;

// Forward declarations for C++ Mp3Player wrapper functions (defined in xingzhi-cube-1.54tft-wifi.cc)
typedef void Mp3Player_t;
bool Mp3Player_Play(Mp3Player_t* player, const char* path);
int Mp3Player_GetState(Mp3Player_t* player);  // 0=STOPPED, 1=PLAYING, 2=PAUSED
void Mp3Player_Stop(Mp3Player_t* player);

void set_music_player_ptr(void* mp3_player_ptr) {
    g_mp3_player = mp3_player_ptr;
}

bool ninja_play_music(const char* song_name) {
    if (!g_mp3_player) {
        ESP_LOGW(TAG, "Mp3Player not initialized");
        return false;
    }
    
    char path[256];
    
    // If already a full path (starts with /sdcard/), use directly
    if (strncmp(song_name, "/sdcard/", 8) == 0) {
        // Full path provided - try as-is first
        ESP_LOGI(TAG, "Attempting to play (full path): %s", song_name);
        if (Mp3Player_Play((Mp3Player_t*)g_mp3_player, song_name)) {
            g_music_playing = true;
            return true;
        }
        // If path doesn't end with .mp3, try adding it
        size_t len = strlen(song_name);
        if (len < 4 || strcasecmp(song_name + len - 4, ".mp3") != 0) {
            snprintf(path, sizeof(path), "%s.mp3", song_name);
            ESP_LOGI(TAG, "Attempting to play: %s", path);
            if (Mp3Player_Play((Mp3Player_t*)g_mp3_player, path)) {
                g_music_playing = true;
                return true;
            }
        }
        ESP_LOGW(TAG, "Failed to play: %s", song_name);
        return false;
    }
    
    // Short name - try /sdcard/song_name.mp3 first
    size_t len = strlen(song_name);
    bool has_mp3 = (len >= 4 && strcasecmp(song_name + len - 4, ".mp3") == 0);
    
    if (!has_mp3) {
        snprintf(path, sizeof(path), "/sdcard/%s.mp3", song_name);
        ESP_LOGI(TAG, "Attempting to play: %s", path);
        if (Mp3Player_Play((Mp3Player_t*)g_mp3_player, path)) {
            g_music_playing = true;
            return true;
        }
    }
    
    // Try /sdcard/song_name (as-is)
    snprintf(path, sizeof(path), "/sdcard/%s", song_name);
    ESP_LOGI(TAG, "Attempting to play: %s", path);
    if (Mp3Player_Play((Mp3Player_t*)g_mp3_player, path)) {
        g_music_playing = true;
        return true;
    }
    
    ESP_LOGW(TAG, "Failed to play song: %s", song_name);
    return false;
}

bool ninja_is_music_playing(void) {
    if (!g_mp3_player || !g_music_playing) return false;
    
    // Check player state: 1 = PLAYING
    int state = Mp3Player_GetState((Mp3Player_t*)g_mp3_player);
    if (state != 1) {
        g_music_playing = false;
        return false;
    }
    return true;
}

void ninja_stop_music(void) {
    if (g_mp3_player) {
        Mp3Player_Stop((Mp3Player_t*)g_mp3_player);
        g_music_playing = false;
    }
}

// ========== LED STRIP CONTROL ==========

static led_strip_handle_t led_strip = NULL;
static led_state_t led_state = {
    .r = 255,
    .g = 255,
    .b = 255,
    .brightness = 128,
    .mode = LED_MODE_SOLID,
    .speed = 50
};

static bool led_initialized = false;
static uint32_t led_animation_step = 0;
static uint32_t led_last_update = 0;

// Music-reactive audio energy (set by audio decoders, read by LED update)
static volatile float audio_energy_level = 0.0f;

void ninja_led_set_audio_energy(float energy) {
    if (energy < 0.0f) energy = 0.0f;
    if (energy > 1.0f) energy = 1.0f;
    audio_energy_level = energy;
}

float ninja_led_get_audio_energy(void) {
    return audio_energy_level;
}

void ninja_led_init(void) {
    if (led_initialized) return;
    
    ESP_LOGI(TAG, "Initializing LED strip on GPIO %d with %d LEDs", LED_STRIP_PIN, LED_STRIP_COUNT);
    
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_PIN,
        .max_leds = LED_STRIP_COUNT,
        .led_model = LED_MODEL_WS2812,
        .flags.invert_out = false,
    };
    
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };
    
    esp_err_t ret = led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create LED strip: %s", esp_err_to_name(ret));
        return;
    }
    
    led_initialized = true;
    load_led_state_from_nvs();
    
    // Ensure LED is visible on first boot (if mode is OFF or brightness is 0, set defaults)
    if (led_state.mode == LED_MODE_OFF || led_state.brightness == 0) {
        ESP_LOGI(TAG, "LED mode was OFF or brightness 0, setting defaults");
        led_state.mode = LED_MODE_SOLID;
        led_state.brightness = 128;
        led_state.r = 255;
        led_state.g = 255;
        led_state.b = 255;
    }
    
    ESP_LOGI(TAG, "LED state: R=%d G=%d B=%d, Brightness=%d, Mode=%d", 
             led_state.r, led_state.g, led_state.b, led_state.brightness, led_state.mode);
    
    // Force immediate LED test - light all LEDs RED for 1 second
    ESP_LOGI(TAG, "Testing LED strip - RED...");
    for (int i = 0; i < LED_STRIP_COUNT; i++) {
        esp_err_t err = led_strip_set_pixel(led_strip, i, 255, 0, 0);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "led_strip_set_pixel failed: %s", esp_err_to_name(err));
        }
    }
    esp_err_t refresh_err = led_strip_refresh(led_strip);
    if (refresh_err != ESP_OK) {
        ESP_LOGE(TAG, "led_strip_refresh failed: %s", esp_err_to_name(refresh_err));
    } else {
        ESP_LOGI(TAG, "LED refresh success - should see RED now!");
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Then set proper state
    ninja_led_update();
    
    ESP_LOGI(TAG, "LED strip initialized on GPIO %d!", LED_STRIP_PIN);
}

static uint8_t apply_brightness(uint8_t color, uint8_t brightness) {
    return (uint8_t)((color * brightness) / 255);
}

void ninja_led_set_color(uint8_t r, uint8_t g, uint8_t b) {
    led_state.r = r;
    led_state.g = g;
    led_state.b = b;
    led_state.mode = LED_MODE_SOLID;
}

void ninja_led_set_pixel(uint8_t index, uint8_t r, uint8_t g, uint8_t b) {
    if (!led_initialized || !led_strip || index >= LED_STRIP_COUNT) return;
    
    uint8_t br = apply_brightness(r, led_state.brightness);
    uint8_t bg = apply_brightness(g, led_state.brightness);
    uint8_t bb = apply_brightness(b, led_state.brightness);
    
    led_strip_set_pixel(led_strip, index, br, bg, bb);
}

void ninja_led_set_brightness(uint8_t brightness) {
    led_state.brightness = brightness;
}

void ninja_led_set_mode(led_mode_t mode) {
    led_state.mode = mode;
    led_animation_step = 0;
}

void ninja_led_set_speed(uint16_t speed_ms) {
    led_state.speed = speed_ms;
}

void ninja_led_off(void) {
    if (!led_initialized || !led_strip) return;
    
    led_state.mode = LED_MODE_OFF;
    led_strip_clear(led_strip);
    led_strip_refresh(led_strip);
}

static void hsv_to_rgb(uint16_t h, uint8_t s, uint8_t v, uint8_t *r, uint8_t *g, uint8_t *b) {
    uint8_t region = h / 60;
    uint8_t remainder = (h - (region * 60)) * 255 / 60;
    
    uint8_t p = (v * (255 - s)) / 255;
    uint8_t q = (v * (255 - ((s * remainder) / 255))) / 255;
    uint8_t t = (v * (255 - ((s * (255 - remainder)) / 255))) / 255;
    
    switch (region) {
        case 0: *r = v; *g = t; *b = p; break;
        case 1: *r = q; *g = v; *b = p; break;
        case 2: *r = p; *g = v; *b = t; break;
        case 3: *r = p; *g = q; *b = v; break;
        case 4: *r = t; *g = p; *b = v; break;
        default: *r = v; *g = p; *b = q; break;
    }
}

void ninja_led_update(void) {
    if (!led_initialized || !led_strip) {
        static bool warned = false;
        if (!warned) {
            ESP_LOGW(TAG, "LED update skipped: initialized=%d, strip=%p", led_initialized, led_strip);
            warned = true;
        }
        return;
    }
    
    static uint32_t update_count = 0;
    if (update_count++ % 10000 == 0) {
        ESP_LOGI(TAG, "LED update #%lu: mode=%d, br=%d", 
                 update_count, led_state.mode, led_state.brightness);
    }
    
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    
    if (led_state.mode != LED_MODE_SOLID && led_state.mode != LED_MODE_OFF && led_state.mode != LED_MODE_MUSIC_REACTIVE) {
        if (now - led_last_update < led_state.speed) {
            return;
        }
        led_last_update = now;
    }
    
    uint8_t br, bg, bb;
    
    switch (led_state.mode) {
        case LED_MODE_OFF:
            led_strip_clear(led_strip);
            break;
            
        case LED_MODE_SOLID:
            br = apply_brightness(led_state.r, led_state.brightness);
            bg = apply_brightness(led_state.g, led_state.brightness);
            bb = apply_brightness(led_state.b, led_state.brightness);
            for (int i = 0; i < LED_STRIP_COUNT; i++) {
                led_strip_set_pixel(led_strip, i, br, bg, bb);
            }
            break;
            
        case LED_MODE_RAINBOW: {
            for (int i = 0; i < LED_STRIP_COUNT; i++) {
                uint16_t hue = (led_animation_step + i * 360 / LED_STRIP_COUNT) % 360;
                uint8_t r, g, b;
                hsv_to_rgb(hue, 255, 255, &r, &g, &b);
                br = apply_brightness(r, led_state.brightness);
                bg = apply_brightness(g, led_state.brightness);
                bb = apply_brightness(b, led_state.brightness);
                led_strip_set_pixel(led_strip, i, br, bg, bb);
            }
            led_animation_step = (led_animation_step + 5) % 360;
            break;
        }
        
        case LED_MODE_BREATHING: {
            float phase = (led_animation_step % 100) / 100.0f * 3.14159f * 2;
            float breath = (sinf(phase) + 1.0f) / 2.0f;
            uint8_t breath_brightness = (uint8_t)(breath * led_state.brightness);
            
            br = apply_brightness(led_state.r, breath_brightness);
            bg = apply_brightness(led_state.g, breath_brightness);
            bb = apply_brightness(led_state.b, breath_brightness);
            
            for (int i = 0; i < LED_STRIP_COUNT; i++) {
                led_strip_set_pixel(led_strip, i, br, bg, bb);
            }
            led_animation_step++;
            break;
        }
            
        case LED_MODE_CHASE: {
            led_strip_clear(led_strip);
            int lit_led = led_animation_step % LED_STRIP_COUNT;
            br = apply_brightness(led_state.r, led_state.brightness);
            bg = apply_brightness(led_state.g, led_state.brightness);
            bb = apply_brightness(led_state.b, led_state.brightness);
            led_strip_set_pixel(led_strip, lit_led, br, bg, bb);
            int trail1 = (lit_led - 1 + LED_STRIP_COUNT) % LED_STRIP_COUNT;
            int trail2 = (lit_led - 2 + LED_STRIP_COUNT) % LED_STRIP_COUNT;
            led_strip_set_pixel(led_strip, trail1, br/2, bg/2, bb/2);
            led_strip_set_pixel(led_strip, trail2, br/4, bg/4, bb/4);
            led_animation_step++;
            break;
        }
        
        case LED_MODE_BLINK: {
            bool on = (led_animation_step % 2) == 0;
            if (on) {
                br = apply_brightness(led_state.r, led_state.brightness);
                bg = apply_brightness(led_state.g, led_state.brightness);
                bb = apply_brightness(led_state.b, led_state.brightness);
                for (int i = 0; i < LED_STRIP_COUNT; i++) {
                    led_strip_set_pixel(led_strip, i, br, bg, bb);
                }
            } else {
                led_strip_clear(led_strip);
            }
            led_animation_step++;
            break;
        }
        
        case LED_MODE_STROBE: {
            // Fast strobe effect
            bool on = (led_animation_step % 10) < 2;
            if (on) {
                br = apply_brightness(led_state.r, led_state.brightness);
                bg = apply_brightness(led_state.g, led_state.brightness);
                bb = apply_brightness(led_state.b, led_state.brightness);
                for (int i = 0; i < LED_STRIP_COUNT; i++) {
                    led_strip_set_pixel(led_strip, i, br, bg, bb);
                }
            } else {
                led_strip_clear(led_strip);
            }
            led_animation_step++;
            break;
        }
        
        case LED_MODE_FADE: {
            // Fade in and out
            uint32_t cycle_pos = led_animation_step % 200;
            float fade_factor;
            if (cycle_pos < 100) {
                fade_factor = cycle_pos / 100.0f;
            } else {
                fade_factor = (200 - cycle_pos) / 100.0f;
            }
            uint8_t fade_brightness = (uint8_t)(fade_factor * led_state.brightness);
            
            br = apply_brightness(led_state.r, fade_brightness);
            bg = apply_brightness(led_state.g, fade_brightness);
            bb = apply_brightness(led_state.b, fade_brightness);
            
            for (int i = 0; i < LED_STRIP_COUNT; i++) {
                led_strip_set_pixel(led_strip, i, br, bg, bb);
            }
            led_animation_step++;
            break;
        }
        
        case LED_MODE_COMET: {
            // Comet with long tail
            led_strip_clear(led_strip);
            int head = led_animation_step % LED_STRIP_COUNT;
            br = apply_brightness(led_state.r, led_state.brightness);
            bg = apply_brightness(led_state.g, led_state.brightness);
            bb = apply_brightness(led_state.b, led_state.brightness);
            
            // Head
            led_strip_set_pixel(led_strip, head, br, bg, bb);
            
            // Tail with gradient
            for (int i = 1; i < 5 && i < LED_STRIP_COUNT; i++) {
                int tail_pos = (head - i + LED_STRIP_COUNT) % LED_STRIP_COUNT;
                uint8_t tail_brightness = led_state.brightness * (5 - i) / 5;
                uint8_t tr = apply_brightness(led_state.r, tail_brightness);
                uint8_t tg = apply_brightness(led_state.g, tail_brightness);
                uint8_t tb = apply_brightness(led_state.b, tail_brightness);
                led_strip_set_pixel(led_strip, tail_pos, tr, tg, tb);
            }
            led_animation_step++;
            break;
        }
        
        case LED_MODE_SPARKLE: {
            // Random sparkle effect
            led_strip_clear(led_strip);
            
            // Light up 2-3 random LEDs
            int num_sparkles = 2 + (led_animation_step % 2);
            for (int s = 0; s < num_sparkles; s++) {
                int pos = (led_animation_step * 37 + s * 17) % LED_STRIP_COUNT;
                br = apply_brightness(led_state.r, led_state.brightness);
                bg = apply_brightness(led_state.g, led_state.brightness);
                bb = apply_brightness(led_state.b, led_state.brightness);
                led_strip_set_pixel(led_strip, pos, br, bg, bb);
            }
            led_animation_step++;
            break;
        }
        
        case LED_MODE_THEATER_CHASE: {
            // Theater chase effect - every 3rd LED
            led_strip_clear(led_strip);
            int offset = led_animation_step % 3;
            br = apply_brightness(led_state.r, led_state.brightness);
            bg = apply_brightness(led_state.g, led_state.brightness);
            bb = apply_brightness(led_state.b, led_state.brightness);
            
            for (int i = offset; i < LED_STRIP_COUNT; i += 3) {
                led_strip_set_pixel(led_strip, i, br, bg, bb);
            }
            led_animation_step++;
            break;
        }
        
        case LED_MODE_MUSIC_REACTIVE: {
            // Music-reactive LED effect: VU meter + beat flash + peak hold
            static float smooth_energy = 0.0f;
            static float peak_val = 0.0f;
            static uint8_t beat_flash = 0;
            static uint16_t hue_offset = 0;
            static uint32_t music_last_update = 0;
            
            // Rate limit to ~30fps (33ms) for smooth visual
            uint32_t music_now = (uint32_t)(esp_timer_get_time() / 1000);
            if (music_now - music_last_update < 33) {
                return; // skip refresh below too
            }
            music_last_update = music_now;
            
            float energy = ninja_led_get_audio_energy();
            
            // Exponential smoothing for baseline
            smooth_energy = smooth_energy * 0.85f + energy * 0.15f;
            
            // Peak hold with slow decay
            if (energy > peak_val) {
                peak_val = energy;
            } else {
                peak_val *= 0.95f;
            }
            
            // Beat detection: energy spike above smoothed baseline
            if (energy > smooth_energy * 1.8f + 0.08f && beat_flash == 0) {
                beat_flash = 6; // flash for 6 frames (~200ms)
                hue_offset = (hue_offset + 40) % 360; // shift color on each beat
            }
            
            // How many LEDs to light based on energy
            int num_leds = (int)(energy * LED_STRIP_COUNT + 0.5f);
            if (num_leds > LED_STRIP_COUNT) num_leds = LED_STRIP_COUNT;
            
            // Peak LED marker position
            int peak_led = (int)(peak_val * (LED_STRIP_COUNT - 1));
            if (peak_led >= LED_STRIP_COUNT) peak_led = LED_STRIP_COUNT - 1;
            
            if (beat_flash > 0) {
                // === BEAT FLASH: all LEDs burst in shifting hue ===
                uint8_t flash_intensity = (uint8_t)(beat_flash * 255 / 6);
                flash_intensity = apply_brightness(flash_intensity, led_state.brightness);
                uint8_t fr, fg, fb;
                hsv_to_rgb(hue_offset, 255, 255, &fr, &fg, &fb);
                fr = apply_brightness(fr, flash_intensity);
                fg = apply_brightness(fg, flash_intensity);
                fb = apply_brightness(fb, flash_intensity);
                for (int i = 0; i < LED_STRIP_COUNT; i++) {
                    led_strip_set_pixel(led_strip, i, fr, fg, fb);
                }
                beat_flash--;
            } else {
                // === VU METER: LEDs light up proportional to energy ===
                for (int i = 0; i < LED_STRIP_COUNT; i++) {
                    if (i < num_leds) {
                        // Color gradient: green(120) → yellow(60) → red(0) based on position
                        uint16_t hue = (uint16_t)(120 - (i * 120 / LED_STRIP_COUNT));
                        hue = (hue + hue_offset) % 360;
                        uint8_t val = (uint8_t)(energy * 255);
                        if (val < 80) val = 80; // minimum brightness for lit LEDs
                        uint8_t vr, vg, vb;
                        hsv_to_rgb(hue, 255, val, &vr, &vg, &vb);
                        vr = apply_brightness(vr, led_state.brightness);
                        vg = apply_brightness(vg, led_state.brightness);
                        vb = apply_brightness(vb, led_state.brightness);
                        led_strip_set_pixel(led_strip, i, vr, vg, vb);
                    } else if (i == peak_led && peak_val > 0.05f) {
                        // Peak hold indicator - white dot
                        uint8_t pw = apply_brightness(180, led_state.brightness);
                        led_strip_set_pixel(led_strip, i, pw, pw, pw);
                    } else {
                        // Background: dim blue glow based on ambient energy
                        uint8_t bg_glow = (uint8_t)(smooth_energy * 30);
                        bg_glow = apply_brightness(bg_glow, led_state.brightness);
                        led_strip_set_pixel(led_strip, i, 0, 0, bg_glow);
                    }
                }
            }
            break;
        }
    }
    
    led_strip_refresh(led_strip);
}

led_state_t* get_led_state(void) {
    return &led_state;
}

void save_led_state_to_nvs(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("robot_led", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) return;
    
    nvs_set_u8(nvs_handle, "led_r", led_state.r);
    nvs_set_u8(nvs_handle, "led_g", led_state.g);
    nvs_set_u8(nvs_handle, "led_b", led_state.b);
    nvs_set_u8(nvs_handle, "led_br", led_state.brightness);
    nvs_set_u8(nvs_handle, "led_mode", (uint8_t)led_state.mode);
    nvs_set_u16(nvs_handle, "led_speed", led_state.speed);
    
    nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    
    ESP_LOGI(TAG, "LED state saved to NVS");
}

void load_led_state_from_nvs(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("robot_led", NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) return;
    
    uint8_t val8;
    uint16_t val16;
    
    if (nvs_get_u8(nvs_handle, "led_r", &val8) == ESP_OK) led_state.r = val8;
    if (nvs_get_u8(nvs_handle, "led_g", &val8) == ESP_OK) led_state.g = val8;
    if (nvs_get_u8(nvs_handle, "led_b", &val8) == ESP_OK) led_state.b = val8;
    if (nvs_get_u8(nvs_handle, "led_br", &val8) == ESP_OK) led_state.brightness = val8;
    if (nvs_get_u8(nvs_handle, "led_mode", &val8) == ESP_OK) led_state.mode = (led_mode_t)val8;
    if (nvs_get_u16(nvs_handle, "led_speed", &val16) == ESP_OK) led_state.speed = val16;
    
    nvs_close(nvs_handle);
    
    ESP_LOGI(TAG, "LED state loaded from NVS");
}
