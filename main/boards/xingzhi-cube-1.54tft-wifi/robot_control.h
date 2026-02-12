/*
 * Xingzhi Cube Robot Control Header
 * Based on Otto Ninja - 4 Servo Version with LED Strip
 */

#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// GPIO Pin for LED strip (WS2812)
// Note: GPIO 43/44 = UART, power control disabled for LED
#define LED_STRIP_PIN           21
#define LED_STRIP_COUNT         8

// Servo channel assignments
typedef enum {
    SERVO_CH_LEFT_FOOT = 0,
    SERVO_CH_LEFT_LEG,
    SERVO_CH_RIGHT_FOOT,
    SERVO_CH_RIGHT_LEG,
    SERVO_CH_MAX
} servo_channel_t;

// Robot mode
typedef enum {
    MODE_WALK = 0,
    MODE_ROLL = 1
} robot_mode_t;

// Calibration settings structure
typedef struct {
    // Foot neutral points
    int lf_neutral;     // Left Foot Neutral (default 90)
    int rf_neutral;     // Right Foot Neutral (default 90)
    
    // Walking speeds
    int lffwrs;         // Left Foot Forward Walking Rotation Speed
    int rffwrs;         // Right Foot Forward Walking Rotation Speed
    int lfbwrs;         // Left Foot Backward Walking Rotation Speed
    int rfbwrs;         // Right Foot Backward Walking Rotation Speed
    
    // Standing positions
    int la0;            // Left Leg standing Position
    int ra0;            // Right Leg standing position
    
    // Walking tilt positions
    int latl;           // Left Leg tilt left position
    int ratl;           // Right Leg tilt left position
    int latr;           // Left Leg tilt right position
    int ratr;           // Right Leg tilt right position
    
    // Roll positions
    int la1;            // Left Leg roll Position
    int ra1;            // Right Leg roll position
    
    // Roll speed settings
    int roll_lf_fwd_speed;
    int roll_lf_bwd_speed;
    int roll_rf_fwd_speed;
    int roll_rf_bwd_speed;
    
    // Transform speeds
    int transform_ll_speed;
    int transform_rl_speed;
    
    // Turn speeds
    int turn_left_speed;
    int turn_right_speed;
    
    // Combo speeds
    int combo_lf_speed;
    int combo_rf_speed;
    
    // Battery alert settings
    bool battery_alert_enabled;
} calibration_t;

// Control state structure
typedef struct {
    int8_t j_x;         // Joystick X (-100 to 100)
    int8_t j_y;         // Joystick Y (-100 to 100)
    uint8_t button_a;   // Left arm
    uint8_t button_b;   // Right arm
    uint8_t button_x;   // Roll mode
    uint8_t button_y;   // Walk mode
    bool manual_mode;
    int move_duration_ms;
    bool test_mode_active;
    int test_cycles_remaining;
} control_state_t;

// Default calibration values
#define CAL_DEFAULT_LF_NEUTRAL  90
#define CAL_DEFAULT_RF_NEUTRAL  90
#define CAL_DEFAULT_LFFWRS      18
#define CAL_DEFAULT_RFFWRS      18
#define CAL_DEFAULT_LFBWRS      18
#define CAL_DEFAULT_RFBWRS      18
#define CAL_DEFAULT_LA0         60
#define CAL_DEFAULT_RA0         135
#define CAL_DEFAULT_LATL        100
#define CAL_DEFAULT_RATL        175
#define CAL_DEFAULT_LATR        5
#define CAL_DEFAULT_RATR        80
#define CAL_DEFAULT_LA1         160
#define CAL_DEFAULT_RA1         25
#define CAL_DEFAULT_ROLL_LF_FWD 45
#define CAL_DEFAULT_ROLL_LF_BWD 45
#define CAL_DEFAULT_ROLL_RF_FWD 45
#define CAL_DEFAULT_ROLL_RF_BWD 45
#define CAL_DEFAULT_TL_SPEED    5
#define CAL_DEFAULT_TR_SPEED    5
#define CAL_DEFAULT_TURN_L      500
#define CAL_DEFAULT_TURN_R      500
#define CAL_DEFAULT_COMBO_LF    1000
#define CAL_DEFAULT_COMBO_RF    1000
#define CAL_DEFAULT_TRANSFORM_LL 5
#define CAL_DEFAULT_TRANSFORM_RL 5
#define CAL_DEFAULT_BATTERY_ALERT true

// Action recording
#define MAX_ACTIONS 20
#define MAX_ACTION_SLOTS 3

typedef enum {
    ACTION_NONE = 0,
    ACTION_JOYSTICK,
    ACTION_BUTTON_A,
    ACTION_BUTTON_B,
    ACTION_WALK_MODE,
    ACTION_ROLL_MODE,
    ACTION_HOME,
    ACTION_TILT_LEFT,
    ACTION_TILT_RIGHT,
    ACTION_SERVO,
    ACTION_DELAY,
    ACTION_MOVE_FWD,
    ACTION_MOVE_BWD,
    ACTION_MOVE_STOP,
    ACTION_TURN_LEFT,
    ACTION_TURN_RIGHT,
    ACTION_RHYTHM_LEFT,
    ACTION_RHYTHM_RIGHT,
    ACTION_COMBO1,
    ACTION_COMBO2,
    ACTION_WALK_COMBO_123,
    ACTION_WALK_COMBO_345,
    ACTION_TEST_LF,
    ACTION_TEST_RF,
    ACTION_TEST_BOTH,
    ACTION_TEST_STOP
} action_type_t;

typedef struct {
    action_type_t type;
    int16_t param1;
    int16_t param2;
    uint16_t duration_ms;
} action_step_t;

typedef struct {
    uint8_t count;
    action_step_t steps[MAX_ACTIONS];
} action_slot_t;

typedef struct {
    bool is_recording;
    uint8_t current_slot;
    uint8_t step_count;
} recording_state_t;

// Initialize robot control
void robot_control_init(void);

// Prepare for sleep: turn off LED strip and detach all servos
void robot_prepare_sleep(void);

// Servo control functions
void servo_write(servo_channel_t channel, int angle);
void servo_attach(servo_channel_t channel);
void servo_detach(servo_channel_t channel);
bool servo_attached(servo_channel_t channel);

// Movement functions
void go_home(void);
void ninja_set_walk(void);
void ninja_set_roll(void);
void ninja_walk(void);
void ninja_roll(void);
void ninja_walk_stop(void);
void ninja_roll_stop(void);

// Calibration functions
calibration_t* get_calibration(void);
void set_calibration(const calibration_t* cal);
void reset_calibration_to_defaults(void);
void save_calibration_to_nvs(void);
void load_calibration_from_nvs(void);

// Control state
control_state_t* get_control_state(void);
robot_mode_t get_robot_mode(void);

// Tilt functions

// Arm functions (no-ops on 4-servo)
void ninja_left_arm_up(void);
void ninja_left_arm_down(void);
void ninja_right_arm_up(void);
void ninja_right_arm_down(void);
void ninja_tilt_left(void);
void ninja_tilt_right(void);

// Test functions
void test_left_foot(void);
void test_right_foot(void);
void test_both_feet(void);
void stop_feet_test(void);

// Dance/rhythm functions
void left_leg_rhythm(void);
void right_leg_rhythm(void);

// Combo functions
void ninja_combo1(void);
void ninja_combo2(void);

// Direct servo control
void servo_direct_write(int channel, int angle);
void set_manual_mode(bool enable);

// Action recording
void start_recording(uint8_t slot);
void stop_recording(void);
void record_action(action_type_t type, int16_t param1, int16_t param2, uint16_t duration);
void play_action(uint8_t slot);
void save_actions_to_nvs(uint8_t slot);
void load_actions_from_nvs(uint8_t slot);
recording_state_t* get_recording_state(void);
action_slot_t* get_action_slot(uint8_t slot);

// Walk phase control functions
void ninja_walk_phase1(void);
void ninja_walk_phase2(void);
void ninja_walk_phase3(void);
void ninja_walk_phase4(void);
void ninja_walk_phase5(void);
void ninja_walk_combo_123(void);
void ninja_walk_combo_345(void);

// OttoDIY oscillator-based dance functions
void ninja_moonwalk(int steps, int T, int dir);
void ninja_swing(int steps, int T, int h);
void ninja_updown(int steps, int T, int h);
void ninja_crusaito(int steps, int T, int h, int dir);
void ninja_flapping(int steps, int T, int h, int dir);
void ninja_jitter(int steps, int T, int h);
void ninja_ascending_turn(int steps, int T, int h);
void ninja_tiptoe_swing(int steps, int T, int h);

// Battery alert
bool is_battery_alert_enabled(void);

// ========== LED STRIP CONTROL ==========
// LED modes
typedef enum {
    LED_MODE_OFF = 0,
    LED_MODE_SOLID,
    LED_MODE_RAINBOW,
    LED_MODE_BREATHING,
    LED_MODE_CHASE,
    LED_MODE_BLINK,
    LED_MODE_STROBE,        // Strobe flash
    LED_MODE_FADE,          // Fade in/out
    LED_MODE_COMET,         // Comet tail
    LED_MODE_SPARKLE,       // Random sparkle
    LED_MODE_THEATER_CHASE  // Theater chase
} led_mode_t;

// LED state structure
typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t brightness;
    led_mode_t mode;
    uint16_t speed;
} led_state_t;

// LED control functions
void ninja_led_init(void);
void ninja_led_set_color(uint8_t r, uint8_t g, uint8_t b);
void ninja_led_set_pixel(uint8_t index, uint8_t r, uint8_t g, uint8_t b);
void ninja_led_set_brightness(uint8_t brightness);
void ninja_led_set_mode(led_mode_t mode);
void ninja_led_set_speed(uint16_t speed_ms);
void ninja_led_off(void);
void ninja_led_update(void);
led_state_t* get_led_state(void);
void save_led_state_to_nvs(void);
void load_led_state_from_nvs(void);

// Music playback functions (for dance)
void set_music_player_ptr(void* mp3_player_ptr);
bool ninja_play_music(const char* song_name);
bool ninja_is_music_playing(void);
void ninja_stop_music(void);

// Foot rotation functions (for web control)
void ninja_rotate_left_foot(int speed, int duration_ms);
void ninja_rotate_right_foot(int speed, int duration_ms);

// Main control loop task
void robot_control_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif // ROBOT_CONTROL_H
