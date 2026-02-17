/*
 * Xingzhi Cube Robot Web Server Header
 * Based on Otto Ninja - 4 Servo Version
 */

#ifndef WEBSERVER_H
#define WEBSERVER_H

#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

// Start the web server
httpd_handle_t webserver_start(void);

// Stop the web server
void webserver_stop(httpd_handle_t server);

// Sleep config callbacks (implemented in board .cc, called from webserver.c)
void set_sleep_config(int sleep_sec, int shutdown_sec);
void get_sleep_config(int* sleep_sec, int* shutdown_sec);

// Music power save mode (implemented in board .cc, called from webserver.c)
void set_music_power_save(int enable);
int get_music_power_save(void);

// Chat AI functions (implemented in board .cc, called from webserver.c)
void send_text_to_ai(const char* text);
const char* get_chat_history_json(void);
const char* get_last_ai_response(void);
void add_chat_message(const char* role, const char* content);

// Alarm functions (for MCP tool access)
int alarm_add_from_mcp(const char* type, int hour, int minute, const char* repeat, const char* message, const char* music, const char* music_name);

// Music search in SD card (for MCP tool access)
// Returns number of files found, result_buffer contains '\n' separated paths
int search_music_files_in_sdcard(const char* keyword, char* result_buffer, size_t buffer_size, int max_results);

// Emoji/display bridge (implemented in board .cc, called from webserver.c)
void set_robot_emoji(const char* emotion);

// ==================== KEYWORD DETECTION (Local, no LLM) ====================
// Trigger play dead animation (non-blocking, spawns FreeRTOS task)
void trigger_play_dead(void);

// Emoji lock mechanism - locks emoji during keyword-triggered actions
// When locked, LLM emotion changes are blocked until TTS ends
void set_emoji_lock(int locked);  // 1=lock, 0=unlock
int  get_emoji_lock(void);        // returns current lock state

// Check STT text for local keyword triggers (called from application.cc)
// Returns 1 if a keyword was detected and action triggered, 0 otherwise
int check_stt_keywords(const char* text);

// ==================== QR CODE DISPLAY ====================
// Show QR code with URL on the TFT display (auto-hides after duration_ms)
void show_qr_code(const char* url, int duration_ms);

// Hide QR code from display (manual dismiss)
void hide_qr_code(void);

// Show QR code for a specific page ("control" or "settings")
// Automatically gets WiFi IP and builds the URL
void show_qr_for_page(const char* page);

#ifdef __cplusplus
}
#endif

#endif // WEBSERVER_H
