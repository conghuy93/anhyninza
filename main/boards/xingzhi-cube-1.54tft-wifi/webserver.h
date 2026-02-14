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

#ifdef __cplusplus
}
#endif

#endif // WEBSERVER_H
