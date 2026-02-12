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

#ifdef __cplusplus
}
#endif

#endif // WEBSERVER_H
