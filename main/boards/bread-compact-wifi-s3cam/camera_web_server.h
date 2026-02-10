#pragma once

#include <esp_http_server.h>
#include "esp32_camera.h"

class CameraWebServer {
private:
    httpd_handle_t server_handle_ = nullptr;
    Esp32Camera* camera_ = nullptr;
    bool is_streaming_ = false;
    
    static CameraWebServer* instance_;
    
    static esp_err_t index_handler(httpd_req_t *req);
    static esp_err_t capture_handler(httpd_req_t *req);
    static esp_err_t stream_handler(httpd_req_t *req);

public:
    CameraWebServer(Esp32Camera* camera);
    ~CameraWebServer();
    
    bool Start(int port = 80);
    void Stop();
    
    bool IsRunning() const { return server_handle_ != nullptr; }
};
