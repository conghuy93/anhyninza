#include "camera_web_server.h"
#include <esp_log.h>
#include <esp_timer.h>
#include <img_converters.h>
#include <cstring>

static const char* TAG = "CameraWebServer";

CameraWebServer* CameraWebServer::instance_ = nullptr;

// HTML page with camera viewer
static const char* INDEX_HTML = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>XiaoZhi Camera</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { 
            font-family: Arial, sans-serif; 
            background: #1a1a2e; 
            color: #eee;
            min-height: 100vh;
            display: flex;
            flex-direction: column;
            align-items: center;
            padding: 20px;
        }
        h1 { 
            color: #00d4ff; 
            margin-bottom: 20px;
            text-shadow: 0 0 10px #00d4ff50;
        }
        .container {
            background: #16213e;
            border-radius: 15px;
            padding: 20px;
            box-shadow: 0 10px 30px rgba(0,0,0,0.3);
            max-width: 100%;
        }
        #stream {
            max-width: 100%;
            border-radius: 10px;
            border: 3px solid #00d4ff;
        }
        .controls {
            margin-top: 15px;
            display: flex;
            gap: 10px;
            justify-content: center;
            flex-wrap: wrap;
        }
        button {
            background: linear-gradient(135deg, #00d4ff, #0066ff);
            border: none;
            color: white;
            padding: 12px 25px;
            border-radius: 25px;
            cursor: pointer;
            font-size: 14px;
            font-weight: bold;
            transition: all 0.3s ease;
        }
        button:hover {
            transform: scale(1.05);
            box-shadow: 0 5px 20px rgba(0,212,255,0.4);
        }
        button:active { transform: scale(0.95); }
        .status {
            margin-top: 15px;
            padding: 10px;
            background: #0f3460;
            border-radius: 8px;
            text-align: center;
            font-size: 12px;
        }
        .status.online { border-left: 4px solid #00ff88; }
        .status.offline { border-left: 4px solid #ff4444; }
    </style>
</head>
<body>
    <h1>📷 XiaoZhi Camera</h1>
    <div class="container">
        <img id="stream" src="/stream" alt="Camera Stream">
        <div class="controls">
            <button onclick="capturePhoto()">📸 Chụp ảnh</button>
            <button onclick="toggleStream()">🔄 Bật/Tắt Stream</button>
            <button onclick="location.reload()">🔃 Làm mới</button>
        </div>
        <div id="status" class="status online">
            ✅ Camera đang hoạt động
        </div>
    </div>
    <script>
        let streamEnabled = true;
        const streamImg = document.getElementById('stream');
        const statusDiv = document.getElementById('status');
        
        streamImg.onerror = function() {
            statusDiv.className = 'status offline';
            statusDiv.innerHTML = '❌ Không thể kết nối camera';
        };
        
        streamImg.onload = function() {
            statusDiv.className = 'status online';
            statusDiv.innerHTML = '✅ Camera đang hoạt động';
        };
        
        function capturePhoto() {
            window.open('/capture', '_blank');
        }
        
        function toggleStream() {
            streamEnabled = !streamEnabled;
            if (streamEnabled) {
                streamImg.src = '/stream?' + Date.now();
                statusDiv.innerHTML = '✅ Stream đã bật';
            } else {
                streamImg.src = '';
                statusDiv.innerHTML = '⏸️ Stream đã tắt';
            }
        }
    </script>
</body>
</html>
)rawliteral";

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

CameraWebServer::CameraWebServer(Esp32Camera* camera) : camera_(camera) {
    instance_ = this;
}

CameraWebServer::~CameraWebServer() {
    Stop();
    instance_ = nullptr;
}

esp_err_t CameraWebServer::index_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, INDEX_HTML, strlen(INDEX_HTML));
}

esp_err_t CameraWebServer::capture_handler(httpd_req_t *req) {
    if (instance_ == nullptr || instance_->camera_ == nullptr) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "Camera capture failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    esp_err_t res = ESP_OK;
    
    if (fb->format == PIXFORMAT_JPEG) {
        res = httpd_resp_send(req, (const char*)fb->buf, fb->len);
    } else {
        // Convert to JPEG
        uint8_t* jpg_buf = nullptr;
        size_t jpg_len = 0;
        bool converted = frame2jpg(fb, 80, &jpg_buf, &jpg_len);
        if (converted) {
            res = httpd_resp_send(req, (const char*)jpg_buf, jpg_len);
            free(jpg_buf);
        } else {
            ESP_LOGE(TAG, "JPEG conversion failed");
            res = ESP_FAIL;
        }
    }

    esp_camera_fb_return(fb);
    return res;
}

esp_err_t CameraWebServer::stream_handler(httpd_req_t *req) {
    if (instance_ == nullptr || instance_->camera_ == nullptr) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    if (res != ESP_OK) {
        return res;
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "X-Framerate", "15");

    instance_->is_streaming_ = true;
    char part_buf[64];
    
    ESP_LOGI(TAG, "MJPEG stream started");

    while (instance_->is_streaming_) {
        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        uint8_t* jpg_buf = nullptr;
        size_t jpg_len = 0;
        bool need_free = false;

        if (fb->format == PIXFORMAT_JPEG) {
            jpg_buf = fb->buf;
            jpg_len = fb->len;
        } else {
            need_free = frame2jpg(fb, 70, &jpg_buf, &jpg_len);
            if (!need_free) {
                ESP_LOGE(TAG, "JPEG conversion failed");
                esp_camera_fb_return(fb);
                continue;
            }
        }

        // Send boundary
        res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
        if (res != ESP_OK) {
            if (need_free) free(jpg_buf);
            esp_camera_fb_return(fb);
            break;
        }

        // Send part header
        size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, jpg_len);
        res = httpd_resp_send_chunk(req, part_buf, hlen);
        if (res != ESP_OK) {
            if (need_free) free(jpg_buf);
            esp_camera_fb_return(fb);
            break;
        }

        // Send JPEG data
        res = httpd_resp_send_chunk(req, (const char*)jpg_buf, jpg_len);
        
        if (need_free) {
            free(jpg_buf);
        }
        esp_camera_fb_return(fb);

        if (res != ESP_OK) {
            break;
        }

        // Limit framerate
        vTaskDelay(pdMS_TO_TICKS(66)); // ~15 FPS
    }

    instance_->is_streaming_ = false;
    ESP_LOGI(TAG, "MJPEG stream stopped");
    
    return res;
}

bool CameraWebServer::Start(int port) {
    if (server_handle_ != nullptr) {
        ESP_LOGW(TAG, "Server already running");
        return true;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = port;
    config.max_uri_handlers = 8;
    config.stack_size = 8192;
    config.core_id = 1;

    // Index page
    httpd_uri_t index_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = index_handler,
        .user_ctx = nullptr
    };

    // Capture endpoint
    httpd_uri_t capture_uri = {
        .uri = "/capture",
        .method = HTTP_GET,
        .handler = capture_handler,
        .user_ctx = nullptr
    };

    // Stream endpoint
    httpd_uri_t stream_uri = {
        .uri = "/stream",
        .method = HTTP_GET,
        .handler = stream_handler,
        .user_ctx = nullptr
    };

    ESP_LOGI(TAG, "Starting camera web server on port %d", port);
    
    if (httpd_start(&server_handle_, &config) == ESP_OK) {
        httpd_register_uri_handler(server_handle_, &index_uri);
        httpd_register_uri_handler(server_handle_, &capture_uri);
        httpd_register_uri_handler(server_handle_, &stream_uri);
        ESP_LOGI(TAG, "Camera web server started successfully");
        ESP_LOGI(TAG, "Access camera at: http://<device-ip>:%d/", port);
        return true;
    }

    ESP_LOGE(TAG, "Failed to start camera web server");
    return false;
}

void CameraWebServer::Stop() {
    is_streaming_ = false;
    if (server_handle_) {
        httpd_stop(server_handle_);
        server_handle_ = nullptr;
        ESP_LOGI(TAG, "Camera web server stopped");
    }
}
