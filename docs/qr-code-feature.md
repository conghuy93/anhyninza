# QR Code Display Feature - Tính Năng Hiển Thị Mã QR

> **Tổng quan**: Hiển thị mã QR trên màn hình robot để truy cập giao diện web điều khiển hoặc trang cài đặt.

## 📱 Giới Thiệu

Tính năng QR Code cho phép robot tự động hiển thị mã QR chứa địa chỉ IP và đường dẫn trang web khi:
1. **Người dùng nói**: "trang điều khiển", "trang cài đặt", "mã qr"
2. **LLM gọi MCP tool**: `self.qr_code` với tham số `page`

### ✨ Đặc Điểm
- ✅ Tự động lấy địa chỉ IP WiFi của robot
- ✅ Tạo URL tương ứng (http://IP hoặc http://IP/#cai-dat)
- ✅ Hiển thị QR code ở giữa màn hình (140×140px, scale 2-4×)
- ✅ Tự động ẩn sau 30 giây
- ✅ Hỗ trợ RGB565 format (LVGL 9.4)

---

## 🎯 Cách Sử Dụng

### 1️⃣ Voice Commands (Keywords Detection)

Người dùng có thể nói các câu sau để hiển thị QR code:

#### **Trang Điều Khiển**
```
"cho tôi xem mã QR trang điều khiển"
"hiển thị mã QR điều khiển"
"mã QR điều khiển"
"trang điều khiển"
```
→ Hiển thị QR code chứa URL: `http://192.168.1.100` (IP của robot)

#### **Trang Cài Đặt**
```
"cho tôi xem mã QR trang cài đặt"
"hiển thị mã QR cài đặt"
"mã QR cài đặt"
"trang cài đặt"
```
→ Hiển thị QR code chứa URL: `http://192.168.1.100/#cai-dat`

### 2️⃣ MCP Tool (Programmatic Control)

LLM có thể gọi tool để hiển thị QR code:

#### **Tool Definition**
```cpp
mcp_server.AddTool("self.qr_code",
    "Hiển thị mã QR trên màn hình để truy cập trang web điều khiển hoặc cài đặt. "
    "Mã QR sẽ tự động ẩn sau 30 giây.",
    PropertyList({
        Property("page", kPropertyTypeString)
            .SetEnum({"control", "settings"})
            .SetDescription("Loại trang: 'control' = trang điều khiển, 'settings' = trang cài đặt")
    }),
    [](const PropertyList& properties) -> ReturnValue {
        std::string page = properties["page"].value<std::string>();
        show_qr_for_page(page.c_str());
        return "Đã hiển thị mã QR trang " + page + " trên màn hình (30 giây)";
    });
```

#### **JSON-RPC Request Example**
```json
{
  "jsonrpc": "2.0",
  "method": "tools/call",
  "params": {
    "name": "self.qr_code",
    "arguments": {
      "page": "control"
    }
  },
  "id": 123
}
```

#### **Response**
```json
{
  "jsonrpc": "2.0",
  "result": {
    "content": [
      {
        "type": "text",
        "text": "Đã hiển thị mã QR trang control trên màn hình (30 giây)"
      }
    ]
  },
  "id": 123
}
```

---

## 🛠️ Technical Implementation

### Architecture

```
User Voice → STT → check_stt_keywords()
                        ↓
                   "trang điều khiển" match?
                        ↓ YES
                   show_qr_for_page("control")
                        ↓
         ┌──────────────┴──────────────┐
         ▼                             ▼
   Get WiFi IP              Build URL: http://192.168.1.100
         ▼                             ▼
   qrcodegen_encodeText()    Create lv_canvas (RGB565)
         ▼                             ▼
   Draw scaled modules       Center on screen (240×240)
         ▼                             ▼
   Start 30s timer          Display QR code
         ▼
   (after 30s) hide_qr_code()
         ▼
   Delete canvas + Free buffer
```

### Key Functions

#### **1. show_qr_code(url, duration_ms)**
```cpp
void show_qr_code(const char* url, uint32_t duration_ms)
```
- Generates QR code using `qrcodegen_encodeText()` (version 5, max 134 chars)
- Creates LVGL canvas with RGB565 format
- Scales QR modules (2-4× based on size)
- Centers on 240×240 screen
- Starts auto-hide timer

#### **2. hide_qr_code()**
```cpp
void hide_qr_code(void)
```
- Stops timer
- Deletes canvas object
- Frees allocated buffer (SPIRAM)

#### **3. show_qr_for_page(page)**
```cpp
void show_qr_for_page(const char* page)
```
- Gets WiFi IP: `esp_netif_get_ip_info()`
- Builds URL:
  - `"control"` → `http://192.168.1.100`
  - `"settings"` → `http://192.168.1.100/#cai-dat`
- Calls `show_qr_code()` với 30s timeout

#### **4. check_stt_keywords(stt_text)**
```cpp
extern "C" void check_stt_keywords(const char* stt_text)
```
- Called from `application.cc` STT handler
- Detects keywords (case-insensitive):
  - **Control page**: "trang điều khiển", "điều khiển", "mã qr" + "điều khiển"
  - **Settings page**: "trang cài đặt", "cài đặt", "mã qr" + "cài đặt"
- Calls `show_qr_for_page()` on match

### Code Files

| File | Purpose | Key Code |
|------|---------|----------|
| **webserver.c** | Implementation | - Lines 20-46: qrcodegen extern declarations<br>- Lines 2407-2476: `show_qr_code()`<br>- Lines 2478-2488: `hide_qr_code()`<br>- Lines 2490-2506: `show_qr_for_page()`<br>- Lines 2577-2609: Keywords detection |
| **webserver.h** | C/C++ Bridge | - Lines 54-63: Function exports |
| **robot_mcp_controller.h** | MCP Tool | - Lines 724-742: `self.qr_code` tool registration |
| **application.cc** | STT Integration | - Line 612: `check_stt_keywords()` call |

---

## 🧪 Testing

### Test 1: Voice Trigger - Control Page
```bash
# Monitor logs
idf.py monitor | grep "QR"

# User says: "cho tôi xem trang điều khiển"
# Expected logs:
[webserver] 🎯 QR Code: Detected keywords for control page
[webserver] 📱 QR Code: Generating for http://192.168.1.100
[webserver] ✅ QR Code: Displayed (30s timeout)

# After 30s:
[webserver] ⏰ QR Code: Timer expired, hiding
```

### Test 2: Voice Trigger - Settings Page
```bash
# User says: "hiển thị mã QR cài đặt"
# Expected:
[webserver] 🎯 QR Code: Detected keywords for settings page
[webserver] 📱 QR Code: Generating for http://192.168.1.100/#cai-dat
[webserver] ✅ QR Code: Displayed (30s timeout)
```

### Test 3: MCP Tool Call
```bash
# Send JSON-RPC request:
{
  "method": "tools/call",
  "params": {
    "name": "self.qr_code",
    "arguments": {"page": "control"}
  }
}

# Expected response:
{
  "result": {
    "content": [{
      "type": "text",
      "text": "Đã hiển thị mã QR trang control trên màn hình (30 giây)"
    }]
  }
}
```

### Test 4: Scan QR Code
1. Display QR on robot screen
2. Scan with phone camera
3. Verify opens correct URL:
   - Control: `http://192.168.1.100`
   - Settings: `http://192.168.1.100/#cai-dat`
4. Verify web interface loads

---

## 🔧 Configuration

### QR Code Parameters
```cpp
// In webserver.c - show_qr_code()

#define QR_VERSION 5              // QR version (5 = 37×37 modules, max 134 chars)
#define QR_ECC qrcodegen_Ecc_LOW  // Error correction: LOW (fastest)
#define QR_TARGET_SIZE 140        // Target display size in pixels
#define QR_MIN_SCALE 2            // Minimum scale factor
#define QR_MAX_SCALE 4            // Maximum scale factor
#define QR_AUTO_HIDE_MS 30000     // Auto-hide timeout: 30 seconds
```

### Display Properties
```cpp
// Screen size: 240×240 pixels
// QR position: Centered ((240 - canvas_size) / 2, (240 - canvas_size) / 2)
// Color format: RGB565 (16 bpp)
// Memory: Allocated in SPIRAM with malloc()
```

---

## 🔍 Troubleshooting

### Issue 1: QR Code Not Displayed
**Symptoms**: Keywords detected but no QR appears

**Debug Steps**:
```cpp
// Check WiFi connected
esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
// Should not be NULL

// Check IP obtained
esp_netif_ip_info_t ip_info;
esp_netif_get_ip_info(netif, &ip_info);
ESP_LOGI(TAG, "IP: " IPSTR, IP2STR(&ip_info.ip));
// Should show valid IP (not 0.0.0.0)
```

**Solution**: Ensure WiFi is connected before triggering QR display.

### Issue 2: Buffer Allocation Failed
**Symptoms**: Log shows "Failed to allocate canvas buffer"

**Debug Steps**:
```cpp
ESP_LOGI(TAG, "Free PSRAM: %d", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
```

**Solution**: 
- Check available SPIRAM (should have > 50KB free)
- Reduce QR_VERSION or QR_MAX_SCALE if memory is tight

### Issue 3: QR Code Not Readable
**Symptoms**: Phone can't scan QR code

**Possible Causes**:
1. **Scale too small**: QR pixels are < 2×2 screen pixels
   - Solution: Increase QR_MIN_SCALE to 3
2. **URL too long**: Exceeds QRv5 capacity (134 chars)
   - Solution: Use shorter URLs or increase QR_VERSION
3. **Screen brightness low**: Hard to scan
   - Solution: Increase display brightness

### Issue 4: Memory Leak
**Symptoms**: Heap slowly decreases after repeated QR displays

**Debug Steps**:
```cpp
// Before show_qr_code()
size_t heap_before = esp_get_free_heap_size();

// After hide_qr_code()
size_t heap_after = esp_get_free_heap_size();

ESP_LOGI(TAG, "Heap difference: %d bytes", heap_before - heap_after);
// Should be 0 (or very small)
```

**Solution**: Verify `hide_qr_code()` is called and frees buffer:
```cpp
if (qr_canvas_buf) {
    free(qr_canvas_buf);
    qr_canvas_buf = NULL;  // ⚠️ Important: set to NULL!
}
```

---

## 📊 Performance Metrics

| Metric | Value | Notes |
|--------|-------|-------|
| **QR Generation Time** | ~50ms | qrcodegen_encodeText() for v5 |
| **Canvas Rendering Time** | ~20-40ms | Depends on scale (2-4×) |
| **Memory Usage** | 10-40KB | Canvas buffer size varies by scale |
| **Auto-hide Timer** | 30s | Configurable via QR_AUTO_HIDE_MS |
| **Max URL Length** | 134 chars | QR version 5 limit |

---

## 🎯 Best Practices

### 1. **WiFi Check Before Display**
```cpp
// Always verify WiFi connected
esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
if (netif == NULL) {
    ESP_LOGW(TAG, "WiFi not connected, cannot show QR");
    return;
}
```

### 2. **Thread-Safe LVGL Operations**
```cpp
// Use LVGL lock when creating/deleting objects
if (qr_canvas) {
    DisplayLockGuard lock(display); // Lock LVGL mutex
    lv_obj_del(qr_canvas);
    qr_canvas = NULL;
}
```

### 3. **Timer Management**
```cpp
// Always stop existing timer before starting new one
if (qr_hide_timer) {
    esp_timer_stop(qr_hide_timer);  // Prevent multiple timers
}
esp_timer_start_once(qr_hide_timer, duration_ms * 1000);
```

### 4. **Memory Management**
```cpp
// Allocate large buffers in SPIRAM
void* buf = malloc(size);  // Uses SPIRAM by default
if (!buf) {
    ESP_LOGE(TAG, "Failed to allocate %d bytes", size);
    return;
}

// Always free on error paths
if (!some_condition) {
    free(buf);
    return;
}
```

### 5. **Error Handling**
```cpp
// Check qrcodegen return value
bool ok = qrcodegen_encodeText(url, tempBuffer, qrcode, ...);
if (!ok) {
    ESP_LOGE(TAG, "❌ QR encode failed (URL too long?)");
    return;
}
```

---

## 🔗 Related Documentation

- **[mcp-usage.md](./mcp-usage.md)**: MCP protocol and tools overview
- **[mcp-protocol.md](./mcp-protocol.md)**: Detailed MCP protocol specification
- **[QR Code Generator Library](https://www.nayuki.io/page/qr-code-generator-library)**: qrcodegen library documentation
- **[LVGL Canvas](https://docs.lvgl.io/9.4/widgets/canvas.html)**: LVGL canvas widget guide

---

## 📝 Keywords Reference

| Category | Vietnamese Keywords | English Keywords |
|----------|-------------------|------------------|
| **Control Page** | "trang điều khiển"<br>"điều khiển"<br>"mã qr điều khiển" | "control page"<br>"control"<br>"qr control" |
| **Settings Page** | "trang cài đặt"<br>"cài đặt"<br>"mã qr cài đặt" | "settings page"<br>"settings"<br>"qr settings" |
| **Generic** | "mã qr"<br>"qr code"<br>"hiển thị mã qr" | "qr code"<br>"show qr"<br>"display qr" |

---

## 🆕 Future Enhancements

- [ ] Support custom QR content via MCP tool
- [ ] Add QR display duration parameter
- [ ] Support different QR error correction levels
- [ ] Cache generated QR codes for faster re-display
- [ ] Add QR display animation (fade in/out)
- [ ] Support displaying text below QR code
- [ ] Add screenshot/save QR to file feature

---

**Version**: 1.0  
**Last Updated**: 2025-02-15  
**Board**: XingZhi Cube 1.54" TFT WiFi  
**Display**: 240×240 SPI LCD (RGB565)  
**LVGL Version**: 9.4.0  
**QR Library**: qrcodegen (from esp_emote_gfx component)
