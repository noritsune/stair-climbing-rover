// ESP32-CAM (AI-Thinker) standalone camera streamer.
// The board runs as a Wi-Fi access point. Connect a phone to the AP
// and open http://192.168.4.1/ in a browser to view the live stream.
//
// Board setting in Arduino IDE: "AI Thinker ESP32-CAM"
// (Tools > Board > esp32 > AI Thinker ESP32-CAM)

#include <WiFi.h>
#include "esp_camera.h"
#include "esp_http_server.h"

// ---------- Access point settings ----------
static const char *AP_SSID = "RoverCam";
static const char *AP_PASSWORD = "12345678";  // 8+ chars required
static const int AP_CHANNEL = 1;
static const int AP_MAX_CONNECTIONS = 2;

// ---------- Camera pin map: AI-Thinker ESP32-CAM ----------
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

static const char *STREAM_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary=frame";
static const char *STREAM_BOUNDARY = "\r\n--frame\r\n";
static const char *STREAM_PART_HEADER =
    "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static const char INDEX_HTML[] = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Rover Camera</title>
<style>
  body { margin: 0; background: #111; color: #eee;
         font-family: sans-serif; text-align: center; }
  h1 { font-size: 1.2rem; padding: 0.5rem; margin: 0; }
  img { width: 100%; max-width: 640px; height: auto; }
</style>
</head>
<body>
<h1>Rover Camera</h1>
<img src="/stream" alt="camera stream">
</body>
</html>
)rawliteral";

static httpd_handle_t httpServer = NULL;

static esp_err_t indexHandler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t streamHandler(httpd_req_t *req) {
  esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    return res;
  }

  char partHeader[64];
  while (true) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
      break;
    }

    res = httpd_resp_send_chunk(req, STREAM_BOUNDARY,
                                strlen(STREAM_BOUNDARY));
    if (res == ESP_OK) {
      int headerLen = snprintf(partHeader, sizeof(partHeader),
                               STREAM_PART_HEADER, fb->len);
      res = httpd_resp_send_chunk(req, partHeader, headerLen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
    }

    esp_camera_fb_return(fb);

    if (res != ESP_OK) {
      // Client disconnected or send failed; end this stream session.
      break;
    }
  }
  return res;
}

static bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_LATEST;

  if (psramFound()) {
    config.frame_size = FRAMESIZE_VGA;  // 640x480
    config.jpeg_quality = 12;           // 0-63, lower = better quality
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
    config.frame_size = FRAMESIZE_QVGA;  // 320x240
    config.jpeg_quality = 15;
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }
  return true;
}

static bool startHttpServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  if (httpd_start(&httpServer, &config) != ESP_OK) {
    Serial.println("Failed to start HTTP server");
    return false;
  }

  httpd_uri_t indexUri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = indexHandler,
    .user_ctx = NULL,
  };
  httpd_uri_t streamUri = {
    .uri = "/stream",
    .method = HTTP_GET,
    .handler = streamHandler,
    .user_ctx = NULL,
  };
  httpd_register_uri_handler(httpServer, &indexUri);
  httpd_register_uri_handler(httpServer, &streamUri);
  return true;
}

void setup() {
  Serial.begin(115200);
  Serial.println();

  if (!initCamera()) {
    Serial.println("Halting: camera not available");
    while (true) {
      delay(1000);
    }
  }

  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL, false,
                   AP_MAX_CONNECTIONS)) {
    Serial.println("Halting: failed to start access point");
    while (true) {
      delay(1000);
    }
  }

  if (!startHttpServer()) {
    Serial.println("Halting: HTTP server not available");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("Camera streamer ready");
  Serial.printf("SSID: %s / Password: %s\n", AP_SSID, AP_PASSWORD);
  Serial.print("Open http://");
  Serial.print(WiFi.softAPIP());
  Serial.println("/ in a browser");
}

void loop() {
  // Everything is handled by the HTTP server task.
  delay(1000);
}
