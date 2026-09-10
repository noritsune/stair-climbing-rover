// ESP32-CAM (AI-Thinker) standalone camera streamer.
// The board runs as a Wi-Fi access point. Connect a phone to the AP
// and open http://192.168.4.1/ in a browser to view the live stream.
//
// Two HTTP servers run side by side:
//   port 80  : the viewer page, /control (camera settings) and /status
//   port 81  : /stream (MJPEG)
// They are separate because the stream handler occupies its HTTP worker for as
// long as a client is watching; on a single server /control would never answer.
//
// Board setting in Arduino IDE: "AI Thinker ESP32-CAM"
// (Tools > Board > esp32 > AI Thinker ESP32-CAM)

#include <WiFi.h>
#include <esp_wifi.h>
#include "esp_camera.h"
#include "esp_http_server.h"

#include "index_html.h"

// ---------- Access point settings ----------
static const char *AP_SSID = "RoverCam";
static const char *AP_PASSWORD = "12345678";  // 8+ chars required
static const int AP_CHANNEL = 1;
static const int AP_MAX_CONNECTIONS = 2;

// 802.11b only. Its lowest rates are DSSS, which a receiver can pull out of
// roughly 10 dB more noise than the OFDM rates 11g/n fall back to -- two to
// three times the usable distance, and the single biggest lever on range here.
// The trade is a 11 Mbit/s PHY ceiling, still far above what the stream needs.
// If a phone refuses to associate (a few Wi-Fi 6 clients have dropped 11b),
// widen this to WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N.
static const uint8_t AP_PROTOCOL_BITMAP = WIFI_PROTOCOL_11B;

// ---------- Server settings ----------
static const int WEB_PORT = 80;
static const int STREAM_PORT = 81;
static const int STREAM_MAX_CLIENTS = 3;

// esp_http_server runs a single worker task, so a socket that stops draining
// (the phone walked out of range) blocks every other request on that server
// until the send finally times out. Short socket timeouts plus TCP keep-alive
// bound how long a client that vanished can hold the server hostage, which is
// what makes the stream come back once the rover is in range again.
static const uint16_t SOCKET_SEND_TIMEOUT_S = 2;
static const uint16_t SOCKET_RECV_TIMEOUT_S = 2;
static const int KEEP_ALIVE_IDLE_S = 3;
static const int KEEP_ALIVE_INTERVAL_S = 1;
static const int KEEP_ALIVE_RETRIES = 3;

// ---------- Camera defaults and limits ----------
// Range beats picture quality for a rover camera: QVGA at quality 18 and
// 10 fps is roughly 0.5-1 Mbit/s, which an 11b link still carries at the
// distance where a VGA stream has already stalled. All three stay adjustable
// from the viewer page when the rover is close enough to spend the bandwidth.
static const framesize_t DEFAULT_FRAMESIZE = FRAMESIZE_QVGA;  // 320x240
static const int DEFAULT_JPEG_QUALITY = 18;  // 0-63, lower = better quality
static const int DEFAULT_FPS_LIMIT = 10;
static const int MIN_JPEG_QUALITY = 10;      // below this the encoder can stall
static const int MAX_JPEG_QUALITY = 63;
static const int MAX_FPS_LIMIT = 30;
static const int MS_PER_SECOND = 1000;

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

static httpd_handle_t webServer = NULL;
static httpd_handle_t streamServer = NULL;

// Largest frame size the buffers were allocated for. Set once at init.
static framesize_t maxFramesize = FRAMESIZE_VGA;

// Minimum gap between frames, 0 = send as fast as the sensor delivers.
// Written by the /control task, read by the stream task.
static volatile uint32_t frameIntervalMs = MS_PER_SECOND / DEFAULT_FPS_LIMIT;

static esp_err_t indexHandler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

// Link quality of the connected station, measured by the AP. The browser
// cannot read its own RSSI, so this is the only signal reading available to
// the viewer page. With one phone connected there is a single entry; the
// strongest is reported so a stray second client cannot drag the reading down.
static bool peakStationRssi(int8_t *rssiOut, int *clientsOut) {
  wifi_sta_list_t stations;
  if (esp_wifi_ap_get_sta_list(&stations) != ESP_OK) {
    return false;
  }
  *clientsOut = stations.num;
  if (stations.num <= 0) {
    return false;
  }

  int8_t peak = stations.sta[0].rssi;
  for (int i = 1; i < stations.num; i++) {
    if (stations.sta[i].rssi > peak) {
      peak = stations.sta[i].rssi;
    }
  }
  *rssiOut = peak;
  return true;
}

static esp_err_t statusHandler(httpd_req_t *req) {
  sensor_t *sensor = esp_camera_sensor_get();
  if (!sensor) {
    return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                               "camera sensor unavailable");
  }

  const uint32_t interval = frameIntervalMs;
  const int fps = interval > 0 ? MS_PER_SECOND / (int)interval : 0;

  int8_t rssi = 0;
  int clients = 0;
  char rssiJson[8];
  if (peakStationRssi(&rssi, &clients)) {
    snprintf(rssiJson, sizeof(rssiJson), "%d", (int)rssi);
  } else {
    strcpy(rssiJson, "null");
  }

  char json[220];
  snprintf(json, sizeof(json),
           "{\"framesize\":%d,\"quality\":%d,\"fps\":%d,"
           "\"maxFramesize\":%d,\"streamPort\":%d,"
           "\"rssi\":%s,\"clients\":%d}",
           (int)sensor->status.framesize, (int)sensor->status.quality, fps,
           (int)maxFramesize, STREAM_PORT, rssiJson, clients);

  httpd_resp_set_type(req, "application/json");
  return httpd_resp_sendstr(req, json);
}

// GET /control?var=<framesize|quality|fps>&val=<number>
static esp_err_t controlHandler(httpd_req_t *req) {
  char query[64];
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "query required");
  }

  char var[16];
  char val[16];
  if (httpd_query_key_value(query, "var", var, sizeof(var)) != ESP_OK ||
      httpd_query_key_value(query, "val", val, sizeof(val)) != ESP_OK) {
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                               "var and val required");
  }

  sensor_t *sensor = esp_camera_sensor_get();
  if (!sensor) {
    return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                               "camera sensor unavailable");
  }

  const int value = atoi(val);
  int result = -1;
  if (strcmp(var, "framesize") == 0) {
    if (value < 0 || value > (int)maxFramesize) {
      return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                 "framesize out of range");
    }
    result = sensor->set_framesize(sensor, (framesize_t)value);
  } else if (strcmp(var, "quality") == 0) {
    if (value < MIN_JPEG_QUALITY || value > MAX_JPEG_QUALITY) {
      return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                 "quality out of range");
    }
    result = sensor->set_quality(sensor, value);
  } else if (strcmp(var, "fps") == 0) {
    if (value < 0 || value > MAX_FPS_LIMIT) {
      return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                 "fps out of range");
    }
    frameIntervalMs = value > 0 ? MS_PER_SECOND / (uint32_t)value : 0;
    result = 0;
  } else {
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "unknown var");
  }

  if (result != 0) {
    return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                               "camera rejected the setting");
  }

  Serial.printf("Control: %s = %d\n", var, value);
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_sendstr(req, "ok");
}

static esp_err_t streamHandler(httpd_req_t *req) {
  esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    return res;
  }
  // The viewer page is served from port 80 and reads this stream with fetch(),
  // so to the browser it is a cross-origin request -- a different port is a
  // different origin. Without this header the read is blocked. (An <img> tag
  // would not need it, but it cannot detect a stalled stream.)
  res = httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  if (res != ESP_OK) {
    return res;
  }

  char partHeader[64];
  uint32_t lastFrameMs = 0;
  while (true) {
    const uint32_t interval = frameIntervalMs;
    if (interval > 0) {
      const uint32_t elapsed = millis() - lastFrameMs;
      if (elapsed < interval) {
        delay(interval - elapsed);
      }
    }
    lastFrameMs = millis();

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      // Also happens for a frame or two right after a resolution change;
      // the browser reconnects on its own.
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
  config.jpeg_quality = DEFAULT_JPEG_QUALITY;

  // The frame buffers are sized at init, so init at the largest resolution the
  // board can hold and step down afterwards. That is what lets /control raise
  // the resolution later without reinitialising the driver.
  if (psramFound()) {
    maxFramesize = FRAMESIZE_UXGA;  // 1600x1200
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
    maxFramesize = FRAMESIZE_QVGA;  // 320x240, all DRAM can spare
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }
  config.frame_size = maxFramesize;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }

  sensor_t *sensor = esp_camera_sensor_get();
  if (!sensor) {
    Serial.println("Camera sensor not found");
    return false;
  }
  const framesize_t startSize =
      DEFAULT_FRAMESIZE < maxFramesize ? DEFAULT_FRAMESIZE : maxFramesize;
  sensor->set_framesize(sensor, startSize);
  sensor->set_quality(sensor, DEFAULT_JPEG_QUALITY);
  return true;
}

static bool registerUri(httpd_handle_t server, const char *uri,
                        esp_err_t (*handler)(httpd_req_t *)) {
  httpd_uri_t definition = {
    .uri = uri,
    .method = HTTP_GET,
    .handler = handler,
    .user_ctx = NULL,
  };
  if (httpd_register_uri_handler(server, &definition) != ESP_OK) {
    Serial.printf("Failed to register %s\n", uri);
    return false;
  }
  return true;
}

// Shared by both servers: bound how long a client that stopped acknowledging
// can occupy a socket, and let the server drop the least recently used one so
// a reconnecting browser always finds a free slot.
static void applyConnectionPolicy(httpd_config_t *config) {
  config->send_wait_timeout = SOCKET_SEND_TIMEOUT_S;
  config->recv_wait_timeout = SOCKET_RECV_TIMEOUT_S;
  config->keep_alive_enable = true;
  config->keep_alive_idle = KEEP_ALIVE_IDLE_S;
  config->keep_alive_interval = KEEP_ALIVE_INTERVAL_S;
  config->keep_alive_count = KEEP_ALIVE_RETRIES;
  config->lru_purge_enable = true;
}

static bool startWebServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = WEB_PORT;
  applyConnectionPolicy(&config);

  if (httpd_start(&webServer, &config) != ESP_OK) {
    Serial.println("Failed to start web server");
    return false;
  }
  return registerUri(webServer, "/", indexHandler) &&
         registerUri(webServer, "/status", statusHandler) &&
         registerUri(webServer, "/control", controlHandler);
}

static bool startStreamServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = STREAM_PORT;
  config.ctrl_port += 1;  // must differ from the web server's control socket
  config.max_open_sockets = STREAM_MAX_CLIENTS;
  applyConnectionPolicy(&config);

  if (httpd_start(&streamServer, &config) != ESP_OK) {
    Serial.println("Failed to start stream server");
    return false;
  }
  return registerUri(streamServer, "/stream", streamHandler);
}

static void halt(const char *reason) {
  Serial.printf("Halting: %s\n", reason);
  while (true) {
    delay(1000);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println();

  if (!initCamera()) {
    halt("camera not available");
  }

  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL, false,
                   AP_MAX_CONNECTIONS)) {
    halt("failed to start access point");
  }
  // Range is the whole game here, and the board is mains/battery powered while
  // driving, so buy every dB available: full TX power, no modem sleep, and the
  // slow-but-tough 802.11b PHY.
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.setSleep(false);
  const esp_err_t protocolErr =
      esp_wifi_set_protocol(WIFI_IF_AP, AP_PROTOCOL_BITMAP);
  if (protocolErr != ESP_OK) {
    // Not fatal: the AP keeps serving on the default 11b/g/n mix, with less
    // reach. Worth seeing on the console, so log it instead of halting.
    Serial.printf("Failed to fix the AP to 802.11b: 0x%x\n", protocolErr);
  }

  if (!startWebServer() || !startStreamServer()) {
    halt("HTTP server not available");
  }

  Serial.println("Camera streamer ready");
  Serial.printf("SSID: %s / Password: %s\n", AP_SSID, AP_PASSWORD);
  Serial.print("Open http://");
  Serial.print(WiFi.softAPIP());
  Serial.println("/ in a browser");
}

void loop() {
  // Everything is handled by the HTTP server tasks.
  delay(1000);
}
