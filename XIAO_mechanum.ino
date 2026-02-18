///////////////////////////////////////////////////////////////////////////
// XIAO ESP32S3 Sense - Mechanum Wheel Robot + WebSocket AI Control
//
// Features:
//   1. VL53L0X 거리 기반 자동 정지 (STOP_DISTANCE 조절 가능)
//   2. 카메라 → OpenAI Vision API → OLED 물체명 표시
//   3. WebSocket 클라이언트 → Node.js 서버 실시간 통신
//   4. AI 자연어 명령 수신 → 모터/카메라/OLED 제어
//   5. 웹 카메라 뷰어 (포트 80) - JPEG 스냅샷 폴링
//      http://<IP>/        → 카메라 뷰 + AI 인식 중 오버레이
//      http://<IP>/capture → JPEG 1장 반환 (JS가 ~10fps로 반복 호출)
//      http://<IP>/status  → {"capturing":true/false}
//
// I2C 구성:
//   - VL53L0X + OLED: Wire (I2C_NUM_0)
//   - 카메라 SCCB: I2C_NUM_1 (sccb_i2c_port=1로 분리)
//   - 카메라: 부팅 시 초기화 후 상시 유지
//
// 필요 라이브러리:
//   ArduinoWebsockets (gilmaimon), ArduinoJson (v6)
//
///////////////////////////////////////////////////////////////////////////

#include <Wire.h>
#include <VL53L0X.h>
#include <U8x8lib.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include "esp_camera.h"
#include <base64.h>
#include <ArduinoWebsockets.h>
#include <ArduinoJson.h>
#include <WebServer.h>

using namespace websockets;

// ===== 빌드 전 조절 가능한 설정값 =====
#define STOP_DISTANCE         300    // mm - 이 거리 이내 물체 감지 시 정지
#define MOTOR_SPEED           200    // 0-255 기본 전진 속도
#define WS_RECONNECT_INTERVAL 5000   // ms - WS 재접속 간격
#define DIST_STREAM_INTERVAL  500    // ms - 거리 데이터 스트리밍 간격
#define RECOGNITION_COOLDOWN  3000   // ms - 인식 후 재감지 금지 시간
#define DETECT_CONFIRM_COUNT  3      // 연속 N회 감지해야 트리거
#define MAX_DURATION_MS       5000   // ms - 모터 명령 최대 지속시간

// ===== WiFi & API 설정 =====
const char* WIFI_SSID      = "asdf";
const char* WIFI_PASSWORD   = "qwe789zxc123";
const char* OPENAI_API_KEY  = "";

// ===== WebSocket 서버 설정 (★ 서버 IP를 실제 주소로 변경하세요) =====
const char* WS_SERVER_URL = "ws://192.168.43.148:3000/ws/esp32";

// ===== Motor Pins =====
#define MOTOR_A_P    7
#define MOTOR_A_D    44
#define MOTOR_B_P    8
#define MOTOR_B_D    9
#define MOTOR_C_P    3
#define MOTOR_C_D    4
#define MOTOR_D_P    2
#define MOTOR_D_D    1

// ===== Camera Pins (XIAO ESP32S3 Sense 내부 연결) =====
#define PWDN_GPIO_NUM   -1
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM   10
#define SIOD_GPIO_NUM   40
#define SIOC_GPIO_NUM   39
#define Y9_GPIO_NUM     48
#define Y8_GPIO_NUM     11
#define Y7_GPIO_NUM     12
#define Y6_GPIO_NUM     14
#define Y5_GPIO_NUM     16
#define Y4_GPIO_NUM     18
#define Y3_GPIO_NUM     17
#define Y2_GPIO_NUM     15
#define VSYNC_GPIO_NUM  38
#define HREF_GPIO_NUM   47
#define PCLK_GPIO_NUM   13

// ===== Global Objects =====
VL53L0X distSensor;
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(U8X8_PIN_NONE);
WebsocketsClient wsClient;
WebServer webServer(80);

// ===== MJPEG Stream (FreeRTOS, Core 0) =====
SemaphoreHandle_t camMutex    = NULL;
volatile bool     streamRunning = false;
WiFiClient        streamCl;

// State
bool objectDetected = false;
bool wifiReady = false;
bool wsConnected = false;
bool autonomousMode = true;
bool cameraReady = false;
volatile bool cameraInUse = false;  // 카메라 하드웨어 사용 중
bool recognizing = false;           // 전체 인식 과정 중 (캡처 + API)
unsigned long lastRecognitionTime = 0;
int detectConfirmCount = 0;
unsigned long lastWsReconnect = 0;
unsigned long lastDistStream = 0;

// Camera command state (WS 끊김 대응)
bool cameraRequested = false;
bool cameraPending = false;
String cameraCmdId;
String cameraResult;

///////////////////////////////////////////////////////////////////////////
// Motor Control
///////////////////////////////////////////////////////////////////////////

void forward(unsigned int speed) {
  digitalWrite(MOTOR_A_D, HIGH); digitalWrite(MOTOR_B_D, HIGH);
  digitalWrite(MOTOR_C_D, LOW);  digitalWrite(MOTOR_D_D, LOW);
  analogWrite(MOTOR_A_P, speed); analogWrite(MOTOR_B_P, speed);
  analogWrite(MOTOR_C_P, speed); analogWrite(MOTOR_D_P, speed);
}

void backward(unsigned int speed) {
  digitalWrite(MOTOR_A_D, LOW);  digitalWrite(MOTOR_B_D, LOW);
  digitalWrite(MOTOR_C_D, HIGH); digitalWrite(MOTOR_D_D, HIGH);
  analogWrite(MOTOR_A_P, speed); analogWrite(MOTOR_B_P, speed);
  analogWrite(MOTOR_C_P, speed); analogWrite(MOTOR_D_P, speed);
}

void stopMotors() {
  analogWrite(MOTOR_A_P, 0); analogWrite(MOTOR_B_P, 0);
  analogWrite(MOTOR_C_P, 0); analogWrite(MOTOR_D_P, 0);
}

void t_l(unsigned int speed) {
  digitalWrite(MOTOR_A_D, LOW);  digitalWrite(MOTOR_B_D, HIGH);
  digitalWrite(MOTOR_C_D, LOW);  digitalWrite(MOTOR_D_D, HIGH);
  analogWrite(MOTOR_A_P, speed); analogWrite(MOTOR_B_P, speed);
  analogWrite(MOTOR_C_P, speed); analogWrite(MOTOR_D_P, speed);
}

void t_r(unsigned int speed) {
  digitalWrite(MOTOR_A_D, HIGH); digitalWrite(MOTOR_B_D, LOW);
  digitalWrite(MOTOR_C_D, HIGH); digitalWrite(MOTOR_D_D, LOW);
  analogWrite(MOTOR_A_P, speed); analogWrite(MOTOR_B_P, speed);
  analogWrite(MOTOR_C_P, speed); analogWrite(MOTOR_D_P, speed);
}

void s_r(unsigned int speed) {
  digitalWrite(MOTOR_A_D, HIGH); digitalWrite(MOTOR_B_D, LOW);
  digitalWrite(MOTOR_C_D, LOW);  digitalWrite(MOTOR_D_D, HIGH);
  analogWrite(MOTOR_A_P, speed); analogWrite(MOTOR_B_P, speed);
  analogWrite(MOTOR_C_P, speed); analogWrite(MOTOR_D_P, speed);
}

void s_l(unsigned int speed) {
  digitalWrite(MOTOR_A_D, LOW);  digitalWrite(MOTOR_B_D, HIGH);
  digitalWrite(MOTOR_C_D, HIGH); digitalWrite(MOTOR_D_D, LOW);
  analogWrite(MOTOR_A_P, speed); analogWrite(MOTOR_B_P, speed);
  analogWrite(MOTOR_C_P, speed); analogWrite(MOTOR_D_P, speed);
}

///////////////////////////////////////////////////////////////////////////
// WebSocket Helpers
///////////////////////////////////////////////////////////////////////////

void wsSendEvent(const char* eventName) {
  if (!wsConnected) return;
  String msg = "{\"type\":\"event\",\"event\":\""; msg += eventName; msg += "\"}";
  wsClient.send(msg);
}

void wsSendEventData(const char* eventName, const String& dataJson) {
  if (!wsConnected) return;
  String msg = "{\"type\":\"event\",\"event\":\""; msg += eventName;
  msg += "\",\"data\":"; msg += dataJson; msg += "}";
  wsClient.send(msg);
}

void wsSendResponse(const char* cmdId, const char* status, const String& dataJson) {
  if (!wsConnected) return;
  String msg = "{\"type\":\"response\",\"id\":\""; msg += cmdId;
  msg += "\",\"status\":\""; msg += status; msg += "\",\"data\":"; msg += dataJson; msg += "}";
  wsClient.send(msg);
}

///////////////////////////////////////////////////////////////////////////
// OLED
///////////////////////////////////////////////////////////////////////////

void oledShowLine(int row, const char* text) {
  char buf[17];
  snprintf(buf, sizeof(buf), "%-16s", text);
  u8x8.drawString(0, row, buf);
}

///////////////////////////////////////////////////////////////////////////
// WebSocket Command Handler
///////////////////////////////////////////////////////////////////////////

void handleWsMessage(WebsocketsMessage message) {
  String payload = message.data();
  Serial.println("WS << " + payload);

  StaticJsonDocument<512> doc;
  if (deserializeJson(doc, payload)) { Serial.println("JSON parse error"); return; }

  const char* id = doc["id"] | "0";
  const char* command = doc["command"];
  if (!command) return;

  String cmd(command);
  JsonObject params = doc["params"];
  int speed    = params["speed"] | MOTOR_SPEED;
  int duration = params["duration_ms"] | 0;
  if (duration > MAX_DURATION_MS) duration = MAX_DURATION_MS;

  bool isMotorCmd = false;

  if      (cmd == "move_forward")   { forward(speed);   isMotorCmd = true; }
  else if (cmd == "move_backward")  { backward(speed);  isMotorCmd = true; }
  else if (cmd == "stop_motors")    { stopMotors(); }
  else if (cmd == "turn_left")      { t_l(speed);       isMotorCmd = true; }
  else if (cmd == "turn_right")     { t_r(speed);       isMotorCmd = true; }
  else if (cmd == "strafe_left")    { s_l(speed);       isMotorCmd = true; }
  else if (cmd == "strafe_right")   { s_r(speed);       isMotorCmd = true; }
  else if (cmd == "take_photo_and_recognize") {
    cameraRequested = true;
    cameraCmdId = String(id);
    wsSendEvent("recognition_start");
    return;
  }
  else if (cmd == "display_text") {
    oledShowLine(params["row"] | 0, params["text"] | "");
    wsSendResponse(id, "ok", "{\"displayed\":true}");
    return;
  }
  else if (cmd == "set_autonomous_mode") {
    autonomousMode = params["enabled"] | true;
    if (!autonomousMode) stopMotors();
    wsSendResponse(id, "ok", autonomousMode ? "{\"autonomous_mode\":true}" : "{\"autonomous_mode\":false}");
    return;
  }
  else { wsSendResponse(id, "error", "{\"message\":\"unknown command\"}"); return; }

  if (isMotorCmd && duration > 0) { delay(duration); stopMotors(); }
  wsSendResponse(id, "ok", "{\"executed\":true}");
}

///////////////////////////////////////////////////////////////////////////
// WebSocket Setup & Connect
///////////////////////////////////////////////////////////////////////////

void wsSetup() {
  wsClient.onMessage(handleWsMessage);
  wsClient.onEvent([](WebsocketsEvent event, String data) {
    if (event == WebsocketsEvent::ConnectionOpened) {
      wsConnected = true;
      Serial.println("WS connected!");
      // 브라우저가 카메라 폴링 URL을 알 수 있도록 ESP32 IP 전송
      String ip = WiFi.localIP().toString();
      String msg = "{\"type\":\"event\",\"event\":\"device_info\",\"data\":{\"ip\":\"" + ip + "\"}}";
      wsClient.send(msg);
    } else if (event == WebsocketsEvent::ConnectionClosed) {
      wsConnected = false;
      Serial.println("WS disconnected");
    }
  });
}

void wsConnect() {
  if (WiFi.status() != WL_CONNECTED || wsConnected) return;
  Serial.print("WS connecting to "); Serial.println(WS_SERVER_URL);
  bool ok = wsClient.connect(WS_SERVER_URL);
  lastWsReconnect = millis();
  wsConnected = ok;
  Serial.println(ok ? "WS connected!" : "WS connect failed");
}

///////////////////////////////////////////////////////////////////////////
// 이벤트 핸들러
///////////////////////////////////////////////////////////////////////////

void onObjectDetected(uint16_t distance) {
  StaticJsonDocument<64> d; d["distance_mm"] = distance;
  String json; serializeJson(d, json);
  wsSendEventData("object_detected", json);
}

void onRecognitionStart()  { wsSendEvent("recognition_start"); }

void onRecognitionResult(const String& objectName) {
  StaticJsonDocument<256> d; d["result"] = objectName;
  String json; serializeJson(d, json);
  wsSendEventData("recognition_result", json);
}

///////////////////////////////////////////////////////////////////////////
// WiFi
///////////////////////////////////////////////////////////////////////////

void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.mode(WIFI_STA); WiFi.disconnect(true); delay(100);

  Serial.println("  Scanning networks...");
  int n = WiFi.scanNetworks();
  bool found = false;
  for (int i = 0; i < n; i++) {
    Serial.printf("    %s (RSSI:%d)\n", WiFi.SSID(i).c_str(), WiFi.RSSI(i));
    if (WiFi.SSID(i) == WIFI_SSID) found = true;
  }
  WiFi.scanDelete();
  if (!found) { Serial.printf("  '%s' not found!\n", WIFI_SSID); return; }

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("WiFi connecting to '%s'", WIFI_SSID);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) { delay(500); Serial.print("."); attempts++; }
  if (WiFi.status() == WL_CONNECTED) { Serial.println(" OK"); Serial.println(WiFi.localIP()); }
  else Serial.printf(" FAILED (status:%d)\n", WiFi.status());
}

///////////////////////////////////////////////////////////////////////////
// Camera
///////////////////////////////////////////////////////////////////////////

bool cameraInit() {
  camera_config_t config;
  memset(&config, 0, sizeof(config));
  config.ledc_channel  = LEDC_CHANNEL_5;
  config.ledc_timer    = LEDC_TIMER_2;
  config.pin_d0        = Y2_GPIO_NUM;  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2        = Y4_GPIO_NUM;  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4        = Y6_GPIO_NUM;  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6        = Y8_GPIO_NUM;  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk      = XCLK_GPIO_NUM;
  config.pin_pclk      = PCLK_GPIO_NUM;
  config.pin_vsync     = VSYNC_GPIO_NUM;
  config.pin_href      = HREF_GPIO_NUM;
  config.pin_sscb_sda  = SIOD_GPIO_NUM;
  config.pin_sscb_scl  = SIOC_GPIO_NUM;
  config.pin_pwdn      = PWDN_GPIO_NUM;
  config.pin_reset     = RESET_GPIO_NUM;
  config.xclk_freq_hz  = 20000000;
  config.pixel_format  = PIXFORMAT_JPEG;
  config.frame_size    = FRAMESIZE_QVGA;
  config.jpeg_quality  = 12;
  config.grab_mode     = CAMERA_GRAB_LATEST;
  config.sccb_i2c_port = 1;

  if (psramFound()) { config.fb_count = 2; config.fb_location = CAMERA_FB_IN_PSRAM; }
  else              { config.fb_count = 1; config.fb_location = CAMERA_FB_IN_DRAM;  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) { Serial.printf("Camera init error: 0x%x\n", err); return false; }
  return true;
}

///////////////////////////////////////////////////////////////////////////
// MJPEG 스트림 태스크 (Core 0에서 실행)
// multipart/x-mixed-replace 포맷으로 프레임을 연속 전송.
// cameraInUse 플래그로 AI 캡처 중 일시 정지.
///////////////////////////////////////////////////////////////////////////

void camStreamTask(void*) {
  for (;;) {
    if (!streamRunning) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }
    if (!streamCl.connected()) { streamRunning = false; vTaskDelay(pdMS_TO_TICKS(50)); continue; }
    if (cameraInUse)  { vTaskDelay(pdMS_TO_TICKS(50)); continue; }

    if (xSemaphoreTake(camMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
      vTaskDelay(pdMS_TO_TICKS(30)); continue;
    }
    camera_fb_t* fb = esp_camera_fb_get();
    xSemaphoreGive(camMutex);
    if (!fb) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }

    char hdr[96];
    snprintf(hdr, sizeof(hdr),
      "--f\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
      (unsigned)fb->len);
    streamCl.print(hdr);
    size_t written = streamCl.write(fb->buf, fb->len);
    esp_camera_fb_return(fb);

    if (!written) { streamRunning = false; continue; }
    streamCl.print("\r\n");
    vTaskDelay(pdMS_TO_TICKS(40)); // ~25fps 상한
  }
}

void handleStream() {
  if (!cameraReady) { webServer.send(503, "text/plain", "no camera"); return; }
  streamRunning = false;
  vTaskDelay(pdMS_TO_TICKS(80)); // 태스크가 현재 쓰기 완료할 때까지 대기
  streamCl = webServer.client();
  streamCl.print(
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: multipart/x-mixed-replace;boundary=f\r\n"
    "Cache-Control: no-cache\r\n"
    "Access-Control-Allow-Origin: *\r\n"
    "Connection: close\r\n\r\n");
  streamRunning = true;
}

///////////////////////////////////////////////////////////////////////////
// 웹 서버 핸들러 (포트 80)
//
// /stream   MJPEG 연속 스트림 (CORS 허용, 브라우저 img 태그로 직접 수신)
// /capture  JPEG 스냅샷 1장 (폴백용)
// /status   {"capturing":true/false}
// /         HTML 뷰어 페이지
///////////////////////////////////////////////////////////////////////////

void handleCapture() {
  if (!cameraReady) { webServer.send(503, "text/plain", "no camera"); return; }
  if (cameraInUse)  { webServer.send(503, "text/plain", "busy");      return; }

  xSemaphoreTake(camMutex, portMAX_DELAY);
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { xSemaphoreGive(camMutex); webServer.send(500, "text/plain", "capture failed"); return; }

  // WebServer 는 이진 데이터를 직접 지원하지 않으므로 raw 소켓으로 전송
  WiFiClient client = webServer.client();
  char hdr[200];
  snprintf(hdr, sizeof(hdr),
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: image/jpeg\r\n"
    "Content-Length: %u\r\n"
    "Cache-Control: no-cache, no-store\r\n"
    "Access-Control-Allow-Origin: *\r\n\r\n",
    (unsigned)fb->len);
  client.print(hdr);

  const uint8_t* buf = fb->buf;
  size_t remaining   = fb->len;
  while (remaining > 0) {
    size_t chunk = (remaining > 1024) ? 1024 : remaining;
    size_t sent  = client.write(buf, chunk);
    if (sent == 0) break;
    buf += sent; remaining -= sent;
  }
  esp_camera_fb_return(fb);
  xSemaphoreGive(camMutex);
}

void handleStatus() {
  String json = recognizing ? "{\"capturing\":true}" : "{\"capturing\":false}";
  webServer.sendHeader("Cache-Control", "no-cache");
  webServer.send(200, "application/json", json);
}

void handleRoot() {
  String html =
    "<!DOCTYPE html><html><head>"
    "<meta charset='UTF-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>XIAO Cam</title>"
    "<style>"
    "body{margin:0;background:#111;display:flex;flex-direction:column;"
    "align-items:center;justify-content:center;min-height:100vh;"
    "font-family:sans-serif;color:#fff;gap:12px}"
    "h1{font-size:1rem;margin:0;color:#aaa}"
    ".wrap{position:relative;display:inline-block}"
    "#cam{display:block;width:320px;height:240px;border-radius:10px;"
    "border:2px solid #333;background:#1a1a1a;object-fit:cover}"
    ".ov{display:none;position:absolute;inset:0;border-radius:10px;"
    "background:rgba(0,0,0,.75);flex-direction:column;"
    "align-items:center;justify-content:center;gap:10px;pointer-events:none}"
    ".ov.on{display:flex}"
    ".sp{width:40px;height:40px;border:4px solid rgba(255,255,255,.2);"
    "border-top-color:#fff;border-radius:50%;animation:sp .7s linear infinite}"
    "@keyframes sp{to{transform:rotate(360deg)}}"
    ".badge{font-size:.72rem;background:#e74c3c;padding:3px 10px;"
    "border-radius:20px}"
    ".msg{font-size:.85rem;color:#ddd;text-align:center;line-height:1.6}"
    ".st{font-size:.73rem;color:#555}"
    ".dot{display:inline-block;width:7px;height:7px;border-radius:50%;"
    "background:#2ecc71;margin-right:4px;vertical-align:middle}"
    ".dot.busy{background:#e74c3c}"
    "</style></head><body>"
    "<h1>XIAO ESP32S3 카메라</h1>"
    "<div class='wrap'>"
    "<img id='cam' alt=''>"
    "<div class='ov' id='ov'>"
    "<div class='sp'></div>"
    "<div class='badge'>AI 인식 중</div>"
    "<div class='msg'>사진 촬영 후 OpenAI 요청 중<br>잠시 멈춥니다...</div>"
    "</div>"
    "</div>"
    "<div class='st'><span class='dot' id='dot'></span>"
    "<span id='st'>연결 중...</span>"
    "<span id='fps' style='color:#444;margin-left:6px'></span>"
    "</div>"
    "<script>"
    "const cam=document.getElementById('cam'),"
    "ov=document.getElementById('ov'),"
    "dot=document.getElementById('dot'),"
    "stEl=document.getElementById('st'),"
    "fpsEl=document.getElementById('fps');"
    "let busy=false,capturing=false,fc=0,ft=Date.now();"

    // 프레임 루프: /capture?t=<ts> 를 반복 요청
    "function nextFrame(){"
    "if(busy||capturing){setTimeout(nextFrame,capturing?300:30);return;}"
    "busy=true;"
    "const img=new Image();"
    "img.onload=function(){"
    "cam.src=img.src;busy=false;fc++;"
    "const now=Date.now();"
    "if(now-ft>=1000){fpsEl.textContent=fc+'fps';fc=0;ft=now;}"
    "setTimeout(nextFrame,80);};"   // ~12fps
    "img.onerror=function(){busy=false;setTimeout(nextFrame,500);};"
    "img.src='/capture?t='+Date.now();"
    "}"
    "nextFrame();"

    // 상태 폴링: /status 를 0.5초마다 확인
    "async function chkStatus(){"
    "try{"
    "const d=await(await fetch('/status',{cache:'no-store'})).json();"
    "capturing=d.capturing;"
    "if(capturing){"
    "ov.classList.add('on');dot.classList.add('busy');"
    "stEl.textContent='AI 인식 중 (카메라 일시 중단)';"
    "}else{"
    "ov.classList.remove('on');dot.classList.remove('busy');"
    "stEl.textContent='스트리밍 중';}"
    "}catch(e){stEl.textContent='연결 확인 중...';}"
    "}"
    "setInterval(chkStatus,500);chkStatus();"
    "</script></body></html>";

  webServer.send(200, "text/html; charset=utf-8", html);
}

///////////////////////////////////////////////////////////////////////////
// 사진 촬영 + OpenAI Vision API 호출
///////////////////////////////////////////////////////////////////////////

String takePhotoAndRecognize() {
  if (!cameraReady) return "Camera Error";

  cameraInUse = true;
  Serial.println(">>> Taking photo...");
  xSemaphoreTake(camMutex, portMAX_DELAY);
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { xSemaphoreGive(camMutex); cameraInUse = false; return "Capture Error"; }
  Serial.printf(">>> Photo: %d bytes\n", fb->len);

  String b64 = base64::encode(fb->buf, fb->len);
  esp_camera_fb_return(fb);
  xSemaphoreGive(camMutex);
  cameraInUse = false;

  if (WiFi.status() != WL_CONNECTED) { b64 = ""; return "WiFi Error"; }

  Serial.printf(">>> Heap: %d bytes\n", ESP.getFreeHeap());
  Serial.println(">>> Calling OpenAI API...");

  String payload;
  payload.reserve(b64.length() + 400);
  payload  = "{\"model\":\"gpt-4o-mini\",\"messages\":[{\"role\":\"user\",\"content\":["
             "{\"type\":\"text\",\"text\":\"What object is in this image? "
             "Reply with ONLY the object name, maximum 3 words. No explanation.\"},"
             "{\"type\":\"image_url\",\"image_url\":{\"url\":\"data:image/jpeg;base64,";
  payload += b64; b64 = "";
  payload += "\"}}]}],\"max_tokens\":30}";

  WiFiClientSecure client; client.setInsecure();
  String response; int httpCode = -1;

  for (int retry = 0; retry < 3; retry++) {
    if (retry > 0) { Serial.printf(">>> Retry %d/3...\n", retry + 1); delay(1000); }
    HTTPClient http;
    http.begin(client, "https://api.openai.com/v1/chat/completions");
    http.addHeader("Authorization", "Bearer " + String(OPENAI_API_KEY));
    http.addHeader("Content-Type", "application/json");
    http.setTimeout(30000);
    httpCode = http.POST(payload);
    if (httpCode == 200) { response = http.getString(); http.end(); break; }
    Serial.printf("HTTP error: %d\n", httpCode); http.end();
  }

  payload = "";
  if (httpCode != 200) return "API Error";
  Serial.println("Response: " + response);

  int idx = response.indexOf("\"content\""); if (idx < 0) return "Parse Error";
  idx = response.indexOf(":", idx);          if (idx < 0) return "Parse Error";
  idx = response.indexOf("\"", idx + 1);    if (idx < 0) return "Parse Error";
  idx++;
  int endIdx = response.indexOf("\"", idx); if (endIdx < 0) return "Parse Error";

  String result = response.substring(idx, endIdx);
  Serial.println(">>> Recognized: " + result);
  return result;
}

///////////////////////////////////////////////////////////////////////////
// Setup
///////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
  delay(500);

  Serial.println("\n========================================");
  Serial.println("  XIAO ESP32S3 - WebSocket AI Control");
  Serial.println("========================================");

  // ---- 0. Motors ----
  int motorPins[] = {MOTOR_A_P,MOTOR_A_D,MOTOR_B_P,MOTOR_B_D,
                     MOTOR_C_P,MOTOR_C_D,MOTOR_D_P,MOTOR_D_D};
  for (int p : motorPins) pinMode(p, OUTPUT);
  stopMotors();
  Serial.println("[OK] Motors");

  // ---- 1. Wire + VL53L0X + OLED ----
  Wire.begin(); delay(100); Wire.setClock(100000);
  for (int i = 1; i <= 3; i++) {
    distSensor.setTimeout(500);
    if (distSensor.init()) {
      distSensor.setSignalRateLimit(0.1);
      distSensor.setMeasurementTimingBudget(33000);
      distSensor.startContinuous(50); delay(100);
      Serial.printf("[OK] VL53L0X (%dmm) attempt %d\n",
                    distSensor.readRangeContinuousMillimeters(), i);
      break;
    }
    Serial.printf("[FAIL] VL53L0X %d/3\n", i);
    Wire.end(); delay(500); Wire.begin(); delay(100); Wire.setClock(100000);
  }
  u8x8.begin(); u8x8.setPowerSave(0);
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.clear(); u8x8.drawString(0, 0, "Booting...");
  Serial.println("[OK] OLED");

  // ---- 2. WiFi ----
  u8x8.drawString(0, 2, "WiFi...");
  connectWiFi();
  wifiReady = (WiFi.status() == WL_CONNECTED);
  u8x8.drawString(0, 2, wifiReady ? "WiFi OK" : "WiFi FAIL");

  // ---- 3. Camera ----
  u8x8.drawString(0, 4, "Camera...");
  cameraReady = cameraInit();
  if (cameraReady) {
    Serial.printf("[OK] Camera (PSRAM:%s)\n", psramFound() ? "yes" : "no");
    for (int i = 0; i < 5; i++) {  // AE/AWB 워밍업
      camera_fb_t* fb = esp_camera_fb_get();
      if (fb) esp_camera_fb_return(fb);
      delay(100);
    }
  } else Serial.println("[FAIL] Camera");

  // ---- 4. Web Server (포트 80) ----
  if (wifiReady) {
    camMutex = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(camStreamTask, "stream", 4096, NULL, 1, NULL, 0); // Core 0
    webServer.on("/",        handleRoot);
    webServer.on("/stream",  handleStream);
    webServer.on("/capture", handleCapture);
    webServer.on("/status",  handleStatus);
    webServer.begin();
    Serial.printf("[OK] Web: http://%s/\n", WiFi.localIP().toString().c_str());
    u8x8.drawString(0, 4, cameraReady ? "Cam+Web OK" : "Web OK");
  }

  // ---- 5. WebSocket ----
  u8x8.drawString(0, 6, "WS connecting...");
  wsSetup();
  if (wifiReady) wsConnect();
  u8x8.drawString(0, 6, wsConnected ? "WS OK" : "WS FAIL");

  Serial.println("========================================");
  Serial.printf("  Free heap: %d bytes\n", ESP.getFreeHeap());
  if (wifiReady) Serial.printf("  Web: http://%s/\n", WiFi.localIP().toString().c_str());
  Serial.println("========================================");

  delay(1500);
  u8x8.clear();
  u8x8.drawString(0, 0, "Ready!");
  if (wifiReady) {
    char buf[17];
    snprintf(buf, sizeof(buf), "%-16s", WiFi.localIP().toString().c_str());
    u8x8.drawString(0, 2, buf);
  }
  Serial.println(">>> Loop start\n");
}

///////////////////////////////////////////////////////////////////////////
// Main Loop
///////////////////////////////////////////////////////////////////////////

void loop() {
  // 1. WS 통신
  if (wsConnected) wsClient.poll();

  // 2. WS 자동 재접속
  if (WiFi.status() == WL_CONNECTED && !wsConnected)
    if (millis() - lastWsReconnect > WS_RECONNECT_INTERVAL) wsConnect();

  // 3. 웹 서버 요청 처리 (/capture, /status, /)
  webServer.handleClient();

  // 4. 카메라 명령 처리 (WS 명령)
  if (cameraRequested) {
    cameraRequested = false;
    recognizing = true;
    oledShowLine(0, "Recognizing..."); oledShowLine(2, "");
    distSensor.stopContinuous();

    String result = takePhotoAndRecognize();

    distSensor.startContinuous(50); delay(200);
    lastRecognitionTime = millis();
    recognizing = false;

    char buf[17]; strncpy(buf, result.c_str(), 16); buf[16] = '\0';
    oledShowLine(0, ""); oledShowLine(2, buf);
    cameraResult = result; cameraPending = true;
    return;
  }

  // 5. 카메라 결과 전송
  if (cameraPending && wsConnected) {
    StaticJsonDocument<256> d; d["result"] = cameraResult;
    String json; serializeJson(d, json);
    wsSendResponse(cameraCmdId.c_str(), "ok", json);
    wsSendEventData("recognition_result", json);
    cameraPending = false; cameraCmdId = ""; cameraResult = "";
  }

  // 6. 거리 센서
  uint16_t distance = distSensor.readRangeContinuousMillimeters();
  if (distSensor.timeoutOccurred()) { Serial.println("VL53L0X timeout!"); delay(100); return; }

  // 7. 거리 스트리밍
  if (millis() - lastDistStream > DIST_STREAM_INTERVAL) {
    lastDistStream = millis();
    Serial.printf("Dist: %dmm\n", distance);
    if (wsConnected) {
      StaticJsonDocument<64> d; d["distance_mm"] = distance;
      String json; serializeJson(d, json);
      wsSendEventData("distance", json);
    }
  }

  // 8. 범위 밖
  if (distance > 8000 || distance == 0) { detectConfirmCount = 0; delay(50); return; }

  // 9. 자율 모드
  if (autonomousMode && !cameraPending) {
    if (distance < STOP_DISTANCE) {
      if (!objectDetected) {
        if (millis() - lastRecognitionTime < RECOGNITION_COOLDOWN) { delay(50); return; }
        if (++detectConfirmCount < DETECT_CONFIRM_COUNT) { delay(50); return; }
        detectConfirmCount = 0;

        objectDetected = true;
        stopMotors();
        Serial.printf("\n>>> DETECTED at %dmm!\n", distance);
        onObjectDetected(distance);
        oledShowLine(0, "Recognizing..."); oledShowLine(2, "");
        distSensor.stopContinuous();
        onRecognitionStart();

        recognizing = true;
        String result = takePhotoAndRecognize();
        recognizing = false;

        distSensor.startContinuous(50); delay(200);
        lastRecognitionTime = millis();
        if (!wsConnected && WiFi.status() == WL_CONNECTED) wsConnect();

        onRecognitionResult(result);
        oledShowLine(0, "Stopped");
        char buf[17]; strncpy(buf, result.c_str(), 16); buf[16] = '\0';
        oledShowLine(2, buf);
      }
    } else {
      detectConfirmCount = 0;
      if (objectDetected) {
        objectDetected = false;
        Serial.printf(">>> Cleared at %dmm\n", distance);
        oledShowLine(0, "Ready"); oledShowLine(2, "");
      }
    }
  }

  delay(50);
}
