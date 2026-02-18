///////////////////////////////////////////////////////////////////////////
// XIAO ESP32S3 Sense - Mechanum Wheel Robot + WebSocket AI Control
//
// Features:
//   1. VL53L0X 거리 기반 자동 정지 (STOP_DISTANCE 조절 가능)
//   2. 카메라 → OpenAI Vision API → OLED 물체명 표시
//   3. WebSocket 클라이언트 → Node.js 서버 실시간 통신
//   4. AI 자연어 명령 수신 → 모터/카메라/OLED 제어
//
// I2C 구성:
//   - VL53L0X + OLED: Wire (I2C_NUM_0)
//   - 카메라 SCCB: I2C_NUM_1 (sccb_i2c_port=1로 분리)
//   - 카메라: 필요 시에만 init → 촬영 → deinit (메모리 절약)
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

// State
bool objectDetected = false;
bool wifiReady = false;
bool wsConnected = false;
bool autonomousMode = true;
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
  digitalWrite(MOTOR_A_D, HIGH);
  digitalWrite(MOTOR_B_D, HIGH);
  digitalWrite(MOTOR_C_D, LOW);
  digitalWrite(MOTOR_D_D, LOW);
  analogWrite(MOTOR_A_P, speed);
  analogWrite(MOTOR_B_P, speed);
  analogWrite(MOTOR_C_P, speed);
  analogWrite(MOTOR_D_P, speed);
}

void backward(unsigned int speed) {
  digitalWrite(MOTOR_A_D, LOW);
  digitalWrite(MOTOR_B_D, LOW);
  digitalWrite(MOTOR_C_D, HIGH);
  digitalWrite(MOTOR_D_D, HIGH);
  analogWrite(MOTOR_A_P, speed);
  analogWrite(MOTOR_B_P, speed);
  analogWrite(MOTOR_C_P, speed);
  analogWrite(MOTOR_D_P, speed);
}

void stopMotors() {
  analogWrite(MOTOR_A_P, 0);
  analogWrite(MOTOR_B_P, 0);
  analogWrite(MOTOR_C_P, 0);
  analogWrite(MOTOR_D_P, 0);
}

void t_l(unsigned int speed) {
  digitalWrite(MOTOR_A_D, LOW);
  digitalWrite(MOTOR_B_D, HIGH);
  digitalWrite(MOTOR_C_D, LOW);
  digitalWrite(MOTOR_D_D, HIGH);
  analogWrite(MOTOR_A_P, speed);
  analogWrite(MOTOR_B_P, speed);
  analogWrite(MOTOR_C_P, speed);
  analogWrite(MOTOR_D_P, speed);
}

void t_r(unsigned int speed) {
  digitalWrite(MOTOR_A_D, HIGH);
  digitalWrite(MOTOR_B_D, LOW);
  digitalWrite(MOTOR_C_D, HIGH);
  digitalWrite(MOTOR_D_D, LOW);
  analogWrite(MOTOR_A_P, speed);
  analogWrite(MOTOR_B_P, speed);
  analogWrite(MOTOR_C_P, speed);
  analogWrite(MOTOR_D_P, speed);
}

void s_r(unsigned int speed) {
  digitalWrite(MOTOR_A_D, HIGH);
  digitalWrite(MOTOR_B_D, LOW);
  digitalWrite(MOTOR_C_D, LOW);
  digitalWrite(MOTOR_D_D, HIGH);
  analogWrite(MOTOR_A_P, speed);
  analogWrite(MOTOR_B_P, speed);
  analogWrite(MOTOR_C_P, speed);
  analogWrite(MOTOR_D_P, speed);
}

void s_l(unsigned int speed) {
  digitalWrite(MOTOR_A_D, LOW);
  digitalWrite(MOTOR_B_D, HIGH);
  digitalWrite(MOTOR_C_D, HIGH);
  digitalWrite(MOTOR_D_D, LOW);
  analogWrite(MOTOR_A_P, speed);
  analogWrite(MOTOR_B_P, speed);
  analogWrite(MOTOR_C_P, speed);
  analogWrite(MOTOR_D_P, speed);
}

///////////////////////////////////////////////////////////////////////////
// WebSocket Helpers
///////////////////////////////////////////////////////////////////////////

void wsSendRaw(const String& json) {
  if (wsConnected) {
    wsClient.send(json);
  }
}

void wsSendEvent(const char* eventName) {
  if (!wsConnected) return;
  String msg = "{\"type\":\"event\",\"event\":\"";
  msg += eventName;
  msg += "\"}";
  wsClient.send(msg);
}

void wsSendEventData(const char* eventName, const String& dataJson) {
  if (!wsConnected) return;
  String msg = "{\"type\":\"event\",\"event\":\"";
  msg += eventName;
  msg += "\",\"data\":";
  msg += dataJson;
  msg += "}";
  wsClient.send(msg);
}

void wsSendResponse(const char* cmdId, const char* status, const String& dataJson) {
  if (!wsConnected) return;
  String msg = "{\"type\":\"response\",\"id\":\"";
  msg += cmdId;
  msg += "\",\"status\":\"";
  msg += status;
  msg += "\",\"data\":";
  msg += dataJson;
  msg += "}";
  wsClient.send(msg);
}

///////////////////////////////////////////////////////////////////////////
// WebSocket Command Handler
///////////////////////////////////////////////////////////////////////////

void handleWsMessage(WebsocketsMessage message) {
  String payload = message.data();
  Serial.println("WS << " + payload);

  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, payload);
  if (err) {
    Serial.println("JSON parse error");
    return;
  }

  const char* id = doc["id"] | "0";
  const char* command = doc["command"];
  if (!command) return;

  String cmd(command);
  JsonObject params = doc["params"];
  int speed = params["speed"] | MOTOR_SPEED;
  int duration = params["duration_ms"] | 0;
  if (duration > MAX_DURATION_MS) duration = MAX_DURATION_MS;

  bool isMotorCmd = false;

  if (cmd == "move_forward")       { forward(speed); isMotorCmd = true; }
  else if (cmd == "move_backward") { backward(speed); isMotorCmd = true; }
  else if (cmd == "stop_motors")   { stopMotors(); }
  else if (cmd == "turn_left")     { t_l(speed); isMotorCmd = true; }
  else if (cmd == "turn_right")    { t_r(speed); isMotorCmd = true; }
  else if (cmd == "strafe_left")   { s_l(speed); isMotorCmd = true; }
  else if (cmd == "strafe_right")  { s_r(speed); isMotorCmd = true; }
  else if (cmd == "take_photo_and_recognize") {
    cameraRequested = true;
    cameraCmdId = String(id);
    wsSendEvent("recognition_start");
    return;  // loop()에서 처리 후 응답 전송
  }
  else if (cmd == "display_text") {
    int row = params["row"] | 0;
    const char* text = params["text"] | "";
    oledShowLine(row, text);
    wsSendResponse(id, "ok", "{\"displayed\":true}");
    return;
  }
  else if (cmd == "set_autonomous_mode") {
    autonomousMode = params["enabled"] | true;
    if (!autonomousMode) {
      stopMotors();  // 자율 모드 끄면 정지
    }
    String d = autonomousMode ? "{\"autonomous_mode\":true}" : "{\"autonomous_mode\":false}";
    wsSendResponse(id, "ok", d);
    Serial.printf("Autonomous mode: %s\n", autonomousMode ? "ON" : "OFF");
    return;
  }
  else {
    wsSendResponse(id, "error", "{\"message\":\"unknown command\"}");
    return;
  }

  // 모터 명령 duration 처리
  if (isMotorCmd && duration > 0) {
    delay(duration);
    stopMotors();
  }

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
    } else if (event == WebsocketsEvent::ConnectionClosed) {
      wsConnected = false;
      Serial.println("WS disconnected");
    }
  });
}

void wsConnect() {
  if (WiFi.status() != WL_CONNECTED || wsConnected) return;

  Serial.print("WS connecting to ");
  Serial.println(WS_SERVER_URL);

  bool ok = wsClient.connect(WS_SERVER_URL);
  lastWsReconnect = millis();

  if (ok) {
    wsConnected = true;
    Serial.println("WS connected!");
  } else {
    wsConnected = false;
    Serial.println("WS connect failed");
  }
}

///////////////////////////////////////////////////////////////////////////
// 이벤트 핸들러
///////////////////////////////////////////////////////////////////////////

void onObjectDetected(uint16_t distance) {
  Serial.printf(">>> onObjectDetected (%dmm)\n", distance);
  StaticJsonDocument<64> d;
  d["distance_mm"] = distance;
  String json;
  serializeJson(d, json);
  wsSendEventData("object_detected", json);
}

void onRecognitionStart() {
  Serial.println(">>> onRecognitionStart");
  wsSendEvent("recognition_start");
}

void onRecognitionResult(const String& objectName) {
  Serial.println(">>> onRecognitionResult: " + objectName);
  StaticJsonDocument<256> d;
  d["result"] = objectName;
  String json;
  serializeJson(d, json);
  wsSendEventData("recognition_result", json);
}

///////////////////////////////////////////////////////////////////////////
// WiFi
///////////////////////////////////////////////////////////////////////////

void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(100);

  Serial.println("  Scanning networks...");
  int n = WiFi.scanNetworks();
  bool found = false;
  for (int i = 0; i < n; i++) {
    Serial.printf("    %s (RSSI:%d)\n", WiFi.SSID(i).c_str(), WiFi.RSSI(i));
    if (WiFi.SSID(i) == WIFI_SSID) found = true;
  }
  WiFi.scanDelete();

  if (!found) {
    Serial.printf("  '%s' not found!\n", WIFI_SSID);
    return;
  }

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("WiFi connecting to '%s'", WIFI_SSID);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(" OK");
    Serial.println(WiFi.localIP());
  } else {
    Serial.printf(" FAILED (status:%d)\n", WiFi.status());
  }
}

///////////////////////////////////////////////////////////////////////////
// Camera
///////////////////////////////////////////////////////////////////////////

bool cameraInit() {
  camera_config_t config;
  memset(&config, 0, sizeof(config));
  config.ledc_channel = LEDC_CHANNEL_5;
  config.ledc_timer = LEDC_TIMER_2;
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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;
  config.fb_count = 1;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_DRAM;
  config.sccb_i2c_port = 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init error: 0x%x\n", err);
    return false;
  }
  return true;
}

void cameraDeinit() {
  esp_camera_deinit();
}

///////////////////////////////////////////////////////////////////////////
// 사진 촬영 + API 호출
///////////////////////////////////////////////////////////////////////////

String takePhotoAndRecognize() {
  Serial.println(">>> [1/5] Camera init...");
  if (!cameraInit()) {
    return "Camera Error";
  }

  Serial.println(">>> [2/5] Taking photo...");
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println(">>> Capture failed");
    cameraDeinit();
    return "Capture Error";
  }
  Serial.printf(">>> Photo: %d bytes\n", fb->len);

  Serial.println(">>> [3/5] Base64 encoding...");
  String b64 = base64::encode(fb->buf, fb->len);
  esp_camera_fb_return(fb);

  Serial.println(">>> [4/5] Camera deinit...");
  cameraDeinit();
  delay(200);

  Serial.printf(">>> Heap: %d bytes, WiFi status: %d\n", ESP.getFreeHeap(), WiFi.status());
  WiFi.disconnect(true);
  delay(100);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print(">>> WiFi reconnecting");
  int wifiAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifiAttempts < 30) {
    delay(500);
    Serial.print(".");
    wifiAttempts++;
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(" FAILED");
    return "WiFi Error";
  }
  Serial.println(" OK");

  Serial.println(">>> [5/5] Calling OpenAI API...");
  String payload;
  payload.reserve(b64.length() + 400);
  payload = "{\"model\":\"gpt-4o-mini\",\"messages\":[{\"role\":\"user\",\"content\":["
            "{\"type\":\"text\",\"text\":\"What object is in this image? "
            "Reply with ONLY the object name, maximum 3 words. No explanation.\"},"
            "{\"type\":\"image_url\",\"image_url\":{\"url\":\"data:image/jpeg;base64,";
  payload += b64;
  b64 = "";
  payload += "\"}}]}],\"max_tokens\":30}";

  WiFiClientSecure client;
  client.setInsecure();

  String response;
  int httpCode = -1;

  for (int retry = 0; retry < 3; retry++) {
    if (retry > 0) {
      Serial.printf(">>> Retry %d/3...\n", retry + 1);
      delay(1000);
    }

    HTTPClient http;
    http.begin(client, "https://api.openai.com/v1/chat/completions");
    http.addHeader("Authorization", "Bearer " + String(OPENAI_API_KEY));
    http.addHeader("Content-Type", "application/json");
    http.setTimeout(30000);

    httpCode = http.POST(payload);

    if (httpCode == 200) {
      response = http.getString();
      http.end();
      break;
    }

    Serial.printf("HTTP error: %d\n", httpCode);
    http.end();
  }

  payload = "";

  if (httpCode != 200) {
    return "API Error";
  }
  Serial.println("Response: " + response);

  int idx = response.indexOf("\"content\"");
  if (idx < 0) return "Parse Error";
  idx = response.indexOf(":", idx);
  if (idx < 0) return "Parse Error";
  idx = response.indexOf("\"", idx + 1);
  if (idx < 0) return "Parse Error";
  idx++;
  int endIdx = response.indexOf("\"", idx);
  if (endIdx < 0) return "Parse Error";

  String result = response.substring(idx, endIdx);
  Serial.println(">>> Recognized: " + result);
  return result;
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
// Setup
///////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(115200);
  unsigned long serialStart = millis();
  while (!Serial && millis() - serialStart < 3000) {
    delay(10);
  }
  delay(500);

  Serial.println();
  Serial.println("========================================");
  Serial.println("  XIAO ESP32S3 - WebSocket AI Control");
  Serial.println("========================================");

  // ---- 0. Motor Pins ----
  pinMode(MOTOR_A_P, OUTPUT);
  pinMode(MOTOR_A_D, OUTPUT);
  pinMode(MOTOR_B_P, OUTPUT);
  pinMode(MOTOR_B_D, OUTPUT);
  pinMode(MOTOR_C_P, OUTPUT);
  pinMode(MOTOR_C_D, OUTPUT);
  pinMode(MOTOR_D_P, OUTPUT);
  pinMode(MOTOR_D_D, OUTPUT);
  stopMotors();
  Serial.println("[OK] Motor pins");

  // ---- 1. Wire + VL53L0X + OLED ----
  Serial.println("[1] Wire + VL53L0X + OLED init...");
  Wire.begin();
  delay(100);
  Wire.setClock(100000);

  bool sensorOK = false;
  for (int attempt = 1; attempt <= 3; attempt++) {
    distSensor.setTimeout(500);
    if (distSensor.init()) {
      distSensor.setSignalRateLimit(0.1);
      distSensor.setMeasurementTimingBudget(33000);
      distSensor.startContinuous(50);
      delay(100);
      uint16_t test = distSensor.readRangeContinuousMillimeters();
      Serial.printf("[OK] VL53L0X (test: %dmm) attempt %d\n", test, attempt);
      sensorOK = true;
      break;
    }
    Serial.printf("[FAIL] VL53L0X attempt %d/3\n", attempt);
    Wire.end();
    delay(500);
    Wire.begin();
    delay(100);
    Wire.setClock(100000);
  }

  u8x8.begin();
  u8x8.setPowerSave(0);
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.clear();
  u8x8.drawString(0, 0, "Booting...");
  Serial.println("[OK] OLED");

  // ---- 2. WiFi ----
  Serial.println("[2] WiFi...");
  u8x8.drawString(0, 2, "WiFi...");
  connectWiFi();
  wifiReady = (WiFi.status() == WL_CONNECTED);
  u8x8.drawString(0, 2, wifiReady ? "WiFi OK" : "WiFi FAIL");

  // ---- 3. WebSocket ----
  Serial.println("[3] WebSocket...");
  u8x8.drawString(0, 4, "WS connecting...");
  wsSetup();
  if (wifiReady) {
    wsConnect();
  }
  u8x8.drawString(0, 4, wsConnected ? "WS OK" : "WS FAIL");

  Serial.println("[!] Camera: on-demand (not init at boot)");
  u8x8.drawString(0, 6, "Cam: on-demand");

  Serial.println("========================================");
  Serial.printf("  Free heap: %d bytes\n", ESP.getFreeHeap());
  Serial.printf("  Autonomous: %s\n", autonomousMode ? "ON" : "OFF");
  Serial.println("========================================");

  delay(1500);
  u8x8.clear();
  u8x8.drawString(0, 0, "Ready!");
  Serial.println(">>> Entering main loop...\n");
}

///////////////////////////////////////////////////////////////////////////
// Main Loop
///////////////////////////////////////////////////////////////////////////

void loop() {
  // 1. WS 통신
  if (wsConnected) {
    wsClient.poll();
  }

  // 2. WS 자동 재접속
  if (WiFi.status() == WL_CONNECTED && !wsConnected) {
    if (millis() - lastWsReconnect > WS_RECONNECT_INTERVAL) {
      wsConnect();
    }
  }

  // 3. 카메라 명령 처리 (WS 명령으로 요청됨)
  if (cameraRequested) {
    cameraRequested = false;

    oledShowLine(0, "Recognizing...");
    oledShowLine(2, "");
    distSensor.stopContinuous();

    String result = takePhotoAndRecognize();

    distSensor.startContinuous(50);
    delay(200);
    lastRecognitionTime = millis();

    // OLED 표시
    char nameBuf[17];
    strncpy(nameBuf, result.c_str(), 16);
    nameBuf[16] = '\0';
    oledShowLine(0, "");
    oledShowLine(2, nameBuf);

    // WiFi 복구 후 WS 재접속 → 응답 전송 (다음 루프에서)
    cameraResult = result;
    cameraPending = true;
    return;
  }

  // 4. 카메라 결과 전송 (WS 재접속 후)
  if (cameraPending && wsConnected) {
    StaticJsonDocument<256> d;
    d["result"] = cameraResult;
    String json;
    serializeJson(d, json);
    wsSendResponse(cameraCmdId.c_str(), "ok", json);
    wsSendEventData("recognition_result", json);

    cameraPending = false;
    cameraCmdId = "";
    cameraResult = "";
  }

  // 5. 거리 센서 읽기
  uint16_t distance = distSensor.readRangeContinuousMillimeters();

  if (distSensor.timeoutOccurred()) {
    Serial.println("VL53L0X timeout!");
    delay(100);
    return;
  }

  // 6. 거리 출력 + WS 스트리밍
  if (millis() - lastDistStream > DIST_STREAM_INTERVAL) {
    lastDistStream = millis();
    Serial.printf("Dist: %dmm\n", distance);

    if (wsConnected) {
      StaticJsonDocument<64> d;
      d["distance_mm"] = distance;
      String json;
      serializeJson(d, json);
      wsSendEventData("distance", json);
    }
  }

  // 7. 범위 밖
  if (distance > 8000 || distance == 0) {
    detectConfirmCount = 0;
    delay(50);
    return;
  }

  // 8. 자율 모드: 전진 → 물체 감지 시 정지 & 인식 → 물체 사라지면 재전진
  if (autonomousMode && !cameraPending) {
    if (distance < STOP_DISTANCE) {
      // === 물체 감지 ===
      if (!objectDetected) {
        if (millis() - lastRecognitionTime < RECOGNITION_COOLDOWN) {
          delay(50);
          return;
        }

        detectConfirmCount++;
        if (detectConfirmCount < DETECT_CONFIRM_COUNT) {
          delay(50);
          return;
        }
        detectConfirmCount = 0;

        // 정지 + 인식 시작
        objectDetected = true;
        stopMotors();
        Serial.printf("\n>>> DETECTED at %dmm! Motors stopped.\n", distance);
        onObjectDetected(distance);

        oledShowLine(0, "Recognizing...");
        oledShowLine(2, "");
        distSensor.stopContinuous();
        onRecognitionStart();

        String result = takePhotoAndRecognize();

        distSensor.startContinuous(50);
        delay(200);
        lastRecognitionTime = millis();

        // WiFi 복구 후 WS 재접속
        if (!wsConnected && WiFi.status() == WL_CONNECTED) {
          wsConnect();
        }

        Serial.println(">>> Final result: " + result);
        onRecognitionResult(result);

        oledShowLine(0, "Stopped");
        char nameBuf[17];
        strncpy(nameBuf, result.c_str(), 16);
        nameBuf[16] = '\0';
        oledShowLine(2, nameBuf);
      }
      // objectDetected == true 상태에서는 정지 유지
    } else {
      // === 물체 없음 ===
      detectConfirmCount = 0;
      if (objectDetected) {
        objectDetected = false;
        Serial.printf(">>> Cleared at %dmm\n", distance);
        oledShowLine(0, "Ready");
        oledShowLine(2, "");
      }
    }
  }

  delay(50);
}
