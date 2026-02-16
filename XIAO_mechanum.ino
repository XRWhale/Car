///////////////////////////////////////////////////////////////////////////
// XIAO ESP32S3 Sense - Mechanum Wheel Robot
//
// Features:
//   1. VL53L0X 거리 기반 자동 정지 (STOP_DISTANCE 조절 가능)
//   2. 카메라 → OpenAI Vision API → OLED 물체명 표시
//
// I2C 충돌 해결:
//   - VL53L0X: Wire (HW I2C) 사용
//   - OLED: Wire 공유 (같은 버스, 같은 주소 다름)
//   - 카메라: 물체 감지 시에만 init → 촬영 → deinit
//     (카메라 SCCB가 I2C를 점유하므로 상시 공존 불가)
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

// ===== 빌드 전 조절 가능한 설정값 =====
#define STOP_DISTANCE  300   // mm - 이 거리 이내 물체 감지 시 정지
#define MOTOR_SPEED    200   // 0-255 기본 전진 속도

// ===== WiFi & API 설정 =====
const char* WIFI_SSID      = "asdf";
const char* WIFI_PASSWORD   = "qwe789zxc123";
const char* OPENAI_API_KEY  = "";

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

bool objectDetected = false;
bool wifiReady = false;
unsigned long lastRecognitionTime = 0;       // 마지막 인식 완료 시각
int detectConfirmCount = 0;                  // 연속 감지 카운터
#define RECOGNITION_COOLDOWN  3000           // 인식 후 재감지 금지 시간 (ms)
#define DETECT_CONFIRM_COUNT  3              // 연속 N회 감지해야 트리거

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
// WiFi
///////////////////////////////////////////////////////////////////////////

void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(100);

  // 주변 네트워크 스캔
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
// Camera - 필요할 때만 init/deinit (I2C 충돌 방지)
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
  config.sccb_i2c_port = 1;               // I2C_NUM_1 사용 → Wire(I2C_NUM_0) 충돌 방지

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
// 사진 촬영 + API 호출 (카메라 init/deinit 포함)
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

  // 카메라 deinit 후 WiFi가 깨질 수 있으므로 강제 재연결
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

  // API 호출
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

  // "content" 파싱
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
// VL53L0X + Wire 재초기화
///////////////////////////////////////////////////////////////////////////

bool initSensor() {
  for (int attempt = 1; attempt <= 3; attempt++) {
    Wire.end();
    delay(100);
    Wire.begin();
    delay(100);
    Wire.setClock(100000);

    distSensor.setTimeout(500);
    if (distSensor.init()) {
      distSensor.setSignalRateLimit(0.1);
      distSensor.setMeasurementTimingBudget(33000);
      distSensor.startContinuous(50);

      // OLED도 같은 I2C 버스이므로 재초기화
      u8x8.begin();
      u8x8.setPowerSave(0);
      u8x8.setFont(u8x8_font_chroma48medium8_r);

      // 검증: 실제 읽기 테스트
      delay(200);
      uint16_t test = distSensor.readRangeContinuousMillimeters();
      Serial.printf("[OK] VL53L0X (attempt %d, test: %dmm)\n", attempt, test);
      return true;
    }
    Serial.printf("[FAIL] VL53L0X restore attempt %d/3\n", attempt);
    delay(500);
  }
  return false;
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
  Serial.println("  XIAO ESP32S3 - Test Mode");
  Serial.println("========================================");

  // ---- 1. Wire + VL53L0X + OLED (같은 HW I2C 버스) ----
  Serial.println("[1] Wire + VL53L0X + OLED init...");
  Wire.begin();
  delay(100);
  Wire.setClock(100000);  // 100kHz (안정적)

  // VL53L0X (최대 3회 재시도)
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

  // OLED (같은 Wire 버스 공유)
  u8x8.begin();
  u8x8.setPowerSave(0);
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.clear();
  u8x8.drawString(0, 0, "Booting...");
  Serial.println("[OK] OLED");

  // VL53L0X 다시 테스트 (OLED init 후에도 작동하는지)
  uint16_t test2 = distSensor.readRangeContinuousMillimeters();
  Serial.printf("[CHECK] VL53L0X after OLED: %dmm\n", test2);

  // ---- 2. WiFi ----
  Serial.println("[2] WiFi...");
  u8x8.drawString(0, 2, "WiFi...");
  connectWiFi();
  wifiReady = (WiFi.status() == WL_CONNECTED);
  u8x8.drawString(0, 2, wifiReady ? "WiFi OK" : "WiFi FAIL");

  // ---- 카메라는 여기서 초기화하지 않음! ----
  // 카메라 SCCB가 I2C를 점유하므로, 물체 감지 시에만 init/deinit
  Serial.println("[!] Camera: on-demand (not init at boot)");
  u8x8.drawString(0, 4, "Cam: on-demand");

  // ---- VL53L0X 최종 테스트 ----
  uint16_t test3 = distSensor.readRangeContinuousMillimeters();
  Serial.printf("[CHECK] VL53L0X final: %dmm\n", test3);

  Serial.println("========================================");
  Serial.printf("  Free heap: %d bytes\n", ESP.getFreeHeap());
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
  uint16_t distance = distSensor.readRangeContinuousMillimeters();

  if (distSensor.timeoutOccurred()) {
    Serial.println("VL53L0X timeout!");
    delay(100);
    return;
  }

  // 시리얼 출력 (0.5초마다)
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    lastPrint = millis();
    Serial.printf("Dist: %dmm\n", distance);
  }

  // 범위 밖
  if (distance > 8000 || distance == 0) {
    detectConfirmCount = 0;  // 연속 감지 끊김
    delay(50);
    return;
  }

  // OLED 거리 업데이트
  static unsigned long lastOled = 0;
  if (millis() - lastOled > 300) {
    lastOled = millis();
    char buf[17];
    snprintf(buf, sizeof(buf), "Dist: %4dmm", distance);
    oledShowLine(2, buf);
    if (!objectDetected) {
      oledShowLine(0, "Monitoring...");
    }
  }

  if (distance < STOP_DISTANCE) {
    // === 물체 감지! ===
    if (!objectDetected) {
      // 쿨다운 체크: 인식 직후엔 재트리거 방지
      if (millis() - lastRecognitionTime < RECOGNITION_COOLDOWN) {
        delay(50);
        return;
      }

      // 디바운싱: 연속 N회 확인
      detectConfirmCount++;
      if (detectConfirmCount < DETECT_CONFIRM_COUNT) {
        delay(50);
        return;
      }
      detectConfirmCount = 0;

      objectDetected = true;
      Serial.printf("\n>>> DETECTED at %dmm!\n", distance);

      oledShowLine(0, "== DETECTED ==");
      char buf[17];
      snprintf(buf, sizeof(buf), "Dist: %4dmm", distance);
      oledShowLine(2, buf);
      oledShowLine(4, "Recognizing...");

      // 카메라는 I2C_NUM_1(sccb_i2c_port=1)을 쓰므로
      // Wire(I2C_NUM_0)는 끄지 않고, 레이저만 정지
      distSensor.stopContinuous();

      // 사진 촬영 + API 호출 (카메라 init/deinit 포함)
      String result = takePhotoAndRecognize();

      // 레이저 재시작 (Wire/센서 재초기화 불필요)
      distSensor.startContinuous(50);
      delay(200);

      lastRecognitionTime = millis();  // 쿨다운 시작

      // 결과 표시
      Serial.println(">>> Final result: " + result);
      oledShowLine(4, "Object:");
      char nameBuf[17];
      strncpy(nameBuf, result.c_str(), 16);
      nameBuf[16] = '\0';
      oledShowLine(5, nameBuf);
    }
  } else {
    // === 물체 없음 ===
    detectConfirmCount = 0;  // 연속 감지 끊김 → 카운터 리셋
    if (objectDetected) {
      objectDetected = false;
      Serial.printf(">>> Cleared at %dmm\n", distance);
      oledShowLine(4, "");
      oledShowLine(5, "");
    }
  }

  delay(50);
}
