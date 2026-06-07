#include "USB.h"
#include "USBHIDGamepad.h"
#include <esp_now.h>
#include <WiFi.h>

// =====================================================
// ============ CẤU HÌNH ESP-NOW =======================
// =====================================================
// MAC xe (RX): CC:DB:A7:99:87:90
uint8_t carMacAddress[] = {0xCC, 0xDB, 0xA7, 0x99, 0x87, 0x90};

// Cấu trúc dữ liệu gửi sang xe (PHẢI GIỐNG HỆT bên RX)
typedef struct __attribute__((packed)) {
  int32_t  steerCount;     // encoder count vô lăng (có dấu)
  int32_t  steerMaxCount;  // = PPR * MAX_TURNS, để RX tự tính tỉ lệ
  uint8_t  throttle;       // 0-255 (ga)
  uint8_t  brake;          // 0-255 (phanh)
  uint8_t  clutch;         // 0-255 (côn)
  uint8_t  gear;           // 0=N, 1..6, 7=R(lùi), 8=handbrake
} ControlData;

ControlData txData;
esp_now_peer_info_t peerInfo;
unsigned long lastEspNowSend = 0;
volatile bool lastSendOK = true;

void onDataSent(const uint8_t *mac, esp_now_send_status_t status) {
  lastSendOK = (status == ESP_NOW_SEND_SUCCESS);
}

// =====================================================
// ============ CẤU HÌNH GỐC ===========================
// =====================================================

// ---- Encoder vô lăng ----
#define PIN_A       4
#define PIN_B       5
#define PPR         600
#define MAX_TURNS   1.5f

// ---- GPIO Pedal ----
#define PIN_GA      1
#define PIN_PHANH   2
#define PIN_CON     3

// ---- Calibrate Pedal ----
#define GA_MIN      0
#define GA_MAX      280
#define PHANH_MIN   0
#define PHANH_MAX   720
#define CON_MIN     0
#define CON_MAX     400

// ---- Hộp số (8 công tắc) ----
#define SW_1    6
#define SW_2    7
#define SW_3    8
#define SW_4    9
#define SW_5    10
#define SW_6    11
#define SW_R    12
#define SW_8    13

#define BTN_1   (1UL << 0)
#define BTN_2   (1UL << 1)
#define BTN_3   (1UL << 2)
#define BTN_4   (1UL << 3)
#define BTN_5   (1UL << 4)
#define BTN_6   (1UL << 5)
#define BTN_R   (1UL << 6)
#define BTN_8   (1UL << 7)

#define DEBOUNCE_MS 50

// ---- Motor FFB ----
#define PIN_RPWM    16
#define PIN_LPWM    17
#define PWM_FREQ    20000
#define PWM_RES     8
#define MAX_FORCE   255

// ========================================
// FFB Parameters
// ========================================
#define SPRING_STRENGTH   4.0f
#define SPRING_CURVE      1.4f
#define SPRING_MIN_PWM    30

#define DAMPER_STRENGTH   0.0f
#define DAMPER_COEFF      25.0f
#define FRICTION_STRENGTH 0.f

#define ENDSTOP_ZONE      0.90f
#define ENDSTOP_FORCE     255.0f

// ========================================
// BIẾN TOÀN CỤC
// ========================================
USBHIDGamepad Gamepad;
volatile long encoderCount = 0;

uint32_t currentGear          = 0;
bool     prevState[8]         = {false};
unsigned long lastDebounce[8] = {0};

const int      swPins[8] = {SW_1, SW_2, SW_3, SW_4, SW_5, SW_6, SW_R, SW_8};
const uint32_t btnMap[8] = {BTN_1, BTN_2, BTN_3, BTN_4, BTN_5, BTN_6, BTN_R, BTN_8};

long  prevEncoderCount = 0;
long  encoderVelocity  = 0;
unsigned long lastFFBTime = 0;
unsigned long lastPrintTime = 0;

// ========================================
// INTERRUPT ENCODER
// ========================================
void IRAM_ATTR onEncoderChange() {
  static int lastA = HIGH;
  int a = digitalRead(PIN_A);
  int b = digitalRead(PIN_B);
  if (a != lastA) {
    if (a == LOW) encoderCount += (b == HIGH) ? 1 : -1;
    else          encoderCount += (b == LOW)  ? 1 : -1;
    lastA = a;
  }
}

// ========================================
// ĐỌC ADC LỌC NHIỄU + ĐẢO NGƯỢC
// ========================================
int readSmoothed(int pin) {
  long sum = 0;
  for (int i = 0; i < 8; i++) {
    sum += analogRead(pin);
    delayMicroseconds(100);
  }
  int avg = (int)(sum / 8);
  return 4095 - avg;
}

// ========================================
// PEDAL CONVERSION
// ========================================
int8_t pedalToAxis(int raw, int minVal, int maxVal) {
  int clamped = constrain(raw, minVal, maxVal);
  return (int8_t)map(clamped, minVal, maxVal, -127, 127);
}

uint8_t pedalTo255(int raw, int minVal, int maxVal) {
  int clamped = constrain(raw, minVal, maxVal);
  return (uint8_t)map(clamped, minVal, maxVal, 0, 255);
}

// ========================================
// ĐIỀU KHIỂN MOTOR FFB
// ========================================
void setMotor(int force) {
  force = constrain(force, -MAX_FORCE, MAX_FORCE);
  if (force > 0) {
    ledcWrite(0, force);
    ledcWrite(1, 0);
  } else if (force < 0) {
    ledcWrite(0, 0);
    ledcWrite(1, -force);
  } else {
    ledcWrite(0, 0);
    ledcWrite(1, 0);
  }
}

// ========================================
// TÍNH LỰC FFB
// ========================================
int calcFFBForce(long count) {
  long maxCount = (long)(PPR * MAX_TURNS);

  float ratio = (float)count / (float)maxCount;
  float springMag = powf(fabsf(ratio), SPRING_CURVE) * 255.0f * SPRING_STRENGTH;
  float spring = (ratio > 0) ? -springMag : springMag;

  if (count > 5)       spring -= SPRING_MIN_PWM;
  else if (count < -5) spring += SPRING_MIN_PWM;

  float damper = -(float)encoderVelocity * DAMPER_COEFF * DAMPER_STRENGTH;

  float friction = 0;
  if      (encoderVelocity > 0) friction = -255.0f * FRICTION_STRENGTH;
  else if (encoderVelocity < 0) friction =  255.0f * FRICTION_STRENGTH;

  float endstop = 0;
  if (count >  maxCount * ENDSTOP_ZONE) endstop = -ENDSTOP_FORCE;
  if (count < -maxCount * ENDSTOP_ZONE) endstop =  ENDSTOP_FORCE;

  float total = spring + damper + friction + endstop;
  return (int)constrain(total, -255, 255);
}

// ========================================
// ĐỌC HỘP SỐ
// ========================================
uint32_t readGearbox() {
  unsigned long now = millis();
  for (int i = 0; i < 8; i++) {
    bool pressed = digitalRead(swPins[i]) == LOW;
    if (now - lastDebounce[i] < DEBOUNCE_MS) continue;
    if (pressed && !prevState[i]) {
      lastDebounce[i] = now;
      currentGear = (currentGear == btnMap[i]) ? 0 : btnMap[i];
    }
    if (pressed != prevState[i]) lastDebounce[i] = now;
    prevState[i] = pressed;
  }
  return currentGear;
}

// Đổi bitmask số → mã 1 byte gửi xe
uint8_t gearToCode(uint32_t btn) {
  if (btn & BTN_1) return 1;
  if (btn & BTN_2) return 2;
  if (btn & BTN_3) return 3;
  if (btn & BTN_4) return 4;
  if (btn & BTN_5) return 5;
  if (btn & BTN_6) return 6;
  if (btn & BTN_R) return 7;   // LÙI
  if (btn & BTN_8) return 8;   // HANDBRAKE
  return 0;                    // NEUTRAL
}

const char* gearName(uint32_t btn) {
  if (btn & BTN_1) return "SO 1";
  if (btn & BTN_2) return "SO 2";
  if (btn & BTN_3) return "SO 3";
  if (btn & BTN_4) return "SO 4";
  if (btn & BTN_5) return "SO 5";
  if (btn & BTN_6) return "SO 6";
  if (btn & BTN_R) return "LUI";
  if (btn & BTN_8) return "HANDBRAKE";
  return "NEUTRAL";
}

// ========================================
// SETUP
// ========================================
void setup() {
  Serial.begin(115200);

  pinMode(PIN_A, INPUT_PULLUP);
  pinMode(PIN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_A), onEncoderChange, CHANGE);

  for (int i = 0; i < 8; i++) pinMode(swPins[i], INPUT_PULLUP);
  delay(200);
  for (int i = 0; i < 8; i++) {
    prevState[i]    = digitalRead(swPins[i]) == LOW;
    lastDebounce[i] = millis();
  }
  currentGear = 0;

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  ledcSetup(0, PWM_FREQ, PWM_RES);
  ledcSetup(1, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_RPWM, 0);
  ledcAttachPin(PIN_LPWM, 1);
  setMotor(0);

  // ---- USB HID ----
  USB.begin();
  Gamepad.begin();
  delay(1000);

  // ---- ESP-NOW ----
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setSleep(false);

  Serial.print("TX MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("!!! ESP-NOW INIT FAILED !!!");
  } else {
    esp_now_register_send_cb(onDataSent);
    memcpy(peerInfo.peer_addr, carMacAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("!!! ADD PEER FAILED !!!");
    } else {
      Serial.println("ESP-NOW peer added OK.");
    }
  }

  Serial.println("=====================================");
  Serial.println("=== VO LANG + ESP-NOW READY ===");
  Serial.printf("Gui den xe: CC:DB:A7:99:87:90\n");
  Serial.println("=====================================");
}

// ========================================
// LOOP
// ========================================
void loop() {
  unsigned long now = millis();

  noInterrupts();
  long count = encoderCount;
  interrupts();

  long maxCount  = (long)(PPR * MAX_TURNS);
  long clamped   = constrain(count, -maxCount, maxCount);
  float sf       = (float)clamped / (float)maxCount * 127.0f;
  int8_t steerVal = (int8_t)constrain((int)sf, -127, 127);

  // ---- FFB ----
  if (now - lastFFBTime >= 20) {
    encoderVelocity  = count - prevEncoderCount;
    prevEncoderCount = count;
    lastFFBTime      = now;
    int force = calcFFBForce(count);
    setMotor(force);
  }

  // ---- Đọc pedal ----
  int conRaw   = readSmoothed(PIN_CON);
  int phanhRaw = readSmoothed(PIN_PHANH);
  int gaRaw    = readSmoothed(PIN_GA);

  int8_t gaVal    = pedalToAxis(gaRaw,    GA_MIN,    GA_MAX);
  int8_t phanhVal = pedalToAxis(phanhRaw, PHANH_MIN, PHANH_MAX);
  int8_t conVal   = pedalToAxis(conRaw,   CON_MIN,   CON_MAX);

  uint32_t buttons = readGearbox();

  // =====================================================
  // ==== GỬI ESP-NOW SANG XE (50Hz) =====================
  // =====================================================
  if (now - lastEspNowSend >= 20) {
    lastEspNowSend = now;

    txData.steerCount    = (int32_t)clamped;
    txData.steerMaxCount = (int32_t)maxCount;
    txData.throttle      = pedalTo255(gaRaw,    GA_MIN,    GA_MAX);
    txData.brake         = pedalTo255(phanhRaw, PHANH_MIN, PHANH_MAX);
    txData.clutch        = pedalTo255(conRaw,   CON_MIN,   CON_MAX);
    txData.gear          = gearToCode(buttons);

    esp_now_send(carMacAddress, (uint8_t *)&txData, sizeof(txData));
  }

  // ---- Serial debug ----
  if (now - lastPrintTime >= 200) {
    lastPrintTime = now;
    Serial.printf("ENC:%5ld | GA:%3u BR:%3u CO:%3u | %s | TX:%s\n",
      count,
      txData.throttle, txData.brake, txData.clutch,
      gearName(buttons),
      lastSendOK ? "OK" : "FAIL");
  }

  // ---- GỬI HID cho PC game ----
  Gamepad.send(steerVal, 0, 0, conVal, gaVal, phanhVal, 0, buttons);
  delay(5);
}