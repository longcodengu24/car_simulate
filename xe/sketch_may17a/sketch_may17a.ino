// =====================================================
// XE RX - CLOSED-LOOP SPEED + STEER LUC KEO CAO + BRAKE REVERSE
// =====================================================
#include <esp_now.h>
#include <WiFi.h>

// =====================================================
// PINOUT
// =====================================================
#define DRIVE_IN1   25
#define DRIVE_IN2   26
#define DRIVE_ENA   27
#define STEER_IN3   32
#define STEER_IN4   33
#define STEER_ENB   14
#define DRIVE_ENC_A 18
#define DRIVE_ENC_B 19
#define STEER_ENC_A 22
#define STEER_ENC_B 23

#define CH_DRIVE       0
#define CH_STEER       1
#define PWM_FREQ_DRIVE 20000      // Drive: 20kHz - chay em
#define PWM_FREQ_STEER 5000       // Steer: 5kHz - LUC KEO MANH HON
#define PWM_RES        8

// =====================================================
// HIEU CHINH
// =====================================================
#define MAX_DRIVE_SPEED       600
#define STEER_MAX_COUNT_CAR   500

#define INVERT_DRIVE_MOTOR    false
#define INVERT_DRIVE_ENCODER  false
#define INVERT_STEER_MOTOR    false
#define INVERT_STEER_ENCODER  false

// PID STEER - TANG LUC KEO TOI DA
float Kp_steer = 2.5f;            // Tang manh
float Ki_steer = 0.3f;            // Bu sai so tinh
float Kd_steer = 0.15f;
#define STEER_DEADBAND   3
#define STEER_MIN_PWM    100      // Vuot ma sat tinh
#define STEER_MAX_PWM    255      // CHO LEN HET MAX

// PI DRIVE
float Kp_drive = 0.4f;
float Ki_drive = 0.5f;
#define DRIVE_MIN_PWM    40
#define DRIVE_MAX_PWM    255
#define COMM_TIMEOUT_MS  500

// =====================================================
// STRUCT DATA
// =====================================================
typedef struct __attribute__((packed)) {
  int32_t  steerCount;
  int32_t  steerMaxCount;
  uint8_t  throttle;
  uint8_t  brake;
  uint8_t  clutch;
  uint8_t  gear;
} ControlData;

ControlData rxData;
volatile unsigned long lastRxTime = 0;

// =====================================================
// ENCODERS
// =====================================================
volatile long driveEncCount = 0;
volatile long steerEncCount = 0;
long          prevDriveCount = 0;
long          driveSpeed = 0;
unsigned long lastSpeedTime = 0;

void IRAM_ATTR onDriveEnc() {
  static int lastA = HIGH;
  int a = digitalRead(DRIVE_ENC_A);
  int b = digitalRead(DRIVE_ENC_B);
  if (a != lastA) {
    int delta;
    if (a == LOW) delta = (b == HIGH) ? 1 : -1;
    else          delta = (b == LOW)  ? 1 : -1;
    if (INVERT_DRIVE_ENCODER) delta = -delta;
    driveEncCount += delta;
    lastA = a;
  }
}

void IRAM_ATTR onSteerEnc() {
  static int lastA = HIGH;
  int a = digitalRead(STEER_ENC_A);
  int b = digitalRead(STEER_ENC_B);
  if (a != lastA) {
    int delta;
    if (a == LOW) delta = (b == HIGH) ? 1 : -1;
    else          delta = (b == LOW)  ? 1 : -1;
    if (INVERT_STEER_ENCODER) delta = -delta;
    steerEncCount += delta;
    lastA = a;
  }
}

// =====================================================
// ESP-NOW
// =====================================================
void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len != sizeof(ControlData)) return;
  memcpy((void*)&rxData, data, sizeof(ControlData));
  lastRxTime = millis();
}

// =====================================================
// MOTOR CONTROL
// =====================================================
void setDriveMotor(int pwm) {
  if (INVERT_DRIVE_MOTOR) pwm = -pwm;
  pwm = constrain(pwm, -DRIVE_MAX_PWM, DRIVE_MAX_PWM);
  if (pwm > 0) {
    digitalWrite(DRIVE_IN1, HIGH);
    digitalWrite(DRIVE_IN2, LOW);
    ledcWrite(CH_DRIVE, pwm);
  } else if (pwm < 0) {
    digitalWrite(DRIVE_IN1, LOW);
    digitalWrite(DRIVE_IN2, HIGH);
    ledcWrite(CH_DRIVE, -pwm);
  } else {
    digitalWrite(DRIVE_IN1, LOW);
    digitalWrite(DRIVE_IN2, LOW);
    ledcWrite(CH_DRIVE, 0);
  }
}

void setSteerMotor(int pwm) {
  if (INVERT_STEER_MOTOR) pwm = -pwm;
  pwm = constrain(pwm, -STEER_MAX_PWM, STEER_MAX_PWM);
  if (pwm > 0) {
    digitalWrite(STEER_IN3, HIGH);
    digitalWrite(STEER_IN4, LOW);
    ledcWrite(CH_STEER, pwm);
  } else if (pwm < 0) {
    digitalWrite(STEER_IN3, LOW);
    digitalWrite(STEER_IN4, HIGH);
    ledcWrite(CH_STEER, -pwm);
  } else {
    digitalWrite(STEER_IN3, LOW);
    digitalWrite(STEER_IN4, LOW);
    ledcWrite(CH_STEER, 0);
  }
}

// =====================================================
// PID STEER - TANG LUC KEO + STALL BOOST
// =====================================================
float steerIntegral  = 0;
float steerLastError = 0;
unsigned long lastSteerPIDTime = 0;

int computeSteerPID(long target, long current) {
  unsigned long now = millis();
  float dt = (now - lastSteerPIDTime) / 1000.0f;
  if (dt <= 0 || dt > 0.1f) dt = 0.01f;
  lastSteerPIDTime = now;

  float error = (float)(target - current);
  if (fabsf(error) < STEER_DEADBAND) {
    steerIntegral = 0;
    steerLastError = error;
    return 0;
  }

  // Anti-windup
  static bool wasSaturated = false;
  if (!wasSaturated) {
    steerIntegral += error * dt;
    steerIntegral = constrain(steerIntegral, -300.0f, 300.0f);
  }

  float derivative = (error - steerLastError) / dt;
  steerLastError = error;

  float output = Kp_steer * error + Ki_steer * steerIntegral + Kd_steer * derivative;

  // STALL BOOST: sai so > 50 → ep PWM max
  if (fabsf(error) > 50) {
    output = (error > 0) ? STEER_MAX_PWM : -STEER_MAX_PWM;
  }

  int outInt = (int)constrain(output, -(float)STEER_MAX_PWM, (float)STEER_MAX_PWM);
  wasSaturated = (abs(outInt) >= STEER_MAX_PWM);

  if (outInt > 0 && outInt < STEER_MIN_PWM) outInt = STEER_MIN_PWM;
  if (outInt < 0 && outInt > -STEER_MIN_PWM) outInt = -STEER_MIN_PWM;
  return outInt;
}

// =====================================================
// PI DRIVE
// =====================================================
float driveIntegral = 0;
int   driveTargetSpeed = 0;
int   drivePWM_out = 0;

void updateDriveSpeedPID() {
  if (driveTargetSpeed == 0) {
    driveIntegral = 0;
    drivePWM_out = 0;
    return;
  }

  float error = (float)driveTargetSpeed - (float)driveSpeed;
  driveIntegral += error * 0.1f;
  driveIntegral = constrain(driveIntegral, -400.0f, 400.0f);

  float output = Kp_drive * error + Ki_drive * driveIntegral;
  int outInt = (int)constrain(output, -DRIVE_MAX_PWM, DRIVE_MAX_PWM);

  if (outInt > 0 && outInt < DRIVE_MIN_PWM) outInt = DRIVE_MIN_PWM;
  if (outInt < 0 && outInt > -DRIVE_MIN_PWM) outInt = -DRIVE_MIN_PWM;

  drivePWM_out = outInt;
}

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(DRIVE_IN1, OUTPUT);
  pinMode(DRIVE_IN2, OUTPUT);
  pinMode(STEER_IN3, OUTPUT);
  pinMode(STEER_IN4, OUTPUT);
  digitalWrite(DRIVE_IN1, LOW);
  digitalWrite(DRIVE_IN2, LOW);
  digitalWrite(STEER_IN3, LOW);
  digitalWrite(STEER_IN4, LOW);

  // PWM TAN SO RIENG CHO TUNG MOTOR
  ledcSetup(CH_DRIVE, PWM_FREQ_DRIVE, PWM_RES);
  ledcSetup(CH_STEER, PWM_FREQ_STEER, PWM_RES);
  ledcAttachPin(DRIVE_ENA, CH_DRIVE);
  ledcAttachPin(STEER_ENB, CH_STEER);
  ledcWrite(CH_DRIVE, 0);
  ledcWrite(CH_STEER, 0);

  pinMode(DRIVE_ENC_A, INPUT_PULLUP);
  pinMode(DRIVE_ENC_B, INPUT_PULLUP);
  pinMode(STEER_ENC_A, INPUT_PULLUP);
  pinMode(STEER_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(DRIVE_ENC_A), onDriveEnc, CHANGE);
  attachInterrupt(digitalPinToInterrupt(STEER_ENC_A), onSteerEnc, CHANGE);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setSleep(false);

  Serial.println();
  Serial.println("=====================================");
  Serial.print  ("RX MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.println("STEER: Tang luc keo + stall boost");
  Serial.println("BRAKE: Dap phanh → motor quay nguoc");
  Serial.printf ("PWM Drive=%dHz | Steer=%dHz\n", PWM_FREQ_DRIVE, PWM_FREQ_STEER);
  Serial.println("=====================================");

  if (esp_now_init() != ESP_OK) {
    Serial.println("!!! ESP-NOW FAIL !!!");
    while (true) delay(1000);
  }
  esp_now_register_recv_cb(onDataRecv);

  Serial.println("San sang. Cho TX...");
  lastSpeedTime    = millis();
  lastSteerPIDTime = millis();
}

// =====================================================
// LOOP
// =====================================================
unsigned long lastDebugTime = 0;

void loop() {
  unsigned long now = millis();

  // Cap nhat speed va Drive PID moi 100ms
  if (now - lastSpeedTime >= 100) {
    noInterrupts();
    long cur = driveEncCount;
    interrupts();
    driveSpeed = cur - prevDriveCount;
    prevDriveCount = cur;
    lastSpeedTime = now;
    updateDriveSpeedPID();
  }

  // An toan
  if (lastRxTime == 0 || (now - lastRxTime) > COMM_TIMEOUT_MS) {
    setDriveMotor(0);
    setSteerMotor(0);
    driveTargetSpeed = 0;
    driveIntegral = 0;
    if (now - lastDebugTime > 1000) {
      lastDebugTime = now;
      Serial.println("[WAIT] Khong co tin hieu TX...");
    }
    delay(10);
    return;
  }

  noInterrupts();
  ControlData d = rxData;
  long curSteer = steerEncCount;
  long curDrive = driveEncCount;
  interrupts();

  // ===== STEER: PID position =====
  float ratio = 0;
  if (d.steerMaxCount > 0) {
    ratio = (float)d.steerCount / (float)d.steerMaxCount;
    ratio = constrain(ratio, -1.0f, 1.0f);
  }
  long targetSteer = (long)(ratio * (float)STEER_MAX_COUNT_CAR);
  int steerPWM = computeSteerPID(targetSteer, curSteer);
  if (curSteer >=  STEER_MAX_COUNT_CAR && steerPWM > 0) steerPWM = 0;
  if (curSteer <= -STEER_MAX_COUNT_CAR && steerPWM < 0) steerPWM = 0;
  setSteerMotor(steerPWM);

  // ===== DRIVE: GA = TIEN, PHANH = LUI =====
  // Tinh net: ga - phanh, ra so co dau -255..+255
  int net = (int)d.throttle - (int)d.brake;
  // Map sang target speed co dau
  int targetSpeedSigned = (int)((float)net * (float)MAX_DRIVE_SPEED / 255.0f);

  if (d.gear == 8) {
    driveTargetSpeed = 0;                       // Handbrake
  } else if (d.gear == 7) {
    driveTargetSpeed = -targetSpeedSigned;      // So R: dao chieu
  } else {
    driveTargetSpeed = targetSpeedSigned;       // So tien (0, 1-6)
  }

  setDriveMotor(drivePWM_out);

  // Debug
  if (now - lastDebugTime > 200) {
    lastDebugTime = now;
    Serial.printf(
      "STEER tgt:%5ld cur:%5ld pwm:%4d | DRIVE tgtSpd:%5d curSpd:%5ld pwm:%4d | T:%3u B:%3u G:%u\n",
      targetSteer, curSteer, steerPWM,
      driveTargetSpeed, driveSpeed, drivePWM_out,
      d.throttle, d.brake, d.gear
    );
  }

  delay(5);
}