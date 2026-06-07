#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <NimBLEDevice.h>

#define SENSITIVITY_X  10.0f
#define SENSITIVITY_Y  8.0f
#define DEADZONE       0.25f
#define SMOOTH         0.12f

static const uint8_t mouseReportDesc[] = {
  0x05, 0x01, 0x09, 0x02, 0xA1, 0x01,
  0x09, 0x01, 0xA1, 0x00, 0x05, 0x09,
  0x19, 0x01, 0x29, 0x03, 0x15, 0x00,
  0x25, 0x01, 0x95, 0x03, 0x75, 0x01,
  0x81, 0x02, 0x95, 0x01, 0x75, 0x05,
  0x81, 0x03, 0x05, 0x01, 0x09, 0x30,
  0x09, 0x31, 0x15, 0x81, 0x25, 0x7F,
  0x75, 0x08, 0x95, 0x02, 0x81, 0x06,
  0xC0, 0xC0
};

NimBLEServer*         pServer     = nullptr;
NimBLECharacteristic* pReportChar = nullptr;
Adafruit_MPU6050      mpu;
bool connected = false;
float smoothX  = 0;
float smoothY  = 0;

class ServerCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* s) {
    connected = true;
    Serial.println("=== CONNECTED ===");
  }
  void onDisconnect(NimBLEServer* s) {
    connected = false;
    NimBLEDevice::startAdvertising();
    Serial.println("Disconnected...");
  }
};

void setup() {
  Serial.begin(115200);
  delay(1000);

  // MPU-6050
  Wire.begin(21, 22);
  if (!mpu.begin()) {
    Serial.println("KHONG TIM THAY MPU-6050!");
    while (1) delay(500);
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("MPU-6050 OK!");

  // BLE
  NimBLEDevice::init("HeadMouse");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEDevice::setSecurityAuth(false, false, false);

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCB());

  NimBLEService* hid = pServer->createService("1812");

  NimBLECharacteristic* info = hid->createCharacteristic(
    "2a4a", NIMBLE_PROPERTY::READ);
  uint8_t hidInfo[] = {0x11, 0x01, 0x00, 0x02};
  info->setValue(hidInfo, 4);

  NimBLECharacteristic* rmap = hid->createCharacteristic(
    "2a4b", NIMBLE_PROPERTY::READ);
  rmap->setValue(mouseReportDesc, sizeof(mouseReportDesc));

  hid->createCharacteristic("2a4c", NIMBLE_PROPERTY::WRITE_NR);

  NimBLECharacteristic* proto = hid->createCharacteristic(
    "2a4e", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE_NR);
  uint8_t pm = 0x01;
  proto->setValue(&pm, 1);

  pReportChar = hid->createCharacteristic(
    "2a4d", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  NimBLEDescriptor* rref = pReportChar->createDescriptor(
    "2908", NIMBLE_PROPERTY::READ, 2);
  uint8_t rrefVal[] = {0x00, 0x01};
  rref->setValue(rrefVal, 2);

  hid->start();

  NimBLEService* bat = pServer->createService("180f");
  NimBLECharacteristic* batLvl = bat->createCharacteristic(
    "2a19", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  uint8_t b = 100;
  batLvl->setValue(&b, 1);
  bat->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID("1812");
  adv->setAppearance(0x03C2);
  adv->setScanResponse(true);
  adv->start();

  Serial.println("=== HeadMouse READY ===");
}

void loop() {
  if (!connected) {
    delay(100);
    return;
  }

  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  float rawX = -g.gyro.z;  // Xoay trái/phải
  float rawY = g.gyro.x;  // Cúi/ngước

  if (abs(rawX) < DEADZONE) rawX = 0;
  if (abs(rawY) < DEADZONE) rawY = 0;

  smoothX = smoothX * (1.0f - SMOOTH) + rawX * SMOOTH;
  smoothY = smoothY * (1.0f - SMOOTH) + rawY * SMOOTH;

  int8_t mx = (int8_t)constrain(smoothX * SENSITIVITY_X, -127, 127);
  int8_t my = (int8_t)constrain(smoothY * SENSITIVITY_Y, -127, 127);

  if (mx != 0 || my != 0) {
    uint8_t report[3] = {0x00, (uint8_t)mx, (uint8_t)my};
    pReportChar->setValue(report, 3);
    pReportChar->notify();
  }

  Serial.printf("X:%5.2f Y:%5.2f | MX:%4d MY:%4d\n",
    g.gyro.z, g.gyro.x, mx, my);

  delay(10);
}
