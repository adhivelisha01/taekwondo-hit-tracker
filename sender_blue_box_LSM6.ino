// =====================================================================
//  BLUE BOX  ---  SENDER  (LSM6DSL / LSM6DS0 / LSM6DS3 family)
//  Reads the LSM6 IMU and transmits packets over ESP-NOW to the black box.
//  NODE_ID = 0  -> this is the BLUE player's tracker.
// =====================================================================
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>

uint8_t LSM6_ADDR = 0x6A;   // if WHO_AM_I reads 0xFF, change to 0x6B and re-flash
#define WHO_AM_I   0x0F
#define CTRL1_XL   0x10
#define CTRL2_G    0x11
#define CTRL3_C    0x12
#define OUTX_L_G   0x22
#define OUTX_L_XL  0x28

// Black box address (it forces itself to this MAC).
uint8_t receiverMAC[] = {0xB0, 0xCB, 0xD8, 0xEA, 0xB1, 0x50};

typedef struct __attribute__((packed)) {
  uint32_t timestamp_ms;
  float    ax, ay, az;
  float    gx, gy, gz;
  uint8_t  node_id;
} IMUPacket;

IMUPacket packet;

const uint8_t NODE_ID   = 0;    // BLUE = 0
const int     SEND_RATE = 100;  // packets per second
uint32_t      lastSend  = 0;
uint32_t      lastPrint = 0;

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(LSM6_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(LSM6_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)LSM6_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

void readBytes(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(LSM6_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)LSM6_ADDR, len);
  for (uint8_t i = 0; i < len && Wire.available(); i++) buf[i] = Wire.read();
}

void initIMU() {
  uint8_t id = readReg(WHO_AM_I);
  Serial.printf("WHO_AM_I = 0x%02X  ", id);
  if (id == 0xFF || id == 0x00)
    Serial.println("<-- SENSOR NOT RESPONDING (check wiring, or try LSM6_ADDR 0x6B)");
  else
    Serial.println("<-- sensor is alive");

  writeReg(CTRL3_C, 0x44);   // BDU + auto-increment
  delay(20);
  writeReg(CTRL1_XL, 0x6C);  // accel 416Hz, +/-8g
  writeReg(CTRL2_G,  0x6C);  // gyro  416Hz, 2000dps
  Serial.println("IMU configured (416Hz, +/-8g, 2000dps)");
}

void readIMU() {
  uint8_t raw[12];
  readBytes(OUTX_L_G,  raw,     6);   // LSM6 = little-endian (low byte first)
  readBytes(OUTX_L_XL, raw + 6, 6);

  int16_t rawGx = (int16_t)(raw[1]  << 8 | raw[0]);
  int16_t rawGy = (int16_t)(raw[3]  << 8 | raw[2]);
  int16_t rawGz = (int16_t)(raw[5]  << 8 | raw[4]);
  int16_t rawAx = (int16_t)(raw[7]  << 8 | raw[6]);
  int16_t rawAy = (int16_t)(raw[9]  << 8 | raw[8]);
  int16_t rawAz = (int16_t)(raw[11] << 8 | raw[10]);

  const float ACC_SCALE = 0.244f * 9.80665f / 1000.0f;  // +/-8g -> m/s^2
  const float GYR_SCALE = 0.070f;                        // 2000dps -> deg/s

  packet.ax = rawAx * ACC_SCALE;
  packet.ay = rawAy * ACC_SCALE;
  packet.az = rawAz * ACC_SCALE;
  packet.gx = rawGx * GYR_SCALE;
  packet.gy = rawGy * GYR_SCALE;
  packet.gz = rawGz * GYR_SCALE;
}

void onSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  static uint32_t okCount = 0, failCount = 0, lastReport = 0;
  if (status == ESP_NOW_SEND_SUCCESS) okCount++; else failCount++;
  if (millis() - lastReport > 1000) {
    lastReport = millis();
    Serial.printf("[TX in last 1s]  delivered=%lu  failed=%lu\n", okCount, failCount);
    okCount = 0; failCount = 0;
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n--- BLUE SENDER (node 0) BOOT ---");

  Wire.begin(21, 22);
  Wire.setClock(400000);
  initIMU();

  WiFi.mode(WIFI_STA);
  Serial.print("Sender MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init FAILED"); while (true);
  }
  Serial.println("ESP-NOW init OK");
  esp_now_register_send_cb(onSent);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, receiverMAC, 6);
  peer.channel = 0;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Add peer FAILED"); while (true);
  }
  Serial.println("Peer added OK");

  packet.node_id = NODE_ID;
  Serial.printf("Node %d ready - sending at %d Hz\n", NODE_ID, SEND_RATE);
}

void loop() {
  uint32_t now = millis();
  if (now - lastSend >= (1000 / SEND_RATE)) {
    lastSend = now;
    readIMU();
    packet.timestamp_ms = now;
    esp_now_send(receiverMAC, (uint8_t *)&packet, sizeof(packet));

    if (now - lastPrint >= 200) {   // ~5x/sec, readable
      lastPrint = now;
      Serial.printf("ax=%.2f ay=%.2f az=%.2f | gx=%.1f gy=%.1f gz=%.1f\n",
                    packet.ax, packet.ay, packet.az,
                    packet.gx, packet.gy, packet.gz);
    }
  }
}
