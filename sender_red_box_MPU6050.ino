// =====================================================================
//  RED BOX  ---  SENDER  (MPU-6050 / GY-521)
//  Different sensor from the blue box, so this has its own driver:
//    - address 0x68 (not 0x6A)
//    - must be woken from sleep on boot
//    - data bytes are BIG-endian (high byte first) -- opposite of LSM6
//  BUT the transmitted packet is byte-for-byte identical to the blue
//  box, so the black box receives it the same way. NODE_ID = 1.
// =====================================================================
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>

uint8_t MPU_ADDR = 0x68;    // if WHO_AM_I != 0x68, try 0x69 and re-flash
#define WHO_AM_I      0x75
#define PWR_MGMT_1    0x6B
#define SMPLRT_DIV    0x19
#define CONFIG        0x1A
#define GYRO_CONFIG   0x1B
#define ACCEL_CONFIG  0x1C
#define ACCEL_XOUT_H  0x3B

// Black box address (it forces itself to this MAC).
uint8_t receiverMAC[] = {0xB0, 0xCB, 0xD8, 0xEA, 0xB1, 0x50};

typedef struct __attribute__((packed)) {
  uint32_t timestamp_ms;
  float    ax, ay, az;
  float    gx, gy, gz;
  uint8_t  node_id;
} IMUPacket;

IMUPacket packet;

const uint8_t NODE_ID   = 1;    // RED = 1
const int     SEND_RATE = 100;  // packets per second
uint32_t      lastSend  = 0;
uint32_t      lastPrint = 0;

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

void readBytes(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, len);
  for (uint8_t i = 0; i < len && Wire.available(); i++) buf[i] = Wire.read();
}

void initIMU() {
  uint8_t id = readReg(WHO_AM_I);
  Serial.printf("WHO_AM_I = 0x%02X  ", id);
  if (id == 0x68)
    Serial.println("<-- MPU-6050 detected");
  else if (id == 0xFF || id == 0x00)
    Serial.println("<-- SENSOR NOT RESPONDING (check wiring, or try MPU_ADDR 0x69)");
  else
    Serial.println("<-- responded, but not the expected 0x68 (wrong chip / address?)");

  writeReg(PWR_MGMT_1,   0x00);   // wake from sleep (internal oscillator)
  delay(50);
  writeReg(CONFIG,       0x00);   // DLPF off -> fast response for impacts
  writeReg(SMPLRT_DIV,   0x00);
  writeReg(GYRO_CONFIG,  0x18);   // +/-2000 dps
  writeReg(ACCEL_CONFIG, 0x10);   // +/-8g   (matches the blue box range)
  Serial.println("MPU configured (+/-8g, 2000dps)");
}

void readIMU() {
  uint8_t raw[14];
  // One burst: accel(6) + temp(2) + gyro(6). MPU = BIG-endian (high first).
  readBytes(ACCEL_XOUT_H, raw, 14);

  int16_t rawAx = (int16_t)(raw[0]  << 8 | raw[1]);
  int16_t rawAy = (int16_t)(raw[2]  << 8 | raw[3]);
  int16_t rawAz = (int16_t)(raw[4]  << 8 | raw[5]);
  // raw[6], raw[7] = temperature (ignored)
  int16_t rawGx = (int16_t)(raw[8]  << 8 | raw[9]);
  int16_t rawGy = (int16_t)(raw[10] << 8 | raw[11]);
  int16_t rawGz = (int16_t)(raw[12] << 8 | raw[13]);

  // +/-8g  -> 4096 LSB/g ;  convert to m/s^2 (same units as the blue box)
  const float ACC_SCALE = 9.80665f / 4096.0f;
  // +/-2000dps -> 16.4 LSB/(deg/s)
  const float GYR_SCALE = 1.0f / 16.4f;

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
  Serial.println("\n--- RED SENDER (node 1) BOOT ---");

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
