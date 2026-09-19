// =====================================================================
//  BLACK BOX  (MC_1)  ---  RECEIVER
//  Receives ESP-NOW packets from both trackers and prints them as CSV.
//  Key fix: forces this board's MAC to the exact address the trackers
//  transmit to (B0:CB:D8:EA:B1:50), so no tracker needs reflashing.
// =====================================================================
#include <esp_now.h>
#include <WiFi.h>
#include "esp_wifi.h"

typedef struct __attribute__((packed)) {
  uint32_t timestamp_ms;
  float    ax, ay, az;
  float    gx, gy, gz;
  uint8_t  node_id;          // 0 = blue tracker, 1 = red tracker
} IMUPacket;

void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(IMUPacket)) return;

  IMUPacket pkt;
  memcpy(&pkt, data, sizeof(pkt));

  // CSV: timestamp, node, ax, ay, az, gx, gy, gz
  Serial.printf("%lu,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                (unsigned long)pkt.timestamp_ms,
                pkt.node_id,
                pkt.ax, pkt.ay, pkt.az,
                pkt.gx, pkt.gy, pkt.gz);
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  // Force THIS board to be the address the trackers already aim at
  uint8_t newMAC[] = {0xB0, 0xCB, 0xD8, 0xEA, 0xB1, 0x50};
  esp_wifi_set_mac(WIFI_IF_STA, newMAC);

  // Read the ACTIVE mac back to prove the change took
  uint8_t mac[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac);
  Serial.printf("Master MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed"); while (true);
  }
  esp_now_register_recv_cb(onReceive);

  Serial.println("timestamp_ms,node_id,ax,ay,az,gx,gy,gz");
  Serial.println("# Master ready");
}

void loop() {
  delay(10);
}
