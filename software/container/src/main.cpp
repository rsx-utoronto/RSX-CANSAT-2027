#include <Arduino.h>
#include "espnow_link.hpp"
#include "protocol.hpp"
#include "config.hpp"

static uint32_t packetSequence = 0;

void setup() {
  Serial.begin(115200);

  // Initialize ESP-NOW on designated Wi-Fi channel
  if (EspNowLink::begin(Config::WIFI_CHANNEL)) {
    Serial.println("[CONTAINER] ESP-NOW Initialized.");
    
    // Register Ground Station peer receiver
    if (EspNowLink::addPeer(Config::GROUND_STATION_MAC)) {
      Serial.println("[CONTAINER] Ground Station Peer Registered.");
    }
  } else {
    Serial.println("[CONTAINER] ESP-NOW Init Failed!");
  }
}

void loop() {
  static uint32_t lastTxTime = 0;

  // Broadcast telemetry frame at 2 Hz (every 500 ms)
  if (millis() - lastTxTime >= 500) {
    lastTxTime = millis();
    packetSequence++;

    // Instantiate exact struct from protocol.hpp
    ContainerPacket packet = {};
    packet.team_id = Config::TEAM_ID;
    packet.mission_time_ms = millis();
    packet.packet_count = packetSequence;
    packet.command_count = 0;
    packet.mode = static_cast<char>(Mode::FLIGHT);
    
    // Placeholder values: connect to sensor_manager in production
    packet.altitude_m = 150.2f;
    packet.pressure_pa = 100130;
    packet.temperature_c = 21.5f;
    packet.battery_v = 11.8f;
    packet.battery_ma = 450;
    packet.mech_state = 0x01;
    packet.state = OpState::ASCENT;
    
    memset(packet.cmd_echo, 0, sizeof(packet.cmd_echo));
    strncpy(packet.cmd_echo, "NONE", sizeof(packet.cmd_echo) - 1);

    // Cast pointer to raw uint8_t bytes and pass structure size
    bool success = EspNowLink::send(
      Config::GROUND_STATION_MAC, 
      reinterpret_cast<const uint8_t*>(&packet), 
      sizeof(ContainerPacket)
    );

    if (success) {
      Serial.printf("[CONTAINER] Packet #%u Sent (%u bytes)\n", packetSequence, sizeof(ContainerPacket));
    } else {
      Serial.println("[CONTAINER] Packet Send Failed!");
    }
  }
}


// #include <Arduino.h>
// #include <WiFi.h>
// #include <esp_wifi.h>

// void readMacAddress() {
//   uint8_t baseMac[6];
//   esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
//   if (ret == ESP_OK) {
//     Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X\n",
//                   baseMac[0], baseMac[1], baseMac[2],
//                   baseMac[3], baseMac[4], baseMac[5]);
//   } else {
//     Serial.println("Failed to read MAC address");
//   }
// }

// void setup() {
//   Serial.begin(115200);
//   delay(1000); // Give serial monitor time to initialize

//   // Initialize Wi-Fi hardware in Station mode
//   WiFi.mode(WIFI_STA);

//   Serial.print("\n[DEFAULT] ESP32 Board MAC Address: ");
//   readMacAddress();
// }
 
// void loop() {
//   // Nothing needed here
// }