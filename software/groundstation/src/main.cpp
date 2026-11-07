// #include <Arduino.h>
// #include <WiFi.h>
// #include "espnow_link.hpp"
// #include "protocol.hpp"
// #include "config.hpp"

// // Shared between the ESP-NOW receive callback (WiFi task) and loop()
// static ContainerPacket latestPacket;
// static uint8_t latestSenderMac[6];
// static volatile bool newPacket = false;
// static volatile uint32_t badLengthCount = 0;
// static portMUX_TYPE rxMux = portMUX_INITIALIZER_UNLOCKED;

// // Called by EspNowLink whenever any packet arrives.
// // Keep this short: copy the data and let loop() do the printing.
// static void onRadioData(const uint8_t* mac, const uint8_t* data, int len) {
//   if (len != (int)sizeof(ContainerPacket)) {
//     badLengthCount++;  // wrong size: struct mismatch or a different sender
//     return;
//   }
//   portENTER_CRITICAL(&rxMux);
//   memcpy(&latestPacket, data, sizeof(ContainerPacket));
//   memcpy(latestSenderMac, mac, sizeof(latestSenderMac));
//   newPacket = true;
//   portEXIT_CRITICAL(&rxMux);
// }

// void setup() {
//   Serial.begin(115200);
//   delay(500);

//   if (!EspNowLink::begin(Config::WIFI_CHANNEL)) {
//     Serial.println("[GROUND] ESP-NOW Init Failed!");
//     return;
//   }
//   Serial.println("[GROUND] ESP-NOW Initialized.");

//   // Put THIS address into GROUND_STATION_MAC in container/include/config.hpp
//   Serial.print("[GROUND] My MAC address: ");
//   Serial.println(WiFi.macAddress());

//   // Register the container as a peer (needed later for sending commands)
//   if (EspNowLink::addPeer(Config::CONTAINER_MAC)) {
//     Serial.println("[GROUND] Container Peer Registered.");
//   } else {
//     Serial.println("[GROUND] Failed to register Container peer.");
//   }

//   EspNowLink::onReceive(onRadioData);
//   Serial.println("[GROUND] Waiting for telemetry...");
// }

// void loop() {
//   if (!newPacket) {
//     return;
//   }

//   // Take a private copy so the callback can't overwrite it mid-print
//   ContainerPacket pkt;
//   uint8_t senderMac[6];
//   portENTER_CRITICAL(&rxMux);
//   memcpy(&pkt, &latestPacket, sizeof(ContainerPacket));
//   memcpy(senderMac, latestSenderMac, sizeof(senderMac));
//   newPacket = false;
//   portEXIT_CRITICAL(&rxMux);

//   Serial.printf("[GROUND] Telemetry received from: %02X:%02X:%02X:%02X:%02X:%02X\n",
//                 senderMac[0], senderMac[1], senderMac[2],
//                 senderMac[3], senderMac[4], senderMac[5]);

//   // cmd_echo isn't guaranteed to be null-terminated over the air
//   char echo[sizeof(pkt.cmd_echo) + 1];
//   memcpy(echo, pkt.cmd_echo, sizeof(pkt.cmd_echo));
//   echo[sizeof(pkt.cmd_echo)] = '\0';

//   Serial.printf(
//     "[GROUND] team=%u t=%lu ms pkt=%lu cmds=%u mode=%c alt=%.1f m "
//     "press=%lu Pa temp=%.1f C batt=%.2f V %u mA mech=0x%02X state=%u echo=%s\n",
//     pkt.team_id,
//     (unsigned long)pkt.mission_time_ms,
//     (unsigned long)pkt.packet_count,
//     pkt.command_count,
//     pkt.mode,
//     pkt.altitude_m,
//     (unsigned long)pkt.pressure_pa,
//     pkt.temperature_c,
//     pkt.battery_v,
//     pkt.battery_ma,
//     pkt.mech_state,
//     (unsigned)pkt.state,
//     echo);

//   if (badLengthCount > 0) {
//     Serial.printf("[GROUND] Warning: %lu packet(s) with wrong size ignored\n",
//                   (unsigned long)badLengthCount);
//     badLengthCount = 0;
//   }
// }

