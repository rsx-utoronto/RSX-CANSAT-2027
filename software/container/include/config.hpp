#pragma once
#include <stdint.h>

namespace Config {
  constexpr uint8_t TEAM_ID = 1;
  // constexpr uint16_t TEAM_ID = 2026; // Set your CanSat team ID
  constexpr uint8_t WIFI_CHANNEL = 1;

  // Replace with actual Ground Station ESP32 MAC address
  const uint8_t GROUND_STATION_MAC[6] = {0x8C, 0x94, 0xDF, 0x57, 0x46, 0x4C}; //8C:94:DF:57:46:4C
}