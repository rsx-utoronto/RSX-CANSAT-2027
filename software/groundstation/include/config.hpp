#pragma once
#include <stdint.h>

namespace Config {
  // Must match the channel used by the container
  constexpr uint8_t WIFI_CHANNEL = 1;

  // Replace with the actual Container ESP32 MAC address
  const uint8_t CONTAINER_MAC[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
}