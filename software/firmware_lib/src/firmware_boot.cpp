#include "firmware_boot.hpp"

#include <Arduino.h>
#include <esp_mac.h>

namespace FirmwareBoot {
namespace {

const char* roleName(NodeRole role) {
  switch (role) {
    case NodeRole::GroundStation:
      return "GROUND_STATION";
    case NodeRole::PocketQube:
      return "PQ";
    case NodeRole::Container:
      return "CONTAINER";
  }
  return "UNKNOWN";
}

}  // namespace

void begin(NodeRole role) {
  Serial.begin(115200);
  delay(250);  // Bounded settling time; never wait for a serial connection.
  Serial.printf("ROLE=%s\n", roleName(role));

  uint8_t mac[6] = {};
  const esp_err_t result = esp_read_mac(mac, ESP_MAC_WIFI_STA);
  if (result == ESP_OK) {
    Serial.printf("MAC=%02X:%02X:%02X:%02X:%02X:%02X\n",
                  static_cast<unsigned>(mac[0]),
                  static_cast<unsigned>(mac[1]),
                  static_cast<unsigned>(mac[2]),
                  static_cast<unsigned>(mac[3]),
                  static_cast<unsigned>(mac[4]),
                  static_cast<unsigned>(mac[5]));
  } else {
    Serial.printf("MAC=UNAVAILABLE (error=%d)\n", static_cast<int>(result));
  }

  Serial.println("MODE=BOOT_ONLY");
}

}  // namespace FirmwareBoot
