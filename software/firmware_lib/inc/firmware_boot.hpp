#pragma once

#include <stdint.h>

namespace FirmwareBoot {

enum class NodeRole : uint8_t {
  GroundStation,
  PocketQube,
  Container,
};

// Starts the diagnostic serial port and prints one boot banner. Does not
// initialize Wi-Fi, ESP-NOW, sensors, or actuators, or wait for a terminal.
void begin(NodeRole role);

}  // namespace FirmwareBoot
