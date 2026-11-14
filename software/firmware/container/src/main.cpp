#include <Arduino.h>

#include "config.hpp"

void setup() {
  FirmwareBoot::begin(FirmwareConfig::kNodeRole);
}

void loop() {
  delay(10);
}
