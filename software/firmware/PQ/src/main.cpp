#include <Arduino.h>

#include "config.hpp"

void setup() {
  FirmwareBoot::begin(FirmwareConfig::kNodeRole);
  // TODO: start the PQ modules here: MissionManager::begin(), the camera
  // drive, and the 100 Hz control task. Update
  // software/tests/test_boot_skeleton.py in the same change, because it
  // compiles this file with only the boot code.
}

void loop() {
  delay(10);
}
