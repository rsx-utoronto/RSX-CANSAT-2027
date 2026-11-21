#include "release_mechanism.hpp"

#include <Arduino.h>

#include "config.hpp"

using namespace FirmwareConfig;

namespace {

// **CHECK:** sensors are assumed to be switches to GND with the internal
// pull-up enabled, so LOW means "deployed" / "cleared". Change if the
// electrical design differs.
bool readActiveLow(int pin) { return digitalRead(pin) == LOW; }

}  // namespace

void ReleaseMechanism::begin(uint8_t restored_mech_state) {
  if (kReleaseDeployedSensorPin >= 0) pinMode(kReleaseDeployedSensorPin, INPUT_PULLUP);
  if (kPqClearedSensorPin >= 0) pinMode(kPqClearedSensorPin, INPUT_PULLUP);

  mech_state_ = restored_mech_state;
  if (activated()) driveActuator();  // Re-assert after a reset.
}

void ReleaseMechanism::trigger() {
  mech_state_ |= kBitActivated;
  driveActuator();
  update();
}

bool ReleaseMechanism::deploymentConfirmed() const {
  // CTR11. Without a sensor we can't confirm anything.
  return kReleaseDeployedSensorPin >= 0 && readActiveLow(kReleaseDeployedSensorPin);
}

void ReleaseMechanism::update() {
  // CTR12: PQ released. Once seen, it stays set even if the sensor bounces.
  if (kPqClearedSensorPin >= 0) {
    if (readActiveLow(kPqClearedSensorPin)) mech_state_ |= kBitReleased;
  } else if (activated()) {
    mech_state_ |= kBitReleased;  // Placeholder, see header.
  }
}

void ReleaseMechanism::driveActuator() {
  // **CHECK:** intentionally empty until the release hardware exists.
}
