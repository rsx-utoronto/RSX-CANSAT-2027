#pragma once

// ReleaseMechanism: the PocketQube release (CTR10) and its two sensors,
// "mechanism fully deployed" (CTR11) and "PQ cleared the rail" (CTR12).
//
// It produces the MECH_STATE telemetry bits:
//   bit 0  deployment mechanism activated
//   bit 1  PocketQube released
//
// **CHECK:** the actuator is not designed yet (servo? latch? burn wire?), so
// trigger() currently records the release WITHOUT driving any hardware. Put
// the real actuator code in driveActuator(). If it is a heat-based release
// (nichrome), M2 requires it not be exposed outside, and it must be switched
// off again after a bounded time.
//
// **CHECK:** while kPqClearedSensorPin is -1 (no sensor fitted), bit 1 is
// assumed to follow bit 0, i.e. "activated" is reported as "released". That
// is only a placeholder: CTR12 requires a real sensor.

#include <stdint.h>

class ReleaseMechanism {
public:
  static constexpr uint8_t kBitActivated = 0x01;
  static constexpr uint8_t kBitReleased  = 0x02;

  // Sets up sensor pins. restored_mech_state comes from the flight record:
  // if the release already happened before a reset, the actuator is driven
  // again so it is left in the released position.
  void begin(uint8_t restored_mech_state);

  // Releases the PocketQube. Safe to call more than once (MEC retries).
  void trigger();

  // Call every loop: reads the deployment sensors.
  void update();

  uint8_t mechState() const { return mech_state_; }
  bool activated() const { return (mech_state_ & kBitActivated) != 0; }

  // CTR11 sensor: true when the mechanism reports it is fully deployed.
  // Always false while no sensor is fitted. Printed in the debug output;
  // the guide's MECH_STATE has no bit for it.
  bool deploymentConfirmed() const;

private:
  void driveActuator();

  uint8_t mech_state_ = 0;
};
