#pragma once
#include <stdint.h>
#include <Wire.h>

// AMS AS5600 12-bit absolute magnetic angle sensor, over I2C.
//
// A diametrically magnetized disc on the end of the shaft sits 0.5 to 3 mm
// above the chip. The chip reports the shaft angle at any moment, including
// right after power-up, so no homing is needed. 4096 counts per turn is
// 0.088 deg per count. The I2C address is fixed at 0x36.
//
// Only needed for camera drive Option B (DC gearmotor).
//
// **CHECK:** written from the datasheet and NOT run on hardware. Verify the
// register addresses and status bits against the AS5600 datasheet, then
// bench-test against a protractor.
class As5600 {
public:
  static constexpr uint8_t kAddress = 0x36;

  // Returns false if the chip doesn't answer. A chip that answers but sees
  // no magnet still returns true; check get_magnetDetected().
  bool begin(TwoWire& bus = Wire, uint8_t addr = kAddress);

  // Read the chip once. Returns false on a failed read or a missing magnet;
  // the last good angle is kept so callers never see garbage.
  bool update();

  float get_angleDeg()        const { return angle_deg_; }  // [0, 360)
  bool  get_magnetDetected()  const { return magnet_detected_; }
  bool  get_magnetTooWeak()   const { return magnet_too_weak_; }
  bool  get_magnetTooStrong() const { return magnet_too_strong_; }
  bool  get_health()          const { return ok_ && magnet_detected_ && fail_streak_ < 5; }
  uint32_t readFailures()     const { return fail_total_; }

private:
  bool readRegisters(uint8_t reg, uint8_t* out, uint8_t count);
  bool readStatus();
  void recordFailure();

  TwoWire* bus_  = nullptr;
  uint8_t  addr_ = kAddress;
  float    angle_deg_ = 0.0f;
  bool     magnet_detected_   = false;
  bool     magnet_too_weak_   = false;
  bool     magnet_too_strong_ = false;
  bool     ok_          = false;
  uint8_t  fail_streak_ = 0;
  uint32_t fail_total_  = 0;
};
