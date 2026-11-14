#pragma once

// HeadingEstimator: the PQ's heading about body +z, shared by the camera
// loop and the panel loop.
//
// Complementary filter: integrate the gyro every cycle, and pull the result
// slowly toward the boom magnetometer's heading. The gyro is smooth but
// drifts; the magnetometer doesn't drift but is noisy.
//
// Plain C++ with no Arduino.h, so it compiles on a laptop for tests.
//
// TODO: the caller (control_task) must pass NAN for the magnetometer
// heading until the boom detector (PQ33) fires. While the boom is stowed
// the magnetometer sits next to the motors.
// TODO: work out the magnetometer heading with tilt compensation (using the
// accelerometer), because the PQ swings under its parachute. The MMC5983MA
// driver isn't ported from electrical/test/mmc5983 yet.

#include <math.h>

#include "config.hpp"

// Wraps an angle to [-180, 180) degrees. Use it on every heading difference,
// so 350 deg to 10 deg counts as a 20 deg turn, not -340 deg.
inline float wrap180(float deg) {
  float w = fmodf(deg + 180.0f, 360.0f);
  if (w < 0.0f) w += 360.0f;
  return w - 180.0f;
}

// Wraps an angle to [0, 360) degrees, like a compass.
inline float wrap360(float deg) {
  float w = fmodf(deg, 360.0f);
  if (w < 0.0f) w += 360.0f;
  return w;
}

// Tunables, defaulting to config.hpp. Tests can pass different values.
struct HeadingEstimatorParams {
  float filter_tau_s  = FirmwareConfig::kHeadingFilterTauS;
  float gyro_bias_dps = FirmwareConfig::kGyroBiasZDefaultDps;
};

class HeadingEstimator {
public:
  using Params = HeadingEstimatorParams;

  explicit HeadingEstimator(const Params& params = Params()) : p_(params) {}

  // Set the gyro bias, e.g. the value measured on the ground and loaded
  // from NVS.
  void setGyroBiasDps(float bias_dps) { p_.gyro_bias_dps = bias_dps; }

  // Start from a known heading. Without a reset, the first valid
  // magnetometer reading sets the starting heading, or 0 if there is none.
  void reset(float heading_deg);

  // Call every control cycle.
  //   gyro_rate_z_dps : raw gyro z rate, already multiplied by kImuRateZSign
  //   mag_heading_deg : magnetometer heading, or NAN when it shouldn't be used
  //   dt_s            : time since the last call
  void update(float gyro_rate_z_dps, float mag_heading_deg, float dt_s);

  float headingDeg() const { return heading_deg_; }  // [0, 360)
  float rateDps() const    { return rate_dps_; }     // bias-corrected
  bool  initialized() const { return initialized_; }

private:
  Params p_;
  float heading_deg_ = 0.0f;
  float rate_dps_    = 0.0f;
  bool  initialized_ = false;
};
