#pragma once

// SpinController: the slow panel loop. Keeps the body's spin slow and its
// heading near the target, so the camera never runs out of travel.
//
//   outer loop: heading error -> rate target   (P, clamped to +/-30 deg/s)
//   inner loop: rate error    -> panel twist   (PI with anti-windup,
//                                               clamped to +/-10 deg)
//
// A degree of twist sets a spin RATE, not an angle, which is why the inner
// loop controls rate. Gains are scaled by (V_ref / V)^2 because the panels'
// authority grows with the square of descent speed.
//
// It only computes the twist.
// Plain C++ with no Arduino.h, so it compiles on a laptop for tests.
//
// TODO: control_task (not written yet) must turn twistDeg() into a servo
// angle and write it to the panel servo:
//   servo deg = kPanelServoTrimDeg + kPanelTwistSign * kPanelGearRatio * twist
// TODO: descent_speed_mps must come from the PQ's barometer, but the PQ
// sensor_manager is still a placeholder.
// TODO: send twistDeg(), rateTargetDps() and gainScale() in the
// IMAGE_STABILIZATION telemetry fields. PQPacket in protocol.hpp still has a
// 32-byte text placeholder there.

#include <stdint.h>

#include "config.hpp"

// Tunables, defaulting to config.hpp. Tests can pass different values.
struct SpinControllerParams {
  float heading_kp       = FirmwareConfig::kSpinHeadingKp;
  float rate_limit_dps   = FirmwareConfig::kSpinRateLimitDps;
  float rate_kp          = FirmwareConfig::kSpinRateKp;
  float rate_ki          = FirmwareConfig::kSpinRateKi;
  float twist_limit_deg  = FirmwareConfig::kSpinTwistLimitDeg;
  float ref_speed_mps    = FirmwareConfig::kSpinRefSpeedMps;
  float gain_scale_min   = FirmwareConfig::kSpinGainScaleMin;
  float gain_scale_max   = FirmwareConfig::kSpinGainScaleMax;
};

class SpinController {
public:
  enum class Mode : uint8_t {
    Off,   // panels flat: release phase, or after landing
    Damp,  // rate target 0: slow the spin after the panels deploy
    Hold,  // heading hold: after lock-on
  };

  using Params = SpinControllerParams;

  explicit SpinController(const Params& params = Params()) : p_(params) {}

  // Changing mode clears the integrator, so old error doesn't carry over.
  void setMode(Mode mode);
  Mode mode() const { return mode_; }

  // Clears the integrator and the last output.
  void reset();

  // Call every control cycle. Returns the panel twist command in degrees.
  //   target_heading_deg : heading to hold (used in Hold mode only)
  //   body_heading_deg   : from HeadingEstimator
  //   rate_z_dps         : from HeadingEstimator
  //   descent_speed_mps  : from the barometer; NAN if unknown (no scaling)
  //   dt_s               : time since the last call
  float update(float target_heading_deg, float body_heading_deg, float rate_z_dps,
               float descent_speed_mps, float dt_s);

  // For telemetry (IMAGE_STABILIZATION fields) and tests.
  float twistDeg() const       { return twist_deg_; }
  float rateTargetDps() const  { return rate_target_dps_; }
  float gainScale() const      { return gain_scale_; }

private:
  float gainScaleFor(float descent_speed_mps) const;

  Params p_;
  Mode  mode_            = Mode::Off;
  float integral_deg_    = 0.0f;
  float twist_deg_       = 0.0f;
  float rate_target_dps_ = 0.0f;
  float gain_scale_      = 1.0f;
};
