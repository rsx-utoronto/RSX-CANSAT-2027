#include "spin_controller.hpp"

#include <math.h>

#include "heading_estimator.hpp"  // wrap180()

namespace {
float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}
}  // namespace

void SpinController::setMode(Mode mode) {
  if (mode != mode_) reset();
  mode_ = mode;
}

void SpinController::reset() {
  integral_deg_ = 0.0f;
  twist_deg_ = 0.0f;
  rate_target_dps_ = 0.0f;
}

float SpinController::gainScaleFor(float descent_speed_mps) const {
  // No usable speed: keep the reference gains.
  if (!isfinite(descent_speed_mps) || descent_speed_mps < 0.5f) return 1.0f;
  const float ratio = p_.ref_speed_mps / descent_speed_mps;
  return clampf(ratio * ratio, p_.gain_scale_min, p_.gain_scale_max);
}

float SpinController::update(float target_heading_deg, float body_heading_deg,
                             float rate_z_dps, float descent_speed_mps, float dt_s) {
  if (mode_ == Mode::Off || !isfinite(rate_z_dps) || !(dt_s > 0.0f)) {
    if (mode_ == Mode::Off) reset();
    return twist_deg_;
  }

  // Outer loop: in Hold, turn heading error into a rate target.
  rate_target_dps_ = 0.0f;
  if (mode_ == Mode::Hold && isfinite(target_heading_deg) && isfinite(body_heading_deg)) {
    const float heading_error = wrap180(target_heading_deg - body_heading_deg);
    rate_target_dps_ = clampf(p_.heading_kp * heading_error,
                              -p_.rate_limit_dps, p_.rate_limit_dps);
  }

  // Inner loop: PI on rate error, both terms scaled for descent speed.
  gain_scale_ = gainScaleFor(descent_speed_mps);
  const float rate_error = rate_target_dps_ - rate_z_dps;
  const float p_term = gain_scale_ * p_.rate_kp * rate_error;
  const float new_integral = integral_deg_ + gain_scale_ * p_.rate_ki * rate_error * dt_s;
  const float unclamped = p_term + new_integral;

  // Anti-windup: only keep the new integral if the output isn't saturated,
  // or if the integral is moving back out of saturation.
  const bool saturated_high = unclamped > p_.twist_limit_deg;
  const bool saturated_low  = unclamped < -p_.twist_limit_deg;
  const bool winding_up = (saturated_high && rate_error > 0.0f) ||
                          (saturated_low && rate_error < 0.0f);
  if (!winding_up) integral_deg_ = new_integral;

  twist_deg_ = clampf(p_term + integral_deg_, -p_.twist_limit_deg, p_.twist_limit_deg);
  return twist_deg_;
}
