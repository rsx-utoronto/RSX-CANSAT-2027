#include "heading_estimator.hpp"

void HeadingEstimator::reset(float heading_deg) {
  heading_deg_ = wrap360(heading_deg);
  initialized_ = true;
}

void HeadingEstimator::update(float gyro_rate_z_dps, float mag_heading_deg, float dt_s) {
  const bool mag_ok = isfinite(mag_heading_deg);

  // First call: start from the magnetometer if it's usable, else from 0.
  if (!initialized_) {
    heading_deg_ = mag_ok ? wrap360(mag_heading_deg) : 0.0f;
    initialized_ = true;
  }

  if (!isfinite(gyro_rate_z_dps) || !(dt_s > 0.0f)) return;  // Skip a bad sample.

  // Gyro step: integrate the bias-corrected rate.
  rate_dps_ = gyro_rate_z_dps - p_.gyro_bias_dps;
  heading_deg_ += rate_dps_ * dt_s;

  // Magnetometer correction: move a small fraction of the way toward the
  // magnetometer each cycle. Over filter_tau_s the gap shrinks by about 63%.
  if (mag_ok && p_.filter_tau_s > 0.0f) {
    float k = dt_s / p_.filter_tau_s;
    if (k > 1.0f) k = 1.0f;
    heading_deg_ += k * wrap180(mag_heading_deg - heading_deg_);
  }

  heading_deg_ = wrap360(heading_deg_);
}
