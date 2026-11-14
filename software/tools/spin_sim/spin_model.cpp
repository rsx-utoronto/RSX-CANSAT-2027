#include "spin_model.hpp"

#include <math.h>

namespace {
// Smooths the swivel's friction near zero spin, so the simulation doesn't
// chatter between +friction and -friction.
constexpr float kFrictionSmoothingRadS = 0.05f;
constexpr float kTwoPi = 6.2831853f;
}  // namespace

SpinModel::SpinModel(const SpinModelParams& params) : p_(params) {
  const float r0 = p_.root_radius_m;
  const float r1 = p_.root_radius_m + p_.panel_span_m;
  area_m2_ = 2.0f * p_.panel_span_m * p_.panel_chord_m;
  r_bar_m_ = 0.5f * (r0 + r1);
  r2_mean_m2_ = (r1 * r1 * r1 - r0 * r0 * r0) / (3.0f * p_.panel_span_m);
}

void SpinModel::reset(float psi_rad, float omega_rad_s) {
  psi_rad_ = psi_rad;
  omega_rad_s_ = omega_rad_s;
}

float SpinModel::dynamicPressure(float speed_mps) const {
  return 0.5f * p_.air_density_kgm3 * speed_mps * speed_mps;
}

float SpinModel::controlGainNmPerRad(float speed_mps) const {
  return dynamicPressure(speed_mps) * p_.lift_slope_per_rad * area_m2_ * r_bar_m_;
}

float SpinModel::dampingNmsPerRad(float speed_mps) const {
  if (speed_mps <= 0.0f) return 0.0f;
  return dynamicPressure(speed_mps) * p_.lift_slope_per_rad * area_m2_ * r2_mean_m2_ /
         speed_mps;
}

float SpinModel::steadySpinPerTwist(float speed_mps) const {
  const float d = dampingNmsPerRad(speed_mps);
  return d > 0.0f ? controlGainNmPerRad(speed_mps) / d : 0.0f;
}

float SpinModel::responseTimeS(float speed_mps) const {
  const float d = dampingNmsPerRad(speed_mps);
  return d > 0.0f ? p_.izz_kgm2 / d : INFINITY;
}

float SpinModel::aeroTorqueNm(float twist_rad, float omega_rad_s, float speed_mps) const {
  return controlGainNmPerRad(speed_mps) * twist_rad - dampingNmsPerRad(speed_mps) * omega_rad_s;
}

float SpinModel::lineTorqueNm(float psi_rad, float omega_rad_s) const {
  if (!p_.use_line_torque) return 0.0f;
  if (p_.has_swivel) {
    // Friction opposes the spin. Assumes the canopy itself doesn't turn.
    return -p_.swivel_friction_nm * tanhf(omega_rad_s / kFrictionSmoothingRadS);
  }
  // No swivel: the risers twist up like a torsion spring around the
  // heading at release (psi = 0).
  return -p_.riser_stiffness_nm_per_rad * psi_rad;
}

float SpinModel::disturbanceTorqueNm(float t_s) const {
  if (!p_.use_disturbance) return 0.0f;
  return p_.dist_bias_nm + p_.dist_gust_nm * sinf(kTwoPi * p_.dist_gust_hz * t_s);
}

void SpinModel::step(float twist_rad, float speed_mps, float t_s, float dt_s,
                     bool panels_deployed) {
  float torque = lineTorqueNm(psi_rad_, omega_rad_s_) + disturbanceTorqueNm(t_s);
  if (panels_deployed) torque += aeroTorqueNm(twist_rad, omega_rad_s_, speed_mps);

  // Semi-implicit Euler: update the rate first, then the angle with it.
  omega_rad_s_ += torque / p_.izz_kgm2 * dt_s;
  psi_rad_ += omega_rad_s_ * dt_s;
}
