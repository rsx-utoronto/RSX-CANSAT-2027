#pragma once

// SpinModel: the PocketQube's spin about body +z, from the camera pointing
// plan. Simulation and tuning only: the flight code never runs this model.
//
//   Izz * d(omega)/dt = b_delta * delta - d_omega * omega + tau_line + tau_d
//
//   q       = 0.5 * rho * V^2
//   b_delta = q * CL_alpha * S * r_bar            control term, per rad of twist
//   d_omega = q * CL_alpha * S * mean(r^2) / V    aerodynamic damping
//
// S is the area of both panels, r_bar the radius of their mid-span, and
// mean(r^2) the mean of r^2 across the span. The model is linear, so it is
// only valid below panel stall (about 10 deg of twist).
//
// tau_line and tau_d can each be switched off. Defaults come from the
// kModel... constants in config.hpp, where both are off for now.
//
// Units inside: SI (rad, rad/s, N*m).

#include "config.hpp"

struct SpinModelParams {
  // Aerodynamics and geometry (layout B by default).
  float air_density_kgm3   = FirmwareConfig::kModelAirDensity;
  float lift_slope_per_rad = FirmwareConfig::kModelLiftSlopePerRad;
  float panel_span_m       = FirmwareConfig::kModelPanelSpanM;
  float panel_chord_m      = FirmwareConfig::kModelPanelChordM;
  float root_radius_m      = FirmwareConfig::kModelPanelRootRadiusM;
  float izz_kgm2           = FirmwareConfig::kModelIzzKgM2;

  // tau_line: swivel friction, or riser twist when there is no swivel.
  bool  use_line_torque           = FirmwareConfig::kModelUseLineTorque;
  bool  has_swivel                = FirmwareConfig::kModelHasSwivel;
  float swivel_friction_nm        = FirmwareConfig::kModelSwivelFrictionNm;
  float riser_stiffness_nm_per_rad = FirmwareConfig::kModelRiserStiffnessNmPerRad;

  // tau_d: constant asymmetry plus a sinusoidal gust.
  bool  use_disturbance = FirmwareConfig::kModelUseDisturbance;
  float dist_bias_nm    = FirmwareConfig::kModelDistBiasNm;
  float dist_gust_nm    = FirmwareConfig::kModelDistGustNm;
  float dist_gust_hz    = FirmwareConfig::kModelDistGustHz;
};

class SpinModel {
public:
  explicit SpinModel(const SpinModelParams& params = SpinModelParams());

  // Start from a heading and spin rate.
  void reset(float psi_rad, float omega_rad_s);

  // Advance one time step.
  //   twist_rad       : panel twist delta (each panel, about its own axis)
  //   speed_mps       : descent speed V
  //   t_s             : current time (for the gust)
  //   panels_deployed : false before C6 deployment; stowed panels make no torque
  void step(float twist_rad, float speed_mps, float t_s, float dt_s, bool panels_deployed);

  float psiRad() const   { return psi_rad_; }
  float omegaRadS() const { return omega_rad_s_; }

  // Each torque term on its own, so a run can log them.
  float aeroTorqueNm(float twist_rad, float omega_rad_s, float speed_mps) const;
  float lineTorqueNm(float psi_rad, float omega_rad_s) const;
  float disturbanceTorqueNm(float t_s) const;

  // Derived numbers to compare with the fan rig.
  float controlGainNmPerRad(float speed_mps) const;  // b_delta
  float dampingNmsPerRad(float speed_mps) const;     // d_omega
  float steadySpinPerTwist(float speed_mps) const;   // b/d: deg/s of spin per deg of twist
  float responseTimeS(float speed_mps) const;        // Izz / d_omega

  const SpinModelParams& params() const { return p_; }

private:
  float dynamicPressure(float speed_mps) const;

  SpinModelParams p_;
  float area_m2_;       // S, both panels
  float r_bar_m_;       // mid-span radius
  float r2_mean_m2_;    // mean of r^2 across the span
  float psi_rad_     = 0.0f;
  float omega_rad_s_ = 0.0f;
};
