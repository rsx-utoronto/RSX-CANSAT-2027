// spin_sim: runs the real PQ control modules (HeadingEstimator,
// SpinController, CameraPointing) against SpinModel, then checks the plan's
// targets: lock-on, picture held within +/-10 deg (C8), and the 90 deg turn
// (C11) settling within 2 s.
//
// Build and run from the RSX-CANSAT-2027 folder:
//
//   c++ -std=c++17 -O2 -Wall \
//     -Isoftware/firmware/PQ/include -Isoftware/firmware_lib/inc -Isoftware/tools/spin_sim \
//     software/tools/spin_sim/spin_sim.cpp software/tools/spin_sim/spin_model.cpp \
//     software/firmware/PQ/src/heading_estimator.cpp \
//     software/firmware/PQ/src/spin_controller.cpp \
//     software/firmware/PQ/src/camera_pointing.cpp \
//     -o spin_sim
//
//   ./spin_sim                  tau_line and tau_d as set in config.hpp (off for now)
//   ./spin_sim --line --dist    switch both on for this run
//
// Options:
//   --line / --no-line   swivel friction (or riser twist) on or off
//   --no-swivel          with --line: risers twist like a spring instead
//   --dist / --no-dist   asymmetry + gust disturbance on or off
//   --speed <m/s>        descent speed (default 4)
//   --spin0 <deg/s>      spin rate at release (default 90)
//   --turn <deg>         commanded camera turn at 30 s (default 90)
//   --lift-scale <x>     multiply the panels' lift slope, e.g. 0.6 or 1.4
//   --delay <s>          servo delay for both servos (default 0.02)
//   --backlash <deg>     panel backlash, measured at the panel (default 0.29)
//   --csv <file>         write every control cycle to a CSV file
//
// The plan's robustness check (build order step 2):
//   ./spin_sim --spin0 300 --delay 0.05 --backlash 1 --lift-scale 0.6
//   ./spin_sim --spin0 300 --delay 0.05 --backlash 1 --lift-scale 1.4
//
// Gains and limits come from config.hpp, so tuning there changes this run.
// Exit code 0 means every target passed.
//
// TODO: the model still lacks parachute swing, camera-servo deadband,
// sensor faults and random variation between runs (Monte Carlo). Add them
// before trusting a pass.
// TODO: fit the model to fan-rig and drop-test logs, then update the
// kModel... values in config.hpp.

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <deque>
#include <random>
#include <vector>

#include "camera_pointing.hpp"
#include "config.hpp"
#include "heading_estimator.hpp"
#include "spin_controller.hpp"
#include "spin_model.hpp"

using namespace FirmwareConfig;

namespace {

constexpr float kDegPerRad = 57.2957795f;
constexpr float kRadPerDeg = 1.0f / kDegPerRad;

// The world and the hardware's imperfections. Simulation only.
struct SimSettings {
  float speed_mps         = 4.0f;    // descent speed (C13: 3 to 5 m/s)
  float spin0_dps         = 90.0f;   // spin at release
  float t_end_s           = 60.0f;
  float t_mag_s           = 8.0f;    // boom out: magnetometer usable from here
  float t_turn_s          = 30.0f;   // CAM command (C11)
  float turn_deg          = 90.0f;
  float gyro_bias_err_dps = 0.1f;    // bias left over after ground calibration
  float gyro_noise_dps    = 0.2f;
  float mag_noise_deg     = 2.0f;
  float panel_delay_s     = 0.02f;
  float panel_slew_dps    = 600.0f;  // at the servo; divided by the gear ratio at the panel
  float panel_backlash_deg = 1.0f;   // at the servo
  float cam_delay_s       = 0.02f;
  float cam_slew_dps      = 600.0f;
  const char* csv_path    = nullptr;
};

// A servo seen from outside: it starts moving after a delay, can't move
// faster than its slew rate, and has slop (backlash) in its gears.
class Actuator {
public:
  Actuator(float delay_s, float slew_dps, float backlash_deg, float dt_s)
      : slew_step_(slew_dps * dt_s), half_backlash_(0.5f * backlash_deg) {
    const int steps = static_cast<int>(lroundf(delay_s / dt_s));
    delay_line_.assign(steps > 0 ? steps : 0, 0.0f);
  }

  float step(float command) {
    delay_line_.push_back(command);
    const float delayed = delay_line_.front();
    delay_line_.pop_front();

    // Slew limit at the motor.
    const float gap = delayed - motor_;
    motor_ += gap > slew_step_ ? slew_step_ : (gap < -slew_step_ ? -slew_step_ : gap);

    // Backlash: the output only moves once the motor takes up the slop.
    if (motor_ - output_ > half_backlash_) output_ = motor_ - half_backlash_;
    if (output_ - motor_ > half_backlash_) output_ = motor_ + half_backlash_;
    return output_;
  }

private:
  std::deque<float> delay_line_;
  float slew_step_;
  float half_backlash_;
  float motor_  = 0.0f;
  float output_ = 0.0f;
};

bool parseArgs(int argc, char** argv, SimSettings& sim, SpinModelParams& model) {
  for (int i = 1; i < argc; ++i) {
    const char* a = argv[i];
    const bool has_value = i + 1 < argc;
    if (!strcmp(a, "--line"))              model.use_line_torque = true;
    else if (!strcmp(a, "--no-line"))      model.use_line_torque = false;
    else if (!strcmp(a, "--no-swivel"))    model.has_swivel = false;
    else if (!strcmp(a, "--dist"))         model.use_disturbance = true;
    else if (!strcmp(a, "--no-dist"))      model.use_disturbance = false;
    else if (!strcmp(a, "--speed") && has_value) sim.speed_mps = strtof(argv[++i], nullptr);
    else if (!strcmp(a, "--spin0") && has_value) sim.spin0_dps = strtof(argv[++i], nullptr);
    else if (!strcmp(a, "--turn") && has_value)  sim.turn_deg = strtof(argv[++i], nullptr);
    else if (!strcmp(a, "--lift-scale") && has_value) {
      model.lift_slope_per_rad *= strtof(argv[++i], nullptr);
    } else if (!strcmp(a, "--delay") && has_value) {
      sim.panel_delay_s = sim.cam_delay_s = strtof(argv[++i], nullptr);
    } else if (!strcmp(a, "--backlash") && has_value) {
      sim.panel_backlash_deg = strtof(argv[++i], nullptr) * kPanelGearRatio;  // given at the panel
    }
    else if (!strcmp(a, "--csv") && has_value)   sim.csv_path = argv[++i];
    else {
      fprintf(stderr, "unknown option: %s\n", a);
      return false;
    }
  }
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  SimSettings sim;
  SpinModelParams model_params;
  if (!parseArgs(argc, argv, sim, model_params)) return 2;

  SpinModel model(model_params);
  const float v = sim.speed_mps;

  printf("Model at %.1f m/s: tau_line %s, tau_d %s\n", v,
         model_params.use_line_torque ? (model_params.has_swivel ? "ON (swivel)" : "ON (risers)") : "off",
         model_params.use_disturbance ? "ON" : "off");
  printf("  torque at 10 deg twist   %6.2f mN*m\n",
         model.controlGainNmPerRad(v) * 10.0f * kRadPerDeg * 1000.0f);
  printf("  steady spin per 1 deg    %6.1f deg/s\n", model.steadySpinPerTwist(v));
  printf("  response time            %6.2f s\n", model.responseTimeS(v));

  // ---- Set up -------------------------------------------------------------
  const float dt = 0.001f;                                    // physics step
  const int   ctrl_every = static_cast<int>(kControlPeriodMs); // 1 ms steps per control cycle
  const float ctrl_dt = kControlPeriodMs / 1000.0f;
  const float deploy_s = kPanelDeployDelayMs / 1000.0f;

  model.reset(0.0f, sim.spin0_dps * kRadPerDeg);
  HeadingEstimator estimator;
  SpinController spin;
  CameraPointing camera;
  Actuator panel_servo(sim.panel_delay_s, sim.panel_slew_dps, sim.panel_backlash_deg, dt);
  Actuator camera_servo(sim.cam_delay_s, sim.cam_slew_dps, 0.0f, dt);

  std::mt19937 rng(1);
  std::normal_distribution<float> unit(0.0f, 1.0f);

  FILE* csv = sim.csv_path ? fopen(sim.csv_path, "w") : nullptr;
  if (csv) {
    fprintf(csv, "t_s,psi_true_deg,rate_true_dps,heading_est_deg,rate_est_dps,mode,"
                 "twist_cmd_deg,twist_deg,cam_cmd_deg,cam_deg,picture_err_deg,"
                 "tau_aero_mnm,tau_line_mnm,tau_d_mnm\n");
  }

  // ---- Run ----------------------------------------------------------------
  bool  locked = false;
  float lock_time_s = NAN;
  float low_rate_since_s = -1.0f;
  float body_target_deg = 0.0f;
  float true_picture_at_lock_deg = 0.0f;

  float twist_cmd_deg = 0.0f, cam_cmd_deg = 0.0f;
  float twist_deg = 0.0f, cam_deg = 0.0f;
  float max_hold_err = 0.0f, max_twist = 0.0f, max_cam = 0.0f;
  int   saturated_cycles = 0;
  std::vector<float> after_turn_t, after_turn_err;

  const int total_steps = static_cast<int>(sim.t_end_s / dt);
  for (int n = 0; n < total_steps; ++n) {
    const float t = n * dt;
    const bool deployed = t >= deploy_s;

    // -- Control cycle (100 Hz) --
    if (n % ctrl_every == 0) {
      const float psi_true = model.psiRad() * kDegPerRad;
      const float rate_true = model.omegaRadS() * kDegPerRad;

      // Sensors: gyro with leftover bias and noise; magnetometer once the boom is out.
      const float gyro = rate_true + sim.gyro_bias_err_dps + sim.gyro_noise_dps * unit(rng);
      const float mag = t >= sim.t_mag_s ? wrap360(psi_true + sim.mag_noise_deg * unit(rng)) : NAN;
      estimator.update(gyro, mag, ctrl_dt);
      const float heading = estimator.headingDeg();
      const float rate = estimator.rateDps();

      // Phases from the plan: Release -> Damp -> Lock (-> Turn).
      if (!deployed) {
        spin.setMode(SpinController::Mode::Off);
        camera.park();
      } else if (!locked) {
        spin.setMode(SpinController::Mode::Damp);
        if (fabsf(rate) < kLockRateDps) {
          if (low_rate_since_s < 0.0f) low_rate_since_s = t;
          if ((t - low_rate_since_s) * 1000.0f >= kLockHoldMs) {
            camera.lock(heading);
            body_target_deg = heading;  // Body holds here; the camera absorbs the turn.
            spin.setMode(SpinController::Mode::Hold);
            locked = true;
            lock_time_s = t;
            true_picture_at_lock_deg = psi_true + cam_deg;
          }
        } else {
          low_rate_since_s = -1.0f;
        }
      }

      const float offset = t >= sim.t_turn_s ? sim.turn_deg : 0.0f;
      camera.setTargetOffsetDeg(offset);
      twist_cmd_deg = spin.update(body_target_deg, heading, rate, sim.speed_mps, ctrl_dt);
      cam_cmd_deg = camera.update(heading, rate);

      // Scoring uses the TRUE picture, so estimator drift counts against us.
      float picture_err = NAN;
      if (locked) {
        picture_err = wrap180(psi_true + cam_deg - (true_picture_at_lock_deg + offset));
        if (t >= lock_time_s + 2.0f && t < sim.t_turn_s) {
          max_hold_err = fmaxf(max_hold_err, fabsf(picture_err));
        }
        if (t >= sim.t_turn_s) {
          after_turn_t.push_back(t);
          after_turn_err.push_back(picture_err);
        }
        if (camera.saturated()) ++saturated_cycles;
      }
      max_twist = fmaxf(max_twist, fabsf(twist_cmd_deg));
      max_cam = fmaxf(max_cam, fabsf(cam_cmd_deg));

      if (csv) {
        fprintf(csv, "%.3f,%.2f,%.2f,%.2f,%.2f,%d,%.3f,%.3f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f\n",
                t, psi_true, rate_true, heading, rate, static_cast<int>(spin.mode()),
                twist_cmd_deg, twist_deg, cam_cmd_deg, cam_deg, picture_err,
                model.aeroTorqueNm(twist_deg * kRadPerDeg, model.omegaRadS(), v) * 1000.0f,
                model.lineTorqueNm(model.psiRad(), model.omegaRadS()) * 1000.0f,
                model.disturbanceTorqueNm(t) * 1000.0f);
      }
    }

    // -- Physics (1 kHz) --
    twist_deg = panel_servo.step(twist_cmd_deg * kPanelGearRatio) / kPanelGearRatio;
    cam_deg = camera_servo.step(cam_cmd_deg);
    model.step(twist_deg * kRadPerDeg, sim.speed_mps, t, dt, deployed);
  }
  if (csv) fclose(csv);

  // ---- Score the turn -----------------------------------------------------
  // Settled = the last moment the picture error was outside +/-10 deg.
  float settle_s = 0.0f;
  for (size_t i = 0; i < after_turn_t.size(); ++i) {
    if (fabsf(after_turn_err[i]) > 10.0f) settle_s = after_turn_t[i] - sim.t_turn_s;
  }
  // Hold after the turn, scored from the 2 s settle target onward, so the
  // camera's own slewing during the turn doesn't count.
  float max_after_settle = 0.0f;
  for (size_t i = 0; i < after_turn_t.size(); ++i) {
    if (after_turn_t[i] - sim.t_turn_s >= 2.0f) {
      max_after_settle = fmaxf(max_after_settle, fabsf(after_turn_err[i]));
    }
  }

  // ---- Report -------------------------------------------------------------
  const bool lock_ok   = locked && lock_time_s < sim.t_turn_s - 3.0f;
  const bool hold_ok   = lock_ok && max_hold_err <= 10.0f;
  const bool turn_ok   = lock_ok && settle_s <= 2.0f;
  const bool after_ok  = lock_ok && max_after_settle <= 10.0f;

  printf("Run: release spin %.0f deg/s, turn %+.0f deg at %.0f s\n",
         sim.spin0_dps, sim.turn_deg, sim.t_turn_s);
  printf("  %s lock-on at %.1f s (needs to be before %.0f s)\n",
         lock_ok ? "PASS" : "FAIL", lock_time_s, sim.t_turn_s - 3.0f);
  printf("  %s picture held within %.1f deg before the turn (limit 10)\n",
         hold_ok ? "PASS" : "FAIL", max_hold_err);
  printf("  %s turn settled in %.2f s (target 2.0)\n", turn_ok ? "PASS" : "FAIL", settle_s);
  printf("  %s picture held within %.1f deg from 2 s after the turn (limit 10)\n",
         after_ok ? "PASS" : "FAIL", max_after_settle);
  printf("  largest panel twist %.1f deg, largest camera angle %.1f deg, "
         "camera at a travel limit for %d cycles\n", max_twist, max_cam, saturated_cycles);

  return (lock_ok && hold_ok && turn_ok && after_ok) ? 0 : 1;
}
