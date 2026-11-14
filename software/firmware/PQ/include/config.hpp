#pragma once

#include <stdint.h>

#include "firmware_boot.hpp"

// Tags used in this file and the PQ modules. Search for them to find every
// open item:
//   MECH:       a value or decision the mechanical subteam owns. Agree it
//               with mech before relying on it. Every MECH item is in
//               this file.
//   **CHECK:**  a starting guess to verify on the bench or in a test.
//   TODO:       work that still has to be done.

namespace FirmwareConfig {
constexpr FirmwareBoot::NodeRole kNodeRole = FirmwareBoot::NodeRole::PocketQube;

// ---------------------------------------------------------------------------
// Mission timing
// ---------------------------------------------------------------------------
// Solar panels deploy this long after power on (C6: five seconds after the
// PocketQube powers up).
constexpr uint32_t kPanelDeployDelayMs = 5000;

// ===========================================================================
// Spin control and camera pointing
// ===========================================================================
// Source: "PocketQube z-rotation and camera pointing plan".
//
// Axes: body +z is the long axis, pointing toward the camera (nadir) end.
// Positive z-rotation is clockwise seen from above, like compass heading.
// Units: angles in degrees and rates in deg/s, unless the name says otherwise.

// ---------------------------------------------------------------------------
// Loop timing
// ---------------------------------------------------------------------------
// The heading estimator, camera loop and panel loop all run at this period.
// TODO: write control_task.cpp, the 100 Hz FreeRTOS task that runs these
// loops. Until then only software/tools/spin_sim runs them.
constexpr uint32_t kControlPeriodMs = 10;  // 100 Hz

// ---------------------------------------------------------------------------
// IMU axis mapping
// ---------------------------------------------------------------------------
// MECH: how is the main board mounted in the PQ? The gyro's z axis must lie
// along the PQ's long axis. If the board is rotated, the axis mapping
// changes, not just this sign.
// **CHECK:** on the bench, turn the PQ clockwise seen from above. The gyro's
// z rate must then read positive; if it reads negative, set this to -1.
constexpr float kImuRateZSign = 1.0f;

// ---------------------------------------------------------------------------
// Heading estimator (complementary filter)
// ---------------------------------------------------------------------------
// How slowly the estimate is pulled toward the magnetometer. Shorter trusts
// the magnetometer more (noisier); longer trusts the gyro more (drifts).
constexpr float kHeadingFilterTauS = 8.0f;  // plan: 5 to 10 s
// Gyro z bias used when NVS holds no ground measurement. Measure the real
// bias over the umbilical (PQ5) before flight, because the PQ boots while
// falling and can't measure it then.
// TODO: measure the bias over the umbilical, save it to NVS, and load it
// at boot. None of that code exists yet.
constexpr float kGyroBiasZDefaultDps = 0.0f;

// ---------------------------------------------------------------------------
// Camera drive: choose ONE option
// ---------------------------------------------------------------------------
enum class CameraDriveType : uint8_t {
  Servo,    // Option A: positional hobby servo. No encoder needed. Start here.
  DcMotor,  // Option B: DC gearmotor + H-bridge + AS5600 absolute encoder.
};
// MECH: does a 270 deg positional servo fit the camera stage inside the 2P
// envelope? If yes, use Option A. If not, Option B.
constexpr CameraDriveType kCameraDrive = CameraDriveType::Servo;

// ---------------------------------------------------------------------------
// Camera pointing (used by both options)
// ---------------------------------------------------------------------------
// L in the plan: servo-plus-loop delay. The camera command leads the body's
// spin by rate_z * L so the picture doesn't lag.
constexpr float kCamLeadS = 0.040f;  // plan: 0.03 to 0.05 s
// Camera travel relative to the body.
// MECH: how far can the camera stage turn each way before it hits the
// structure or the wire service loop runs out? The 90 deg turn (C11) needs
// at least +/-135 deg (plan). With only +/-90 deg, see the re-centring TODO
// in camera_pointing.hpp.
// MECH: does the boom stay in the camera's view after the camera has turned
// 90 deg (C10, PQ12)?
constexpr float kCamTravelMinDeg = -135.0f;
constexpr float kCamTravelMaxDeg =  135.0f;

// ---------------------------------------------------------------------------
// Option A: camera servo
// ---------------------------------------------------------------------------
// **CHECK:** the PQ board isn't drawn yet. -1 means "not wired", and the
// drive then reports itself unhealthy instead of driving a random pin.
constexpr int kCamServoPin = -1;
// Pulse-width-to-angle map. **CHECK:** measure on the bench; datasheet
// ranges vary from part to part. These suit a typical 270 deg micro servo.
constexpr float kCamServoMinUs  = 500.0f;    // pulse at kCamServoMinDeg
constexpr float kCamServoMaxUs  = 2500.0f;   // pulse at kCamServoMaxDeg
constexpr float kCamServoMinDeg = -135.0f;
constexpr float kCamServoMaxDeg =  135.0f;

// ---------------------------------------------------------------------------
// Option B: camera DC gearmotor + AS5600 encoder
// ---------------------------------------------------------------------------
// MECH: mount a diametrically magnetized disc on the end of the camera
// shaft, 0.5 to 3 mm above the AS5600 chip, and keep any magnet as far from
// the boom magnetometer as possible.
// **CHECK:** pins unknown until the board is drawn. -1 means "not wired".
constexpr int      kCamMotorPinA    = -1;
constexpr int      kCamMotorPinB    = -1;
constexpr uint32_t kCamMotorPwmHz   = 20000;  // above hearing; check the H-bridge's limit
constexpr uint8_t  kCamMotorPwmBits = 10;
// Encoder reading when the camera points straight along the body (theta = 0),
// and +1 or -1 so a clockwise camera turn reads positive.
// **CHECK:** measure both after assembly.
constexpr float kCamEncoderZeroDeg = 0.0f;
constexpr float kCamEncoderSign    = 1.0f;
// Position loop: motor output (-1..1) per degree of camera error.
// **CHECK:** tune on the bench. Start with Ki = Kd = 0 and raise Kp slowly.
constexpr float kCamMotorKp = 0.05f;
constexpr float kCamMotorKi = 0.0f;
constexpr float kCamMotorKd = 0.0f;
constexpr float kCamMotorMaxOutput = 0.8f;   // fraction of full PWM
constexpr float kCamMotorDeadbandDeg = 0.3f; // stop driving inside this error
// The plan asks for the motor's position loop at 200 Hz or faster, which is
// faster than kControlPeriodMs. See the TODO in camera_drive.hpp.
constexpr uint32_t kCamMotorLoopHz = 200;

// ---------------------------------------------------------------------------
// Shared servo PWM settings (camera and panel servos)
// ---------------------------------------------------------------------------
constexpr uint32_t kServoPwmHz   = 50;
constexpr uint8_t  kServoPwmBits = 16;  // 0.31 us per count at 50 Hz

// ---------------------------------------------------------------------------
// Panel twist servo (the spin actuator)
// ---------------------------------------------------------------------------
constexpr int kPanelServoPin = -1;  // **CHECK:** not wired yet
// Servo degrees per panel degree.
// MECH: which reversing linkage (bevel gears or crossed pushrods), and what
// ratio? Plan: gear down 3:1 to 4:1.
constexpr float kPanelGearRatio = 3.5f;
// +1 or -1.
// MECH: the linkage's direction decides this sign.
// **CHECK:** after assembly, a positive twist command must spin the PQ
// clockwise seen from above. Flip this if it spins the other way.
constexpr float kPanelTwistSign = 1.0f;
// Servo angle that sets both panels flat (zero twist).
// MECH: trim screws at each panel root should zero the two panels to within
// about 0.2 deg of each other (plan). Set this after they are trimmed.
constexpr float kPanelServoTrimDeg = 0.0f;
// The servo's range must cover the hard stops times the gear ratio, e.g.
// 12 deg x 3.5 = 42 deg each way.
// **CHECK:** measure this servo's pulse map like the camera servo's.
constexpr float kPanelServoMinUs  = 1000.0f;
constexpr float kPanelServoMaxUs  = 2000.0f;
constexpr float kPanelServoMinDeg = -45.0f;
constexpr float kPanelServoMaxDeg =  45.0f;

// ---------------------------------------------------------------------------
// Spin controller (panel loop)
// ---------------------------------------------------------------------------
// Outer loop: heading error -> rate target.
constexpr float kSpinHeadingKp    = 1.5f;   // deg/s of rate target per deg of error
constexpr float kSpinRateLimitDps = 30.0f;  // plan: clamp the rate target to +/-30 deg/s
// Inner loop: rate error -> panel twist (PI with anti-windup).
constexpr float kSpinRateKp = 0.25f;  // deg of twist per deg/s of rate error
constexpr float kSpinRateKi = 0.25f;  // deg of twist per deg/s of error, per second
// MECH: this must stay inside the panels' hard stops (plan: +/-12 deg).
// Confirm the hard-stop angle once the linkage is designed.
constexpr float kSpinTwistLimitDeg = 10.0f;
// Gain scheduling: panel authority grows with V^2, so gains are tuned at
// this descent speed and scaled by (kSpinRefSpeedMps / V)^2, within limits.
// MECH: what descent speed will the PQ's parachute give? C13 allows 3 to
// 5 m/s.
constexpr float kSpinRefSpeedMps  = 4.0f;
constexpr float kSpinGainScaleMin = 0.5f;
constexpr float kSpinGainScaleMax = 2.0f;
// Lock-on: switch from damping to heading hold once |rate_z| stays below
// this for kLockHoldMs (plan's "Lock" phase).
constexpr float    kLockRateDps = 20.0f;
constexpr uint32_t kLockHoldMs  = 1000;

// ---------------------------------------------------------------------------
// Plant model: simulation and tuning only
// ---------------------------------------------------------------------------
// software/tools/spin_sim/ uses these. The flight code never runs the model.
//   Izz * d(omega)/dt = b_delta * delta - d_omega * omega + tau_line + tau_d
// TODO: replace these defaults with fits from fan-rig and drop-test logs.
constexpr float kModelAirDensity      = 1.15f;  // kg/m^3, hot summer field
// Lift slope of one panel.
// MECH: the fan rig (plan, build order step 5) measures the steady spin per
// degree of twist and the response time; fit this to them.
// **CHECK:** Helmbold estimate cut 15% for low Reynolds number.
constexpr float kModelLiftSlopePerRad = 2.6f;
// Panel geometry. These defaults are layout B (root on the short edge), so
// the span runs outward from the body.
// MECH: layout A (root on the long edge) or layout B? For layout A, swap
// span and chord. Final panel size and root position come from the CAD.
constexpr float kModelPanelSpanM       = 0.114f;
constexpr float kModelPanelChordM      = 0.045f;
constexpr float kModelPanelRootRadiusM = 0.030f;  // axis to panel root
// MECH: send the CAD mass properties of the deployed PQ now, then measure
// with a bifilar pendulum once it's built.
// Estimate: 450 g 50 x 50 mm body plus two 15 g panels at layout B.
constexpr float kModelIzzKgM2 = 4.5e-4f;

// Optional torque terms. Both are off for now, so the model is just the
// panels' control and damping terms. Turn them on to test robustness.
// tau_line: swivel friction, or riser twist if there is no swivel.
// MECH: swivel or not, and which swivel? Measure its friction torque under
// a 0.5 kg load.
constexpr bool  kModelUseLineTorque        = false;
constexpr bool  kModelHasSwivel            = true;
constexpr float kModelSwivelFrictionNm     = 0.5e-3f;
// MECH: only without a swivel: the riser lines' spacing and length set
// this stiffness (plan: 5 to 10 mN*m/rad).
constexpr float kModelRiserStiffnessNmPerRad = 7.5e-3f;
// tau_d: build asymmetry (a constant bias) plus a gust (a sine).
// MECH: how closely can the two panels be aligned? Any mismatch acts as a
// constant torque, which is part of this bias.
constexpr bool  kModelUseDisturbance = false;
constexpr float kModelDistBiasNm     = 1.0e-3f;
constexpr float kModelDistGustNm     = 1.0e-3f;
constexpr float kModelDistGustHz     = 0.8f;
}  // namespace FirmwareConfig
