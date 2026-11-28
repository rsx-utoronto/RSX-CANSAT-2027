#include "camera_drive.hpp"

#include <Arduino.h>
#include <Wire.h>

#include "config.hpp"
#include "drivers/as5600.hpp"
#include "drivers/dc_motor.hpp"
#include "heading_estimator.hpp"  // wrap180()

using namespace FirmwareConfig;

namespace {

As5600  encoder;
DcMotor motor;

// Past the travel limit by this much, cut the motor. It protects the wires
// across the joint if the loop or the encoder misbehaves.
constexpr float kOverTravelDeg = 5.0f;

float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

}  // namespace

bool CameraDrive::begin() {
  target_deg_ = 0.0f;
  integral_ = 0.0f;
  have_prev_ = false;

  // Wire must already be running (started by the PQ sensor code).
  const bool encoder_up = encoder.begin(Wire);
  const bool motor_up = motor.begin(kCamMotorPinA, kCamMotorPinB,
                                    kCamMotorPwmHz, kCamMotorPwmBits);
  healthy_ = encoder_up && motor_up;
  return healthy_;
}

void CameraDrive::setTargetDeg(float deg) {
  if (!isfinite(deg)) return;  // Keep the last target on a bad value.
  target_deg_ = clampf(deg, kCamTravelMinDeg, kCamTravelMaxDeg);
}

void CameraDrive::update(float dt_s) {
  if (!encoder.update()) {
    // No trustworthy angle: stop rather than drive blind.
    motor.stop();
    healthy_ = false;
    return;
  }
  healthy_ = motor.get_health() && encoder.get_health();

  // Camera angle relative to the body, in (-180, 180].
  measured_deg_ = wrap180(kCamEncoderSign * (encoder.get_angleDeg() - kCamEncoderZeroDeg));

  // Safety: outside the travel limits plus margin, cut the motor.
  if (measured_deg_ < kCamTravelMinDeg - kOverTravelDeg ||
      measured_deg_ > kCamTravelMaxDeg + kOverTravelDeg) {
    motor.stop();
    integral_ = 0.0f;
    return;
  }

  // Plain subtraction, not wrap180(): with limited travel the camera must
  // never take the "short way" through the +/-180 deg zone it can't reach.
  const float error = target_deg_ - measured_deg_;
  if (!(dt_s > 0.0f)) return;

  // Inside the deadband, stop driving so the motor doesn't buzz.
  if (fabsf(error) < kCamMotorDeadbandDeg) {
    motor.stop();
    have_prev_ = false;
    return;
  }

  // PID. The integral is clamped to the output limit (anti-windup), and the
  // derivative uses the measurement so a target step doesn't kick the motor.
  integral_ = clampf(integral_ + kCamMotorKi * error * dt_s,
                     -kCamMotorMaxOutput, kCamMotorMaxOutput);
  float derivative = 0.0f;
  if (have_prev_) derivative = -(measured_deg_ - prev_measured_deg_) / dt_s;
  prev_measured_deg_ = measured_deg_;
  have_prev_ = true;

  const float output = kCamMotorKp * error + integral_ + kCamMotorKd * derivative;
  motor.setOutput(clampf(output, -kCamMotorMaxOutput, kCamMotorMaxOutput));
}
