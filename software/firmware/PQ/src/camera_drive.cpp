#include "camera_drive.hpp"

#include <Arduino.h>

#include "config.hpp"
#include "drivers/servo_pwm.hpp"

using namespace FirmwareConfig;

namespace {

ServoPwm camera_servo;

float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

}  // namespace

bool CameraDrive::begin() {
  target_deg_ = 0.0f;
  const ServoPwm::PulseMap map = {kCamServoMinUs, kCamServoMaxUs,
                                  kCamServoMinDeg, kCamServoMaxDeg};
  healthy_ = camera_servo.begin(kCamServoPin, map, kServoPwmHz, kServoPwmBits);
  if (healthy_) camera_servo.writeDeg(0.0f);  // Park at centre.
  return healthy_;
}

void CameraDrive::setTargetDeg(float deg) {
  if (!isfinite(deg)) return;  // Keep the last target on a bad value.
  target_deg_ = clampf(deg, kCamTravelMinDeg, kCamTravelMaxDeg);
}

void CameraDrive::update() {
  camera_servo.writeDeg(target_deg_);
  healthy_ = camera_servo.get_health();
}

float CameraDrive::get_measuredDeg() const {
  return NAN;
}
