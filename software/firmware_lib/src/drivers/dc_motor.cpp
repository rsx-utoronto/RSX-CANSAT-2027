#include "drivers/dc_motor.hpp"

#include <Arduino.h>

bool DcMotor::begin(int pin_a, int pin_b, uint32_t freq_hz, uint8_t bits) {
  attached_ = false;
  if (pin_a < 0 || pin_b < 0 || freq_hz == 0 || bits == 0 || bits > 20) return false;

  pin_a_ = pin_a;
  pin_b_ = pin_b;
  max_duty_ = (1UL << bits) - 1;

  attached_ = ledcAttach(static_cast<uint8_t>(pin_a_), freq_hz, bits) &&
              ledcAttach(static_cast<uint8_t>(pin_b_), freq_hz, bits);
  if (attached_) stop();
  return attached_;
}

void DcMotor::setOutput(float output) {
  if (!attached_) return;
  if (!isfinite(output)) output = 0.0f;  // Fail safe: coast on a bad value.
  if (output > 1.0f) output = 1.0f;
  if (output < -1.0f) output = -1.0f;
  output_ = output;

  const uint32_t duty = static_cast<uint32_t>(lroundf(fabsf(output) * max_duty_));
  if (output > 0.0f) {
    ledcWrite(static_cast<uint8_t>(pin_a_), duty);
    ledcWrite(static_cast<uint8_t>(pin_b_), 0);
  } else if (output < 0.0f) {
    ledcWrite(static_cast<uint8_t>(pin_a_), 0);
    ledcWrite(static_cast<uint8_t>(pin_b_), duty);
  } else {
    ledcWrite(static_cast<uint8_t>(pin_a_), 0);
    ledcWrite(static_cast<uint8_t>(pin_b_), 0);
  }
}
