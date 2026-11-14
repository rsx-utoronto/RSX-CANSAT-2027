#pragma once
#include <stdint.h>

// Brushed DC motor through a two-input H-bridge (DRV8833 style: one PWM
// input per direction), driven by the ESP32's LEDC PWM hardware.
//
//   output > 0 : PWM on pin A, pin B low  -> one direction
//   output < 0 : PWM on pin B, pin A low  -> the other direction
//   output = 0 : both low                 -> coast
//
// Only needed for camera drive Option B (DC gearmotor).
//
// **CHECK:** not run on hardware, and the H-bridge part isn't chosen yet.
// Confirm its input truth table matches the one above, or change
// setOutput() to match the chosen part.
class DcMotor {
public:
  // Returns false if either pin is negative (not wired) or LEDC can't
  // attach. Outputs do nothing until this succeeds.
  bool begin(int pin_a, int pin_b, uint32_t freq_hz, uint8_t bits);

  // Drive at a fraction of full power, -1 to 1. Clamped.
  void setOutput(float output);

  // Both inputs low: the motor coasts.
  void stop() { setOutput(0.0f); }

  float get_output() const { return output_; }
  bool  get_health() const { return attached_; }

private:
  int      pin_a_    = -1;
  int      pin_b_    = -1;
  uint32_t max_duty_ = 0;
  float    output_   = 0.0f;
  bool     attached_ = false;
};
