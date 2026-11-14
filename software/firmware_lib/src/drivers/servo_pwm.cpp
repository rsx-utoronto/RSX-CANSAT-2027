#include "drivers/servo_pwm.hpp"

#include <Arduino.h>

bool ServoPwm::begin(int pin, const PulseMap& map, uint32_t freq_hz, uint8_t bits) {
  attached_ = false;
  if (pin < 0 || freq_hz == 0 || bits == 0 || bits > 20) return false;
  if (map.max_deg <= map.min_deg) return false;

  pin_ = pin;
  map_ = map;
  period_us_ = 1e6f / static_cast<float>(freq_hz);
  max_duty_ = (1UL << bits) - 1;

  attached_ = ledcAttach(static_cast<uint8_t>(pin_), freq_hz, bits);
  return attached_;
}

void ServoPwm::writeDeg(float deg) {
  if (!isfinite(deg)) return;  // Keep the last command on a bad value.
  if (deg < map_.min_deg) deg = map_.min_deg;
  if (deg > map_.max_deg) deg = map_.max_deg;
  command_deg_ = deg;

  // Straight line between the two measured end points.
  const float fraction = (deg - map_.min_deg) / (map_.max_deg - map_.min_deg);
  writeUs(map_.min_us + fraction * (map_.max_us - map_.min_us));
}

void ServoPwm::writeUs(float us) {
  if (!attached_ || !isfinite(us) || us < 0.0f) return;
  // Duty is the fraction of the 20 ms period the pulse is high.
  const float duty = us / period_us_ * static_cast<float>(max_duty_);
  ledcWrite(static_cast<uint8_t>(pin_), static_cast<uint32_t>(lroundf(duty)));
}
