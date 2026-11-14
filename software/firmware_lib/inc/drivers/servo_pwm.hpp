#pragma once
#include <stdint.h>

// Positional hobby servo driven by the ESP32's LEDC PWM hardware.
//
// The servo's own controller holds the angle, so this only sets the pulse
// width: a pulse every 20 ms (50 Hz), whose width picks the angle.
// Arduino-ESP32 core 3 API: ledcAttach(pin, freq, bits) once, then
// ledcWrite(pin, duty).
//
// **CHECK:** not yet run on hardware. Measure each servo's pulse-to-angle
// map on the bench; datasheet ranges vary from part to part.
class ServoPwm {
public:
  // Pulse widths at the two ends of the servo's range.
  struct PulseMap {
    float min_us;   // pulse at min_deg
    float max_us;   // pulse at max_deg
    float min_deg;
    float max_deg;
  };

  // Returns false if pin is negative (not wired) or LEDC can't attach.
  // The caller decides what to do. Writes do nothing until this succeeds.
  bool begin(int pin, const PulseMap& map, uint32_t freq_hz = 50, uint8_t bits = 16);

  // Command an angle in degrees. Clamped to the map's range.
  void writeDeg(float deg);

  // Command a raw pulse width in microseconds, e.g. for bench calibration.
  void writeUs(float us);

  float get_commandDeg() const { return command_deg_; }
  bool  get_health()     const { return attached_; }

private:
  int      pin_         = -1;
  PulseMap map_         = {1000.0f, 2000.0f, -45.0f, 45.0f};
  float    period_us_   = 20000.0f;
  uint32_t max_duty_    = 0;
  float    command_deg_ = 0.0f;
  bool     attached_    = false;
};
