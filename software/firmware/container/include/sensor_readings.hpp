#pragma once

#include <stdint.h>

// One snapshot of everything the container measures. Units are in the names.
//
// This lives in its own header (instead of sensor_manager.hpp) so the mission
// logic can use it without pulling in Arduino or I2C headers. That keeps
// MissionManager compilable by the host tests in software/tests.
struct SensorReadings {
  float    altitude_m         = 0.0f;  // Smoothed, metres above the launch pad.
  float    max_altitude_m     = 0.0f;  // Highest smoothed altitude seen.
  float    vertical_speed_mps = 0.0f;  // Positive when climbing.
  uint32_t pressure_pa        = 0;     // Real or simulated, whichever is in use.
  float    temperature_c      = 0.0f;  // Barometer die temperature.
  float    battery_v          = 0.0f;
  uint16_t battery_ma         = 0;     // Clamped to 0 if current reads negative.
  bool     baro_ok            = false; // Barometer answering and reading.
  bool     power_ok           = false; // INA236 answering and reading.
  uint32_t sample_time_ms     = 0;     // millis() of the last altitude sample.
};
