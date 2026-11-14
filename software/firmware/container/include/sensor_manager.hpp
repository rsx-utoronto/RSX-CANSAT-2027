#pragma once

// SensorManager: the container's single point of access to its sensors.
//
// Responsibilities:
//   * Own and initialise the sensor drivers (BMP581 barometer, INA236
//     voltage/current monitor). A missing sensor is reported through the
//     health flags, never by blocking or halting.
//   * Turn raw readings into mission quantities: altitude above the launch
//     pad (smoothed), maximum altitude, and vertical speed.
//   * In simulation mode, use the pressure sent by the ground station (SIMP
//     command) instead of the real barometer.
//   * Persist the launch-pad reference pressure in flash (NVS), so altitude
//     stays correct after a mid-flight processor reset.
//
// Not its job: deciding the flight state (MissionManager), building or
// sending telemetry (the telemetry layer copies readings() into a
// ContainerPacket), or driving actuators.
//
// **CHECK:** nothing calls this class yet. main.cpp must call begin() in
// setup(), then update() and MissionManager::updateState() every loop. The
// host test test_boot_skeleton.py forbids I2C use in main.cpp, so update
// that test in the same change.
//
// **CHECK:** none of this has run on real hardware. Verify altitude,
// battery voltage and current against known values before flight.

#include <stdint.h>

#include "config.hpp"
#include "drivers/bmp581.hpp"
#include "drivers/ina236.hpp"

// One snapshot of everything the container measures. Units are in the names.
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

class SensorManager {
public:
  // Starts the I2C bus, initialises both sensors and loads the saved
  // launch-pad reference from flash. Returns true only if every sensor came
  // up; the manager still works with whatever did, so the caller may continue.
  bool begin();

  // Call every loop. Samples sensors at most every kSensorPeriodMs and
  // updates the filtered values.
  //   now_ms : current millis()
  //   sim_on : true while simulation mode is active (SIM_ON). Altitude then
  //            comes from setSimulatedPressure() instead of the barometer.
  void update(uint32_t now_ms, bool sim_on);

  const SensorReadings& readings() const { return r_; }

  // CAL command: makes the current pressure the zero-altitude reference and
  // saves it to flash. Uses the simulated pressure if sim_on is true.
  // Returns false if there is no valid pressure to calibrate with.
  // **CHECK:** CAL during simulation overwrites the saved launch-pad
  // reference with a simulated pressure. Run CAL again after leaving
  // simulation, or decide to block CAL while sim_on is true.
  bool calibrateGroundLevel(bool sim_on);

  // Launch-pad reference pressure in Pa, and whether one has been set (by
  // CAL now or in an earlier boot).
  float groundReferencePa() const { return ground_pa_; }
  bool  isGroundCalibrated() const { return ground_calibrated_; }

  // SIMP command: the next simulated pressure in Pa. Each value becomes one
  // altitude sample on the next update().
  void setSimulatedPressure(uint32_t pa);

  // Clears the altitude filter and max altitude. Call when entering or
  // leaving simulation mode, so real and simulated samples are never
  // averaged together.
  void resetAltitudeFilter();

private:
  // Altitude in metres of pressure pa above the launch-pad reference.
  float pressureToAltitude(float pa) const;

  // Feeds one raw altitude sample through the moving-average filter and
  // updates max altitude and vertical speed.
  void pushAltitude(float raw_alt_m, uint32_t now_ms);

  // Saves ground_pa_ to flash.
  void saveGroundReference();

  Bmp581 baro_;
  Ina236 power_;
  SensorReadings r_;

  float ground_pa_         = 101325.0f;  // Standard sea level until calibrated.
  bool  ground_calibrated_ = false;

  uint32_t sim_pa_    = 0;
  bool     sim_fresh_ = false;  // A SIMP value is waiting to be used.

  uint32_t last_sample_ms_ = 0;

  // Moving-average ring buffer. alt_sum_ is kept up to date so each sample
  // costs O(1).
  float   alt_window_[FirmwareConfig::kAltitudeWindow] = {};
  float   alt_sum_    = 0.0f;
  uint8_t alt_idx_    = 0;
  uint8_t alt_count_  = 0;

  // Previous filtered altitude and its time, used for vertical speed.
  float    prev_alt_m_    = 0.0f;
  uint32_t prev_alt_ms_   = 0;
  bool     have_prev_alt_ = false;
};
