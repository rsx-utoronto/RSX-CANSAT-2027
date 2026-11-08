#include "sensor_manager.hpp"

#include <Arduino.h>
#include <Preferences.h>
#include <Wire.h>
#include <math.h>

using namespace FirmwareConfig;

namespace {

// NVS location of the saved launch-pad reference pressure.
constexpr const char* kNvsNamespace = "sensors";
constexpr const char* kNvsGroundKey = "ground_pa";

// Plausible pressure range in Pa, from about 9 km altitude up to a very high
// sea-level pressure. Anything outside this is a bad reading or a bad SIMP
// value and is ignored.
// **CHECK:** confirm the competition's simulation pressure profile stays
// inside this range, or valid SIMP values will be silently dropped.
constexpr float kMinValidPa = 30000.0f;
constexpr float kMaxValidPa = 110000.0f;

bool isValidPressure(float pa) {
  return pa >= kMinValidPa && pa <= kMaxValidPa;
}

}  // namespace

bool SensorManager::begin() {
  Wire.begin(kI2cSdaPin, kI2cSclPin, kI2cClockHz);

  // Each sensor is tried on its own, so one failure doesn't stop the other.
  const bool baro_up = baro_.begin(kBaroAddrPrimary, Wire) ||
                       baro_.begin(kBaroAddrSecondary, Wire);
  const bool power_up = power_.begin(kPowerMonitorAddr, kShuntOhms, Wire);

  // Restore the launch-pad reference from an earlier boot, if one was saved.
  // Without this, a reset mid-flight would measure altitude from sea level.
  // **CHECK:** only the reference pressure survives a reset. Max altitude
  // and the filter start empty, so apogee detection restarts from scratch.
  Preferences prefs;
  if (prefs.begin(kNvsNamespace, /*readOnly=*/true)) {
    const float saved = prefs.getFloat(kNvsGroundKey, NAN);
    prefs.end();
    if (isValidPressure(saved)) {
      ground_pa_ = saved;
      ground_calibrated_ = true;
    }
  }

  r_.baro_ok = baro_up;
  r_.power_ok = power_up;
  return baro_up && power_up;
}

void SensorManager::update(uint32_t now_ms, bool sim_on) {
  // Unsigned subtraction handles millis() wrapping around.
  if (now_ms - last_sample_ms_ < kSensorPeriodMs) return;
  last_sample_ms_ = now_ms;

  // --- Power monitor -------------------------------------------------------
  if (power_.update()) {
    r_.battery_v = power_.get_busVoltageV();
    const float ma = power_.get_currentA() * 1000.0f;
    // The packet field is unsigned. Negative current only happens through
    // noise near 0 or a wiring fault, so report it as 0.
    r_.battery_ma = ma <= 0.0f ? 0 : (ma >= 65535.0f ? 65535 : static_cast<uint16_t>(ma));
  }
  r_.power_ok = power_.get_health();

  // --- Barometer -----------------------------------------------------------
  // Temperature always comes from the real sensor, even in simulation.
  const bool baro_read = baro_.update();
  r_.baro_ok = baro_.get_health();
  if (baro_read) r_.temperature_c = baro_.get_temperatureC();

  // --- Altitude source -----------------------------------------------------
  if (sim_on) {
    // Use each SIMP value once. If the ground station stops sending, the
    // altitude holds instead of reusing a stale value as new samples.
    if (!sim_fresh_) return;
    sim_fresh_ = false;
    r_.pressure_pa = sim_pa_;
    pushAltitude(pressureToAltitude(static_cast<float>(sim_pa_)), now_ms);
    return;
  }

  if (!baro_read) return;
  const float pa = baro_.get_pressurePa();
  if (!isValidPressure(pa)) return;
  r_.pressure_pa = static_cast<uint32_t>(lroundf(pa));
  pushAltitude(pressureToAltitude(pa), now_ms);
}

bool SensorManager::calibrateGroundLevel(bool sim_on) {
  // Calibrate from the newest unfiltered pressure. The filtered altitude is
  // relative to the old reference, so it can't be used here.
  float pa = NAN;
  if (sim_on) {
    pa = static_cast<float>(sim_pa_);
  } else if (baro_.update()) {
    pa = baro_.get_pressurePa();
  }
  if (!isValidPressure(pa)) return false;

  ground_pa_ = pa;
  ground_calibrated_ = true;
  saveGroundReference();

  // Earlier samples were measured against the old reference, so drop them.
  resetAltitudeFilter();
  return true;
}

void SensorManager::setSimulatedPressure(uint32_t pa) {
  if (!isValidPressure(static_cast<float>(pa))) return;  // Ignore bad SIMP values.
  sim_pa_ = pa;
  sim_fresh_ = true;
}

void SensorManager::resetAltitudeFilter() {
  for (float& v : alt_window_) v = 0.0f;
  alt_sum_ = 0.0f;
  alt_idx_ = 0;
  alt_count_ = 0;
  have_prev_alt_ = false;
  r_.altitude_m = 0.0f;
  r_.max_altitude_m = 0.0f;
  r_.vertical_speed_mps = 0.0f;
}

float SensorManager::pressureToAltitude(float pa) const {
  // International barometric formula, with the launch-pad pressure as the
  // reference instead of sea level, so 0 m is the pad. Accurate to within
  // a few metres over the altitudes a CanSat reaches.
  return 44330.0f * (1.0f - powf(pa / ground_pa_, 1.0f / 5.255f));
}

void SensorManager::pushAltitude(float raw_alt_m, uint32_t now_ms) {
  // Moving average: replace the oldest sample and keep the running sum
  // updated. Until the window fills, average only the samples we have.
  if (alt_count_ == kAltitudeWindow) {
    alt_sum_ -= alt_window_[alt_idx_];
  } else {
    ++alt_count_;
  }
  alt_window_[alt_idx_] = raw_alt_m;
  alt_sum_ += raw_alt_m;
  alt_idx_ = static_cast<uint8_t>((alt_idx_ + 1) % kAltitudeWindow);

  const float alt = alt_sum_ / alt_count_;
  r_.altitude_m = alt;
  r_.sample_time_ms = now_ms;
  if (alt > r_.max_altitude_m) r_.max_altitude_m = alt;

  // Vertical speed: change in filtered altitude over time, then smoothed
  // with an exponential moving average, because differentiating amplifies
  // noise.
  if (have_prev_alt_) {
    const uint32_t dt_ms = now_ms - prev_alt_ms_;
    if (dt_ms > 0) {
      const float v = (alt - prev_alt_m_) * 1000.0f / static_cast<float>(dt_ms);
      r_.vertical_speed_mps += kVerticalSpeedAlpha * (v - r_.vertical_speed_mps);
    }
  }
  prev_alt_m_ = alt;
  prev_alt_ms_ = now_ms;
  have_prev_alt_ = true;
}

void SensorManager::saveGroundReference() {
  // Writes only on CAL, a few times per mission, so flash wear is not a
  // concern.
  Preferences prefs;
  if (prefs.begin(kNvsNamespace, /*readOnly=*/false)) {
    prefs.putFloat(kNvsGroundKey, ground_pa_);
    prefs.end();
  }
}
