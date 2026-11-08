#include "drivers/bmp581.hpp"

// Implementation of the Bmp581 wrapper declared in drivers/bmp581.hpp.
//
// The chip runs in NORMAL power mode at a fixed output data rate, so
// update() only reads the latest result registers and never waits for a
// conversion. Settings follow the electrical bench test
// (electrical/test/barometer-bmp581), which was checked on hardware.

bool Bmp581::begin(uint8_t addr, TwoWire& bus) {
  ok_ = false;
  if (!dev_.begin(addr, &bus)) return false;

  // Configuration registers are only safe to change in standby.
  bool cfg = dev_.setPowerMode(BMP5XX_POWERMODE_STANDBY);
  cfg &= dev_.setTemperatureOversampling(BMP5XX_OVERSAMPLING_2X);
  cfg &= dev_.setPressureOversampling(BMP5XX_OVERSAMPLING_16X);
  // A light IIR filter on the chip removes high-frequency noise without
  // adding much lag. The SensorManager does the main smoothing.
  // **CHECK:** the IIR setting was not part of the bench test. Confirm on
  // hardware that it doesn't delay apogee detection noticeably.
  cfg &= dev_.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);
  cfg &= dev_.setOutputDataRate(BMP5XX_ODR_10_HZ);
  cfg &= dev_.enablePressure(true);
  cfg &= dev_.setPowerMode(BMP5XX_POWERMODE_NORMAL);

  ok_ = cfg;
  fail_streak_ = 0;
  return ok_;
}

bool Bmp581::update() {
  if (!ok_ || !dev_.performReading()) {
    ++fail_total_;
    if (fail_streak_ < 255) ++fail_streak_;
    return false;
  }

  // The Adafruit library reports pressure in hPa. This wrapper uses Pa.
  pressure_pa_ = dev_.pressure * 100.0f;
  temp_c_ = dev_.temperature;
  fail_streak_ = 0;
  return true;
}
