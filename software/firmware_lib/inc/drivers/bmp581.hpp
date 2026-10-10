#pragma once
#include <stdint.h>
#include <Wire.h>
#include <Adafruit_BMP5xx.h>

class Bmp581 {
public:
  // Returns false if the chip doesn't answer. Caller decides what to do.
  bool begin(uint8_t addr = BMP5XX_DEFAULT_ADDRESS, TwoWire& bus = Wire);

  // Read the chip once. Returns false on a failed read; the last good
  // values are kept so callers never see garbage.
  bool update();

  float pressurePa()   const { return pressure_pa_; }   // Pa, not hPa
  float temperatureC() const { return temp_c_; }
  bool  healthy()      const { return ok_ && fail_streak_ < 5; }
  uint32_t readFailures() const { return fail_total_; }

private:
  Adafruit_BMP5xx dev_;
  float    pressure_pa_ = 0.0f;
  float    temp_c_      = 0.0f;
  bool     ok_          = false;
  uint8_t  fail_streak_ = 0;
  uint32_t fail_total_  = 0;
};