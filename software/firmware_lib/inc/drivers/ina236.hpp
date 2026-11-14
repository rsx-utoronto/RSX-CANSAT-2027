#pragma once

#include <stdint.h>
#include <Wire.h>

// Minimal driver for the TI INA236 voltage/current monitor.
//
// The chip runs in its continuous shunt + bus mode, so update() only reads
// the latest result registers and never waits on a conversion. Current is
// computed directly from the shunt voltage (I = V / R). That avoids the
// calibration register, so a wrong calibration value can't corrupt readings.
//
// With the default ADC range (+/-81.92 mV full scale) and a 0.1 ohm shunt,
// the maximum measurable current is about 819 mA.
// **CHECK:** measure the container's real peak current (servos, radio
// bursts). If it can exceed ~800 mA, readings will clip; use a smaller shunt.
//
// **CHECK:** this driver was written from the datasheet and has NOT been run
// on hardware. Bench-test it against a multimeter before trusting it.
class Ina236 {
public:
  // Resets and configures the chip. Returns false if it doesn't answer or
  // doesn't identify as a TI part. The caller decides what to do.
  bool begin(uint8_t addr, float shunt_ohms, TwoWire& bus = Wire);

  // Read the chip once. Returns false on a failed read; the last good
  // values are kept so callers never see garbage.
  bool update();

  float    get_busVoltageV() const { return bus_v_; }
  float    get_currentA()    const { return current_a_; }   // signed
  bool     get_health()      const { return ok_ && fail_streak_ < 5; }
  uint32_t readFailures()    const { return fail_total_; }

private:
  bool readRegister(uint8_t reg, uint16_t& value);
  bool writeRegister(uint8_t reg, uint16_t value);

  TwoWire* bus_        = nullptr;
  uint8_t  addr_       = 0;
  float    shunt_ohms_ = 0.1f;
  float    bus_v_      = 0.0f;
  float    current_a_  = 0.0f;
  bool     ok_         = false;
  uint8_t  fail_streak_ = 0;
  uint32_t fail_total_  = 0;
};
