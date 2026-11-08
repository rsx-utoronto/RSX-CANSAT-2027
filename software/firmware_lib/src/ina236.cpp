#include "drivers/ina236.hpp"

namespace {

// Register map (INA236 datasheet, section 7.6).
// **CHECK:** verify every address and constant in this block against the
// INA236 datasheet (not the INA226/INA237, which differ). They were written
// from memory and have not been tested on a chip.
constexpr uint8_t kRegConfig   = 0x00;
constexpr uint8_t kRegShunt    = 0x01;
constexpr uint8_t kRegBus      = 0x02;
constexpr uint8_t kRegMfgId    = 0x3E;

constexpr uint16_t kMfgIdTi    = 0x5449;  // ASCII "TI"
constexpr uint16_t kConfigReset = 0x8000;

// CONFIG = reserved bit 14 set | AVG=16 (010) | VBUSCT=1.1 ms (100)
//        | VSHCT=1.1 ms (100) | MODE=continuous shunt + bus (111).
// Averaging 16 x 1.1 ms gives a fresh result about every 35 ms.
constexpr uint16_t kConfig = 0x4000 | (0b010 << 9) | (0b100 << 6) | (0b100 << 3) | 0b111;

constexpr float kShuntLsbV = 2.5e-6f;  // 2.5 uV per bit at ADCRANGE = 0
constexpr float kBusLsbV   = 1.6e-3f;  // 1.6 mV per bit

}  // namespace

bool Ina236::begin(uint8_t addr, float shunt_ohms, TwoWire& bus) {
  bus_ = &bus;
  addr_ = addr;
  shunt_ohms_ = shunt_ohms;
  ok_ = false;

  // Check the manufacturer ID first, so a different chip at this address
  // isn't reconfigured by mistake.
  // **CHECK:** if begin() always fails on real hardware, print `mfg` here
  // first. A wrong ID constant makes the sensor look absent.
  uint16_t mfg = 0;
  if (!readRegister(kRegMfgId, mfg) || mfg != kMfgIdTi) {
    return false;
  }

  // A reset clears any configuration left over from before an ESP32 reboot.
  if (!writeRegister(kRegConfig, kConfigReset)) return false;
  if (!writeRegister(kRegConfig, kConfig)) return false;

  ok_ = true;
  return true;
}

bool Ina236::update() {
  uint16_t shunt_raw = 0;
  uint16_t bus_raw = 0;
  if (!bus_ || !readRegister(kRegShunt, shunt_raw) || !readRegister(kRegBus, bus_raw)) {
    ++fail_total_;
    if (fail_streak_ < 255) ++fail_streak_;
    return false;
  }

  // The shunt register is two's complement, so negative values mean current
  // flowing backwards. The bus register is unsigned with bit 15 always 0.
  const float shunt_v = static_cast<int16_t>(shunt_raw) * kShuntLsbV;
  bus_v_ = (bus_raw & 0x7FFF) * kBusLsbV;
  current_a_ = shunt_v / shunt_ohms_;
  fail_streak_ = 0;
  return true;
}

bool Ina236::readRegister(uint8_t reg, uint16_t& value) {
  bus_->beginTransmission(addr_);
  bus_->write(reg);
  // Repeated start (false), so nothing else can use the bus between the
  // register select and the read.
  if (bus_->endTransmission(false) != 0) return false;
  if (bus_->requestFrom(addr_, static_cast<uint8_t>(2)) != 2) return false;
  const uint8_t msb = bus_->read();
  const uint8_t lsb = bus_->read();
  value = static_cast<uint16_t>((msb << 8) | lsb);  // INA236 is big-endian.
  return true;
}

bool Ina236::writeRegister(uint8_t reg, uint16_t value) {
  bus_->beginTransmission(addr_);
  bus_->write(reg);
  bus_->write(static_cast<uint8_t>(value >> 8));
  bus_->write(static_cast<uint8_t>(value & 0xFF));
  return bus_->endTransmission() == 0;
}
