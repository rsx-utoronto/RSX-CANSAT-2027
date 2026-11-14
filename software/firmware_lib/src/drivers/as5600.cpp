#include "drivers/as5600.hpp"

namespace {

// Register map (AS5600 datasheet, register table).
// **CHECK:** verify against the datasheet before trusting readings.
constexpr uint8_t kRegStatus   = 0x0B;  // MD, ML, MH flags
constexpr uint8_t kRegRawAngle = 0x0C;  // 12-bit angle: 0x0C bits 11:8, 0x0D bits 7:0

constexpr uint8_t kStatusMagnetTooStrong = 1 << 3;  // MH
constexpr uint8_t kStatusMagnetTooWeak   = 1 << 4;  // ML
constexpr uint8_t kStatusMagnetDetected  = 1 << 5;  // MD

constexpr float kDegPerCount = 360.0f / 4096.0f;

}  // namespace

bool As5600::begin(TwoWire& bus, uint8_t addr) {
  bus_ = &bus;
  addr_ = addr;
  ok_ = readStatus();
  fail_streak_ = 0;
  return ok_;
}

bool As5600::update() {
  uint8_t raw[2] = {0, 0};
  if (!ok_ || !readStatus() || !magnet_detected_ || !readRegisters(kRegRawAngle, raw, 2)) {
    recordFailure();
    return false;
  }

  const uint16_t counts = static_cast<uint16_t>(((raw[0] & 0x0F) << 8) | raw[1]);
  angle_deg_ = counts * kDegPerCount;
  fail_streak_ = 0;
  return true;
}

bool As5600::readStatus() {
  uint8_t status = 0;
  if (!readRegisters(kRegStatus, &status, 1)) return false;
  magnet_detected_   = (status & kStatusMagnetDetected) != 0;
  magnet_too_weak_   = (status & kStatusMagnetTooWeak) != 0;
  magnet_too_strong_ = (status & kStatusMagnetTooStrong) != 0;
  return true;
}

bool As5600::readRegisters(uint8_t reg, uint8_t* out, uint8_t count) {
  if (!bus_) return false;
  bus_->beginTransmission(addr_);
  bus_->write(reg);
  // Repeated start (false), so nothing else can use the bus between the
  // register select and the read.
  if (bus_->endTransmission(false) != 0) return false;
  if (bus_->requestFrom(addr_, count) != count) return false;
  for (uint8_t i = 0; i < count; ++i) out[i] = static_cast<uint8_t>(bus_->read());
  return true;
}

void As5600::recordFailure() {
  ++fail_total_;
  if (fail_streak_ < 255) ++fail_streak_;
}
