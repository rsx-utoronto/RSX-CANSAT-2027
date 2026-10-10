#pragma once
/** @file
 * Read-only INA236 bring-up driver over a borrowed RegisterBus.
 * Single owner; probe/read can perform blocking bus transactions with finite platform timeouts.
 */
#include <cansat/register_bus.hpp>
#include <cansat/sample.hpp>

namespace cansat {
class Ina236 {
public:
    // Explicit configuration: no guessed board address or shunt resistor.
    Ina236(RegisterBus& bus, std::uint8_t address, double shunt_ohms)
        : bus_(bus), address_(address), shunt_ohms_(shunt_ohms) {}
    /// Reset identification, validate address/shunt, then read manufacturer/device IDs.
    /// Borrowed bus must outlive the driver. Only Error::none enables subsequent reads.
    Error probe();
    // Read-only bring-up adapter. Supports continuous shunt+bus conversion only.
    // observed_at_us is the caller's acquisition-start timestamp, not conversion time.
    // Call probe() first. No writes, calibration-register setup, or actuation.
    Sample<PowerReading> read(std::uint64_t observed_at_us);
private:
    bool validConfig() const;
    bool read16(std::uint8_t reg, std::uint16_t& value);
    RegisterBus& bus_;
    std::uint8_t address_;
    double shunt_ohms_;
    bool identified_ = false;
};
}  // namespace cansat
