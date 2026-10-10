#pragma once
/** @file
 * ESP-IDF synchronous repeated-start implementation of RegisterBus.
 * Borrows a configured device; selects no pins and owns no hardware lifetime.
 */
#include <cansat/register_bus.hpp>
#include <driver/i2c_master.h>

namespace cansat {
// One instance per device; single-task ownership. Handle and bus are borrowed.
// PRECONDITION: device is configured for synchronous operation (no callbacks),
// with the supplied 7-bit address. The owner keeps it alive through every read.
// This adapter never creates/deletes devices or selects pins/clock/pull-ups.
class EspIdfRegisterBus final : public RegisterBus {
public:
    static constexpr std::size_t max_read_bytes = 256;
    static constexpr int max_timeout_ms = 1000;
    EspIdfRegisterBus(i2c_master_dev_handle_t device, std::uint8_t address, int timeout_ms)
        : device_(device), address_(address), timeout_ms_(timeout_ms) {}
    /// Address must match; length 1..256; timeout 1..1000 ms. Validate before SDK I/O.
    /// Copy to destination only after full synchronous success; false preserves bytes.
    bool read(std::uint8_t address, std::uint8_t reg,
              std::uint8_t* destination, std::size_t length) override;
    /// ESP_ERR_INVALID_STATE before the first read; latest validation/SDK status thereafter.
    esp_err_t lastError() const { return last_error_; }
private:
    i2c_master_dev_handle_t device_;
    std::uint8_t address_;
    int timeout_ms_;
    esp_err_t last_error_ = ESP_ERR_INVALID_STATE;
};
}  // namespace cansat
