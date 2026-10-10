// Borrowed synchronous I2C device adapter; publish bytes only after full success.
#include <cansat/esp_idf_register_bus.hpp>
#include <array>
#include <cstring>

namespace cansat {
bool EspIdfRegisterBus::read(std::uint8_t address, std::uint8_t reg,
                           std::uint8_t* destination, std::size_t length) {
    if (!device_ || address_ < 0x08 || address_ > 0x77 || address != address_ ||
        !destination || length == 0 || length > max_read_bytes ||
        timeout_ms_ <= 0 || timeout_ms_ > max_timeout_ms) {
        last_error_ = ESP_ERR_INVALID_ARG;
        return false;
    }
    // Never publish a partially written SDK receive buffer after an error.
    // Stack buffers are valid because the borrowed device MUST be synchronous.
    std::array<std::uint8_t, max_read_bytes> received{};
    last_error_ = i2c_master_transmit_receive(device_, &reg, 1, received.data(), length, timeout_ms_);
    if (last_error_ != ESP_OK) return false;
    std::memcpy(destination, received.data(), length);
    return true;
}
}  // namespace cansat
