#pragma once
/** @file
 * Synchronous injected register-read boundary. Device drivers never select pins or own hardware.
 * See documentation/software-specification.md for ownership and failure contracts.
 */
#include <cstddef>
#include <cstdint>

namespace cansat {
class RegisterBus {
public:
    virtual ~RegisterBus() = default;
    // The platform adapter owns bus/pin setup, repeated-start and finite timeouts.
    // True means exactly length bytes transferred, in device wire byte order.
    // NACK, timeout, and short reads must return false; never wait indefinitely.
    virtual bool read(std::uint8_t address, std::uint8_t reg,
                      std::uint8_t* destination, std::size_t length) = 0;
};
}  // namespace cansat
