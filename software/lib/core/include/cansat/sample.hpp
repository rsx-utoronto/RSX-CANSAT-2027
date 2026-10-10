#pragma once
/** @file
 * Sensor result types with explicit absence, error and acquisition-start time.
 * Units belong to each reading type; timestamps use the caller monotonic clock.
 */
#include <cstdint>
#include <optional>

namespace cansat {
enum class Error { none, invalid_config, bus, wrong_device, not_ready, unsupported_mode, invalid_data, saturated };

template<class T> struct Sample {
    // Invalid samples never carry zero-valued measurements masquerading as data.
    std::optional<T> value;
    std::uint64_t observed_at_us = 0; // Acquisition start, microseconds; not conversion completion.
    Error error = Error::not_ready;
    explicit operator bool() const { return value.has_value() && error == Error::none; }
};

struct PowerReading {
    double voltage_v;
    double current_a;  // Signed; negative current is valid.
};
}  // namespace cansat
