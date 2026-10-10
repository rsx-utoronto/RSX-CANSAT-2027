// Read-only identity/status/register decoding; no device configuration writes.
#include <cansat/ina236.hpp>
#include <cmath>

namespace cansat {
bool Ina236::validConfig() const {
    return address_ >= 0x08 && address_ <= 0x77 &&
           std::isfinite(shunt_ohms_) && shunt_ohms_ > 0.0;
}

bool Ina236::read16(std::uint8_t reg, std::uint16_t& value) {
    std::uint8_t bytes[2]{};
    if (!bus_.read(address_, reg, bytes, sizeof(bytes))) return false;
    value = static_cast<std::uint16_t>((std::uint16_t{bytes[0]} << 8) | bytes[1]);
    return true;
}

Error Ina236::probe() {
    identified_ = false;
    if (!validConfig()) return Error::invalid_config;
    std::uint16_t manufacturer{}, device{};
    if (!read16(0x3e, manufacturer) || !read16(0x3f, device)) return Error::bus;
    // Low four device-ID bits identify the silicon revision.
    if (manufacturer != 0x5449 || (device & 0xfff0) != 0xa080) return Error::wrong_device;
    identified_ = true;
    return Error::none;
}

Sample<PowerReading> Ina236::read(std::uint64_t observed_at_us) {
    const auto fail = [observed_at_us](Error error) {
        return Sample<PowerReading>{std::nullopt, observed_at_us, error};
    };
    if (!validConfig()) return fail(Error::invalid_config);
    if (!identified_) return fail(Error::wrong_device);
    std::uint16_t config{}, status{}, shunt{}, voltage{};
    if (!read16(0x00, config)) return fail(Error::bus);
    if ((config & 0x0007) != 0x0007) return fail(Error::unsupported_mode);
    // Reading Mask/Enable acknowledges CVRF. One consumer must own this driver.
    if (!read16(0x06, status)) return fail(Error::bus);
    if (!(status & 0x0008)) return fail(Error::not_ready);
    if (!read16(0x01, shunt) || !read16(0x02, voltage)) return fail(Error::bus);
    if (voltage & 0x8000) return fail(Error::invalid_data);
    if (shunt == 0x7fff || shunt == 0x8000) return fail(Error::saturated);
    const auto signed_shunt = shunt >= 0x8000 ? std::int32_t{shunt} - 65536 : std::int32_t{shunt};
    const double shunt_lsb_v = (config & 0x1000) ? 0.625e-6 : 2.5e-6;
    // Derive current from shunt voltage, not the uncalibrated CURRENT register.
    const double current = signed_shunt * shunt_lsb_v / shunt_ohms_;
    if (!std::isfinite(current)) return fail(Error::invalid_data);
    return {PowerReading{voltage * 0.0016, current}, observed_at_us, Error::none};
}
}  // namespace cansat
