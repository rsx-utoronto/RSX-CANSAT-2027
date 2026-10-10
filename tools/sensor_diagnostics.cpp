// Host-only explicit fixture diagnostic. Never a substitute for hardware acquisition.
#include <cansat/ina236.hpp>
#include <iomanip>
#include <iostream>
#include <string>

namespace {
class FixtureBus : public cansat::RegisterBus {
    bool read(std::uint8_t address, std::uint8_t reg, std::uint8_t* out, std::size_t n) override {
        if (address != 0x40 || n != 2) return false;
        std::uint16_t value{};
        switch (reg) {
            case 0x3e: value = 0x5449; break;
            case 0x3f: value = 0xa080; break;
            case 0x00: value = 0x4127; break;
            case 0x06: value = 0x0008; break;
            case 0x01: value = 800; break;
            case 0x02: value = 7500; break;
            default: return false;
        }
        out[0] = static_cast<std::uint8_t>(value >> 8);
        out[1] = static_cast<std::uint8_t>(value);
        return true;
    }
};
}  // namespace

int main(int argc, char** argv) {
    if (argc != 2 || std::string{argv[1]} != "--fixture") {
        std::cerr << "Usage: sensor_diagnostics --fixture (no hardware backend configured)\n";
        return 2;
    }
    FixtureBus bus;
    cansat::Ina236 sensor{bus, 0x40, 0.020};
    if (sensor.probe() != cansat::Error::none) return 1;
    const auto sample = sensor.read(1000000);
    if (!sample) return 1;
    std::cout << "source=fixture voltage_v=" << std::fixed << std::setprecision(4)
              << sample.value->voltage_v << " current_a=" << std::setprecision(6)
              << sample.value->current_a << " observed_at_us=" << sample.observed_at_us << '\n';
}
