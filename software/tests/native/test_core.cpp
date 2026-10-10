#include <cansat/ina236.hpp>
#include <cansat/serial_frame.hpp>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <stdexcept>

namespace {
int checks = 0;
void check(bool condition, const char* label) {
    ++checks;
    if (!condition) throw std::runtime_error(label);
}
void near(double actual, double expected) {
    check(std::abs(actual - expected) < 1e-10, "numeric conversion");
}
struct FakeBus : cansat::RegisterBus {
    std::map<std::uint8_t, std::uint16_t> registers{
        {0x3e, 0x5449}, {0x3f, 0xa080}, {0x00, 0x4127},
        {0x06, 0x0008}, {0x01, 800}, {0x02, 7500}};
    int failing_register = -1;
    bool read(std::uint8_t address, std::uint8_t reg, std::uint8_t* out, std::size_t length) override {
        if (address != 0x40 || length != 2 || reg == failing_register || !registers.count(reg)) return false;
        const auto value = registers.at(reg);
        out[0] = static_cast<std::uint8_t>(value >> 8);
        out[1] = static_cast<std::uint8_t>(value);
        return true;
    }
};
}  // namespace

int main() {
    try {
        using namespace cansat;
        FakeBus bus;
        Ina236 sensor{bus, 0x40, 0.020};
        check(!sensor.read(0), "probe required");
        check(sensor.probe() == Error::none, "probe");
        auto sample = sensor.read(1234);
        check(bool(sample) && sample.observed_at_us == 1234, "valid timestamped sample");
        near(sample.value->voltage_v, 12.0);
        near(sample.value->current_a, 0.1);
        bus.registers[0x01] = 65536 - 800;
        near(sensor.read(1).value->current_a, -0.1);
        bus.registers[0x00] |= 0x1000;
        near(sensor.read(1).value->current_a, -0.025);
        bus.registers[0x01] = 1;
        near(sensor.read(1).value->current_a, 0.00003125);
        bus.registers[0x00] &= ~0x1000;
        near(sensor.read(1).value->current_a, 0.000125);
        for (int reg : {0x00, 0x06, 0x01, 0x02}) {
            bus.failing_register = reg;
            const auto failed = sensor.read(42);
            check(!failed.value && failed.error == Error::bus, "read failure clears data");
        }
        bus.failing_register = -1;
        bus.registers[0x06] = 0;
        check(sensor.read(0).error == Error::not_ready, "conversion not ready");
        bus.registers[0x06] = 8;
        bus.registers[0x00] = 0x4120;
        check(sensor.read(0).error == Error::unsupported_mode, "shutdown refused");
        bus.registers[0x00] = 0x4127;
        bus.registers[0x02] = 0x8000;
        check(sensor.read(0).error == Error::invalid_data, "reserved voltage bit");
        bus.registers[0x02] = 7500;
        for (auto rail : {0x8000, 0x7fff}) {
            bus.registers[0x01] = rail;
            check(sensor.read(0).error == Error::saturated, "shunt saturation");
        }
        for (double shunt : {0.0, -1.0, std::numeric_limits<double>::infinity(),
                             std::numeric_limits<double>::quiet_NaN()}) {
            Ina236 invalid{bus, 0x40, shunt};
            check(invalid.probe() == Error::invalid_config, "invalid resistor");
            check(!invalid.read(0).value, "invalid resistor has no sample");
        }
        Ina236 invalid_address{bus, 0x00, 0.02};
        check(invalid_address.probe() == Error::invalid_config, "reserved address");
        bus.registers[0x3f] = 0xa081;
        check(sensor.probe() == Error::none, "silicon revision allowed");
        bus.registers[0x3f] = 0xffff;
        check(sensor.probe() == Error::wrong_device, "wrong device refused");
        check(!sensor.read(0).value, "failed probe invalidates previous identification");
        bus.registers[0x3f] = 0xa080;
        bus.registers[0x3e] = 0;
        check(sensor.probe() == Error::wrong_device, "wrong manufacturer refused");
        bus.registers[0x3e] = 0x5449;
        for (int reg : {0x3e, 0x3f}) {
            bus.failing_register = reg;
            check(sensor.probe() == Error::bus, "probe bus error");
        }
        for (auto node : {Node::container, Node::pocketqube}) {
            auto encoded = encodeSerialFrame(node, "CMD,1011,CAL");
            check(encoded.has_value(), "encode");
            auto decoded = decodeSerialFrame(*encoded);
            check(decoded && decoded->node == node && decoded->payload == "CMD,1011,CAL", "roundtrip");
            check(encodeSerialFrame(node, std::string(250, 'x')).has_value(), "250 allowed");
            check(!encodeSerialFrame(node, std::string(251, 'x')), "251 rejected");
            check(!encodeSerialFrame(node, ""), "empty rejected");
            check(!encodeSerialFrame(node, "hello\nworld"), "embedded LF rejected");
            check(!encodeSerialFrame(node, "hello\rworld"), "embedded CR rejected");
            check(!encodeSerialFrame(node, std::string{"x\0y", 3}), "NUL rejected");
        }
        check(!encodeSerialFrame(static_cast<Node>('X'), "test"), "invalid target");
        for (const auto* invalid : {"C:x\r", "C:x\r\n", "C:x", "C:\n", "X:x\n", "C:x\nP:y\n"}) {
            check(!decodeSerialFrame(invalid), "malformed frame rejected");
        }
        std::cout << "native_checks=" << checks << " PASS\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
