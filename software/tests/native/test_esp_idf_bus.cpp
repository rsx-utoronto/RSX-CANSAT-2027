#include <cansat/esp_idf_register_bus.hpp>
#include <cansat/ina236.hpp>
#include <array>
#include <cmath>
#include <iostream>
#include <map>
#include <stdexcept>

struct i2c_master_dev_t {};
namespace {
i2c_master_dev_t device;
esp_err_t result = ESP_OK;
int calls = 0;
int checks = 0;
std::map<std::uint8_t, std::uint16_t> registers{
    {0x3e, 0x5449}, {0x3f, 0xa080}, {0, 0x4127}, {6, 8}, {1, 800}, {2, 7500}};
void check(bool condition, const char* label) {
    ++checks;
    if (!condition) throw std::runtime_error(label);
}
}  // namespace

extern "C" esp_err_t i2c_master_transmit_receive(
    i2c_master_dev_handle_t handle, const std::uint8_t* write_buffer, std::size_t write_size,
    std::uint8_t* read_buffer, std::size_t read_size, int timeout_ms) {
    ++calls;
    check(handle == &device, "correct borrowed device");
    check(write_buffer && write_size == 1, "single register address and repeated-start call");
    check(timeout_ms == 25, "explicit bounded timeout forwarded");
    check(read_buffer && read_size > 0 && read_size <= 256, "receive length");
    // Deliberately mutate the receive buffer even on failure.
    for (std::size_t i = 0; i < read_size; ++i) read_buffer[i] = 0xff;
    if (result != ESP_OK) return result;
    const auto value = registers.at(write_buffer[0]);
    read_buffer[0] = static_cast<std::uint8_t>(value >> 8);
    if (read_size > 1) read_buffer[1] = static_cast<std::uint8_t>(value);
    return ESP_OK;
}

int main() {
    try {
        cansat::EspIdfRegisterBus bus{&device, 0x40, 25};
        std::array<std::uint8_t, 256> bytes{};
        check(bus.read(0x40, 2, bytes.data(), 2), "successful read");
        check(bytes[0] == 0x1d && bytes[1] == 0x4c, "wire byte order preserved");
        check(bus.lastError() == ESP_OK, "success status");
        for (auto error : {ESP_ERR_TIMEOUT, ESP_FAIL}) {
            result = error;
            const auto previous = bytes;
            check(!bus.read(0x40, 2, bytes.data(), 2), "SDK error propagated");
            check(bus.lastError() == error && bytes == previous, "no partial data published");
        }
        result = ESP_OK;
        const int before = calls;
        check(!bus.read(0x41, 2, bytes.data(), 2), "other device refused");
        check(!bus.read(0x40, 2, nullptr, 2), "null output refused");
        check(!bus.read(0x40, 2, bytes.data(), 0), "zero length refused");
        check(!bus.read(0x40, 2, bytes.data(), 257), "oversize refused");
        for (int timeout : {-1, 0, 1001}) {
            cansat::EspIdfRegisterBus invalid{&device, 0x40, timeout};
            check(!invalid.read(0x40, 2, bytes.data(), 2), "invalid timeout refused");
        }
        cansat::EspIdfRegisterBus missing{nullptr, 0x40, 25};
        check(!missing.read(0x40, 2, bytes.data(), 2), "missing handle refused");
        cansat::EspIdfRegisterBus reserved{&device, 0, 25};
        check(!reserved.read(0, 2, bytes.data(), 2), "reserved address refused");
        check(calls == before, "invalid requests never call SDK");
        check(bus.lastError() == ESP_ERR_INVALID_ARG, "argument error retained");
        check(bus.read(0x40, 2, bytes.data(), bytes.size()), "maximum bounded read");
        cansat::Ina236 sensor{bus, 0x40, 0.020};
        check(sensor.probe() == cansat::Error::none, "driver through platform adapter probe");
        const auto sample = sensor.read(5000);
        check(bool(sample), "driver through platform adapter read");
        check(std::abs(sample.value->voltage_v - 12.0) < 1e-10 &&
              std::abs(sample.value->current_a - 0.1) < 1e-10, "fixture conversions unchanged");
        result = ESP_ERR_TIMEOUT;
        check(!sensor.read(6000).value, "timeout invalidates sensor sample");
        std::cout << "esp_idf_bus_checks=" << checks << " PASS source=SDK_test_double hardware=none\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
