#pragma once
// Test-only substitute, NOT an ESP-IDF SDK header or proof of SDK compatibility.
// Prototype checked against Espressif's v6.1 i2c_master.h.
#include <cstddef>
#include <cstdint>
using esp_err_t = int;
constexpr esp_err_t ESP_OK = 0;
constexpr esp_err_t ESP_FAIL = -1;
constexpr esp_err_t ESP_ERR_INVALID_ARG = 0x102;
constexpr esp_err_t ESP_ERR_INVALID_STATE = 0x103;
constexpr esp_err_t ESP_ERR_TIMEOUT = 0x107;
struct i2c_master_dev_t;
using i2c_master_dev_handle_t = i2c_master_dev_t*;
extern "C" esp_err_t i2c_master_transmit_receive(
    i2c_master_dev_handle_t device, const std::uint8_t* write_buffer, std::size_t write_size,
    std::uint8_t* read_buffer, std::size_t read_size, int timeout_ms);
