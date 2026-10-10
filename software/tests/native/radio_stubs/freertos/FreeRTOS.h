#pragma once
#include <cstddef>
#include <cstdint>
using BaseType_t = int;
constexpr BaseType_t pdTRUE = 1;
struct StaticQueue_t {
    std::size_t capacity = 0, width = 0, head = 0, count = 0;
    std::uint8_t* storage = nullptr;
};
using QueueHandle_t = StaticQueue_t*;
