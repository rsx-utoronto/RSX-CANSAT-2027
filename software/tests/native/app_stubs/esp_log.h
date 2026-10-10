#pragma once

// Private host substitute, never on any ESP-IDF component's include path.
void cansat_test_log(const char* tag, const char* format, ...);
#define ESP_LOGW(tag, ...) cansat_test_log(tag, __VA_ARGS__)
