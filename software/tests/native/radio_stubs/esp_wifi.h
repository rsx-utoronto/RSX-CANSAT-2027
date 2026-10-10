#pragma once
#include <cstdint>
using esp_err_t = int;
constexpr esp_err_t ESP_OK = 0;
enum wifi_mode_t { WIFI_MODE_NULL, WIFI_MODE_STA };
enum wifi_second_chan_t { WIFI_SECOND_CHAN_NONE };
enum wifi_interface_t { WIFI_IF_STA };
esp_err_t esp_wifi_get_mode(wifi_mode_t*);
esp_err_t esp_wifi_get_channel(std::uint8_t*, wifi_second_chan_t*);
esp_err_t esp_wifi_get_mac(wifi_interface_t, std::uint8_t*);
