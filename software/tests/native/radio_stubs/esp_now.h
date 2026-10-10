#pragma once
#include <esp_wifi.h>
#include <cstddef>
struct esp_now_recv_info_t { const std::uint8_t* src_addr; const std::uint8_t* des_addr; };
struct esp_now_send_info_t { const std::uint8_t* des_addr; };
enum esp_now_send_status_t { ESP_NOW_SEND_SUCCESS, ESP_NOW_SEND_FAIL };
struct esp_now_peer_info_t {
    std::uint8_t peer_addr[6]{}, lmk[16]{};
    std::uint8_t channel = 0;
    wifi_interface_t ifidx{};
    bool encrypt = false;
};
using esp_now_recv_cb_t = void (*)(const esp_now_recv_info_t*, const std::uint8_t*, int);
using esp_now_send_cb_t = void (*)(const esp_now_send_info_t*, esp_now_send_status_t);
esp_err_t esp_now_init();
esp_err_t esp_now_deinit();
esp_err_t esp_now_set_pmk(const std::uint8_t*);
esp_err_t esp_now_add_peer(const esp_now_peer_info_t*);
esp_err_t esp_now_register_recv_cb(esp_now_recv_cb_t);
esp_err_t esp_now_register_send_cb(esp_now_send_cb_t);
esp_err_t esp_now_unregister_recv_cb();
esp_err_t esp_now_unregister_send_cb();
esp_err_t esp_now_send(const std::uint8_t*, const std::uint8_t*, std::size_t);
