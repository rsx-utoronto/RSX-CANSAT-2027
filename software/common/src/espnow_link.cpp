#include "espnow_link.hpp"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>

namespace {
  RxHandler userRxHandler = nullptr;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  void internalOnDataRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
    if (userRxHandler && info) {
      userRxHandler(info->src_addr, data, len);
    }
  }
#else
  void internalOnDataRecv(const uint8_t* mac, const uint8_t* data, int len) {
    if (userRxHandler) {
      userRxHandler(mac, data, len);
    }
  }
#endif
}

namespace EspNowLink {

  bool begin(uint8_t channel) {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    
    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK) {
      return false;
    }

    esp_now_register_recv_cb(internalOnDataRecv);
    return true;
  }

  bool addPeer(const uint8_t mac[6]) {
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, mac, 6);
    peerInfo.channel = 0; // Use current channel
    peerInfo.encrypt = false;

    return (esp_now_add_peer(&peerInfo) == ESP_OK);
  }

  bool send(const uint8_t mac[6], const uint8_t* data, size_t len) {
    return (esp_now_send(mac, data, len) == ESP_OK);
  }

  void onReceive(RxHandler handler) {
    userRxHandler = handler;
  }
}