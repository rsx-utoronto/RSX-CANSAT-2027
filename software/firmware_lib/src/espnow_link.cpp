#include "espnow_link.hpp"

// ESP-NOW transport implementation (API and topology: espnow_link.hpp).
//
// Written against Arduino-ESP32 3.x / ESP-IDF 5.5, where the receive
// callback gets an esp_now_recv_info_t and the send callback gets an
// esp_now_send_info_t.

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <string.h>

namespace {

// Container and PQ peer only with the ground station; the ground station
// peers with both. 4 leaves headroom.
constexpr size_t kMaxPeers = 4;

uint8_t  g_peers[kMaxPeers][6];
size_t   g_peer_count = 0;
uint8_t  g_channel = 0;
bool     g_up = false;
RxHandler g_handler = nullptr;

// Updated from the Wi-Fi task, read from loop().
volatile uint32_t g_send_ok = 0;
volatile uint32_t g_send_fail = 0;
volatile uint32_t g_rx_rejected = 0;

bool isBroadcast(const uint8_t mac[6]) {
  static const uint8_t kBroadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  return memcmp(mac, kBroadcast, 6) == 0;
}

bool isZero(const uint8_t mac[6]) {
  static const uint8_t kZero[6] = {0, 0, 0, 0, 0, 0};
  return memcmp(mac, kZero, 6) == 0;
}

// Runs in the Wi-Fi task. Packets from MACs we never registered are dropped
// here: ESP-NOW delivers anything on our channel, including other teams'
// traffic (X4, X5).
void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (info == nullptr || data == nullptr || len <= 0) return;
  if (!EspNowLink::isKnownPeer(info->src_addr)) {
    g_rx_rejected = g_rx_rejected + 1;
    return;
  }
  RxHandler handler = g_handler;
  if (handler != nullptr) handler(info->src_addr, data, len);
}

// Runs in the Wi-Fi task once the radio knows whether the peer ACKed.
void onSent(const esp_now_send_info_t* /*info*/, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    g_send_ok = g_send_ok + 1;
  } else {
    g_send_fail = g_send_fail + 1;
  }
}

}  // namespace

namespace EspNowLink {

bool begin(uint8_t channel) {
  // ESP-NOW runs on the Wi-Fi radio in station mode without joining any
  // access point. The channel must match the ground station's (X2).
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE) != ESP_OK) return false;
  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_recv_cb(onRecv);
  esp_now_register_send_cb(onSent);
  g_channel = channel;
  g_up = true;
  return true;
}

bool addPeer(const uint8_t mac[6]) {
  // X6: no radio may use the broadcast address. An all-zero MAC means the
  // address was never configured.
  if (!g_up || mac == nullptr || isBroadcast(mac) || isZero(mac)) return false;
  if (isKnownPeer(mac)) return true;
  if (g_peer_count >= kMaxPeers) return false;

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = g_channel;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) return false;

  // Peers are added during setup, before traffic starts, so the receive
  // callback never sees this list half-written.
  memcpy(g_peers[g_peer_count], mac, 6);
  ++g_peer_count;
  return true;
}

bool send(const uint8_t mac[6], const uint8_t* data, size_t len) {
  if (!g_up || mac == nullptr || data == nullptr || len == 0 || len > ESP_NOW_MAX_DATA_LEN ||
      !isKnownPeer(mac)) {
    g_send_fail = g_send_fail + 1;
    return false;
  }
  // ESP_OK only means "queued"; delivery is reported later to onSent().
  if (esp_now_send(mac, data, len) != ESP_OK) {
    g_send_fail = g_send_fail + 1;
    return false;
  }
  return true;
}

void onReceive(RxHandler handler) { g_handler = handler; }

bool isKnownPeer(const uint8_t mac[6]) {
  if (mac == nullptr) return false;
  for (size_t i = 0; i < g_peer_count; ++i) {
    if (memcmp(g_peers[i], mac, 6) == 0) return true;
  }
  return false;
}

uint32_t sendFailures() { return g_send_fail; }
uint32_t sendOk() { return g_send_ok; }
uint32_t rxRejected() { return g_rx_rejected; }
bool isUp() { return g_up; }

}  // namespace EspNowLink
