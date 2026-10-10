// Explicit ESP-NOW lifecycle and bounded callback-to-task queues.
// Callbacks are transport plumbing only; current app_main entry points never start it.
#include <cansat/esp_now_transport.hpp>
#include <algorithm>
#include <cstring>
#include <esp_wifi.h>

namespace cansat::esp_idf {
namespace {
template<std::size_t N> bool nonzero(const std::array<std::uint8_t, N>& value) {
    return std::any_of(value.begin(), value.end(), [](auto byte) { return byte != 0; });
}
bool unicast(const std::array<std::uint8_t, 6>& mac) { return nonzero(mac) && (mac[0] & 1u) == 0; }
}  // namespace
EspNowTransport& EspNowTransport::instance() { static EspNowTransport adapter; return adapter; }
bool EspNowTransport::validConfig(const RadioConfig& c) {
    if (!com::isNode(c.self) || c.channel < 1 || c.channel > 14 || !nonzero(c.primary_key) ||
        c.peer_count != (c.self == com::NodeId::ground ? 2 : 1)) return false;
    for (unsigned i = 0; i < c.peer_count; ++i) {
        const auto& p = c.peers[i];
        if (!com::isStarLink(c.self, p.node) || !unicast(p.mac) || !nonzero(p.local_key)) return false;
        for (unsigned j = 0; j < i; ++j)
            if (p.mac == c.peers[j].mac || p.node == c.peers[j].node) return false;
    }
    return true;
}
RadioStart EspNowTransport::start(const RadioConfig& c) {
    if (active()) return RadioStart::already_active;
    if (!validConfig(c)) return RadioStart::invalid_config;
    std::uint8_t channel = 0;
    wifi_second_chan_t secondary{};
    wifi_mode_t mode{};
    // Caller must have started STA Wi-Fi on the reviewed, assigned channel.
    // The numeric range above is not regulatory approval; Wi-Fi country policy applies.
    if (esp_wifi_get_mode(&mode) != ESP_OK || mode != WIFI_MODE_STA ||
        esp_wifi_get_channel(&channel, &secondary) != ESP_OK || channel != c.channel ||
        esp_wifi_get_mac(WIFI_IF_STA, own_mac_.data()) != ESP_OK) return RadioStart::wifi_not_ready;
    for (unsigned i = 0; i < c.peer_count; ++i) if (c.peers[i].mac == own_mac_) return RadioStart::invalid_config;
    // Object and queue backing storage remain alive for the entire process.
    if (!rx_queue_) rx_queue_ = xQueueCreateStatic(8, sizeof(com::Received), rx_storage_.data(), &rx_control_);
    if (!tx_queue_) tx_queue_ = xQueueCreateStatic(1, sizeof(com::Sent), tx_storage_.data(), &tx_control_);
    if (!rx_queue_ || !tx_queue_) return RadioStart::sdk_error;
    config_ = c;
    if (esp_now_init() != ESP_OK) return RadioStart::sdk_error;
    bool ok = esp_now_set_pmk(c.primary_key.data()) == ESP_OK;
    for (unsigned i = 0; i < c.peer_count && ok; ++i) {
        esp_now_peer_info_t peer{};
        std::copy(c.peers[i].mac.begin(), c.peers[i].mac.end(), peer.peer_addr);
        std::copy(c.peers[i].local_key.begin(), c.peers[i].local_key.end(), peer.lmk);
        peer.channel = c.channel; peer.ifidx = WIFI_IF_STA; peer.encrypt = true;
        ok = esp_now_add_peer(&peer) == ESP_OK;
    }
    if (ok) ok = esp_now_register_recv_cb(&onReceive) == ESP_OK;
    if (ok) ok = esp_now_register_send_cb(&onSent) == ESP_OK;
    if (!ok) {
        esp_now_unregister_recv_cb(); esp_now_unregister_send_cb(); esp_now_deinit();
        return RadioStart::sdk_error;
    }
    active_.store(true);
    return RadioStart::ok;
}
com::Submit EspNowTransport::send(com::NodeId node, const com::Datagram& data, std::uint32_t token) {
    if (!active() || token == 0 || data.size <= com::header_bytes + com::checksum_bytes ||
        data.size > com::max_radio_bytes) return com::Submit::rejected;
    const RadioPeer* selected = nullptr;
    for (unsigned i = 0; i < config_.peer_count; ++i) if (config_.peers[i].node == node) selected = &config_.peers[i];
    if (!selected) return com::Submit::rejected;
    bool idle = false;
    if (!busy_.compare_exchange_strong(idle, true)) return com::Submit::busy;
    destination_ = selected->mac; token_.store(token);
    if (esp_now_send(destination_.data(), data.bytes.data(), data.size) != ESP_OK) {
        busy_.store(false); return com::Submit::rejected;
    }
    return com::Submit::queued;
}
bool EspNowTransport::receive(com::Received& out) {
    return active() && xQueueReceive(rx_queue_, &out, 0) == pdTRUE;
}
bool EspNowTransport::completion(com::Sent& out) {
    if (!active() || xQueueReceive(tx_queue_, &out, 0) != pdTRUE) return false;
    busy_.store(false); return true;
}
void EspNowTransport::onReceive(const esp_now_recv_info_t* info, const std::uint8_t* data, int size) {
    auto& a = instance();
    if (!a.active()) return;
    if (!info || !info->src_addr || !info->des_addr || !data || size <= int(com::header_bytes + com::checksum_bytes) ||
        size > int(com::max_radio_bytes) || std::memcmp(info->des_addr, a.own_mac_.data(), 6) != 0) {
        ++a.dropped_; return;
    }
    for (unsigned i = 0; i < a.config_.peer_count; ++i) {
        if (std::memcmp(info->src_addr, a.config_.peers[i].mac.data(), 6) != 0) continue;
        com::Received event;
        event.peer = a.config_.peers[i].node; event.datagram.size = static_cast<std::uint16_t>(size);
        std::memcpy(event.datagram.bytes.data(), data, static_cast<std::size_t>(size));
        if (xQueueSend(a.rx_queue_, &event, 0) != pdTRUE) ++a.dropped_;
        return;
    }
    ++a.dropped_;
}
void EspNowTransport::onSent(const esp_now_send_info_t* info, esp_now_send_status_t status) {
    auto& a = instance();
    if (!a.active() || !a.busy_.load()) return;
    if (!info || !info->des_addr || std::memcmp(info->des_addr, a.destination_.data(), 6) != 0) {
        ++a.dropped_; return; // Missing/mismatched completion never means success.
    }
    const com::Sent result{a.token_.load(), status == ESP_NOW_SEND_SUCCESS};
    if (xQueueSend(a.tx_queue_, &result, 0) != pdTRUE) ++a.dropped_;
}
}  // namespace cansat::esp_idf
