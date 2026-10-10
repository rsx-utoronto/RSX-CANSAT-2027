#pragma once
/** @file
 * Inactive-by-default ESP-IDF ESP-NOW adapter with static callback queues.
 * Caller-task APIs have one owner; callbacks only validate and copy bounded records.
 * Requires externally configured STA Wi-Fi and privately provisioned peers/keys.
 */
#include <cansat/radio_transport.hpp>
#include <array>
#include <atomic>
#include <esp_now.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

namespace cansat::esp_idf {
struct RadioPeer {
    com::NodeId node = com::NodeId::ground;
    std::array<std::uint8_t, 6> mac{};
    std::array<std::uint8_t, 16> local_key{};
};
struct RadioConfig {
    com::NodeId self = com::NodeId::ground;
    std::uint8_t channel = 0;
    std::array<RadioPeer, 2> peers{};
    std::uint8_t peer_count = 0;
    std::array<std::uint8_t, 16> primary_key{};
};
enum class RadioStart { ok, already_active, invalid_config, wifi_not_ready, sdk_error };
// Process-lifetime singleton: ESP-NOW has global callbacks. No constructor side
// effects. Caller owns Wi-Fi STA initialization/channel and exclusive ESP-NOW use.
// No stop/reconfigure lifecycle is offered by this first adapter slice.
class EspNowTransport final : public com::IRadioTransport {
public:
    static EspNowTransport& instance();
    /// Pure validation: ground has two peers, a flight node one; nonzero keys and
    /// distinct unicast peers required. Numeric channel validation is not radio approval.
    static bool validConfig(const RadioConfig& config);
    /// Explicit startup only, never called by current apps. Borrow already-started
    /// STA Wi-Fi on the selected channel; configure encrypted unicast ESP-NOW peers.
    /// Already active refuses reconfiguration. SDK setup failures return sdk_error.
    RadioStart start(const RadioConfig& config);
    bool active() const { return active_.load(); }
    /// One outstanding send; queued is submission only. Missing completion keeps busy.
    com::Submit send(com::NodeId peer, const com::Datagram& data, std::uint32_t token) override;
    /// Pop a callback-copied RX record without blocking; false preserves output.
    bool receive(com::Received& output) override;
    /// Pop MAC completion and release busy; false preserves output and busy state.
    bool completion(com::Sent& output) override;
    /// Diagnostic RX/invalid-completion/queue-drop counter; not persisted, may wrap.
    std::uint32_t dropped() const { return dropped_.load(); }
    EspNowTransport(const EspNowTransport&) = delete;
    EspNowTransport& operator=(const EspNowTransport&) = delete;
private:
    EspNowTransport() = default;
    static void onReceive(const esp_now_recv_info_t* info, const std::uint8_t* data, int size);
    static void onSent(const esp_now_send_info_t* info, esp_now_send_status_t status);
    RadioConfig config_{};
    std::array<std::uint8_t, 6> own_mac_{};
    std::atomic<bool> active_{false}, busy_{false};
    std::atomic<std::uint32_t> dropped_{0}, token_{0};
    std::array<std::uint8_t, 6> destination_{};
    StaticQueue_t rx_control_{}, tx_control_{};
    std::array<std::uint8_t, 8 * sizeof(com::Received)> rx_storage_{};
    std::array<std::uint8_t, sizeof(com::Sent)> tx_storage_{};
    QueueHandle_t rx_queue_ = nullptr, tx_queue_ = nullptr;
};
}  // namespace cansat::esp_idf
