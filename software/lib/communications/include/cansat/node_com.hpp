#pragma once
/** @file
 * Single-owner three-node star service with bounded queues, retries and receipts.
 * No SDK, clock reads, payload interpretation, command authorization or actuation.
 * Sessions and state are volatile; this is not durable exactly-once execution.
 */
#include <cansat/radio_transport.hpp>
#include <optional>

namespace cansat::com {
/// Immutable service policy. Zero defaults deliberately fail validation.
/// Valid team 1..9999, nonzero self session, retry_ms > 0, TTL >= retry, attempts 1..8.
/// Unknown peer sessions (zero) prevent commands and reject incoming peer traffic.
struct Config {
    NodeId self = NodeId::ground;
    std::uint16_t team = 0;
    std::uint32_t session = 0;
    // Authenticated peer boot/session IDs must be supplied externally. Zero is unknown.
    std::array<std::uint32_t, 3> peer_sessions{};
    std::uint32_t retry_ms = 0, command_ttl_ms = 0;
    std::uint8_t max_attempts = 0;
};
struct CommandEvent { NodeId peer; std::uint32_t id; ResultCode result; };
struct Stats {
    std::uint32_t malformed = 0, rejected_peer = 0, duplicates = 0, queue_full = 0;
    std::uint32_t tx_queued = 0, mac_failed = 0, tx_rejected = 0, expired = 0;
};
class NodeCom {
public:
    /// Borrow transport for the entire service lifetime; construction performs no I/O.
    NodeCom(IRadioTransport& transport, Config config);
    bool valid() const { return valid_; }
    /// Flight nodes only. Copy 1..220 bytes as the latest snapshot; no immediate send.
    /// Replaces an older snapshot without queueing historical samples. False leaves it unchanged.
    bool publishTelemetry(const std::uint8_t* data, std::size_t size);
    bool setTelemetryRate(unsigned hz);  // 0, 1 or 4; application supplies mission policy.
    /// Ground only; copy payload and reserve one outstanding command per known flight peer.
    /// Returns its session-local ID or nullopt (invalid input, busy peer, clock regression,
    /// unknown session or ID exhaustion). A returned ID is not delivery/acceptance.
    std::optional<std::uint32_t> submitCommand(NodeId peer, const std::uint8_t* data,
                                              std::size_t size, std::uint64_t now_ms);
    /// Same monotonic millisecond clock as submitCommand; equal time is allowed.
    /// Drain at most eight completions and eight receives, expire TTLs, then attempt
    /// one send: queued reply, round-robin command retry, or latest telemetry.
    /// False means invalid configuration/regressing clock, not radio-send failure.
    /// True does not imply progress or delivery. Transport operations must not block.
    bool tick(std::uint64_t now_ms);
    /// Pop one FIFO record; false leaves the output unchanged. Taking is not executing.
    bool takeCommand(Message& command) { return commands_.pop(command); }
    /// Pop one received telemetry record; schema validation is the caller responsibility.
    bool takeTelemetry(Message& telemetry) { return telemetry_in_.pop(telemetry); }
    /// Pop progress/terminal status. UNKNOWN means uncertain execution, not safe to repeat.
    bool takeResult(CommandEvent& result) { return results_.pop(result); }
    /// Flight application transitions: received -> accepted/rejected; accepted ->
    /// completed/failed. False rejects missing IDs/illegal transitions. True updates
    /// cached state even if the bounded reply queue is full; it is not delivery proof.
    bool reportCommandResult(std::uint32_t command_id, ResultCode result);
    /// Single-owner counters; unsigned counters can wrap and are not persisted.
    const Stats& stats() const { return stats_; }
private:
    struct Pending {
        Message message{};
        std::uint64_t created = 0, last_sent = 0;
        unsigned attempts = 0;
        bool active = false, have_progress = false;
        ResultCode progress = ResultCode::received;
    };
    struct Receipt { Message command{}; ResultCode result = ResultCode::received; bool used = false; };
    Message message(Kind kind, NodeId peer);
    void handle(const Received& event);
    void reply(const Message& command, ResultCode code);
    bool send(const Message& message);
    void resultEvent(const CommandEvent& event);
    IRadioTransport& transport_;
    Config config_;
    bool valid_ = false;
    std::uint32_t next_id_ = 1, token_ = 0;
    bool in_flight_ = false;
    std::optional<std::uint64_t> clock_;
    std::array<Pending, 3> pending_{};
    std::array<Receipt, 8> receipts_{};
    std::uint32_t command_watermark_ = 0;
    Queue<Message, 4> commands_;
    Queue<Message, 8> replies_;
    Queue<Message, 8> telemetry_in_;
    Queue<CommandEvent, 16> results_;
    Message telemetry_{};
    bool telemetry_present_ = false;
    unsigned hz_ = 0;
    std::optional<std::uint64_t> telemetry_sent_;
    std::size_t next_peer_ = 1;
    Stats stats_{};
};
}  // namespace cansat::com
