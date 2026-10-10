// Single-owner COM scheduler. Replies outrank command retries, then latest telemetry.
// Receipt caching is session-local; transport completion never executes a command.
#include <cansat/node_com.hpp>
#include <algorithm>
#include <limits>

namespace cansat::com {
namespace {
std::size_t index(NodeId n) { return static_cast<std::size_t>(n); }
bool terminal(ResultCode r) {
    return r != ResultCode::received && r != ResultCode::accepted;
}
bool payloadValid(const std::uint8_t* p, std::size_t n) { return p && n && n <= max_payload_bytes; }
}  // namespace
NodeCom::NodeCom(IRadioTransport& t, Config c) : transport_(t), config_(c) {
    valid_ = isNode(c.self) && c.team > 0 && c.team <= 9999 && c.session != 0 &&
        c.retry_ms > 0 && c.command_ttl_ms >= c.retry_ms && c.max_attempts > 0 && c.max_attempts <= 8;
}
Message NodeCom::message(Kind kind, NodeId peer) {
    Message m;
    if (!valid_ || next_id_ == 0) return m; // No ID wrap within a session.
    m.header = {kind, config_.team, config_.self, peer, config_.session,
        kind == Kind::telemetry ? 0 : config_.peer_sessions[index(peer)], next_id_++, 0};
    return m;
}
bool NodeCom::publishTelemetry(const std::uint8_t* data, std::size_t size) {
    if (!valid_ || config_.self == NodeId::ground || !payloadValid(data, size)) return false;
    telemetry_.size = static_cast<std::uint16_t>(size);
    std::copy_n(data, size, telemetry_.payload.begin()); telemetry_present_ = true; return true;
}
bool NodeCom::setTelemetryRate(unsigned hz) {
    if (!valid_ || config_.self == NodeId::ground || (hz != 0 && hz != 1 && hz != 4)) return false;
    hz_ = hz; return true;
}
std::optional<std::uint32_t> NodeCom::submitCommand(NodeId peer, const std::uint8_t* data,
                                                  std::size_t size, std::uint64_t now) {
    if (!valid_ || config_.self != NodeId::ground || !isStarLink(config_.self, peer) ||
        !payloadValid(data, size) || (clock_ && now < *clock_) ||
        config_.peer_sessions[index(peer)] == 0 || pending_[index(peer)].active || next_id_ == 0) return {};
    clock_ = now;
    auto m = message(Kind::command, peer);
    m.size = static_cast<std::uint16_t>(size); std::copy_n(data, size, m.payload.begin());
    pending_[index(peer)] = {m, now, 0, 0, true}; return m.header.id;
}
void NodeCom::resultEvent(const CommandEvent& event) {
    if (!results_.push(event)) ++stats_.queue_full;
}
void NodeCom::reply(const Message& command, ResultCode code) {
    auto m = message(Kind::result, command.header.source);
    m.header.correlation = command.header.id;
    m.size = 1; m.payload[0] = static_cast<std::uint8_t>(code);
    if (m.header.id == 0 || !replies_.push(m)) ++stats_.queue_full;
}
void NodeCom::handle(const Received& event) {
    Message m;
    if (!decode(event.datagram, m)) { ++stats_.malformed; return; }
    const auto& h = m.header;
    if (h.team != config_.team || h.destination != config_.self || h.source != event.peer ||
        !isStarLink(config_.self, h.source) || config_.peer_sessions[index(h.source)] != h.session ||
        (h.kind != Kind::telemetry && h.destination_session != config_.session)) {
        ++stats_.rejected_peer; return;
    }
    if (h.kind == Kind::telemetry) {
        if (!telemetry_in_.push(m)) ++stats_.queue_full;
    } else if (h.kind == Kind::result) {
        auto& pending = pending_[index(h.source)];
        if (!pending.active || pending.message.header.id != h.correlation) { ++stats_.rejected_peer; return; }
        const auto code = static_cast<ResultCode>(m.payload[0]);
        if (pending.have_progress && !terminal(code) && code <= pending.progress) {
            ++stats_.duplicates; return; // Reordered receipts cannot regress UI state.
        }
        if (pending.have_progress && pending.progress == ResultCode::accepted &&
            (code == ResultCode::rejected || code == ResultCode::busy || code == ResultCode::stale)) {
            ++stats_.malformed; return;
        }
        pending.have_progress = true; pending.progress = code;
        resultEvent({h.source, h.correlation, code});
        if (terminal(code)) pending.active = false;
    } else {
        for (auto& receipt : receipts_) {
            if (!receipt.used || receipt.command.header.id != h.id) continue;
            if (receipt.command.size != m.size || !std::equal(m.payload.begin(), m.payload.begin() + m.size,
                                                             receipt.command.payload.begin())) {
                ++stats_.malformed; return;
            }
            ++stats_.duplicates; reply(m, receipt.result); return;
        }
        if (h.id <= command_watermark_) { reply(m, ResultCode::stale); return; }
        Receipt* slot = nullptr;
        // Never evict an unfinished command. Terminal receipts can be evicted;
        // the watermark still prevents an old command from executing again.
        for (auto& r : receipts_) if (!r.used || terminal(r.result)) { slot = &r; break; }
        if (!slot || !commands_.push(m)) { ++stats_.queue_full; reply(m, ResultCode::busy); return; }
        *slot = {m, ResultCode::received, true}; command_watermark_ = h.id;
        reply(m, ResultCode::received);
    }
}
bool NodeCom::reportCommandResult(std::uint32_t id, ResultCode code) {
    for (auto& receipt : receipts_) {
        if (!receipt.used || receipt.command.header.id != id) continue;
        const auto previous = receipt.result;
        const bool allowed = (previous == ResultCode::received &&
            (code == ResultCode::accepted || code == ResultCode::rejected)) ||
            (previous == ResultCode::accepted && (code == ResultCode::completed || code == ResultCode::failed));
        if (!allowed) return false;
        receipt.result = code; reply(receipt.command, code); return true;
    }
    return false;
}
bool NodeCom::send(const Message& m) {
    Datagram d;
    if (in_flight_ || token_ == std::numeric_limits<std::uint32_t>::max() || !encode(m, d)) return false;
    const auto status = transport_.send(m.header.destination, d, token_ + 1);
    if (status == Submit::rejected) ++stats_.tx_rejected;
    if (status != Submit::queued) return false;
    ++token_; in_flight_ = true; ++stats_.tx_queued; return true;
}
bool NodeCom::tick(std::uint64_t now) {
    if (!valid_ || (clock_ && now < *clock_)) return false;
    clock_ = now;
    Sent completion;
    for (unsigned i = 0; i < 8 && transport_.completion(completion); ++i) {
        if (in_flight_ && completion.token == token_) {
            in_flight_ = false; if (!completion.delivered) ++stats_.mac_failed;
        }
    }
    Received received;
    for (unsigned i = 0; i < 8 && transport_.receive(received); ++i) handle(received);
    for (auto& p : pending_) if (p.active && now - p.created >= config_.command_ttl_ms) {
        p.active = false; ++stats_.expired;
        resultEvent({p.message.header.destination, p.message.header.id, ResultCode::unknown});
    }
    if (const auto* response = replies_.front()) {
        if (send(*response)) { Message ignored; replies_.pop(ignored); }
        return true;
    }
    for (unsigned i = 0; i < 2; ++i) {
        const auto peer = 1 + ((next_peer_ - 1 + i) % 2);
        auto& p = pending_[peer];
        if (p.active && p.attempts < config_.max_attempts &&
            (p.attempts == 0 || now - p.last_sent >= config_.retry_ms)) {
            if (send(p.message)) { ++p.attempts; p.last_sent = now; next_peer_ = 3 - peer; }
            return true;
        }
    }
    if (hz_ && telemetry_present_ && !in_flight_ &&
        (!telemetry_sent_ || now - *telemetry_sent_ >= 1000 / hz_)) {
        auto m = message(Kind::telemetry, NodeId::ground);
        m.payload = telemetry_.payload; m.size = telemetry_.size;
        if (send(m)) telemetry_sent_ = now; // Never catch up by bursting old snapshots.
    }
    return true;
}
}  // namespace cansat::com
