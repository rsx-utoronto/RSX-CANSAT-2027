#pragma once
/** @file
 * Nonblocking peer transport and single-owner bounded queues for NodeCom.
 * SDK callbacks must use a separate synchronized handoff, not Queue directly.
 */
#include <cansat/radio_protocol.hpp>

namespace cansat::com {
enum class Submit { queued, busy, rejected };
/// peer must come from authenticated/provisioned transport identity, not payload bytes.
struct Received { NodeId peer = NodeId::ground; Datagram datagram{}; };
/// Completion echoes the supplied token; delivered is MAC-level status only.
struct Sent { std::uint32_t token = 0; bool delivered = false; };
class IRadioTransport {
public:
    virtual ~IRadioTransport() = default;
    // Copy bytes before returning queued. One completion per queued submission.
    virtual Submit send(NodeId peer, const Datagram& data, std::uint32_t token) = 0;
    /// Nonblocking FIFO pop; false must preserve output.
    virtual bool receive(Received& output) = 0;
    /// Nonblocking completion pop; false must preserve output.
    virtual bool completion(Sent& output) = 0;
};
// Single-owner fixed queue. Not a synchronization primitive for SDK callbacks.
template<class T, std::size_t N> class Queue {
public:
    /// N must be positive. Full returns false without overwriting queued records.
    bool push(const T& item) {
        if (count_ == N) return false;
        data_[(head_ + count_) % N] = item; ++count_; return true;
    }
    /// Empty returns false without modifying item. No synchronization is supplied.
    bool pop(T& item) {
        if (count_ == 0) return false;
        item = data_[head_]; head_ = (head_ + 1) % N; --count_; return true;
    }
    /// Borrow the head until the next mutating queue operation; nullptr when empty.
    const T* front() const { return count_ ? &data_[head_] : nullptr; }
private:
    std::array<T, N> data_{};
    std::size_t head_ = 0, count_ = 0;
};
}  // namespace cansat::com
