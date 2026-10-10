#pragma once
#include <cansat/radio_transport.hpp>
#include <vector>

// Test-only manual network: no OS clock, hardware, or automatic packet delivery.
class FakeRadio final : public cansat::com::IRadioTransport {
public:
    struct Transmission { cansat::com::NodeId peer; cansat::com::Datagram data; std::uint32_t token; };
    cansat::com::Submit behavior = cansat::com::Submit::queued;
    bool mac_success = true, produce_completion = true;
    std::vector<Transmission> sent;
    cansat::com::Queue<cansat::com::Received, 32> incoming;
    cansat::com::Queue<cansat::com::Sent, 32> completions;
    cansat::com::Submit send(cansat::com::NodeId peer, const cansat::com::Datagram& data, std::uint32_t token) override {
        if (behavior != cansat::com::Submit::queued) return behavior;
        sent.push_back({peer, data, token});
        if (produce_completion) completions.push({token, mac_success});
        return behavior;
    }
    bool receive(cansat::com::Received& out) override { return incoming.pop(out); }
    bool completion(cansat::com::Sent& out) override { return completions.pop(out); }
};
