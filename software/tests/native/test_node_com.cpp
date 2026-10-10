#include "fake_radio.hpp"
#include <cansat/node_com.hpp>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <iomanip>

using namespace cansat::com;
namespace {
unsigned checks = 0;
#define CHECK(x) do { ++checks; if (!(x)) { std::cerr << "FAILED line " << __LINE__ << ": " #x "\n"; std::exit(1); } } while (false)
Config config(NodeId self) { return {self, 1234, static_cast<std::uint32_t>(11 * (1 + int(self))), {11,22,33}, 100, 500, 3}; }
Message command(std::uint32_t id = 1) {
    Message m; m.header = {Kind::command,1234,NodeId::ground,NodeId::container,11,22,id,0};
    m.size = 3; m.payload[0] = 0xa5; m.payload[2] = 0xff; return m;
}
Datagram wire(const Message& m) { Datagram d; CHECK(encode(m,d)); return d; }
void inject(FakeRadio& f, NodeId peer, const Datagram& d) { CHECK(f.incoming.push({peer,d})); }
void codec() {
    auto m = command(); const auto d = wire(m);
    std::ostringstream hex;
    for (unsigned i = 0; i < d.size; ++i) hex << std::hex << std::setfill('0') << std::setw(2) << unsigned(d.bytes[i]);
    std::cout << "GOLDEN_COMMAND=" << hex.str() << '\n';
    Message out; CHECK(decode(d,out)); CHECK(out.size == 3 && out.header.destination_session == 22 && out.payload[2] == 0xff);
    for (unsigned i = 0; i < d.size; ++i) { auto bad = d; bad.bytes[i] ^= 1; CHECK(!decode(bad,out)); }
    for (unsigned i = 0; i < d.size; ++i) { auto bad = d; bad.size = i; CHECK(!decode(bad,out)); }
    auto bad = d; bad.size = 251; CHECK(!decode(bad,out));
    m.size = max_payload_bytes; CHECK(encode(m,bad)); CHECK(bad.size == 250); CHECK(decode(bad,out));
    m.size = max_payload_bytes + 1; auto before = bad; CHECK(!encode(m,bad)); CHECK(bad.bytes == before.bytes);
    m = command(); m.header.source = NodeId::pocketqube; CHECK(!encode(m,bad));
    m = command(); m.header.destination_session = 0; CHECK(!encode(m,bad));
    m = command(); m.header.kind = Kind::result; CHECK(!encode(m,bad));
    m = command(); m.header.id = 0; CHECK(!encode(m,bad));
    for (int i = 3; i < 256; ++i) { m = command(); m.header.destination = static_cast<NodeId>(i); CHECK(!encode(m,bad)); }
}
void telemetry() {
    FakeRadio f; NodeCom n(f,config(NodeId::container)); CHECK(n.valid());
    std::uint8_t value = 9; CHECK(n.publishTelemetry(&value,1)); CHECK(n.setTelemetryRate(4));
    CHECK(n.tick(0)); CHECK(f.sent.size() == 1); CHECK(n.tick(249)); CHECK(f.sent.size() == 1);
    CHECK(n.tick(250)); CHECK(f.sent.size() == 2); CHECK(!n.tick(249));
    CHECK(n.setTelemetryRate(1)); CHECK(n.tick(1249)); CHECK(f.sent.size() == 2);
    value = 7; CHECK(n.publishTelemetry(&value,1)); CHECK(n.tick(1250)); CHECK(f.sent.size() == 3);
    Message out; CHECK(decode(f.sent.back().data,out)); CHECK(out.payload[0] == 7);
    CHECK(n.tick(20000)); CHECK(f.sent.size() == 4); CHECK(n.tick(20000)); CHECK(f.sent.size() == 4);
    CHECK(n.setTelemetryRate(0)); CHECK(n.tick(30000)); CHECK(f.sent.size() == 4); CHECK(!n.setTelemetryRate(3));
    FakeRadio g; NodeCom ground(g,config(NodeId::ground)); inject(g,NodeId::container,f.sent.back().data);
    CHECK(ground.tick(0)); CHECK(ground.takeTelemetry(out)); CHECK(out.payload[0] == 7);
    CHECK(!ground.publishTelemetry(&value,1)); CHECK(!ground.setTelemetryRate(4));
}
void commands() {
    FakeRadio g,c; NodeCom ground(g,config(NodeId::ground)), container(c,config(NodeId::container));
    std::uint8_t data = 9;
    auto id = ground.submitCommand(NodeId::container,&data,1,0); CHECK(id); CHECK(!ground.submitCommand(NodeId::container,&data,1,0));
    CHECK(ground.tick(0)); CHECK(g.sent.size() == 1);
    // MAC success without any node receipt cannot complete the command.
    CHECK(ground.tick(99)); CommandEvent event; CHECK(!ground.takeResult(event));
    CHECK(ground.tick(100)); CHECK(g.sent.size() == 2); CHECK(g.sent[0].data.bytes == g.sent[1].data.bytes);
    inject(c,NodeId::ground,g.sent[0].data); inject(c,NodeId::ground,g.sent[1].data);
    CHECK(container.tick(100)); CHECK(container.stats().duplicates == 1);
    Message request; CHECK(container.takeCommand(request)); CHECK(!container.takeCommand(request));
    CHECK(!container.reportCommandResult(*id,ResultCode::completed));
    CHECK(container.reportCommandResult(*id,ResultCode::accepted)); CHECK(container.reportCommandResult(*id,ResultCode::completed));
    CHECK(!container.reportCommandResult(*id,ResultCode::failed));
    for (unsigned t = 101; t < 106; ++t) CHECK(container.tick(t));
    CHECK(c.sent.size() == 4); // two receipts, acceptance, observed completion supplied by caller
    inject(g,NodeId::container,c.sent[2].data); inject(g,NodeId::container,c.sent[0].data);
    CHECK(ground.tick(105)); CHECK(ground.takeResult(event)); CHECK(event.result == ResultCode::accepted);
    CHECK(!ground.takeResult(event)); CHECK(ground.stats().duplicates == 1);
    inject(g,NodeId::container,c.sent.back().data); CHECK(ground.tick(106));
    CHECK(ground.takeResult(event)); CHECK(event.result == ResultCode::completed && event.id == *id);
    CHECK(ground.submitCommand(NodeId::container,&data,1,107));
    // A changed payload with an existing identity must not execute.
    Message changed; CHECK(decode(g.sent[0].data,changed)); changed.payload[0] ^= 1;
    inject(c,NodeId::ground,wire(changed)); CHECK(container.tick(107)); CHECK(container.stats().malformed == 1);
    CHECK(!container.takeCommand(request));
}
void failures() {
    std::uint8_t data = 1; Message request; CommandEvent event;
    FakeRadio g; NodeCom ground(g,config(NodeId::ground)); CHECK(ground.submitCommand(NodeId::container,&data,1,0));
    for (auto t : {0,100,200,300,400,500}) CHECK(ground.tick(t));
    CHECK(g.sent.size() == 3); CHECK(ground.takeResult(event)); CHECK(event.result == ResultCode::unknown);
    CHECK(ground.stats().expired == 1);
    FakeRadio missing; missing.produce_completion = false; NodeCom stalled(missing,config(NodeId::ground));
    CHECK(stalled.submitCommand(NodeId::container,&data,1,0)); CHECK(stalled.tick(0)); CHECK(stalled.tick(500));
    CHECK(stalled.takeResult(event)); CHECK(event.result == ResultCode::unknown); CHECK(missing.sent.size() == 1);
    FakeRadio c; auto cfg = config(NodeId::container); cfg.session = 44; NodeCom rebooted(c,cfg);
    inject(c,NodeId::ground,wire(command())); CHECK(rebooted.tick(0)); CHECK(!rebooted.takeCommand(request));
    CHECK(rebooted.stats().rejected_peer == 1); // No replay into a different receiver boot.
    FakeRadio filtering; NodeCom receiver(filtering,config(NodeId::container));
    auto wrong = command(); wrong.header.team = 4321; inject(filtering,NodeId::ground,wire(wrong));
    wrong = command(); wrong.header.session = 777; inject(filtering,NodeId::ground,wire(wrong));
    inject(filtering,NodeId::pocketqube,wire(command()));
    CHECK(receiver.tick(0)); CHECK(receiver.stats().rejected_peer == 3); CHECK(!receiver.takeCommand(request));
    FakeRadio busy; busy.behavior = Submit::busy; NodeCom waiting(busy,config(NodeId::ground));
    CHECK(waiting.submitCommand(NodeId::container,&data,1,0)); CHECK(waiting.tick(0)); CHECK(busy.sent.empty());
    busy.behavior = Submit::queued; busy.mac_success = false; CHECK(waiting.tick(1)); CHECK(waiting.tick(2));
    CHECK(waiting.stats().mac_failed == 1); CHECK(!waiting.takeResult(event));
    FakeRadio peers; NodeCom two(peers,config(NodeId::ground)); CHECK(two.submitCommand(NodeId::container,&data,1,0));
    CHECK(two.submitCommand(NodeId::pocketqube,&data,1,0)); CHECK(two.tick(0)); CHECK(two.tick(1));
    CHECK(peers.sent.size() == 2 && peers.sent[0].peer != peers.sent[1].peer);
    auto bad = config(NodeId::ground); bad.session = 0; NodeCom invalid(peers,bad); CHECK(!invalid.valid()); CHECK(!invalid.tick(0));
}
void queue_and_order() {
    FakeRadio f; NodeCom receiver(f,config(NodeId::container));
    for (unsigned i = 1; i <= 5; ++i) inject(f,NodeId::ground,wire(command(i)));
    CHECK(receiver.tick(0)); CHECK(receiver.stats().queue_full == 1);
    Message m; unsigned taken = 0; while (receiver.takeCommand(m)) { ++taken; CHECK(receiver.reportCommandResult(m.header.id,ResultCode::rejected)); }
    CHECK(taken == 4);
    inject(f,NodeId::ground,wire(command(9))); CHECK(receiver.tick(1)); CHECK(receiver.takeCommand(m)); CHECK(m.header.id == 9);
    inject(f,NodeId::ground,wire(command(8))); CHECK(receiver.tick(2)); CHECK(!receiver.takeCommand(m));
    CHECK(receiver.reportCommandResult(9,ResultCode::rejected));
    // Eviction of completed receipts cannot allow an older identity to execute.
    inject(f,NodeId::ground,wire(command(1))); CHECK(receiver.tick(3)); CHECK(!receiver.takeCommand(m));
}
}  // namespace
int main() { codec(); telemetry(); commands(); failures(); queue_and_order(); std::cout << "node_com: " << checks << " checks passed; no hardware\n"; }
