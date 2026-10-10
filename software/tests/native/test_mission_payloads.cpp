#include <cansat/mission_payloads.hpp>
#include <cansat/node_com.hpp>
#include "fake_radio.hpp"
#include <algorithm>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

using namespace cansat::com;
namespace p = cansat::com::payload;
namespace {
unsigned checks = 0;
#define CHECK(x) do { ++checks; if (!(x)) { std::cerr << "FAILED line " << __LINE__ << ": " #x "\n"; std::exit(1); } } while (false)
bool equal(const p::Bytes& a, const p::Bytes& b) { return a.size == b.size && a.data == b.data; }
std::string hex(const p::Bytes& b) {
    std::ostringstream out;
    for (unsigned i = 0; i < b.size; ++i) out << std::hex << std::setfill('0') << std::setw(2) << unsigned(b.data[i]);
    return out.str();
}
p::CommonTelemetry common() {
    p::CommonTelemetry c;
    c.mission_time_ms = 4000; c.packet_count = 42; c.command_count = 7; c.mode = p::Mode::flight;
    c.valid = p::container_valid_bits; c.altitude_mm = 123456; c.pressure_pa = 98765;
    c.temperature_milli_c = -5000; c.battery_mv = 4200; c.battery_current_ma = -12;
    c.mechanism_state = 2; c.mechanism_known = 3; CHECK(p::setText(c.command_echo,"ARM")); return c;
}
p::ContainerTelemetry container() { return {common(),p::ContainerState::apogee}; }
p::PocketTelemetry pocket() {
    p::PocketTelemetry x; x.common = common(); x.common.mode = p::Mode::simulation; x.common.valid = p::pocket_valid_bits;
    x.common.altitude_mm = -1200; x.common.pressure_pa = 101325; x.common.temperature_milli_c = 22000;
    x.common.battery_current_ma = 125; x.common.mechanism_state = 5; x.common.mechanism_known = 15;
    CHECK(p::setText(x.common.command_echo,"VIEW-90000"));
    x.gyro_mdeg_s = {123,-456,0}; x.accel_mm_s2 = {0,9810,-1}; x.mag_milligauss = {123,456,-789};
    x.latitude_nanodeg = 43500000000LL; x.longitude_nanodeg = -79400000000LL;
    x.gnss_altitude_mm = 100250; x.gnss_satellites = 12; CHECK(p::setText(x.gnss_time,"123456.789"));
    x.solar_mv = {4123,4056}; return x;
}
void telemetry() {
    CHECK(p::bit(static_cast<p::Field>(255)) == 0);
    const auto c = container(); const auto pq = pocket();
    p::Bytes bytes, again; p::ContainerTelemetry cd; p::PocketTelemetry pd;
    CHECK(p::encodeContainer(c,bytes)); CHECK(bytes.size == p::container_bytes);
    std::cout << "GOLDEN_container=" << hex(bytes) << '\n';
    CHECK(p::decodeContainer(bytes.data.data(),bytes.size,cd)); CHECK(p::encodeContainer(cd,again)); CHECK(equal(bytes,again));
    CHECK(cd.common.temperature_milli_c == -5000 && cd.common.battery_current_ma == -12);
    for (unsigned n = 0; n < bytes.size; ++n) CHECK(!p::decodeContainer(bytes.data.data(),n,cd));
    CHECK(!p::decodeContainer(bytes.data.data(),bytes.size+1,cd)); CHECK(!p::decodeContainer(nullptr,bytes.size,cd));
    CHECK(p::encodePocket(pq,bytes)); CHECK(bytes.size == p::pocket_base_bytes);
    std::cout << "GOLDEN_pocket=" << hex(bytes) << '\n';
    CHECK(p::decodePocket(bytes.data.data(),bytes.size,pd)); CHECK(p::encodePocket(pd,again)); CHECK(equal(bytes,again));
    CHECK(pd.longitude_nanodeg == -79400000000LL && pd.gyro_mdeg_s[1] == -456);
    for (unsigned n = 0; n < bytes.size; ++n) CHECK(!p::decodePocket(bytes.data.data(),n,pd));
    CHECK(!p::decodePocket(bytes.data.data(),bytes.size+1,pd)); CHECK(!p::decodePocket(nullptr,bytes.size,pd));
    auto max = pq; max.stabilization.profile = 1; max.stabilization.version = 1; max.stabilization.size = 20;
    max.science.profile = 2; max.science.version = 3; max.science.size = 23;
    for (unsigned i = 0; i < 20; ++i) max.stabilization.data[i] = i;
    for (unsigned i = 0; i < 23; ++i) max.science.data[i] = 255-i;
    CHECK(p::encodePocket(max,bytes)); CHECK(bytes.size == 220);
    std::cout << "GOLDEN_pocket_max=" << hex(bytes) << '\n';
    CHECK(p::decodePocket(bytes.data.data(),bytes.size,pd)); CHECK(p::encodePocket(pd,again)); CHECK(equal(bytes,again));
    Message m; m.header = {Kind::telemetry,1234,NodeId::pocketqube,NodeId::ground,33,0,1,0};
    m.payload = bytes.data; m.size = bytes.size; Datagram d; CHECK(encode(m,d)); CHECK(d.size == 250);
    Message decoded; CHECK(decode(d,decoded)); CHECK(p::decodePocket(decoded.payload.data(),decoded.size,pd));
    max.science.size = 24; const auto before = bytes; CHECK(!p::encodePocket(max,bytes)); CHECK(equal(before,bytes));
    max.science.size = 44; CHECK(!p::encodePocket(max,bytes));
    auto badp = pq; badp.stabilization.profile = 1; CHECK(!p::encodePocket(badp,bytes));
    badp = pq; badp.stabilization.size = 1; CHECK(!p::encodePocket(badp,bytes));
    badp = pq; badp.common.valid &= ~p::bit(p::Field::longitude); CHECK(!p::encodePocket(badp,bytes));
    badp.longitude_nanodeg = 0; CHECK(!p::encodePocket(badp,bytes)); // half a position fix is not a coordinate pair
    badp = pq; badp.latitude_nanodeg = 90000000001LL; CHECK(!p::encodePocket(badp,bytes));
    badp.latitude_nanodeg = INT64_MIN; CHECK(!p::encodePocket(badp,bytes));
    badp = pq; badp.longitude_nanodeg = -180000000001LL; CHECK(!p::encodePocket(badp,bytes));
    badp = pq; badp.latitude_nanodeg = -90000000000LL; badp.longitude_nanodeg = 180000000000LL; CHECK(p::encodePocket(badp,bytes));
    CHECK(p::decodePocket(bytes.data.data(),bytes.size,pd)); CHECK(pd.latitude_nanodeg == badp.latitude_nanodeg);
    badp = pq; badp.gnss_time = {}; CHECK(!p::encodePocket(badp,bytes));
    badp = pq; badp.common.valid &= ~p::bit(p::Field::gnss_time); CHECK(!p::encodePocket(badp,bytes));
    auto badc = c; badc.common.valid |= 1u << 31; CHECK(!p::encodeContainer(badc,bytes));
    badc = c; badc.common.valid &= ~p::bit(p::Field::battery_current); CHECK(!p::encodeContainer(badc,bytes));
    badc.common.battery_current_ma = 0; CHECK(p::encodeContainer(badc,bytes));
    badc = c; badc.common.battery_current_ma = 0; CHECK(p::encodeContainer(badc,again)); CHECK(!equal(bytes,again)); // real zero != missing
    badc = c; badc.common.pressure_pa = 0; CHECK(!p::encodeContainer(badc,bytes));
    badc = c; badc.common.temperature_milli_c = -273151; CHECK(!p::encodeContainer(badc,bytes));
    badc.common.temperature_milli_c = -273150; CHECK(p::encodeContainer(badc,bytes));
    badc = c; badc.common.mechanism_known = 0; CHECK(!p::encodeContainer(badc,bytes));
    badc = c; badc.common.mechanism_known |= 4; CHECK(!p::encodeContainer(badc,bytes));
    badc = c; badc.common.mode = p::Mode::unset; CHECK(!p::encodeContainer(badc,bytes));
    badc = c; badc.state = static_cast<p::ContainerState>(4); CHECK(!p::encodeContainer(badc,bytes));
    badc = c; CHECK(p::setText(badc.common.command_echo,"=A1")); CHECK(!p::encodeContainer(badc,bytes));
    badc = c; badc.common.command_echo.data[23] = 'X'; CHECK(!p::encodeContainer(badc,bytes));
    p::Text<24> text; CHECK(!p::setText(text,"SIM,ENABLE")); CHECK(!p::setText(text,"CAL\r")); CHECK(!p::setText(text,std::string(25,'A')));
    CHECK(!p::setText(text,std::string_view("A\0B",3))); CHECK(!p::setText(text,"\xff"));
    p::ContainerTelemetry empty; CHECK(!p::encodeContainer(empty,bytes)); empty.common.mode = p::Mode::flight;
    CHECK(p::encodeContainer(empty,bytes)); CHECK(bytes.size == 71);
    std::cout << "GOLDEN_container_missing=" << hex(bytes) << '\n';
    // Transactional decode: a wrong version, padding, or schema never modifies output.
    cd = c; CHECK(p::encodeContainer(cd,again)); bytes = again; bytes.data[0] = 2;
    CHECK(!p::decodeContainer(bytes.data.data(),bytes.size,cd)); CHECK(p::encodeContainer(cd,bytes)); CHECK(equal(bytes,again));
    bytes.data[1] = 2; CHECK(!p::decodeContainer(bytes.data.data(),bytes.size,cd));
    bytes = again; bytes.data[69] = 1; CHECK(!p::decodeContainer(bytes.data.data(),bytes.size,cd));
    auto extremes = c; extremes.common.mission_time_ms = UINT64_MAX; extremes.common.packet_count = UINT32_MAX;
    extremes.common.command_count = UINT32_MAX; extremes.common.altitude_mm = INT32_MIN; extremes.common.battery_current_ma = INT32_MIN;
    CHECK(p::encodeContainer(extremes,bytes)); CHECK(p::decodeContainer(bytes.data.data(),bytes.size,cd));
    CHECK(cd.common.altitude_mm == INT32_MIN && cd.common.mission_time_ms == UINT64_MAX);
}
void commands() {
    struct Case { const char* name; NodeId node; p::Command command; unsigned size; };
    const std::vector<Case> cases = {
        {"cal",NodeId::container,p::Calibrate{},3},{"arm",NodeId::container,p::Arm{},3},
        {"utc",NodeId::container,p::SetUtc{1791648000123ULL},11},
        {"sim_enable",NodeId::container,p::Simulation{p::SimAction::enable},4},
        {"sim_activate",NodeId::pocketqube,p::Simulation{p::SimAction::activate},4},
        {"sim_disable",NodeId::pocketqube,p::Simulation{p::SimAction::disable},4},
        {"pressure",NodeId::container,p::SimulatedPressure{101325},7},
        {"release",NodeId::container,p::Deploy{p::Mechanism::container_release},4},
        {"solar1",NodeId::pocketqube,p::Deploy{p::Mechanism::solar_1},4},
        {"solar2",NodeId::pocketqube,p::Deploy{p::Mechanism::solar_2},4},
        {"boom",NodeId::pocketqube,p::Deploy{p::Mechanism::boom},4},
        {"record_on",NodeId::pocketqube,p::Recording{p::RecordAction::start},4},
        {"record_off",NodeId::pocketqube,p::Recording{p::RecordAction::stop},4},
        {"rotate",NodeId::pocketqube,p::RotateView{-90000},7}
    };
    for (const auto& c : cases) {
        p::Bytes bytes, again; p::Command decoded = p::Arm{}; p::Text<24> echo;
        CHECK(p::encodeCommand(c.node,c.command,bytes)); CHECK(bytes.size == c.size);
        std::cout << "GOLDEN_" << c.name << '=' << hex(bytes) << '\n';
        CHECK(p::decodeCommand(c.node,bytes.data.data(),bytes.size,decoded)); CHECK(p::encodeCommand(c.node,decoded,again)); CHECK(equal(bytes,again));
        CHECK(p::commandEcho(c.node,c.command,echo)); CHECK(echo.size > 0 && echo.size <= 24);
        for (unsigned n = 0; n < bytes.size; ++n) CHECK(!p::decodeCommand(c.node,bytes.data.data(),n,decoded));
        CHECK(!p::decodeCommand(c.node,bytes.data.data(),bytes.size+1,decoded)); CHECK(!p::encodeCommand(NodeId::ground,c.command,again));
        CHECK(!p::decodeCommand(NodeId::ground,bytes.data.data(),bytes.size,decoded));
    }
    p::Bytes bytes; bytes.data[0] = 99; auto before = bytes; p::Command command = p::Arm{};
    CHECK(!p::encodeCommand(NodeId::pocketqube,p::Arm{},bytes)); CHECK(equal(before,bytes));
    CHECK(!p::encodeCommand(NodeId::pocketqube,p::Calibrate{},bytes)); CHECK(!p::encodeCommand(NodeId::pocketqube,p::SetUtc{1},bytes));
    CHECK(!p::encodeCommand(NodeId::container,p::Deploy{p::Mechanism::boom},bytes));
    CHECK(!p::encodeCommand(NodeId::pocketqube,p::Deploy{p::Mechanism::container_release},bytes));
    CHECK(!p::encodeCommand(NodeId::container,p::Recording{p::RecordAction::stop},bytes));
    CHECK(!p::encodeCommand(NodeId::container,p::RotateView{90000},bytes));
    CHECK(!p::encodeCommand(NodeId::pocketqube,p::RotateView{360001},bytes)); CHECK(!p::encodeCommand(NodeId::pocketqube,p::RotateView{INT32_MIN},bytes));
    CHECK(!p::encodeCommand(NodeId::pocketqube,p::RotateView{0},bytes));
    CHECK(p::encodeCommand(NodeId::pocketqube,p::RotateView{-360000},bytes));
    CHECK(p::encodeCommand(NodeId::pocketqube,p::RotateView{360000},bytes));
    CHECK(!p::encodeCommand(NodeId::container,p::SimulatedPressure{0},bytes));
    CHECK(!p::encodeCommand(NodeId::container,p::Command{},bytes));
    CHECK(!p::encodeCommand(NodeId::container,p::Simulation{static_cast<p::SimAction>(0)},bytes));
    CHECK(!p::encodeCommand(NodeId::pocketqube,p::Recording{static_cast<p::RecordAction>(3)},bytes));
    CHECK(!p::encodeCommand(NodeId::pocketqube,p::Deploy{static_cast<p::Mechanism>(5)},bytes));
    p::Text<24> echo; CHECK(p::commandEcho(NodeId::container,p::SetUtc{UINT64_MAX},echo)); CHECK(echo.size == 22);
    const std::uint8_t bad[] = {1,3,255}; CHECK(!p::decodeCommand(NodeId::container,bad,sizeof bad,command)); CHECK(std::holds_alternative<p::Arm>(command));
    CHECK(!p::decodeCommand(NodeId::container,nullptr,0,command));
    // Codec presence does not perform the SIM ENABLE/ACTIVATE gate or any command effect.
    CHECK(p::encodeCommand(NodeId::container,p::Simulation{p::SimAction::activate},bytes));
}
void roundtrip_faults() {
    std::uint32_t rng = 123456789;
    for (unsigned i = 0; i < 1000; ++i) {
        p::Bytes bytes; bytes.size = static_cast<std::uint16_t>(i % 251);
        for (auto& value : bytes.data) { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; value = static_cast<std::uint8_t>(rng); }
        p::ContainerTelemetry c; p::PocketTelemetry q; p::Command cmd; p::Bytes encoded;
        if (p::decodeContainer(bytes.data.data(),bytes.size,c)) { CHECK(p::encodeContainer(c,encoded)); CHECK(std::equal(encoded.data.begin(),encoded.data.begin()+encoded.size,bytes.data.begin())); }
        if (p::decodePocket(bytes.data.data(),bytes.size,q)) { CHECK(p::encodePocket(q,encoded)); CHECK(std::equal(encoded.data.begin(),encoded.data.begin()+encoded.size,bytes.data.begin())); }
        for (auto node : {NodeId::container,NodeId::pocketqube}) if (p::decodeCommand(node,bytes.data.data(),bytes.size,cmd)) {
            CHECK(p::encodeCommand(node,cmd,encoded)); CHECK(std::equal(encoded.data.begin(),encoded.data.begin()+encoded.size,bytes.data.begin()));
        }
        ++checks;
    }
}
void com_boundary() {
    FakeRadio g,c; Config gc{NodeId::ground,1234,11,{11,22,33},100,500,3}; auto cc = gc; cc.self = NodeId::container; cc.session = 22;
    NodeCom ground(g,gc), receiver(c,cc); p::Bytes bytes;
    CHECK(p::encodeCommand(NodeId::container,p::Arm{},bytes)); CHECK(ground.submitCommand(NodeId::container,bytes.data.data(),bytes.size,0));
    CHECK(ground.tick(0)); CHECK(c.incoming.push({NodeId::ground,g.sent[0].data})); CHECK(receiver.tick(0));
    Message request; CHECK(receiver.takeCommand(request)); p::Command decoded;
    CHECK(p::decodeCommand(request.header.destination,request.payload.data(),request.size,decoded)); CHECK(std::holds_alternative<p::Arm>(decoded));
    // No mission state or hardware object exists in this test: receipt is NOT execution.
    Message ack; CHECK(decode(c.sent[0].data,ack)); CHECK(ack.payload[0] == static_cast<unsigned>(ResultCode::received));
}
int cli(int argc, char** argv) {
    if (argc != 4 || std::string(argv[1]) != "--decode") return 2;
    const std::string mode = argv[2], input = argv[3]; p::Bytes bytes, out;
    if (input.size() % 2 || input.size() / 2 > bytes.data.size()) return 2;
    auto digit = [](char c) -> int { if (c >= '0' && c <= '9') return c-'0'; if (c >= 'a' && c <= 'f') return c-'a'+10; if (c >= 'A' && c <= 'F') return c-'A'+10; return -1; };
    for (std::size_t i = 0; i < input.size(); i += 2) { int a = digit(input[i]), b = digit(input[i+1]); if (a < 0 || b < 0) return 2; bytes.data[bytes.size++] = static_cast<std::uint8_t>(16*a+b); }
    bool ok = false; p::ContainerTelemetry c; p::PocketTelemetry q; p::Command command;
    if (mode == "c") ok = p::decodeContainer(bytes.data.data(),bytes.size,c) && p::encodeContainer(c,out);
    if (mode == "p") ok = p::decodePocket(bytes.data.data(),bytes.size,q) && p::encodePocket(q,out);
    if (mode == "cmd_c" || mode == "cmd_p") {
        auto node = mode == "cmd_c" ? NodeId::container : NodeId::pocketqube;
        ok = p::decodeCommand(node,bytes.data.data(),bytes.size,command) && p::encodeCommand(node,command,out);
    }
    if (!ok) return 2;
    std::cout << hex(out) << '\n'; return 0;
}
}  // namespace
int main(int argc, char** argv) {
    if (argc > 1) return cli(argc,argv);
    telemetry(); commands(); roundtrip_faults(); com_boundary();
    std::cout << "mission_payloads: " << checks << " checks passed; no command execution or hardware\n";
}
