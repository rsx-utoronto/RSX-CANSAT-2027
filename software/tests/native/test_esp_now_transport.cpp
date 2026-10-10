#include <cansat/esp_now_transport.hpp>
#include <cstring>
#include <cstdlib>
#include <iostream>

using namespace cansat;
namespace {
unsigned checks = 0, calls = 0, deinit_calls = 0, peers_added = 0;
int add_error = 0, send_error = 0;
wifi_mode_t mode = WIFI_MODE_STA;
std::uint8_t wifi_channel = 6;
const std::uint8_t own_mac[6] = {2,1,2,3,4,5};
esp_now_recv_cb_t rx = nullptr;
esp_now_send_cb_t tx = nullptr;
#define CHECK(x) do { ++checks; if (!(x)) { std::cerr << "FAILED line " << __LINE__ << ": " #x "\n"; std::exit(1); } } while (false)
}
esp_err_t esp_wifi_get_mode(wifi_mode_t* out) { ++calls; *out = mode; return 0; }
esp_err_t esp_wifi_get_channel(std::uint8_t* out, wifi_second_chan_t* secondary) { ++calls; *out = wifi_channel; *secondary = WIFI_SECOND_CHAN_NONE; return 0; }
esp_err_t esp_wifi_get_mac(wifi_interface_t, std::uint8_t* out) { ++calls; std::memcpy(out,own_mac,6); return 0; }
esp_err_t esp_now_init() { ++calls; return 0; }
esp_err_t esp_now_deinit() { ++deinit_calls; return 0; }
esp_err_t esp_now_set_pmk(const std::uint8_t*) { ++calls; return 0; }
esp_err_t esp_now_add_peer(const esp_now_peer_info_t* p) { ++peers_added; CHECK(p->encrypt && p->channel == wifi_channel); return add_error; }
esp_err_t esp_now_register_recv_cb(esp_now_recv_cb_t c) { rx = c; return 0; }
esp_err_t esp_now_register_send_cb(esp_now_send_cb_t c) { tx = c; return 0; }
esp_err_t esp_now_unregister_recv_cb() { rx = nullptr; return 0; }
esp_err_t esp_now_unregister_send_cb() { tx = nullptr; return 0; }
esp_err_t esp_now_send(const std::uint8_t*, const std::uint8_t*, std::size_t) { ++calls; return send_error; }
int main() {
    using namespace esp_idf;
    auto& a = EspNowTransport::instance(); CHECK(!a.active()); CHECK(calls == 0);
    RadioConfig c; CHECK(a.start(c) == RadioStart::invalid_config); CHECK(calls == 0);
    com::Datagram d; CHECK(a.send(com::NodeId::container,d,1) == com::Submit::rejected); CHECK(calls == 0);
    c.self = com::NodeId::ground; c.channel = 6; c.peer_count = 2; c.primary_key.fill(1);
    c.peers[0] = {com::NodeId::container,{2,2,3,4,5,6},{}}; c.peers[0].local_key.fill(2);
    c.peers[1] = {com::NodeId::pocketqube,{2,3,4,5,6,7},{}}; c.peers[1].local_key.fill(3);
    CHECK(EspNowTransport::validConfig(c));
    auto bad = c; bad.peers[0].mac.fill(255); CHECK(!EspNowTransport::validConfig(bad));
    bad = c; bad.peers[1] = bad.peers[0]; CHECK(!EspNowTransport::validConfig(bad));
    bad = c; bad.peers[0].local_key.fill(0); CHECK(!EspNowTransport::validConfig(bad));
    bad = c; bad.channel = 0; CHECK(!EspNowTransport::validConfig(bad));
    mode = WIFI_MODE_NULL; CHECK(a.start(c) == RadioStart::wifi_not_ready); mode = WIFI_MODE_STA;
    wifi_channel = 1; CHECK(a.start(c) == RadioStart::wifi_not_ready); wifi_channel = 6;
    add_error = -1; CHECK(a.start(c) == RadioStart::sdk_error); CHECK(deinit_calls == 1 && !a.active());
    CHECK(!rx && !tx); add_error = 0;
    CHECK(a.start(c) == RadioStart::ok); CHECK(a.active()); CHECK(rx && tx); CHECK(peers_added == 3);
    CHECK(a.start(c) == RadioStart::already_active);
    com::Message m; m.header = {com::Kind::telemetry,1234,com::NodeId::container,com::NodeId::ground,22,0,1,0};
    m.size = 1; m.payload[0] = 99; CHECK(com::encode(m,d));
    esp_now_recv_info_t info{c.peers[0].mac.data(),own_mac};
    rx(&info,d.bytes.data(),d.size); auto saved = d.bytes[0]; d.bytes[0] = 0;
    com::Received out; CHECK(a.receive(out)); CHECK(out.datagram.bytes[0] == saved && out.peer == com::NodeId::container);
    d.bytes[0] = saved; CHECK(!a.receive(out));
    auto before = a.dropped(); rx(nullptr,nullptr,0); rx(&info,d.bytes.data(),251); rx(&info,d.bytes.data(),-1);
    info.des_addr = c.peers[0].mac.data(); rx(&info,d.bytes.data(),d.size);
    info.des_addr = own_mac; info.src_addr = own_mac; rx(&info,d.bytes.data(),d.size);
    CHECK(a.dropped() == before + 5); CHECK(!a.receive(out)); info.src_addr = c.peers[0].mac.data();
    for (int i = 0; i < 9; ++i) rx(&info,d.bytes.data(),d.size);
    unsigned count = 0; while (a.receive(out)) ++count; CHECK(count == 8); CHECK(a.dropped() == before + 6);
    CHECK(a.send(com::NodeId::ground,d,1) == com::Submit::rejected);
    CHECK(a.send(com::NodeId::container,d,0) == com::Submit::rejected);
    send_error = -1; CHECK(a.send(com::NodeId::container,d,1) == com::Submit::rejected); send_error = 0;
    CHECK(a.send(com::NodeId::container,d,42) == com::Submit::queued);
    CHECK(a.send(com::NodeId::container,d,43) == com::Submit::busy);
    esp_now_send_info_t sent_info{c.peers[0].mac.data()}; tx(&sent_info,ESP_NOW_SEND_FAIL);
    CHECK(a.send(com::NodeId::container,d,43) == com::Submit::busy); // wait for owner to consume completion
    com::Sent completion; CHECK(a.completion(completion)); CHECK(completion.token == 42 && !completion.delivered);
    CHECK(!a.completion(completion)); CHECK(a.send(com::NodeId::container,d,43) == com::Submit::queued);
    tx(&sent_info,ESP_NOW_SEND_SUCCESS); CHECK(a.completion(completion)); CHECK(completion.delivered && completion.token == 43);
    std::cout << "esp_now_adapter: " << checks << " sequential SDK-double checks passed; no radio hardware\n";
}
