// Explicit little-endian radio envelope and reflected CRC-32/ISO-HDLC.
// Output assignment happens only after complete bounds/semantic validation.
#include <cansat/radio_protocol.hpp>
#include <algorithm>

namespace cansat::com {
namespace {
void put(std::uint8_t* p, std::uint32_t value, unsigned width) {
    for (unsigned i = 0; i < width; ++i) p[i] = static_cast<std::uint8_t>(value >> (8 * i));
}
std::uint32_t get(const std::uint8_t* p, unsigned width) {
    std::uint32_t value = 0;
    for (unsigned i = 0; i < width; ++i) value |= std::uint32_t{p[i]} << (8 * i);
    return value;
}
std::uint32_t crc(const std::uint8_t* p, std::size_t n) {
    std::uint32_t value = 0xffffffffu;
    for (std::size_t i = 0; i < n; ++i) {
        value ^= p[i];
        for (int j = 0; j < 8; ++j)
            value = (value >> 1) ^ ((value & 1u) ? 0xedb88320u : 0u);
    }
    return ~value;
}
bool valid(const Message& m) {
    const auto& h = m.header;
    if (!isStarLink(h.source, h.destination) || h.team == 0 || h.team > 9999 ||
        h.session == 0 || h.id == 0 || m.size == 0 || m.size > max_payload_bytes) return false;
    switch (h.kind) {
    case Kind::telemetry:
        return h.source != NodeId::ground && h.destination_session == 0 && h.correlation == 0;
    case Kind::command:
        return h.source == NodeId::ground && h.destination_session != 0 && h.correlation == 0;
    case Kind::result:
        return h.destination == NodeId::ground && h.destination_session != 0 && h.correlation != 0 &&
            m.size == 1 && m.payload[0] >= static_cast<unsigned>(ResultCode::received) &&
            m.payload[0] <= static_cast<unsigned>(ResultCode::unknown);
    }
    return false;
}
}  // namespace
bool isNode(NodeId n) { return n == NodeId::ground || n == NodeId::container || n == NodeId::pocketqube; }
bool isStarLink(NodeId s, NodeId d) {
    return isNode(s) && isNode(d) && s != d && (s == NodeId::ground || d == NodeId::ground);
}
bool encode(const Message& m, Datagram& output) {
    if (!valid(m)) return false;
    Datagram d;
    auto* p = d.bytes.data();
    p[0] = 'C'; p[1] = 'S'; p[2] = 1; p[3] = static_cast<std::uint8_t>(m.header.kind);
    put(p + 4, m.header.team, 2);
    p[6] = static_cast<std::uint8_t>(m.header.source); p[7] = static_cast<std::uint8_t>(m.header.destination);
    put(p + 8, m.header.session, 4); put(p + 12, m.header.destination_session, 4);
    put(p + 16, m.header.id, 4); put(p + 20, m.header.correlation, 4); put(p + 24, m.size, 2);
    std::copy_n(m.payload.begin(), m.size, d.bytes.begin() + header_bytes);
    d.size = static_cast<std::uint16_t>(header_bytes + m.size + checksum_bytes);
    put(p + d.size - checksum_bytes, crc(p, d.size - checksum_bytes), 4);
    output = d;
    return true;
}
bool decode(const Datagram& d, Message& output) {
    if (d.size <= header_bytes + checksum_bytes || d.size > max_radio_bytes) return false;
    const auto* p = d.bytes.data();
    if (p[0] != 'C' || p[1] != 'S' || p[2] != 1 ||
        get(p + d.size - checksum_bytes, 4) != crc(p, d.size - checksum_bytes)) return false;
    Message m;
    m.size = static_cast<std::uint16_t>(get(p + 24, 2));
    if (m.size > max_payload_bytes || header_bytes + m.size + checksum_bytes != d.size) return false;
    m.header = {static_cast<Kind>(p[3]), static_cast<std::uint16_t>(get(p + 4, 2)),
        static_cast<NodeId>(p[6]), static_cast<NodeId>(p[7]), get(p + 8, 4), get(p + 12, 4),
        get(p + 16, 4), get(p + 20, 4)};
    std::copy_n(d.bytes.begin() + header_bytes, m.size, m.payload.begin());
    if (!valid(m)) return false;
    output = m;
    return true;
}
}  // namespace cansat::com
