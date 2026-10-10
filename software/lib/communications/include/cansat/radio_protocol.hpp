#pragma once
/** @file
 * Portable version-1 radio envelope. Wire offsets live in communications/protocol.json.
 * Values are serialized explicitly; native struct layout is never the wire ABI.
 */
#include <array>
#include <cstddef>
#include <cstdint>

namespace cansat::com {
enum class NodeId : std::uint8_t { ground = 0, container = 1, pocketqube = 2 };
enum class Kind : std::uint8_t { telemetry = 1, command = 2, result = 3 };
/// received/accepted are nonterminal; every other result ends ground tracking.
/// UNKNOWN includes TTL expiry. MAC success is never an application result.
enum class ResultCode : std::uint8_t {
    received = 1, accepted = 2, rejected = 3, completed = 4, failed = 5,
    busy = 6, stale = 7, unknown = 8
};
constexpr std::size_t max_radio_bytes = 250;
constexpr std::size_t header_bytes = 26;
constexpr std::size_t checksum_bytes = 4;
constexpr std::size_t max_payload_bytes = max_radio_bytes - header_bytes - checksum_bytes;
struct Datagram {
    std::array<std::uint8_t, max_radio_bytes> bytes{};
    std::uint16_t size = 0;
};
/// team/source/destination bind identity; session is sender boot identity.
/// destination_session is zero only for telemetry; id is nonzero/session-local.
/// correlation is nonzero only for a result, pointing to the original command ID.
struct Header {
    Kind kind = Kind::telemetry;
    std::uint16_t team = 0;
    NodeId source = NodeId::container;
    NodeId destination = NodeId::ground;
    std::uint32_t session = 0;
    std::uint32_t destination_session = 0;
    std::uint32_t id = 0;
    std::uint32_t correlation = 0;
};
struct Message {
    Header header{};
    std::array<std::uint8_t, max_payload_bytes> payload{};
    std::uint16_t size = 0;
};
/// Accept only the three enumerated node values; reject invalid enum casts.
bool isNode(NodeId node);
/// True only for distinct ground <-> flight endpoints; no flight <-> flight link.
bool isStarLink(NodeId source, NodeId destination);
// Explicit little-endian encoding; CRC32 detects corruption, NOT forgery.
// Outputs remain unchanged on failure. No dynamic allocation or packed structs.
bool encode(const Message& message, Datagram& output);
bool decode(const Datagram& datagram, Message& output);
}  // namespace cansat::com
