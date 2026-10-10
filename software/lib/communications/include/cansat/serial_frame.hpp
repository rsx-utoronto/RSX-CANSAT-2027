#pragma once
/** @file
 * Experimental complete-frame ASCII codec shared with Python cansat2027.
 * Allocates strings on the host; not a stream parser, radio codec or CSV exporter.
 */
#include <optional>
#include <string>
#include <string_view>

namespace cansat {
enum class Node : char { container = 'C', pocketqube = 'P' };
struct SerialFrame { Node node; std::string payload; };
constexpr std::size_t max_ascii_payload = 250;

// Experimental host/bridge ASCII envelope: C:<payload>\n or P:<payload>\n.
// Not a flight telemetry schema or a binary RF encoding. Full frames only.
/// Payload length 1..250, ASCII 0x20..0x7e, C/P only. Invalid input -> nullopt.
/// Successful full-frame length is payload+3, with one LF (not CR or CRLF).
std::optional<std::string> encodeSerialFrame(Node node, std::string_view payload);
/// Require a complete canonical frame; no buffering, delimiter recovery or partial result.
std::optional<SerialFrame> decodeSerialFrame(std::string_view frame);
}  // namespace cansat
