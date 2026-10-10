// Canonical whole-frame host ASCII codec; intentionally no UART stream state.
#include <cansat/serial_frame.hpp>

namespace cansat {
namespace {
bool validPayload(std::string_view payload) {
    if (payload.empty() || payload.size() > max_ascii_payload) return false;
    for (const unsigned char byte : payload) {
        if (byte < 0x20 || byte > 0x7e) return false;
    }
    return true;
}
bool validNode(Node node) { return node == Node::container || node == Node::pocketqube; }
}  // namespace

std::optional<std::string> encodeSerialFrame(Node node, std::string_view payload) {
    if (!validNode(node) || !validPayload(payload)) return std::nullopt;
    std::string result;
    result.reserve(payload.size() + 3);
    result.push_back(static_cast<char>(node));
    result.push_back(':');
    result.append(payload);
    result.push_back('\n');
    return result;
}

std::optional<SerialFrame> decodeSerialFrame(std::string_view frame) {
    if (frame.size() < 4 || frame.size() > max_ascii_payload + 3 ||
        frame[1] != ':' || frame.back() != '\n') return std::nullopt;
    const auto node = static_cast<Node>(frame.front());
    const auto payload = frame.substr(2, frame.size() - 3);
    if (!validNode(node) || !validPayload(payload)) return std::nullopt;
    return SerialFrame{node, std::string{payload}};
}
}  // namespace cansat
