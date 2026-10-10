#pragma once
/** @file
 * Version-1 mission payload types and canonical little-endian codecs.
 * communications/mission_payloads.json defines offsets, units, enums and CSV mappings.
 * Allocation-free values/codecs; validate wire syntax here and mission authority elsewhere.
 */
#include <cansat/radio_protocol.hpp>
#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <variant>

namespace cansat::com::payload {
constexpr std::uint8_t version = 1;
constexpr std::size_t container_bytes = 71, pocket_base_bytes = 177;
constexpr std::size_t extension_budget = max_payload_bytes - pocket_base_bytes;
static_assert(extension_budget == 43);
/// Encoded bytes: only [0,size) is on wire; never transmit sizeof(Bytes).
struct Bytes {
    std::array<std::uint8_t, max_payload_bytes> data{};
    std::uint16_t size = 0;
};
/// Fixed padded ASCII storage; size is a byte count, NOT a C-string terminator.
/// All positions at/after size must be zero; encoders reject noncanonical padding.
template<std::size_t N> struct Text {
    std::array<char, N> data{};
    std::uint8_t size = 0;
};
/// Copy bounded ASCII excluding whitespace/comma/control; zero-fill padding.
/// False leaves out unchanged. Echo has an additional, narrower token grammar.
template<std::size_t N> bool setText(Text<N>& out, std::string_view value) {
    static_assert(N <= 255);
    if (value.size() > N) return false;
    for (unsigned char c : value) if (c < 0x21 || c > 0x7e || c == ',') return false;
    Text<N> text;
    text.size = static_cast<std::uint8_t>(value.size());
    for (std::size_t i = 0; i < value.size(); ++i) text.data[i] = value[i];
    out = text; return true;
}
enum class Mode : std::uint8_t { unset = 0, flight = 'F', simulation = 'S' };
// Published Container wire states, NOT the internal mission enum ordinal values.
enum class ContainerState : std::uint8_t { launch_pad = 0, ascent = 1, apogee = 2, pq_release = 3 };
enum class Field : unsigned {
    altitude = 0, pressure, temperature, battery_voltage, battery_current,
    gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, mag_x, mag_y, mag_z,
    latitude, longitude, gnss_altitude, gnss_satellites, gnss_time, solar_1, solar_2
};
constexpr std::uint32_t bit(Field f) {
    const auto index = static_cast<unsigned>(f);
    return index <= static_cast<unsigned>(Field::solar_2) ? std::uint32_t{1} << index : 0;
}
constexpr std::uint32_t container_valid_bits = 0x1fu, pocket_valid_bits = 0x1fffffu;
/// Snapshot supplied by acquisition/application code, not a sensor driver.
/// Invalid numeric fields must be zero; valid zero remains distinguishable via valid.
/// Counts/elapsed time are application-owned; no increment/persistence is performed.
struct CommonTelemetry {
    std::uint64_t mission_time_ms = 0; // Elapsed mission time, not a Unix timestamp.
    std::uint32_t packet_count = 0, command_count = 0; // Application-owned; not header IDs.
    Mode mode = Mode::unset;
    std::uint32_t valid = 0;
    std::int32_t altitude_mm = 0;
    std::uint32_t pressure_pa = 0;
    std::int32_t temperature_milli_c = 0;
    std::uint32_t battery_mv = 0;
    std::int32_t battery_current_ma = 0;
    // State bits can be set only where the corresponding known bit is set.
    std::uint8_t mechanism_state = 0, mechanism_known = 0;
    Text<24> command_echo{};
};
struct ContainerTelemetry {
    CommonTelemetry common{};
    ContainerState state = ContainerState::launch_pad;
};
/// Opaque profile/version/data tuple. Empty means profile=version=size=0.
/// Nonempty requires nonzero profile/version; only data[0,size) is serialized.
/// Stabilization and science share extension_budget bytes, not one budget each.
struct Extension {
    std::uint16_t profile = 0;
    std::uint8_t version = 0, size = 0;
    std::array<std::uint8_t, extension_budget> data{};
};
struct PocketTelemetry {
    CommonTelemetry common{};
    std::array<std::int32_t, 3> gyro_mdeg_s{}, accel_mm_s2{}, mag_milligauss{};
    std::int64_t latitude_nanodeg = 0, longitude_nanodeg = 0;
    std::int32_t gnss_altitude_mm = 0;
    std::uint16_t gnss_satellites = 0;
    Text<32> gnss_time{}; // Receiver-provided text, with no inferred time scale/epoch.
    std::array<std::uint32_t, 2> solar_mv{};
    Extension stabilization{}, science{}; // Combined data budget, not 43 bytes each.
};

enum class Op : std::uint8_t {
    calibrate = 1, arm = 2, set_utc = 3, simulation = 4, simulated_pressure = 5,
    deploy = 6, recording = 7, rotate_view = 8
};
enum class SimAction : std::uint8_t { enable = 1, activate = 2, disable = 3 };
enum class Mechanism : std::uint8_t { container_release = 1, solar_1 = 2, solar_2 = 3, boom = 4 };
enum class RecordAction : std::uint8_t { start = 1, stop = 2 };
struct Calibrate {};
struct Arm {};
struct SetUtc { std::uint64_t unix_ms; }; // Sets a UTC reference, not elapsed mission_time_ms.
struct Simulation { SimAction action; };
struct SimulatedPressure { std::uint32_t pa; };
struct Deploy { Mechanism mechanism; };
struct Recording { RecordAction action; };
struct RotateView { std::int32_t delta_mdeg; }; // Signed right-hand rotation about +Z/nadir.
/// monostate is deliberately unencodable; no zero-initialized implicit operation.
/// Wire opcode numbers are Op values, never std::variant alternative indices.
using Command = std::variant<std::monostate, Calibrate, Arm, SetUtc, Simulation,
                             SimulatedPressure, Deploy, Recording, RotateView>;

// Transactional codecs: output is untouched on failure; reject trailing bytes,
// wrong schema/version, noncanonical missing values, invalid enums and node targets.
// Decoding never authorizes or executes a command, changes mode, or claims feedback.
/// Encode exactly 71 bytes after validating values, known bits and text.
bool encodeContainer(const ContainerTelemetry&, Bytes&);
/// Read exactly 71 accessible bytes; reject null, unknown schema/version/state.
bool decodeContainer(const std::uint8_t*, std::size_t, ContainerTelemetry&);
/// Encode 177..220 bytes; enforce the combined 43-byte extension data budget.
bool encodePocket(const PocketTelemetry&, Bytes&);
/// Read one complete 177..220-byte payload; retain unknown extension profiles opaque.
bool decodePocket(const std::uint8_t*, std::size_t, PocketTelemetry&);
/// Encode 3..11 bytes for a flight destination; reject wrong-role operations/arguments.
bool encodeCommand(NodeId destination, const Command&, Bytes&);
/// Validate target and exact command syntax only. Caller binds outer source/kind,
/// authenticates authority, gates mission state and handles command identity before dispatch.
bool decodeCommand(NodeId destination, const std::uint8_t*, std::size_t, Command&);
/// Format a validated command into an at-most-24-byte token without I/O.
/// Caller publishes only after processing; generating text is not processing a command.
bool commandEcho(NodeId destination, const Command&, Text<24>&);
}  // namespace cansat::com::payload
