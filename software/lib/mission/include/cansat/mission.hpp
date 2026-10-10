#pragma once
/** @file
 * Deterministic Container and PocketQube mission models, with no hardware I/O.
 * Single owner per instance; monotonic milliseconds; consume one-shot intents immediately.
 * State is volatile: reconstructing an object is not reset recovery.
 */
#include <array>
#include <cstdint>
#include <optional>

namespace cansat::mission {
// Internal phases, not wire values. Legacy OpState/CSV mapping remains separate.
enum class ContainerPhase { launch_pad, ascent, apogee, release_requested, pq_released, landed };
enum class PocketPhase { off, descending, landed };
enum class FlightEvent { none, launch, apogee, landed };

struct Altitude {
    double agl_m;
    std::uint64_t sampled_at_ms;
};
struct ContainerInput {
    std::uint64_t now_ms = 0;
    std::optional<Altitude> altitude;
    FlightEvent flight_event = FlightEvent::none;
    bool arm_command = false;
    bool release_command = false;  // Explicit ground override; no ARM prerequisite.
    // Qualified sensor evidence, not actuator return codes or unfiltered GPIO.
    bool mechanism_deployed = false;
    bool pq_clear = false;
};
struct ContainerOutput {
    bool input_accepted = true;
    bool altitude_valid = false;
    ContainerPhase phase = ContainerPhase::launch_pad;
    bool armed = false;
    double peak_altitude_m = 0;
    bool release_request = false;  // One-shot intent, NOT deployment confirmation.
    bool mechanism_deployed = false;
    bool pq_clear = false;
    unsigned telemetry_hz = 4;
};

class ContainerMission {
public:
    explicit ContainerMission(std::uint64_t max_altitude_age_ms)
        : max_altitude_age_ms_(max_altitude_age_ms) {}
    /// Reject zero max age/regressing now_ms without changing mission state.
    /// Consume qualified altitude/events; latch feedback independently of command intent.
    /// No persistence/retries: release_request must be handed off in this same update.
    ContainerOutput update(const ContainerInput& input);
private:
    ContainerOutput snapshot(std::uint64_t now_ms) const;
    std::uint64_t max_altitude_age_ms_;
    std::optional<std::uint64_t> last_update_ms_;
    std::optional<std::uint64_t> last_altitude_sample_ms_;
    std::optional<std::uint64_t> released_at_ms_;
    ContainerPhase phase_ = ContainerPhase::launch_pad;
    bool armed_ = false;
    bool release_requested_ = false;
    bool mechanism_deployed_ = false;
    double peak_altitude_m_ = 0;
};

struct RotationCommand {
    std::uint64_t sequence;  // Strictly increasing within this runtime; no wrap.
    double delta_degrees;
};
struct PocketInput {
    std::uint64_t now_ms = 0;
    // Event timestamp on the SAME monotonic timeline. No physical power control.
    std::optional<std::uint64_t> power_on_at_ms;
    bool landed = false;
    std::array<bool, 2> solar_deployed{};
    bool boom_deployed = false;
    std::optional<bool> recording_feedback;
    std::array<bool, 2> solar_commands{};
    bool boom_command = false;
    std::optional<RotationCommand> rotation;
};
struct PocketOutput {
    bool input_accepted = true;
    PocketPhase phase = PocketPhase::off;
    unsigned telemetry_hz = 0;
    bool start_recording = false;
    bool recording_confirmed = false;
    std::array<bool, 2> deploy_solar{};
    std::array<bool, 2> solar_confirmed{};
    bool deploy_boom = false;
    bool boom_confirmed = false;
    std::optional<double> rotate_to_degrees;  // Intent, not measured orientation.
    double commanded_offset_degrees = 0;
};

class PocketMission {
public:
    // Explicit flight/bench policy. This model never operates physical actuators.
    explicit PocketMission(bool automatic_solar_deployment)
        : automatic_solar_deployment_(automatic_solar_deployment) {}
    /// First valid power event anchors recording/panel deadlines on the caller clock.
    /// Reject clock regression/future first power event; never rewind a latched deadline.
    /// Mechanism/rotation requests are one-shot; qualified landing suppresses new ones.
    PocketOutput update(const PocketInput& input);
private:
    PocketOutput snapshot() const;
    bool automatic_solar_deployment_;
    std::optional<std::uint64_t> last_update_ms_;
    std::optional<std::uint64_t> powered_at_ms_;
    std::optional<std::uint64_t> last_rotation_sequence_;
    PocketPhase phase_ = PocketPhase::off;
    std::array<bool, 2> solar_requested_{};
    std::array<bool, 2> solar_confirmed_{};
    bool boom_requested_ = false;
    bool boom_confirmed_ = false;
    bool recording_confirmed_ = false;
    double commanded_offset_degrees_ = 0;
};
}  // namespace cansat::mission
