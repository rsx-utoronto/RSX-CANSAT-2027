#pragma once
/** @file
 * Configurable altitude-only event qualification using sample-time finite differences.
 * Explicit configuration, no flight defaults, no pressure conversion or sensor fusion.
 * Outputs feed mission models; events do not directly operate mechanisms.
 */
#include <cansat/mission.hpp>
#include <cstddef>

namespace cansat::mission {
// No flight defaults: zero-initialized configuration is deliberately invalid.
struct FlightDetectionConfig {
    std::uint64_t max_sample_age_ms = 0;
    std::uint64_t max_sample_gap_ms = 0;
    double max_abs_vertical_speed_mps = 0;
    double launch_altitude_m = 0;
    double launch_min_climb_mps = 0;
    std::uint64_t launch_confirm_ms = 0;
    double apogee_min_drop_m = 0;
    double apogee_min_descent_mps = 0;
    std::uint64_t apogee_confirm_ms = 0;
    double landing_altitude_min_m = 0;
    double landing_altitude_max_m = 0;
    double landing_max_abs_speed_mps = 0;
    double landing_max_span_m = 0;
    std::uint64_t landing_confirm_ms = 0;
    std::size_t min_confirm_samples = 0;
};
enum class DetectorStart { on_pad, descending_after_release };
enum class DetectionPhase { waiting_for_launch, ascending, descending, landed };
enum class AltitudeStatus {
    invalid_configuration, clock_regression, missing, nonfinite, future, stale,
    out_of_order, conflicting_duplicate, duplicate, seeded, gap_reseeded,
    implausible_rate, accepted
};
struct DetectionInput {
    std::uint64_t now_ms = 0;
    std::optional<Altitude> altitude;
    bool armed = false;  // Level from command state; gates launch only.
};
struct DetectionOutput {
    DetectionPhase phase = DetectionPhase::waiting_for_launch;
    FlightEvent event = FlightEvent::none;  // One-shot, consume in the same update.
    AltitudeStatus status = AltitudeStatus::invalid_configuration;
    // Only rate-checked samples are forwarded; seed/rejected/duplicate => empty.
    std::optional<Altitude> qualified_altitude;
    std::optional<double> vertical_speed_mps;
    std::optional<double> peak_altitude_m;
};

class AltitudeFlightDetector {
public:
    AltitudeFlightDetector(FlightDetectionConfig config, DetectorStart start);
    bool configuration_valid() const { return config_valid_; }
    /// Faults clear rate/dwell continuity, but retain phase and peak. Equal fresh
    /// samples are not new votes. Seed/rejected/duplicate inputs forward no altitude.
    /// Consume event and qualified_altitude together; never substitute raw input.
    DetectionOutput update(const DetectionInput& input);
private:
    DetectionOutput snapshot(AltitudeStatus status) const;
    void break_continuity();
    bool confirm(bool eligible, std::uint64_t sample_ms, std::uint64_t dwell_ms);
    FlightDetectionConfig config_;
    bool config_valid_;
    DetectionPhase phase_;
    std::optional<std::uint64_t> last_update_ms_;
    std::optional<Altitude> last_seen_;
    std::optional<Altitude> previous_;
    std::optional<double> peak_;
    std::optional<std::uint64_t> candidate_since_;
    std::size_t candidate_samples_ = 0;
    double candidate_min_m_ = 0;
    double candidate_max_m_ = 0;
};
}  // namespace cansat::mission
