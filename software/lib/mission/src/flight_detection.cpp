// Sample-time qualification. Faults break continuity without rewinding flight phase.
#include <cansat/flight_detection.hpp>
#include <algorithm>
#include <cmath>

namespace cansat::mission {
namespace {
bool positive(double value) { return std::isfinite(value) && value > 0; }
bool valid(const FlightDetectionConfig& c) {
    return c.max_sample_age_ms > 0 && c.max_sample_gap_ms > 0 &&
        positive(c.max_abs_vertical_speed_mps) && positive(c.launch_altitude_m) &&
        positive(c.launch_min_climb_mps) && c.launch_confirm_ms > 0 &&
        positive(c.apogee_min_drop_m) && positive(c.apogee_min_descent_mps) &&
        c.apogee_confirm_ms > 0 && std::isfinite(c.landing_altitude_min_m) &&
        std::isfinite(c.landing_altitude_max_m) &&
        c.landing_altitude_min_m <= c.landing_altitude_max_m &&
        c.landing_altitude_max_m < c.launch_altitude_m &&
        positive(c.landing_max_abs_speed_mps) && positive(c.landing_max_span_m) &&
        c.landing_confirm_ms > 0 && c.min_confirm_samples >= 2 &&
        c.max_abs_vertical_speed_mps >= c.launch_min_climb_mps &&
        c.max_abs_vertical_speed_mps >= c.apogee_min_descent_mps &&
        c.max_abs_vertical_speed_mps >= c.landing_max_abs_speed_mps;
}
}  // namespace

AltitudeFlightDetector::AltitudeFlightDetector(FlightDetectionConfig config, DetectorStart start)
    : config_(config), config_valid_(valid(config) &&
          (start == DetectorStart::on_pad || start == DetectorStart::descending_after_release)),
      phase_(start == DetectorStart::on_pad ? DetectionPhase::waiting_for_launch
                                           : DetectionPhase::descending) {}

DetectionOutput AltitudeFlightDetector::snapshot(AltitudeStatus status) const {
    DetectionOutput out;
    out.phase = phase_;
    out.status = status;
    out.peak_altitude_m = peak_;
    return out;
}
void AltitudeFlightDetector::break_continuity() {
    previous_.reset();
    candidate_since_.reset();
    candidate_samples_ = 0;
}
bool AltitudeFlightDetector::confirm(bool eligible, std::uint64_t sample_ms, std::uint64_t dwell_ms) {
    if (!eligible) {
        candidate_since_.reset();
        candidate_samples_ = 0;
        return false;
    }
    if (!candidate_since_) candidate_since_ = sample_ms;
    if (candidate_samples_ < config_.min_confirm_samples) ++candidate_samples_;
    return candidate_samples_ >= config_.min_confirm_samples && sample_ms - *candidate_since_ >= dwell_ms;
}

DetectionOutput AltitudeFlightDetector::update(const DetectionInput& in) {
    // Faults preserve the phase and peak, but invalidate rate/confirmation history.
    const auto reject = [this](AltitudeStatus status) {
        break_continuity();
        return snapshot(status);
    };
    if (!config_valid_) return reject(AltitudeStatus::invalid_configuration);
    if (last_update_ms_ && in.now_ms < *last_update_ms_)
        return reject(AltitudeStatus::clock_regression);
    last_update_ms_ = in.now_ms;
    // Disarming before launch cancels confirmation even on a duplicate sample.
    if (phase_ == DetectionPhase::waiting_for_launch && !in.armed)
        confirm(false, 0, 0);
    if (!in.altitude) return reject(AltitudeStatus::missing);
    const auto sample = *in.altitude;
    if (!std::isfinite(sample.agl_m)) return reject(AltitudeStatus::nonfinite);
    if (sample.sampled_at_ms > in.now_ms) return reject(AltitudeStatus::future);
    if (in.now_ms - sample.sampled_at_ms > config_.max_sample_age_ms)
        return reject(AltitudeStatus::stale);
    if (last_seen_) {
        if (sample.sampled_at_ms < last_seen_->sampled_at_ms)
            return reject(AltitudeStatus::out_of_order);
        if (sample.sampled_at_ms == last_seen_->sampled_at_ms) {
            if (sample.agl_m != last_seen_->agl_m)
                return reject(AltitudeStatus::conflicting_duplicate);
            return snapshot(AltitudeStatus::duplicate);
        }
    }
    last_seen_ = sample;
    if (!previous_) {
        previous_ = sample;
        return snapshot(AltitudeStatus::seeded);
    }
    const auto dt_ms = sample.sampled_at_ms - previous_->sampled_at_ms;
    if (dt_ms > config_.max_sample_gap_ms) {
        break_continuity();
        previous_ = sample;
        return snapshot(AltitudeStatus::gap_reseeded);
    }
    const double speed = (sample.agl_m - previous_->agl_m) / (static_cast<double>(dt_ms) / 1000.0);
    if (!std::isfinite(speed) || std::abs(speed) > config_.max_abs_vertical_speed_mps)
        return reject(AltitudeStatus::implausible_rate);
    previous_ = sample;
    auto out = snapshot(AltitudeStatus::accepted);
    out.qualified_altitude = sample;
    out.vertical_speed_mps = speed;
    if (phase_ == DetectionPhase::waiting_for_launch) {
        if (confirm(in.armed && sample.agl_m >= config_.launch_altitude_m &&
                    speed >= config_.launch_min_climb_mps, sample.sampled_at_ms, config_.launch_confirm_ms)) {
            phase_ = DetectionPhase::ascending;
            peak_ = sample.agl_m;
            out.event = FlightEvent::launch;
        }
    } else if (phase_ == DetectionPhase::ascending) {
        peak_ = std::max(*peak_, sample.agl_m);
        if (confirm(*peak_ - sample.agl_m >= config_.apogee_min_drop_m &&
                    speed <= -config_.apogee_min_descent_mps, sample.sampled_at_ms, config_.apogee_confirm_ms)) {
            phase_ = DetectionPhase::descending;
            out.event = FlightEvent::apogee;
        }
    } else if (phase_ == DetectionPhase::descending) {
        const bool eligible = sample.agl_m >= config_.landing_altitude_min_m &&
            sample.agl_m <= config_.landing_altitude_max_m &&
            std::abs(speed) <= config_.landing_max_abs_speed_mps;
        if (eligible) {
            if (candidate_since_) {
                candidate_min_m_ = std::min(candidate_min_m_, sample.agl_m);
                candidate_max_m_ = std::max(candidate_max_m_, sample.agl_m);
                if (candidate_max_m_ - candidate_min_m_ > config_.landing_max_span_m)
                    candidate_since_.reset();
            }
            if (!candidate_since_) {
                candidate_samples_ = 0;
                candidate_min_m_ = candidate_max_m_ = sample.agl_m;
            }
        }
        if (confirm(eligible, sample.sampled_at_ms, config_.landing_confirm_ms)) {
            phase_ = DetectionPhase::landed;
            out.event = FlightEvent::landed;
        }
    }
    if (out.event != FlightEvent::none) {
        candidate_since_.reset();
        candidate_samples_ = 0;
    }
    out.phase = phase_;
    out.peak_altitude_m = peak_;
    return out;
}
}  // namespace cansat::mission
