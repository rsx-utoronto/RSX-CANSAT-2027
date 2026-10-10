// Pure mission transitions and one-shot intents; feedback is qualified upstream.
#include <cansat/mission.hpp>
#include <algorithm>
#include <cmath>

namespace cansat::mission {
namespace {
constexpr std::uint64_t deployment_delay_ms = 5000;
constexpr double release_fraction = 0.90;
bool fresh(const std::optional<Altitude>& altitude, std::uint64_t now, std::uint64_t max_age) {
    return altitude && std::isfinite(altitude->agl_m) &&
           altitude->sampled_at_ms <= now && now - altitude->sampled_at_ms <= max_age;
}
}  // namespace

ContainerOutput ContainerMission::snapshot(std::uint64_t now_ms) const {
    ContainerOutput out;
    out.phase = phase_;
    out.armed = armed_;
    out.peak_altitude_m = peak_altitude_m_;
    out.mechanism_deployed = mechanism_deployed_;
    out.pq_clear = released_at_ms_.has_value();
    if (phase_ == ContainerPhase::landed) out.telemetry_hz = 0;
    else if (released_at_ms_ && now_ms >= *released_at_ms_ &&
             now_ms - *released_at_ms_ >= deployment_delay_ms) out.telemetry_hz = 1;
    return out;
}

ContainerOutput ContainerMission::update(const ContainerInput& in) {
    if (max_altitude_age_ms_ == 0 || (last_update_ms_ && in.now_ms < *last_update_ms_)) {
        auto out = snapshot(last_update_ms_.value_or(in.now_ms));
        out.input_accepted = false;
        return out;
    }
    last_update_ms_ = in.now_ms;
    const bool valid_altitude = fresh(in.altitude, in.now_ms, max_altitude_age_ms_) &&
        (!last_altitude_sample_ms_ || in.altitude->sampled_at_ms >= *last_altitude_sample_ms_);
    if (valid_altitude) last_altitude_sample_ms_ = in.altitude->sampled_at_ms;
    // Confirmation latches survive switch bounce; qualification is upstream.
    mechanism_deployed_ = mechanism_deployed_ || in.mechanism_deployed;
    if (in.pq_clear && !released_at_ms_) {
        released_at_ms_ = in.now_ms;
        if (phase_ != ContainerPhase::landed) phase_ = ContainerPhase::pq_released;
    }
    if (in.flight_event == FlightEvent::landed && phase_ != ContainerPhase::launch_pad)
        phase_ = ContainerPhase::landed;
    bool request = false;
    if (phase_ != ContainerPhase::landed && !released_at_ms_) {
        if (phase_ == ContainerPhase::launch_pad) {
            if (in.arm_command) armed_ = true;
            if (armed_ && valid_altitude && in.flight_event == FlightEvent::launch)
                phase_ = ContainerPhase::ascent;
        }
        if (phase_ == ContainerPhase::ascent && valid_altitude) {
            peak_altitude_m_ = std::max(peak_altitude_m_, in.altitude->agl_m);
            if (in.flight_event == FlightEvent::apogee) phase_ = ContainerPhase::apogee;
        }
        const bool auto_release = phase_ == ContainerPhase::apogee && valid_altitude &&
            peak_altitude_m_ > 0 && in.altitude->agl_m <= release_fraction * peak_altitude_m_;
        if (!release_requested_ && (in.release_command || auto_release)) {
            release_requested_ = true;
            phase_ = ContainerPhase::release_requested;
            request = true;
        }
    }
    auto out = snapshot(in.now_ms);
    out.altitude_valid = valid_altitude;
    out.release_request = request;
    return out;
}

PocketOutput PocketMission::snapshot() const {
    PocketOutput out;
    out.phase = phase_;
    // Unlike the container, no automatic landing cutoff is imposed on PQ telemetry.
    out.telemetry_hz = powered_at_ms_ ? 4 : 0;
    out.solar_confirmed = solar_confirmed_;
    out.boom_confirmed = boom_confirmed_;
    out.recording_confirmed = recording_confirmed_;
    out.commanded_offset_degrees = commanded_offset_degrees_;
    return out;
}

PocketOutput PocketMission::update(const PocketInput& in) {
    if ((last_update_ms_ && in.now_ms < *last_update_ms_) ||
        (!powered_at_ms_ && in.power_on_at_ms && *in.power_on_at_ms > in.now_ms)) {
        auto out = snapshot();
        out.input_accepted = false;
        return out;
    }
    last_update_ms_ = in.now_ms;
    bool start_recording = false;
    if (!powered_at_ms_ && in.power_on_at_ms) {
        powered_at_ms_ = in.power_on_at_ms;
        phase_ = PocketPhase::descending;
        start_recording = true;
    }
    if (!powered_at_ms_) return snapshot();
    for (std::size_t i = 0; i < solar_confirmed_.size(); ++i)
        solar_confirmed_[i] = solar_confirmed_[i] || in.solar_deployed[i];
    boom_confirmed_ = boom_confirmed_ || in.boom_deployed;
    if (in.recording_feedback) recording_confirmed_ = *in.recording_feedback;
    if (in.landed) phase_ = PocketPhase::landed;
    auto out = snapshot();
    out.start_recording = start_recording;
    // Conservative policy: no NEW mechanism/rotation intents after qualified landing.
    if (phase_ == PocketPhase::landed) return out;
    const bool solar_due = automatic_solar_deployment_ &&
        in.now_ms - *powered_at_ms_ >= deployment_delay_ms;
    for (std::size_t i = 0; i < solar_requested_.size(); ++i) {
        if (!solar_requested_[i] && !solar_confirmed_[i] && (solar_due || in.solar_commands[i])) {
            solar_requested_[i] = true;
            out.deploy_solar[i] = true;
        }
    }
    if (in.boom_command && !boom_requested_ && !boom_confirmed_) {
        boom_requested_ = true;
        out.deploy_boom = true;
    }
    if (in.rotation && std::isfinite(in.rotation->delta_degrees) &&
        (!last_rotation_sequence_ || in.rotation->sequence > *last_rotation_sequence_)) {
        const double delta = std::fmod(in.rotation->delta_degrees, 360.0);
        commanded_offset_degrees_ = std::fmod(commanded_offset_degrees_ + delta, 360.0);
        if (commanded_offset_degrees_ < 0) commanded_offset_degrees_ += 360.0;
        last_rotation_sequence_ = in.rotation->sequence;
        out.rotate_to_degrees = commanded_offset_degrees_;
        out.commanded_offset_degrees = commanded_offset_degrees_;
    }
    return out;
}
}  // namespace cansat::mission
