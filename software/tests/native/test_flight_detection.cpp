#include <cansat/flight_detection.hpp>
#include <array>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using namespace cansat::mission;
namespace {
int checks = 0;
void check(bool condition, const char* label) {
    ++checks;
    if (!condition) throw std::runtime_error(label);
}
FlightDetectionConfig fixture_config() {
    // Synthetic trace parameters ONLY. No hardware/flight configuration selected.
    FlightDetectionConfig c;
    c.max_sample_age_ms = 1000; c.max_sample_gap_ms = 1000;
    c.max_abs_vertical_speed_mps = 150;
    c.launch_altitude_m = 10; c.launch_min_climb_mps = 2; c.launch_confirm_ms = 2000;
    c.apogee_min_drop_m = 5; c.apogee_min_descent_mps = 1; c.apogee_confirm_ms = 2000;
    c.landing_altitude_min_m = -3; c.landing_altitude_max_m = 3;
    c.landing_max_abs_speed_mps = 0.5; c.landing_max_span_m = 0.5;
    c.landing_confirm_ms = 3000; c.min_confirm_samples = 3;
    return c;
}
DetectionInput at(std::uint64_t t, double height, bool armed = true) {
    return {t, Altitude{height, t}, armed};
}
void launch(AltitudeFlightDetector& d) {
    d.update(at(0, 0)); d.update(at(1000, 10)); d.update(at(2000, 20));
    check(d.update(at(3000, 30)).event == FlightEvent::launch, "launch fixture");
}
void nominal_coupled_flight() {
    const auto c = fixture_config();
    AltitudeFlightDetector ctr_detector(c, DetectorStart::on_pad);
    AltitudeFlightDetector pq_detector(c, DetectorStart::descending_after_release);
    ContainerMission ctr(c.max_sample_age_ms);
    PocketMission pq(true);
    const std::array<double, 18> heights{0, 5, 10, 20, 30, 70, 100, 98, 94, 90, 86, 50, 10, 2, 1.75, 1.5, 1.75, 1.5};
    std::vector<std::pair<std::uint64_t, FlightEvent>> events;
    for (std::size_t i = 0; i < heights.size(); ++i) {
        const auto ms = static_cast<std::uint64_t>(i) * 1000;
        const auto detection = ctr_detector.update(at(ms, heights[i]));
        if (detection.event != FlightEvent::none) events.emplace_back(ms, detection.event);
        ContainerInput ci;
        ci.now_ms = ms; ci.arm_command = i == 0;
        ci.altitude = detection.qualified_altitude; ci.flight_event = detection.event;
        ci.pq_clear = i == 11;
        const auto co = ctr.update(ci);
        check(co.release_request == (i == 10), "detector-driven release only at confirmed apogee below 90 percent");
        check(co.telemetry_hz == (i == 17 ? 0U : i == 16 ? 1U : 4U), "detector-driven CTR telemetry transitions");
        if (i >= 11) {
            const auto pd = pq_detector.update(at(ms, heights[i], false));
            PocketInput pi;
            pi.now_ms = ms; pi.power_on_at_ms = 11000;
            pi.landed = pd.event == FlightEvent::landed;
            const auto po = pq.update(pi);
            check(po.start_recording == (i == 11), "PQ recording intent at power-on only");
            check(po.deploy_solar[0] == (i == 16) && po.deploy_solar[1] == (i == 16), "PQ five-second panel intents");
            check(po.telemetry_hz == 4, "PQ telemetry retained through detected landing");
            check((po.phase == PocketPhase::landed) == (i == 17), "PQ release-start detector needs no launch event");
        }
    }
    check(events == std::vector<std::pair<std::uint64_t, FlightEvent>>{
        {4000, FlightEvent::launch}, {10000, FlightEvent::apogee}, {17000, FlightEvent::landed}}, "exact flight event times and ordering");
    const auto terminal = ctr_detector.update(at(18000, 100));
    check(terminal.phase == DetectionPhase::landed && terminal.event == FlightEvent::none, "landing terminal, no repeated events");
    std::cout << "DETECTED: launch@4000ms apogee@10000ms landed@17000ms\n"
              << "COUPLED: release@10000ms clear@11000ms CTR=1Hz@16000ms/0Hz@17000ms; PQ=solar@16000ms/4Hz after landing\n";
}
void noise_and_phase_gates() {
    const auto c = fixture_config();
    AltitudeFlightDetector pad(c, DetectorStart::on_pad);
    for (std::uint64_t i = 0; i < 128; ++i) {
        const double h = static_cast<double>((i * 37) % 9) / 4.0 - 1;
        const auto out = pad.update(at(i * 250, h));
        check(out.phase == DetectionPhase::waiting_for_launch && out.event == FlightEvent::none,
              "pad noise does not produce launch/apogee/landing");
    }
    AltitudeFlightDetector unarmed(c, DetectorStart::on_pad);
    for (std::uint64_t i = 0; i < 6; ++i)
        check(unarmed.update(at(i * 1000, static_cast<double>(i) * 10, false)).event == FlightEvent::none,
              "unarmed climb suppressed");
    unarmed.update(at(6000, 60)); unarmed.update(at(7000, 70));
    check(unarmed.update(at(8000, 80)).event == FlightEvent::launch, "arming starts fresh confirmation, no retroactive launch");
    check(unarmed.update(at(9000, 90, false)).phase == DetectionPhase::ascending, "loss of ARM level cannot reset in-flight phase");
    AltitudeFlightDetector interrupted(c, DetectorStart::on_pad);
    interrupted.update(at(0, 0)); interrupted.update(at(1000, 10)); interrupted.update(at(2000, 20));
    check(interrupted.update(at(3000, 19)).event == FlightEvent::none, "launch rising trend interruption");
    interrupted.update(at(4000, 30));
    check(interrupted.update(at(5000, 40)).event == FlightEvent::none, "launch dwell restarts");
    check(interrupted.update(at(6000, 50)).event == FlightEvent::launch, "sustained rising trend qualifies");
    AltitudeFlightDetector rebound(c, DetectorStart::on_pad); launch(rebound);
    rebound.update(at(4000, 100)); rebound.update(at(5000, 94));
    check(rebound.update(at(6000, 110)).event == FlightEvent::none, "new peak cancels apogee candidate");
    rebound.update(at(7000, 103));
    check(rebound.update(at(8000, 100)).event == FlightEvent::none, "apogee dwell not yet complete");
    auto out = rebound.update(at(9000, 98));
    check(out.event == FlightEvent::apogee && out.peak_altitude_m == 110, "sustained drop from updated peak");
    AltitudeFlightDetector plateau(c, DetectorStart::on_pad); launch(plateau);
    for (std::uint64_t t = 4000; t <= 10000; t += 1000)
        check(plateau.update(at(t, 30)).event == FlightEvent::none, "plateau alone is neither apogee nor landing");
}
void bad_samples_and_recovery() {
    const std::vector<std::pair<DetectionInput, AltitudeStatus>> bad{
        {{2500, std::nullopt, true}, AltitudeStatus::missing},
        {at(2500, std::numeric_limits<double>::quiet_NaN()), AltitudeStatus::nonfinite},
        {at(2500, std::numeric_limits<double>::infinity()), AltitudeStatus::nonfinite},
        {{2500, Altitude{25, 2501}, true}, AltitudeStatus::future},
        {{2500, Altitude{0, 0}, true}, AltitudeStatus::stale},
        {{2500, Altitude{15, 1500}, true}, AltitudeStatus::out_of_order},
        {{2500, Altitude{21, 2000}, true}, AltitudeStatus::conflicting_duplicate},
        {at(1999, 20), AltitudeStatus::clock_regression},
        {at(2500, 10000), AltitudeStatus::implausible_rate}
    };
    for (const auto& [input, status] : bad) {
        AltitudeFlightDetector d(fixture_config(), DetectorStart::on_pad);
        d.update(at(0, 0)); d.update(at(1000, 10)); d.update(at(2000, 20));
        auto out = d.update(input);
        check(out.status == status && out.event == FlightEvent::none && !out.qualified_altitude,
              "invalid data is classified and cannot emit events or qualified altitude");
        check(d.update(at(3000, 30)).status == AltitudeStatus::seeded, "invalid data breaks rate continuity");
        check(d.update(at(4000, 40)).event == FlightEvent::none, "first recovery vote cannot complete old dwell");
        check(d.update(at(5000, 50)).event == FlightEvent::none, "second recovery vote still waiting");
        check(d.update(at(6000, 60)).event == FlightEvent::launch, "new complete window recovers after bad input");
    }
    AltitudeFlightDetector duplicates(fixture_config(), DetectorStart::on_pad);
    duplicates.update(at(0, 0)); duplicates.update(at(1000, 10)); duplicates.update(at(2000, 20));
    auto out = duplicates.update({2500, Altitude{20, 2000}, true});
    check(out.status == AltitudeStatus::duplicate && !out.qualified_altitude && out.event == FlightEvent::none,
          "duplicate sample cannot advance dwell or be forwarded");
    check(duplicates.update(at(3000, 30)).event == FlightEvent::launch, "benign repeated reads do not destroy a fresh window");
    AltitudeFlightDetector disarm(fixture_config(), DetectorStart::on_pad);
    disarm.update(at(0, 0)); disarm.update(at(1000, 10)); disarm.update(at(2000, 20));
    disarm.update({2500, Altitude{20, 2000}, false});
    check(disarm.update(at(3000, 30)).event == FlightEvent::none, "disarming on a duplicate cancels launch window");
    AltitudeFlightDetector gap(fixture_config(), DetectorStart::on_pad);
    gap.update(at(0, 0)); gap.update(at(1000, 10)); gap.update(at(2000, 20));
    out = gap.update(at(3001, 30));
    check(out.status == AltitudeStatus::gap_reseeded && !out.qualified_altitude && out.event == FlightEvent::none,
          "sample gap above bound reseeds without confirming");
    gap.update(at(4001, 40)); gap.update(at(5001, 50));
    check(gap.update(at(6001, 60)).event == FlightEvent::launch, "full dwell required after a sample gap");
    AltitudeFlightDetector spike(fixture_config(), DetectorStart::on_pad); launch(spike);
    spike.update(at(4000, 100));
    out = spike.update(at(5000, 10000));
    check(out.status == AltitudeStatus::implausible_rate && out.peak_altitude_m == 100, "rejected spike cannot poison peak");
    spike.update(at(6000, 99)); spike.update(at(7000, 94)); spike.update(at(8000, 90));
    check(spike.update(at(9000, 86)).event == FlightEvent::apogee, "descent recovers without spike-induced peak");
}
void landing_guards() {
    auto c = fixture_config();
    AltitudeFlightDetector high(c, DetectorStart::descending_after_release);
    for (std::uint64_t t = 0; t <= 5000; t += 1000)
        check(high.update(at(t, 50, false)).event == FlightEvent::none, "stable high-altitude plateau cannot count as landing");
    AltitudeFlightDetector drift(c, DetectorStart::descending_after_release);
    for (std::uint64_t i = 0; i <= 8; ++i)
        check(drift.update(at(i * 1000, static_cast<double>(i) * 0.25, false)).event == FlightEvent::none,
              "slow drift spanning too much altitude cannot count as landing");
    check(drift.update(at(9000, 2, false)).event == FlightEvent::none, "new stable landing dwell incomplete");
    check(drift.update(at(10000, 2, false)).event == FlightEvent::landed, "three seconds inside permitted span qualifies landing");
    AltitudeFlightDetector bounce(c, DetectorStart::descending_after_release);
    bounce.update(at(0, 2)); bounce.update(at(1000, 2)); bounce.update(at(2000, 2));
    check(bounce.update(at(3000, 3.5)).event == FlightEvent::none, "bounce outside landing band cancels dwell");
    bounce.update(at(4000, 2)); bounce.update(at(5000, 2)); bounce.update(at(6000, 2));
    check(bounce.update(at(7000, 2)).event == FlightEvent::none, "landing window restarted after bounce");
    check(bounce.update(at(8000, 2)).event == FlightEvent::landed, "landing after full new stable dwell");
    AltitudeFlightDetector dropout(c, DetectorStart::descending_after_release);
    dropout.update(at(0, 2)); dropout.update(at(1000, 2)); dropout.update(at(2000, 2));
    auto rejected = dropout.update({2500, std::nullopt, false});
    check(rejected.phase == DetectionPhase::descending && rejected.event == FlightEvent::none,
          "landing dropout preserves flight phase but cannot confirm");
    dropout.update(at(3000, 2)); dropout.update(at(4000, 2)); dropout.update(at(5000, 2));
    check(dropout.update(at(6000, 2)).event == FlightEvent::none, "landing cannot bridge a missing sample");
    check(dropout.update(at(7000, 2)).event == FlightEvent::landed, "landing requires full post-dropout dwell");
    AltitudeFlightDetector stale_vote(c, DetectorStart::descending_after_release);
    stale_vote.update(at(0, 2)); stale_vote.update(at(1000, 2));
    for (std::uint64_t t = 1100; t <= 2000; t += 100)
        check(stale_vote.update({t, Altitude{2, 1000}, false}).event == FlightEvent::none,
              "fast polling the same sample cannot supply landing votes");
    check(stale_vote.update({2001, Altitude{2, 1000}, false}).status == AltitudeStatus::stale,
          "unchanged sample eventually ages out");
    auto low_speed = c; low_speed.landing_max_span_m = 2;
    AltitudeFlightDetector speed_boundary(low_speed, DetectorStart::descending_after_release);
    speed_boundary.update(at(0, 0)); speed_boundary.update(at(1000, 0.5));
    speed_boundary.update(at(2000, 1)); speed_boundary.update(at(3000, 1.5));
    check(speed_boundary.update(at(4000, 2)).event == FlightEvent::landed,
          "inclusive landing speed limit within explicitly permitted span");
    for (double boundary : {-3.0, 3.0}) {
        AltitudeFlightDetector edge(c, DetectorStart::descending_after_release);
        edge.update(at(0, boundary)); edge.update(at(1000, boundary)); edge.update(at(2000, boundary));
        check(edge.update(at(3000, boundary)).event == FlightEvent::none, "2999-or-earlier dwell not assumed from seed");
        check(edge.update(at(4000, boundary)).event == FlightEvent::landed, "inclusive landing altitude and dwell boundaries");
    }
    const auto end = std::numeric_limits<std::uint64_t>::max();
    AltitudeFlightDetector timer(c, DetectorStart::descending_after_release);
    for (std::uint64_t delta = 4000; delta > 0; delta -= 1000) timer.update(at(end - delta, 0));
    check(timer.update(at(end, 0)).event == FlightEvent::landed, "confirmation elapsed arithmetic does not overflow");
}
void configuration_and_boundaries() {
    const auto c = fixture_config();
    check(AltitudeFlightDetector(c, DetectorStart::on_pad).configuration_valid(), "explicit fixture config accepted");
    check(!AltitudeFlightDetector({}, DetectorStart::on_pad).configuration_valid(), "no implicit defaults");
    const std::vector<std::function<void(FlightDetectionConfig&)>> invalid{
        [](auto& x) { x.max_sample_age_ms = 0; }, [](auto& x) { x.max_sample_gap_ms = 0; },
        [](auto& x) { x.max_abs_vertical_speed_mps = 1; }, [](auto& x) { x.launch_altitude_m = 0; },
        [](auto& x) { x.launch_min_climb_mps = 0; }, [](auto& x) { x.launch_confirm_ms = 0; },
        [](auto& x) { x.apogee_min_drop_m = -1; }, [](auto& x) { x.apogee_min_descent_mps = 0; },
        [](auto& x) { x.apogee_confirm_ms = 0; }, [](auto& x) { x.landing_altitude_min_m = 4; },
        [](auto& x) { x.landing_altitude_max_m = 10; }, [](auto& x) { x.landing_max_abs_speed_mps = 0; },
        [](auto& x) { x.landing_max_span_m = 0; }, [](auto& x) { x.landing_confirm_ms = 0; },
        [](auto& x) { x.min_confirm_samples = 1; },
        [](auto& x) { x.apogee_min_drop_m = std::numeric_limits<double>::quiet_NaN(); },
        [](auto& x) { x.landing_altitude_min_m = -std::numeric_limits<double>::infinity(); }
    };
    for (const auto& mutate : invalid) {
        auto x = c; mutate(x);
        AltitudeFlightDetector d(x, DetectorStart::on_pad);
        const auto out = d.update(at(0, 0));
        check(!d.configuration_valid() && out.status == AltitudeStatus::invalid_configuration && out.event == FlightEvent::none,
              "invalid configuration cannot produce events");
    }
    AltitudeFlightDetector age(c, DetectorStart::on_pad);
    age.update(at(0, 0));
    check(age.update({2000, Altitude{10, 1000}, true}).status == AltitudeStatus::accepted, "exact maximum age and gap accepted");
    check(age.update({2001, Altitude{10, 1000}, true}).status == AltitudeStatus::stale, "age boundary plus one rejected before duplicate handling");
    AltitudeFlightDetector rate(c, DetectorStart::on_pad);
    rate.update(at(0, 0));
    check(rate.update(at(1000, 150)).status == AltitudeStatus::accepted, "inclusive maximum speed");
    check(rate.update(at(2000, 301)).status == AltitudeStatus::implausible_rate, "speed just above bound rejected");
    auto dense_config = c; dense_config.min_confirm_samples = 5; dense_config.launch_confirm_ms = 100;
    AltitudeFlightDetector dense(dense_config, DetectorStart::on_pad);
    dense.update(at(0, 0)); dense.update(at(1000, 10));
    check(dense.update(at(2000, 20)).event == FlightEvent::none, "elapsed dwell alone insufficient without sample votes");
    dense.update(at(3000, 30)); dense.update(at(4000, 40));
    check(dense.update(at(5000, 50)).event == FlightEvent::launch, "minimum sample count independently enforced");
    auto dwell = c; dwell.launch_min_climb_mps = 2;
    AltitudeFlightDetector climb(dwell, DetectorStart::on_pad);
    climb.update(at(0, 8)); climb.update(at(1000, 10)); climb.update(at(2000, 12));
    check(climb.update(at(3000, 14)).event == FlightEvent::launch, "exact launch height and climb-rate thresholds inclusive");
    climb.update(at(4000, 20)); climb.update(at(5000, 15)); climb.update(at(6000, 14));
    check(climb.update(at(7000, 13)).event == FlightEvent::apogee, "exact apogee drop and descent thresholds inclusive");
}
}  // namespace
int main() {
    try {
        nominal_coupled_flight(); noise_and_phase_gates(); bad_samples_and_recovery();
        landing_guards(); configuration_and_boundaries();
        std::cout << "flight_detection_checks=" << checks << " PASS clock=sample_ms thresholds=fixture_only hardware=none\n";
    } catch (const std::exception& error) {
        std::cerr << "flight detection failure: " << error.what() << '\n';
        return 1;
    }
}
