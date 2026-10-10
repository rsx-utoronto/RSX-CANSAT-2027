#include <cansat/mission.hpp>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

using namespace cansat::mission;
namespace {
int checks = 0;
void check(bool condition, const char* label) {
    ++checks;
    if (!condition) throw std::runtime_error(label);
}
ContainerInput altitude(std::uint64_t ms, double m, FlightEvent event = FlightEvent::none) {
    ContainerInput in;
    in.now_ms = ms;
    in.altitude = Altitude{m, ms};
    in.flight_event = event;
    return in;
}
void reach_apogee(ContainerMission& mission, std::uint64_t origin = 0) {
    auto in = altitude(origin, 10, FlightEvent::launch);
    in.arm_command = true;
    check(mission.update(in).phase == ContainerPhase::ascent, "armed launch");
    check(mission.update(altitude(origin + 100, 1000, FlightEvent::apogee)).phase == ContainerPhase::apogee,
          "qualified apogee");
}
void container_nominal() {
    ContainerMission mission{1000};
    check(mission.update(altitude(0, 0)).telemetry_hz == 4, "CTR initially 4 Hz");
    check(mission.update(altitude(1, 10, FlightEvent::launch)).phase == ContainerPhase::launch_pad,
          "ARM gates launch detection");
    reach_apogee(mission, 2);
    check(!mission.update(altitude(103, 901)).release_request, "above 90 percent");
    auto out = mission.update(altitude(104, 900));
    check(out.release_request && out.phase == ContainerPhase::release_requested, "exact 90 percent release intent");
    check(!out.pq_clear && !out.mechanism_deployed && out.telemetry_hz == 4, "intent is not confirmation");
    auto in = altitude(5104, 800);
    in.mechanism_deployed = true;
    out = mission.update(in);
    check(!out.release_request && out.mechanism_deployed && !out.pq_clear && out.telemetry_hz == 4,
          "five seconds after REQUEST does not reduce telemetry");
    in.now_ms = 6000; in.pq_clear = true;
    out = mission.update(in);
    check(out.pq_clear && out.phase == ContainerPhase::pq_released, "rail-clear confirmation");
    in = {}; in.now_ms = 10999;
    check(mission.update(in).telemetry_hz == 4, "4999ms after clear still 4 Hz");
    in.now_ms = 11000;
    out = mission.update(in);
    check(out.telemetry_hz == 1 && out.pq_clear && out.mechanism_deployed, "5000ms after clear 1 Hz; feedback latched");
    in.now_ms = 11001; in.flight_event = FlightEvent::landed; in.release_command = true;
    out = mission.update(in);
    check(out.phase == ContainerPhase::landed && out.telemetry_hz == 0 && !out.release_request, "landing cutoff and no new release");
    in.now_ms = 12000; in.flight_event = FlightEvent::launch; in.arm_command = true;
    check(mission.update(in).phase == ContainerPhase::landed, "landing terminal");
    std::cout << "CTR: ARM -> ASCENT -> APOGEE -> RELEASE_REQUEST -> PQ_CLEAR -> 4Hz/4999ms -> 1Hz/5000ms -> LANDED/0Hz\n";
}
void container_faults() {
    ContainerMission mission{100};
    reach_apogee(mission);
    auto in = altitude(200, 800);
    in.altitude->sampled_at_ms = 99;
    auto out = mission.update(in);
    check(!out.altitude_valid && !out.release_request, "stale altitude inhibited");
    in = altitude(201, 800); in.altitude->sampled_at_ms = 202;
    check(!mission.update(in).release_request, "future altitude inhibited");
    for (double value : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
        in = altitude(++in.now_ms, value);
        check(!mission.update(in).release_request, "nonfinite altitude inhibited");
    }
    in = altitude(199, 800); in.release_command = true; in.pq_clear = true;
    out = mission.update(in);
    check(!out.input_accepted && !out.release_request && !out.pq_clear, "clock regression causes no mutation/actions");
    in = altitude(300, 800); in.altitude->sampled_at_ms = 200;
    check(mission.update(in).release_request, "exact freshness boundary accepted");
    in.release_command = true;
    check(!mission.update(in).release_request, "duplicate update/command no second request");
    ContainerMission manual{100};
    in = {}; in.release_command = true;
    check(manual.update(in).release_request, "explicit ground mechanism override works without ARM/altitude");
    check(!manual.update(in).release_request, "manual override idempotent");
    ContainerMission bad_config{0};
    check(!bad_config.update(in).input_accepted, "zero freshness configuration rejected");
    ContainerMission reordered{100}; reach_apogee(reordered);
    in = altitude(101, 800); in.altitude->sampled_at_ms = 99;
    out = reordered.update(in);
    check(!out.altitude_valid && !out.release_request, "fresh but older altitude sample rejected");
    in = altitude(102, 800);
    check(reordered.update(in).release_request, "ordered sample recovers after rejected sample");
    ContainerMission still_ascending{100};
    in = altitude(0, 1000, FlightEvent::launch); in.arm_command = true;
    still_ascending.update(in);
    check(!still_ascending.update(altitude(1, 800)).release_request, "no apogee event means no automatic release");
    ContainerMission zero_peak{100};
    in = altitude(0, 0, FlightEvent::launch); in.arm_command = true;
    zero_peak.update(in);
    check(!zero_peak.update(altitude(1, -1, FlightEvent::apogee)).release_request, "nonpositive peak cannot trigger release");
    ContainerMission landing{100}; reach_apogee(landing);
    in = altitude(101, 800, FlightEvent::landed); in.release_command = true;
    check(!landing.update(in).release_request, "landing priority over release command");
    ContainerMission edge{100};
    in = {}; in.now_ms = std::numeric_limits<std::uint64_t>::max() - 5000; in.pq_clear = true;
    edge.update(in);
    in.now_ms = std::numeric_limits<std::uint64_t>::max();
    check(edge.update(in).telemetry_hz == 1, "elapsed subtraction avoids timer overflow");
}
void pocket_nominal() {
    PocketMission mission{true};
    PocketInput in;
    in.boom_command = true; in.rotation = RotationCommand{1, 90};
    auto out = mission.update(in);
    check(out.phase == PocketPhase::off && out.telemetry_hz == 0 && !out.deploy_boom && !out.rotate_to_degrees,
          "off node cannot act or transmit");
    in = {}; in.now_ms = 1000; in.power_on_at_ms = 1000;
    out = mission.update(in);
    check(out.start_recording && !out.recording_confirmed && out.telemetry_hz == 4, "power-on recording intent and 4 Hz");
    check(!out.deploy_solar[0] && !out.deploy_solar[1], "no early solar deployment");
    in = {}; in.now_ms = 5999; in.recording_feedback = true;
    out = mission.update(in);
    check(!out.start_recording && out.recording_confirmed && !out.deploy_solar[0], "4999ms no solar; camera feedback");
    in.now_ms = 6000;
    out = mission.update(in);
    check(out.deploy_solar[0] && out.deploy_solar[1] && !out.solar_confirmed[0], "5000ms both solar intents, no fabricated confirmation");
    check(!out.deploy_boom && !out.rotate_to_degrees, "no automatic boom or camera rotation");
    out = mission.update(in);
    check(!out.deploy_solar[0] && !out.deploy_solar[1], "no duplicate solar activation");
    in.now_ms = 6001; in.solar_deployed = {true, false}; in.boom_command = true;
    out = mission.update(in);
    check(out.solar_confirmed[0] && !out.solar_confirmed[1] && out.deploy_boom && !out.boom_confirmed,
          "independent panel feedback and boom intent");
    in.now_ms = 6002; in.boom_deployed = true; in.solar_deployed = {false, true};
    out = mission.update(in);
    check(out.solar_confirmed[0] && out.solar_confirmed[1] && out.boom_confirmed && !out.deploy_boom, "latched deployment feedback");
    in = {}; in.now_ms = 31000;
    check(!mission.update(in).rotate_to_degrees, "30 seconds never auto-rotates camera");
    in.rotation = RotationCommand{7, 90};
    out = mission.update(in);
    check(out.rotate_to_degrees && *out.rotate_to_degrees == 90, "commanded positive rotation");
    check(!mission.update(in).rotate_to_degrees, "duplicate sequence ignored");
    in.rotation = RotationCommand{6, 90};
    check(!mission.update(in).rotate_to_degrees, "older sequence ignored");
    in.rotation = RotationCommand{8, -180};
    check(mission.update(in).commanded_offset_degrees == 270, "negative rotation wrap");
    in.now_ms = 31001; in.recording_feedback = false; in.landed = true;
    out = mission.update(in);
    check(out.phase == PocketPhase::landed && out.telemetry_hz == 4 && !out.recording_confirmed, "PQ landing keeps telemetry; camera failure visible");
    in.rotation = RotationCommand{9, 90};
    check(!mission.update(in).rotate_to_degrees, "no new rotation after landing");
    std::cout << "PQ: OFF -> POWER_ON/record+4Hz -> SOLAR_REQUEST/5000ms -> confirmed panels -> commanded boom -> commanded rotation; no automatic 30s rotation\n";
}
void pocket_faults() {
    PocketMission mission{true};
    PocketInput in; in.now_ms = 10; in.power_on_at_ms = 11;
    check(!mission.update(in).input_accepted, "future power timestamp rejected");
    in.power_on_at_ms = 0; in.now_ms = 5000;
    auto out = mission.update(in);
    check(out.start_recording && out.deploy_solar[0], "late first tick uses supplied power timestamp");
    in.now_ms = 4999; in.boom_command = true;
    check(!mission.update(in).input_accepted && !mission.update(in).deploy_boom, "backward PQ clock rejected");
    in = {}; in.now_ms = 5001; in.rotation = RotationCommand{1, std::numeric_limits<double>::quiet_NaN()};
    check(!mission.update(in).rotate_to_degrees, "invalid angle rejected");
    in.rotation = RotationCommand{1, 90};
    check(mission.update(in).commanded_offset_degrees == 90, "invalid angle did not consume sequence");
    in.rotation = RotationCommand{2, std::numeric_limits<double>::max()};
    out = mission.update(in);
    check(std::isfinite(out.commanded_offset_degrees) && out.commanded_offset_degrees >= 0 &&
          out.commanded_offset_degrees < 360, "large finite delta normalized without overflow");
    PocketMission bench{false};
    in = {}; in.power_on_at_ms = 0; bench.update(in);
    in = {}; in.now_ms = 5000;
    check(!bench.update(in).deploy_solar[0], "explicit bench policy inhibits automatic solar");
    in.solar_commands[1] = true;
    out = bench.update(in);
    check(!out.deploy_solar[0] && out.deploy_solar[1], "manual panel command bypasses automatic policy");
    PocketMission confirmed{true};
    in = {}; in.power_on_at_ms = 0; in.solar_deployed[0] = true; confirmed.update(in);
    in = {}; in.now_ms = 5000;
    out = confirmed.update(in);
    check(!out.deploy_solar[0] && out.deploy_solar[1], "already deployed panel not re-fired");
    PocketMission replayed_power{true};
    in = {}; in.power_on_at_ms = 0; replayed_power.update(in);
    in.now_ms = 4999; in.power_on_at_ms = 4999;
    check(!replayed_power.update(in).start_recording, "power event replay cannot restart recording");
    in.now_ms = 5000;
    check(replayed_power.update(in).deploy_solar[0], "power event replay cannot reset deployment timer");
    PocketMission landed{true};
    in = {}; in.power_on_at_ms = 0; in.landed = true; landed.update(in);
    in = {}; in.now_ms = 5000; in.boom_command = true; in.solar_commands = {true,true};
    out = landed.update(in);
    check(!out.deploy_solar[0] && !out.deploy_boom, "landing suppresses new mechanism intents");
}
}  // namespace

int main() {
    try {
        container_nominal(); container_faults(); pocket_nominal(); pocket_faults();
        std::cout << "mission_checks=" << checks << " PASS clock=explicit_ms hardware=none\n";
    } catch (const std::exception& error) {
        std::cerr << "mission failure: " << error.what() << '\n';
        return 1;
    }
}
