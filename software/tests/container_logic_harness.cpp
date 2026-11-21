// Host-side tests for the container's pure-logic modules:
//   MissionManager      (software/firmware/container/src/mission_manager.cpp)
//   container_telemetry (software/firmware/container/src/container_telemetry.cpp)
//   command_parser      (software/firmware/container/src/command_parser.cpp)
//
// Compiled and run on a PC by test_container_logic.py; no Arduino or board
// needed. Usage:
//   harness --list        print every case name
//   harness <case>        run one case; exit 0 = pass, 1 = fail
//
// Flight profiles are synthetic: altitude, max altitude and vertical speed
// are fed in directly, as SensorManager would produce them.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <map>
#include <string>

#include "command_parser.hpp"
#include "config.hpp"
#include "container_telemetry.hpp"
#include "mission_manager.hpp"

namespace {

int g_failures = 0;

#define EXPECT(cond)                                                        \
  do {                                                                      \
    if (!(cond)) {                                                          \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);           \
      ++g_failures;                                                         \
    }                                                                       \
  } while (0)

// Feeds samples into a MissionManager, tracking max altitude and vertical
// speed the way SensorManager does (speed = d(alt)/dt, no smoothing here).
struct FlightSim {
  MissionManager mission;
  SensorReadings r;
  uint32_t t_ms = 1000;
  bool have_prev = false;

  // Releases are recorded here: altitude at the moment PQ_RELEASE began.
  float release_alt = NAN;

  FlightState step(float alt_m, uint32_t dt_ms = 100) {
    t_ms += dt_ms;
    r.vertical_speed_mps = have_prev ? (alt_m - r.altitude_m) * 1000.0f / dt_ms : 0.0f;
    have_prev = true;
    r.altitude_m = alt_m;
    if (alt_m > r.max_altitude_m) r.max_altitude_m = alt_m;
    r.sample_time_ms = t_ms;
    const FlightState before = mission.getFlState();
    const FlightState after = mission.updateState(r);
    if (before != PQ_RELEASE && after == PQ_RELEASE) release_alt = alt_m;
    return after;
  }
};

// --- MissionManager ---------------------------------------------------------

void mission_starts_idle_and_ignores_flight() {
  FlightSim s;
  EXPECT(s.mission.getFlState() == IDLE);
  for (int i = 0; i < 50; ++i) s.step(i * 10.0f);  // climbing hard
  EXPECT(s.mission.getFlState() == IDLE);
}

void mission_arm_only_from_idle() {
  MissionManager m;
  EXPECT(m.arm());
  EXPECT(m.getFlState() == LAUNCH_PAD);
  EXPECT(!m.arm());  // already armed
  m.setFlState(ASCENT);
  EXPECT(!m.arm());
  EXPECT(m.getFlState() == ASCENT);
}

void mission_full_flight_profile() {
  // Pad 2 s, climb at 60 m/s to 700 m, then descend at 10 m/s to the ground,
  // then sit still. Release must happen at 90 % of the 700 m peak (C3).
  FlightSim s;
  s.mission.arm();
  for (int i = 0; i < 20; ++i) s.step(0.0f);
  EXPECT(s.mission.getFlState() == LAUNCH_PAD);

  bool saw_ascent = false, saw_apogee = false;
  for (float alt = 6.0f; alt <= 700.0f; alt += 6.0f) {
    if (s.step(alt) == ASCENT) saw_ascent = true;
  }
  EXPECT(saw_ascent);
  for (float alt = 699.0f; alt >= 0.0f; alt -= 1.0f) {
    const FlightState st = s.step(alt);
    if (st == APOGEE) saw_apogee = true;
  }
  EXPECT(saw_apogee);
  EXPECT(s.mission.getFlState() == PQ_RELEASE);

  const float peak = s.r.max_altitude_m;
  // 0.9 is written out from the guide (C3) rather than read from config.hpp,
  // so a wrong value in the config makes this test fail.
  const float target = 0.9f * peak;
  std::printf("peak %.1f m, release at %.1f m (target <= %.1f m)\n", peak, s.release_alt, target);
  EXPECT(!std::isnan(s.release_alt));
  EXPECT(s.release_alt <= target);
  // Debounce costs a few samples (1 m each here); must still be close.
  EXPECT(s.release_alt >= target - 5.0f);

  // Still descending: not landed.
  EXPECT(!s.mission.isLanded());
  // On the ground, motionless: landed after kLandedConfirmMs.
  for (int i = 0; i < 40; ++i) s.step(0.0f);  // 4 s
  EXPECT(!s.mission.isLanded());
  for (int i = 0; i < 20; ++i) s.step(0.0f);  // +2 s
  EXPECT(s.mission.isLanded());
}

void mission_single_spike_does_not_launch() {
  FlightSim s;
  s.mission.arm();
  for (int i = 0; i < 10; ++i) s.step(0.0f);
  s.step(100.0f);  // one bad sample: huge altitude and speed
  s.step(0.0f);
  s.step(0.0f);
  EXPECT(s.mission.getFlState() == LAUNCH_PAD);
}

void mission_same_sample_counted_once() {
  MissionManager m;
  m.arm();
  SensorReadings r;
  r.altitude_m = 100.0f;
  r.vertical_speed_mps = 50.0f;
  r.sample_time_ms = 5000;
  for (int i = 0; i < 10; ++i) m.updateState(r);  // same timestamp each time
  EXPECT(m.getFlState() == LAUNCH_PAD);
}

void mission_release_after_restore() {
  // Reset during descent: restored to APOGEE with the saved 700 m peak.
  MissionManager m;
  m.restore(APOGEE, false);
  SensorReadings r;
  r.max_altitude_m = 700.0f;
  uint32_t t = 0;
  for (float alt : {650.0f, 640.0f, 635.0f}) {
    r.altitude_m = alt;
    r.sample_time_ms = (t += 100);
    m.updateState(r);
  }
  EXPECT(m.getFlState() == APOGEE);  // still above 630 m
  for (float alt : {629.0f, 628.0f, 627.0f}) {
    r.altitude_m = alt;
    r.sample_time_ms = (t += 100);
    m.updateState(r);
  }
  EXPECT(m.getFlState() == PQ_RELEASE);
}

void mission_landing_needs_continuous_stillness() {
  MissionManager m;
  m.restore(PQ_RELEASE, false);
  SensorReadings r;
  uint32_t t = 0;
  auto feed = [&](float v, int n) {
    for (int i = 0; i < n; ++i) {
      r.vertical_speed_mps = v;
      r.sample_time_ms = (t += 100);
      m.updateState(r);
    }
  };
  feed(0.0f, 40);   // 4 s still
  feed(-5.0f, 1);   // a gust: moving again
  feed(0.0f, 40);   // 4 s still
  EXPECT(!m.isLanded());
  feed(0.0f, 15);
  EXPECT(m.isLanded());
}

void mission_sim_needs_enable_then_activate() {
  MissionManager m;
  EXPECT(m.simCommand("ACTIVATE") == SimCommandResult::Rejected);
  EXPECT(!m.simActive());
  EXPECT(m.simCommand("ENABLE") == SimCommandResult::Enabled);
  EXPECT(!m.simActive());
  EXPECT(m.getOpMode() == OPMODE_FLIGHT);
  EXPECT(m.simCommand("ACTIVATE") == SimCommandResult::Activated);
  EXPECT(m.simActive());
  EXPECT(m.getOpMode() == OPMODE_SIM);
  EXPECT(m.simCommand("BOGUS") == SimCommandResult::Rejected);
  EXPECT(m.simCommand("DISABLE") == SimCommandResult::Disabled);
  EXPECT(!m.simActive());
  EXPECT(m.getOpMode() == OPMODE_FLIGHT);
}

void mission_restore_never_resumes_sim() {
  MissionManager m;
  m.simCommand("ENABLE");
  m.simCommand("ACTIVATE");
  m.restore(ASCENT, false);
  EXPECT(!m.simActive());
  EXPECT(m.getFlState() == ASCENT);
}

// --- Telemetry --------------------------------------------------------------

TelemetryFrame exampleFrame() {
  TelemetryFrame f;
  f.team_id = 1000;
  f.mission_time_ms = 12345;
  f.packet_count = 42;
  f.command_count = 3;
  f.mode = OPMODE_FLIGHT;
  f.altitude_m = 123.44f;
  f.pressure_pa = 101325;
  f.temperature_c = 23.46f;
  f.battery_v = 7.416f;
  f.battery_ma = 123;
  f.mech_state = 0x03;
  f.state = ASCENT;
  f.cmd_echo = "CAL";
  return f;
}

void telemetry_format_matches_guide_columns() {
  char buf[200];
  const size_t n = formatTelemetry(exampleFrame(), buf, sizeof(buf));
  const char* expected = "1000C,12.345,42,3,F,123.4,101325,23.5,7.42,123,03,ASCENT,CAL";
  std::printf("got: %s\n", buf);
  EXPECT(n == std::strlen(expected));
  EXPECT(std::strcmp(buf, expected) == 0);
  // Exactly 13 columns (12 commas), as in the guide.
  int commas = 0;
  for (const char* c = buf; *c; ++c) commas += (*c == ',');
  EXPECT(commas == 12);
}

void telemetry_idle_reported_as_launch_pad() {
  TelemetryFrame f = exampleFrame();
  f.state = IDLE;
  f.mode = OPMODE_SIM;
  char buf[200];
  formatTelemetry(f, buf, sizeof(buf));
  EXPECT(std::strstr(buf, ",LAUNCH_PAD,") != nullptr);
  EXPECT(std::strstr(buf, "IDLE") == nullptr);
  EXPECT(std::strstr(buf, ",S,") != nullptr);
}

void telemetry_mission_time_is_exact_after_hours() {
  TelemetryFrame f = exampleFrame();
  f.mission_time_ms = 3u * 3600u * 1000u + 7;  // 3 h + 7 ms
  char buf[200];
  formatTelemetry(f, buf, sizeof(buf));
  EXPECT(std::strncmp(buf, "1000C,10800.007,", 16) == 0);
}

void telemetry_too_small_buffer_sends_nothing() {
  char buf[20];
  EXPECT(formatTelemetry(exampleFrame(), buf, sizeof(buf)) == 0);
}

void telemetry_fits_espnow_payload() {
  // Worst case: every number at its widest, longest state and echo.
  TelemetryFrame f = exampleFrame();
  f.team_id = 65535;
  f.mission_time_ms = 0xFFFFFFFFu;
  f.packet_count = 0xFFFFFFFFu;
  f.command_count = 0xFFFFFFFFu;
  f.altitude_m = -99999.9f;
  f.pressure_pa = 0xFFFFFFFFu;
  f.temperature_c = -999.9f;
  f.battery_v = 99.99f;
  f.battery_ma = 65535;
  f.state = PQ_RELEASE;
  f.cmd_echo = "SIMACTIVATEXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX";  // 47 chars
  char buf[251];
  const size_t n = formatTelemetry(f, buf, sizeof(buf));
  std::printf("worst-case line is %zu bytes\n", n);
  EXPECT(n > 0);
  EXPECT(n <= 250);
}

void telemetry_rate_schedule() {
  using namespace FirmwareConfig;
  EXPECT(telemetryPeriodMs(false, UINT32_MAX, false, true) == kTelemetryFastPeriodMs);
  EXPECT(telemetryPeriodMs(true, 0, false, true) == kTelemetryFastPeriodMs);
  EXPECT(telemetryPeriodMs(true, 4999, false, true) == kTelemetryFastPeriodMs);
  EXPECT(telemetryPeriodMs(true, 5000, false, true) == kTelemetrySlowPeriodMs);
  EXPECT(telemetryPeriodMs(true, UINT32_MAX, false, true) == kTelemetrySlowPeriodMs);
  EXPECT(telemetryPeriodMs(true, 9000, true, true) == 0);    // landed: stop
  EXPECT(telemetryPeriodMs(false, 0, false, false) == 0);    // CX OFF
  EXPECT(kTelemetryFastPeriodMs == 250);  // 4 Hz (CTR15)
  EXPECT(kTelemetrySlowPeriodMs == 1000); // 1 Hz (CTR16)
}

// --- Command parser ---------------------------------------------------------

ParseResult parse(const char* text, ParsedCommand& out) {
  return parseCommand(text, std::strlen(text), 1000, out);
}

void command_parses_op_and_value() {
  ParsedCommand c;
  EXPECT(parse("CMD,1000,SIMP,101325\r\n", c) == ParseResult::Ok);
  EXPECT(c.op == CommandOp::SIMP);
  EXPECT(std::strcmp(c.value, "101325") == 0);

  EXPECT(parse("CMD,1000,CAL", c) == ParseResult::Ok);
  EXPECT(c.op == CommandOp::CAL);
  EXPECT(c.value[0] == '\0');

  EXPECT(parse("CMD,1000,SIM,ENABLE", c) == ParseResult::Ok);
  EXPECT(c.op == CommandOp::SIM);
  EXPECT(std::strcmp(c.value, "ENABLE") == 0);

  EXPECT(parse("CMD,1000,FOO,1", c) == ParseResult::Ok);
  EXPECT(c.op == CommandOp::Unknown);
}

void command_rejects_other_teams_and_junk() {
  ParsedCommand c;
  EXPECT(parse("CMD,1001,CAL", c) == ParseResult::WrongTeam);
  EXPECT(parse("hello", c) == ParseResult::NotACommand);
  EXPECT(parse("", c) == ParseResult::NotACommand);
  EXPECT(parse("CMD,1000", c) == ParseResult::Malformed);
  EXPECT(parse("CMD,,CAL", c) == ParseResult::Malformed);
  EXPECT(parse("CMD,10a0,CAL", c) == ParseResult::Malformed);
  EXPECT(parse("CMD,1000,", c) == ParseResult::Malformed);
  // Value longer than the buffer is rejected, not truncated.
  EXPECT(parse("CMD,1000,MEC,0123456789012345678901234567890123456789XYZ", c) ==
         ParseResult::Malformed);
}

void command_echo_has_no_commas() {
  ParsedCommand c;
  char echo[48];
  parse("CMD,1000,CX,ON", c);
  buildCommandEcho(c, echo, sizeof(echo));
  EXPECT(std::strcmp(echo, "CXON") == 0);

  parse("CMD,1000,MEC,REL:1,2", c);
  buildCommandEcho(c, echo, sizeof(echo));
  EXPECT(std::strcmp(echo, "MECREL:12") == 0);

  parse("CMD,1000,SIMP,101325", c);
  buildCommandEcho(c, echo, 6);  // truncates safely
  EXPECT(std::strcmp(echo, "SIMP1") == 0);
}

void command_number_and_time_parsing() {
  uint32_t v = 0;
  EXPECT(parseUint32("101325", v) && v == 101325u);
  EXPECT(parseUint32("4294967295", v) && v == 4294967295u);
  EXPECT(!parseUint32("4294967296", v));
  EXPECT(!parseUint32("-5", v));
  EXPECT(!parseUint32("12.5", v));
  EXPECT(!parseUint32("", v));

  EXPECT(parseTimeOfDay("13:45:07", v) && v == 13u * 3600u + 45u * 60u + 7u);
  EXPECT(parseTimeOfDay("00:00:00", v) && v == 0u);
  EXPECT(!parseTimeOfDay("24:00:00", v));
  EXPECT(!parseTimeOfDay("12:60:00", v));
  EXPECT(!parseTimeOfDay("1:2:3", v));
  EXPECT(!parseTimeOfDay("GPS", v));
}

const std::map<std::string, std::function<void()>>& cases() {
  static const std::map<std::string, std::function<void()>> kCases = {
      {"mission_starts_idle_and_ignores_flight", mission_starts_idle_and_ignores_flight},
      {"mission_arm_only_from_idle", mission_arm_only_from_idle},
      {"mission_full_flight_profile", mission_full_flight_profile},
      {"mission_single_spike_does_not_launch", mission_single_spike_does_not_launch},
      {"mission_same_sample_counted_once", mission_same_sample_counted_once},
      {"mission_release_after_restore", mission_release_after_restore},
      {"mission_landing_needs_continuous_stillness", mission_landing_needs_continuous_stillness},
      {"mission_sim_needs_enable_then_activate", mission_sim_needs_enable_then_activate},
      {"mission_restore_never_resumes_sim", mission_restore_never_resumes_sim},
      {"telemetry_format_matches_guide_columns", telemetry_format_matches_guide_columns},
      {"telemetry_idle_reported_as_launch_pad", telemetry_idle_reported_as_launch_pad},
      {"telemetry_mission_time_is_exact_after_hours", telemetry_mission_time_is_exact_after_hours},
      {"telemetry_too_small_buffer_sends_nothing", telemetry_too_small_buffer_sends_nothing},
      {"telemetry_fits_espnow_payload", telemetry_fits_espnow_payload},
      {"telemetry_rate_schedule", telemetry_rate_schedule},
      {"command_parses_op_and_value", command_parses_op_and_value},
      {"command_rejects_other_teams_and_junk", command_rejects_other_teams_and_junk},
      {"command_echo_has_no_commas", command_echo_has_no_commas},
      {"command_number_and_time_parsing", command_number_and_time_parsing},
  };
  return kCases;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::printf("usage: %s --list | <case>\n", argv[0]);
    return 2;
  }
  if (std::strcmp(argv[1], "--list") == 0) {
    for (const auto& entry : cases()) std::printf("%s\n", entry.first.c_str());
    return 0;
  }
  const auto it = cases().find(argv[1]);
  if (it == cases().end()) {
    std::printf("unknown case %s\n", argv[1]);
    return 2;
  }
  it->second();
  return g_failures == 0 ? 0 : 1;
}
