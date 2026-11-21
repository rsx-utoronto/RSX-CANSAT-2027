// Container flight software: the top-level loop.
//
// This file only wires the parts together; each part lives in its own
// module:
//   SensorManager     barometer + power monitor -> SensorReadings
//   MissionManager    flight state machine, ARM, simulation mode
//   ReleaseMechanism  PocketQube release and its sensors (MECH_STATE)
//   FlightStore       what must survive a reset (F1, F2, F8)
//   command_parser    "CMD,<team>,<OP>[,<value>]" commands
//   container_telemetry  CSV telemetry line and its 4 Hz / 1 Hz / off schedule
//   EspNowLink        radio (firmware_lib)
//
// Every loop pass:
//   1. handle any command received over the radio or the USB serial port
//   2. sample sensors (simulated pressure in simulation mode)
//   3. advance the flight state; release the PQ on entering PQ_RELEASE
//   4. save the flight record (now if something changed, else once a second)
//   5. send telemetry if it is due
// Nothing here waits: a missing sensor or a dead radio link can't stop the
// release from happening.
//
// USB serial output (115200 baud):
//   * telemetry lines exactly as sent over the radio ("1000C,...")
//   * everything else starts with "# " so tools can tell them apart
// USB serial input accepts the same commands as the radio, for bench tests
// (see software/tests/README.md).

#include <Arduino.h>
#include <string.h>

#include "command_parser.hpp"
#include "config.hpp"
#include "container_telemetry.hpp"
#include "espnow_link.hpp"
#include "flight_store.hpp"
#include "mission_manager.hpp"
#include "release_mechanism.hpp"
#include "sensor_manager.hpp"

using namespace FirmwareConfig;

namespace {

SensorManager    g_sensors;
MissionManager   g_mission;
ReleaseMechanism g_release;
FlightStore      g_store;

bool g_radio_up          = false;
bool g_telemetry_enabled = true;   // CX ON/OFF. On from power-up (CTR15).
char g_cmd_echo[48]      = "";     // CMD_ECHO telemetry field.

uint32_t g_last_tx_ms  = 0;
bool     g_sent_any    = false;

// Release time, for the 1 Hz switch 5 s after release (CTR16). Unknown after
// a reset that happened post-release; telemetry then goes straight to 1 Hz.
bool     g_release_time_known = false;
uint32_t g_release_ms         = 0;

// --- Radio receive mailbox --------------------------------------------------
// The ESP-NOW callback runs in the Wi-Fi task (possibly on the other CPU
// core). It only copies the bytes into this one-slot mailbox; the command
// is handled later in loop(). The spinlock makes the copy atomic between
// the two cores. Commands arrive at most ~1 per second, so one slot is
// enough; anything arriving while the slot is full is counted and dropped.
portMUX_TYPE     g_rx_mux = portMUX_INITIALIZER_UNLOCKED;
uint8_t          g_rx_buf[250];
size_t           g_rx_len = 0;
bool             g_rx_pending = false;
volatile uint32_t g_rx_dropped = 0;

void onRadioRx(const uint8_t* /*mac*/, const uint8_t* data, int len) {
  // EspNowLink has already dropped packets that aren't from the ground station.
  if (len <= 0) return;
  const size_t n = static_cast<size_t>(len) < sizeof(g_rx_buf) ? static_cast<size_t>(len)
                                                                : sizeof(g_rx_buf);
  portENTER_CRITICAL(&g_rx_mux);
  if (g_rx_pending) {
    g_rx_dropped = g_rx_dropped + 1;
  } else {
    memcpy(g_rx_buf, data, n);
    g_rx_len = n;
    g_rx_pending = true;
  }
  portEXIT_CRITICAL(&g_rx_mux);
}

// --- USB serial command input -----------------------------------------------
char   g_serial_line[128];
size_t g_serial_len = 0;
bool   g_serial_overflow = false;

// True before launch: the only time CAL and RST are allowed.
bool onPad() {
  const FlightState s = g_mission.getFlState();
  return s == IDLE || s == LAUNCH_PAD;
}

void markReleased(uint32_t now) {
  g_release.trigger();
  if (!g_release_time_known) {
    g_release_time_known = true;
    g_release_ms = now;
  }
}

// Applies one parsed command. Returns false if it was rejected.
bool applyCommand(const ParsedCommand& cmd, uint32_t now) {
  FlightRecord& rec = g_store.record();

  switch (cmd.op) {
    case CommandOp::CX:
      if (strcmp(cmd.value, "ON") == 0) { g_telemetry_enabled = true; return true; }
      if (strcmp(cmd.value, "OFF") == 0) { g_telemetry_enabled = false; return true; }
      return false;

    case CommandOp::ST: {
      // F3: UTC time of day as hh:mm:ss. The container has no GPS, so
      // "ST,GPS" is rejected.
      uint32_t secs = 0;
      if (!parseTimeOfDay(cmd.value, secs)) return false;
      g_store.setUtc(now, secs);
      return true;
    }

    case CommandOp::SIM: {
      // Never let simulation start once the rocket has launched.
      const bool turning_on = strcmp(cmd.value, "DISABLE") != 0;
      if (turning_on && !onPad()) return false;
      const bool was_active = g_mission.simActive();
      const SimCommandResult result = g_mission.simCommand(cmd.value);
      if (result == SimCommandResult::Rejected) return false;
      // Never average real and simulated altitudes together.
      if (was_active != g_mission.simActive()) g_sensors.resetAltitudeFilter();
      return true;
    }

    case CommandOp::SIMP: {
      if (!g_mission.simActive()) return false;
      uint32_t pa = 0;
      if (!parseUint32(cmd.value, pa)) return false;
      g_sensors.setSimulatedPressure(pa);
      return true;
    }

    case CommandOp::CAL:
      // Re-zeroing mid-flight would corrupt apogee and release detection.
      if (!onPad()) return false;
      if (!g_sensors.calibrateGroundLevel(g_mission.simActive())) return false;
      rec.max_altitude_m = 0.0f;
      return true;

    case CommandOp::ARM:
      if (!g_sensors.isGroundCalibrated()) {
        Serial.println("# WARNING: ARM before CAL; altitude is relative to sea level");
      }
      return g_mission.arm();

    case CommandOp::MEC:
      // Manual mechanism control, for FRR tests and emergencies. Allowed in
      // any state. **CHECK:** agree the full MEC value format with the GUI;
      // only "REL..." (release the PQ) is implemented.
      if (strncmp(cmd.value, "REL", 3) == 0) {
        markReleased(now);
        return true;
      }
      return false;

    case CommandOp::RST:
      // New mission record: mission time and counts back to 0, state IDLE.
      // Refused in flight, so a stray RST can't wipe a real flight; allowed
      // on the pad, after landing, or during a simulation run.
      if (!(onPad() || g_mission.isLanded() || g_mission.simActive())) return false;
      g_store.clear(now);
      g_mission.restore(IDLE, false);
      g_sensors.resetAltitudeFilter();
      g_release.begin(0);
      g_release_time_known = false;
      g_sent_any = false;
      return true;

    case CommandOp::Unknown:
      return false;
  }
  return false;
}

void handleCommand(const char* text, size_t len, uint32_t now) {
  ParsedCommand cmd;
  const ParseResult result = parseCommand(text, len, kTeamId, cmd);
  if (result == ParseResult::WrongTeam || result == ParseResult::NotACommand) return;
  if (result == ParseResult::Malformed) {
    Serial.println("# CMD malformed");
    return;
  }

  const bool ok = applyCommand(cmd, now);
  // Counted after applying, so RST (which zeroes the count) counts itself.
  g_store.record().command_count++;
  buildCommandEcho(cmd, g_cmd_echo, sizeof(g_cmd_echo));
  Serial.printf("# CMD %s %s\n", g_cmd_echo, ok ? "OK" : "REJECTED");
  g_store.saveNow(now);
}

void pollRadio(uint32_t now) {
  uint8_t local[sizeof(g_rx_buf)];
  size_t len = 0;
  portENTER_CRITICAL(&g_rx_mux);
  if (g_rx_pending) {
    len = g_rx_len;
    memcpy(local, g_rx_buf, len);
    g_rx_pending = false;
  }
  portEXIT_CRITICAL(&g_rx_mux);
  if (len > 0) handleCommand(reinterpret_cast<const char*>(local), len, now);
}

void pollSerial(uint32_t now) {
  // Reads only what has already arrived; never waits for more.
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      if (!g_serial_overflow && g_serial_len > 0) handleCommand(g_serial_line, g_serial_len, now);
      g_serial_len = 0;
      g_serial_overflow = false;
    } else if (g_serial_len < sizeof(g_serial_line)) {
      g_serial_line[g_serial_len++] = c;
    } else {
      g_serial_overflow = true;  // Drop the whole over-long line.
    }
  }
}

void sendTelemetryIfDue(uint32_t now, const SensorReadings& r) {
  const bool released = g_release.activated();
  const uint32_t since_release = g_release_time_known ? now - g_release_ms : UINT32_MAX;
  const uint32_t period =
      telemetryPeriodMs(released, since_release, g_mission.isLanded(), g_telemetry_enabled);
  if (period == 0) return;
  if (g_sent_any && now - g_last_tx_ms < period) return;

  FlightRecord& rec = g_store.record();
  // F1: the count includes this packet, and is saved by FlightStore.
  rec.packet_count++;

  TelemetryFrame f;
  f.team_id         = kTeamId;
  f.mission_time_ms = g_store.missionTimeMs(now);
  f.packet_count    = rec.packet_count;
  f.command_count   = rec.command_count;
  f.mode            = g_mission.getOpMode();
  f.altitude_m      = r.altitude_m;
  f.pressure_pa     = r.pressure_pa;   // Simulated value while in simulation mode.
  f.temperature_c   = r.temperature_c;
  f.battery_v       = r.battery_v;
  f.battery_ma      = r.battery_ma;
  f.mech_state      = g_release.mechState();
  f.state           = g_mission.getFlState();
  f.cmd_echo        = g_cmd_echo;

  char line[200];
  const size_t len = formatTelemetry(f, line, sizeof(line));
  g_last_tx_ms = now;
  g_sent_any = true;
  if (len == 0) return;

  if (g_radio_up) {
    EspNowLink::send(kGroundStationMac, reinterpret_cast<const uint8_t*>(line), len);
  }
  Serial.println(line);
}

}  // namespace

void setup() {
  FirmwareBoot::begin(kNodeRole, "FLIGHT");
  const uint32_t now = millis();

  // Restore the mission first, so everything below starts from it.
  const bool resumed = g_store.begin(now);
  const FlightRecord& rec = g_store.record();
  g_mission.restore(rec.state, rec.landed);

  const bool sensors_ok = g_sensors.begin();
  g_sensors.restoreMaxAltitude(rec.max_altitude_m);
  g_release.begin(rec.mech_state);

  // Register the receive handler before starting the radio, so a packet
  // arriving during start-up has somewhere to go.
  EspNowLink::onReceive(onRadioRx);
  g_radio_up = EspNowLink::begin(kEspNowChannel) && EspNowLink::addPeer(kGroundStationMac);

  Serial.printf("# %s mission record: state=%s packets=%lu mission_time_ms=%lu\n",
                resumed ? "resumed" : "new", flight_state_to_string(rec.state),
                static_cast<unsigned long>(rec.packet_count),
                static_cast<unsigned long>(g_store.missionTimeMs(now)));
  Serial.printf("# sensors: baro=%s power=%s ground_cal=%s\n",
                g_sensors.readings().baro_ok ? "ok" : "FAIL",
                g_sensors.readings().power_ok ? "ok" : "FAIL",
                g_sensors.isGroundCalibrated() ? "yes" : "no");
  if (!sensors_ok) Serial.println("# WARNING: a sensor failed to start; continuing");
  if (!g_radio_up) {
    Serial.println("# WARNING: radio not up (ground station MAC/channel set?); "
                   "telemetry on USB serial only");
  }
}

void loop() {
  const uint32_t now = millis();

  // 1. Commands.
  pollRadio(now);
  pollSerial(now);

  // 2. Sensors.
  g_sensors.update(now, g_mission.simActive());
  const SensorReadings& r = g_sensors.readings();

  // 3. Flight state.
  const FlightState before = g_mission.getFlState();
  const bool landed_before = g_mission.isLanded();
  const FlightState after = g_mission.updateState(r);
  if (after != before) {
    if (after == PQ_RELEASE) markReleased(now);  // C3
    Serial.printf("# STATE %s -> %s (alt %.1f m, max %.1f m)\n", flight_state_to_string(before),
                  flight_state_to_string(after), static_cast<double>(r.altitude_m),
                  static_cast<double>(r.max_altitude_m));
  }
  if (g_mission.isLanded() && !landed_before) Serial.println("# LANDED: telemetry stopped");
  g_release.update();

  // 4. Persist.
  FlightRecord& rec = g_store.record();
  const uint8_t mech_before = rec.mech_state;
  rec.state = after;
  rec.landed = g_mission.isLanded();
  rec.mech_state = g_release.mechState();
  if (r.max_altitude_m > rec.max_altitude_m) rec.max_altitude_m = r.max_altitude_m;
  if (after != before || rec.landed != landed_before || rec.mech_state != mech_before) {
    g_store.saveNow(now);
  } else {
    g_store.saveIfDue(now);
  }

  // 5. Telemetry.
  sendTelemetryIfDue(now, r);
}
