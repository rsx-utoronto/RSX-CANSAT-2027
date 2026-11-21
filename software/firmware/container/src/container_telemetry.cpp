#include "container_telemetry.hpp"

#include <stdio.h>

#include "config.hpp"

using namespace FirmwareConfig;

size_t formatTelemetry(const TelemetryFrame& f, char* buf, size_t size) {
  if (buf == nullptr || size == 0) return 0;

  // Mission time is printed from integers, not a float, so the millisecond
  // digits are exact even after hours of running (a float only has ~7
  // significant digits).
  const unsigned long secs = static_cast<unsigned long>(f.mission_time_ms / 1000u);
  const unsigned long ms   = static_cast<unsigned long>(f.mission_time_ms % 1000u);

  const int n = snprintf(
      buf, size,
      "%uC,%lu.%03lu,%lu,%lu,%c,%.1f,%lu,%.1f,%.2f,%u,%02X,%s,%s",
      static_cast<unsigned>(f.team_id),
      secs, ms,
      static_cast<unsigned long>(f.packet_count),
      static_cast<unsigned long>(f.command_count),
      op_mode_to_char(f.mode),
      static_cast<double>(f.altitude_m),
      static_cast<unsigned long>(f.pressure_pa),
      static_cast<double>(f.temperature_c),
      static_cast<double>(f.battery_v),
      static_cast<unsigned>(f.battery_ma),
      static_cast<unsigned>(f.mech_state),
      telemetry_state_name(f.state),
      f.cmd_echo ? f.cmd_echo : "");

  // snprintf returns the length it *wanted*; if that didn't fit, the line
  // was truncated and must not be sent.
  if (n < 0 || static_cast<size_t>(n) >= size) return 0;
  return static_cast<size_t>(n);
}

uint32_t telemetryPeriodMs(bool released, uint32_t ms_since_release,
                           bool landed, bool enabled) {
  if (!enabled || landed) return 0;
  if (released && ms_since_release >= kSlowRateDelayMs) return kTelemetrySlowPeriodMs;
  return kTelemetryFastPeriodMs;
}
