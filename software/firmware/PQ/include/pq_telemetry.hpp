#pragma once

// PQ telemetry: what the PocketQube sends over the radio, and how often.
//
// TODO: not written yet. Build it the same way as the container's
// container_telemetry module: a frame struct (below), a formatTelemetry()
// that writes one ASCII CSV line into a caller's buffer with snprintf (no
// heap), and a rate function (4 Hz). Plain C++ with no Arduino.h, so the
// host tests can check it.
//
// Wire format: ASCII CSV, one line per ESP-NOW packet, no terminator, never
// a binary struct. The rules and the reasons are in the container's
// software/firmware/container/include/container_telemetry.hpp and apply to
// the PQ unchanged.
//
// **CHECK:** the team still has to confirm CSV for the PQ (the container
// decision of 2026-11-21 implies it). The PQ line is longer than the
// container's: check it stays under the 250-byte ESP-NOW limit once the
// column formats are fixed.
//
// Fields come from the guide's PQ telemetry format (pages 25-26). This list
// replaces the binary PQPacket that was in firmware_lib/inc/protocol.hpp
// (removed; see git history). Column order and formats must be checked
// against the guide when formatTelemetry() is written.

#include <stdint.h>

#include "mission_manager.hpp"  // OperatingMode

struct PqTelemetryFrame {
  uint16_t      team_id            = 0;
  uint32_t      mission_time_ms    = 0;
  uint32_t      packet_count       = 0;
  uint32_t      command_count      = 0;
  OperatingMode mode               = OPMODE_FLIGHT;
  float         altitude_m         = 0.0f;
  uint32_t      pressure_pa        = 0;
  float         temperature_c      = 0.0f;
  float         battery_v          = 0.0f;
  uint16_t      battery_ma         = 0;
  // IMU
  float         rot_rate_x_dps     = 0.0f;
  float         rot_rate_y_dps     = 0.0f;
  float         rot_rate_z_dps     = 0.0f;
  float         accel_x_mps2       = 0.0f;
  float         accel_y_mps2       = 0.0f;
  float         accel_z_mps2       = 0.0f;
  // Magnetometer, in milligauss as the guide asks
  float         mag_x_mG           = 0.0f;
  float         mag_y_mG           = 0.0f;
  float         mag_z_mG           = 0.0f;
  // GNSS
  double        gnss_latitude_deg  = 0.0;
  double        gnss_longitude_deg = 0.0;
  float         gnss_altitude_m    = 0.0f;
  uint32_t      gnss_time_s        = 0;
  uint16_t      gnss_sats          = 0;
  // Solar panel voltages, 0.01 V resolution
  float         solar_1_v          = 0.0f;
  float         solar_2_v          = 0.0f;
  uint8_t       mech_state         = 0;     // bits 0-3, see the guide
  // TODO: decide the IMAGE_STABILIZATION and SCIENCE_EXP contents. Candidates
  // from the camera code: picture error, spin_controller twistDeg(),
  // rateTargetDps(), gainScale(). Text fields, so no commas allowed.
  const char*   image_stabilization = "";
  const char*   science_exp         = "";
  const char*   cmd_echo            = "";
};
