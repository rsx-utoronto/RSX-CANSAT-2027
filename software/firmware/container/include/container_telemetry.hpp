#pragma once

// Container telemetry: what goes over the radio, and how often.
//
// ---------------------------------------------------------------------------
// Wire format rules (team decision 2026-11-21). These apply to the PQ too
// (PQ/include/pq_telemetry.hpp points here).
// ---------------------------------------------------------------------------
//   * Each ESP-NOW packet carries exactly one telemetry line, already in the
//     judges' column format, with no line terminator.
//   * The ground-station radio bridge forwards each received payload to the
//     GUI over USB unchanged, followed by "\r\n" (Serial.println). It must
//     not decode, reformat or re-order fields.
//   * Commands go the other way as ASCII "CMD,<team>,<OP>[,<value>]".
//   * Never send a binary struct over the air (reasons below).
//
// ---------------------------------------------------------------------------
// Why ASCII CSV instead of a binary packet
// ---------------------------------------------------------------------------
// The guide lets each team pick its own over-the-air format (binary or
// ASCII), but the file handed to the judges must be ASCII CSV in a fixed
// column order (GS13). The original plan was to send a packed binary
// ContainerPacket and convert it to CSV on the ground. The team chose ASCII
// CSV instead, and the binary code (firmware_lib protocol.hpp and
// telemetry.*) was removed; it is in git history if ever needed.
//
// We send the CSV line itself. Reasons:
//   * The ground radio bridge forwards each ESP-NOW packet to the PC as one
//     text line ending in "\r\n", and the GUI splits lines on those bytes.
//     A binary packet can contain 0x0A ('\n') or 0x0D ('\r') inside a float
//     or counter, which would split it into broken lines. Text never does.
//   * The ground station can append the line to the CSV file as-is, so the
//     container and the judges' file can't disagree about formatting or
//     rounding.
//   * It is readable in a plain serial monitor, which makes bench testing
//     much easier.
// The cost is size: a line is roughly 80-100 bytes instead of ~45 binary,
// still well under the 250-byte ESP-NOW limit.
//
// What changes because of this:
//   * The ground station must treat container packets as text lines. The
//     GUI recognises them by the "<team>C," prefix (gui-new
//     code/data/process.py, ContainerTelemetry).
//   * The same line is printed on the USB serial port for bench testing.
//
// ---------------------------------------------------------------------------
// Line format (guide, "Container Telemetry Format")
// ---------------------------------------------------------------------------
//   ID,MISSION_TIME,PACKET_COUNT,COMMAND_COUNT,MODE,ALTITUDE,PRESSURE,
//   TEMPERATURE,BATT_V,BATT_I,MECH_STATE,STATE,CMD_ECHO
// e.g.
//   1000C,12.345,42,3,F,123.4,101325,23.5,7.42,123,03,ASCENT,CAL
//
//   ID            team number + 'C'
//   MISSION_TIME  seconds, 3 decimals (1 ms resolution)
//   MODE          'F' flight or 'S' simulation
//   ALTITUDE      metres, 1 decimal (CTR19)
//   PRESSURE      pascals, integer (CTR18)
//   TEMPERATURE   degrees C, 1 decimal (CTR20)
//   BATT_V        volts, 2 decimals (CTR21 asks for 0.01 V)
//   BATT_I        milliamps, integer (CTR22)
//   MECH_STATE    two hex digits: bit 0 mechanism activated, bit 1 PQ released
//   STATE         LAUNCH_PAD / ASCENT / APOGEE / PQ_RELEASE
//   CMD_ECHO      last command processed, commas removed (e.g. "CXON")
//
// No line terminator is sent over the air: each ESP-NOW packet is exactly
// one line, and the ground side adds the line ending when writing the file.
//
// This module is plain C++ (no Arduino) so the host tests can check the
// formatting and the rate schedule.

#include <stddef.h>
#include <stdint.h>

#include "mission_manager.hpp"

// Everything needed to build one telemetry line.
struct TelemetryFrame {
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
  uint8_t       mech_state         = 0;
  FlightState   state              = IDLE;
  const char*   cmd_echo           = "";
};

// Writes the CSV line (no terminator) into buf. Returns the number of
// characters written, or 0 if buf is too small (nothing should be sent).
size_t formatTelemetry(const TelemetryFrame& frame, char* buf, size_t size);

// How often to send telemetry right now (CTR15, CTR16):
//   * 4 Hz (kTelemetryFastPeriodMs) from power-on,
//   * 1 Hz (kTelemetrySlowPeriodMs) from 5 s after the PQ release,
//   * never again once landing is detected.
// Returns the period in ms, or 0 for "don't transmit".
//   released          : the PQ release has happened
//   ms_since_release  : time since release; pass UINT32_MAX if the release
//                       time is unknown (e.g. after a reset), which selects 1 Hz
//   landed            : landing detected
//   enabled           : telemetry switched on (CX ON/OFF command)
uint32_t telemetryPeriodMs(bool released, uint32_t ms_since_release,
                           bool landed, bool enabled);
