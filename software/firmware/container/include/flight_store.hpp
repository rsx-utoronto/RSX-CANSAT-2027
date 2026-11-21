#pragma once

// FlightStore: the part of the mission that must survive a processor reset
// or a momentary power loss, kept in ESP32 flash (NVS).
//
// Requirements covered:
//   F1  packet count keeps counting through resets
//   F2  mission time keeps running through resets
//   F8  configuration state (flight state, armed, calibration) survives a
//       reset. The CAL ground pressure itself is saved by SensorManager.
//
// How it works: everything lives in one FlightRecord. It is written to NVS
// as a single blob on every important event (state change, command) and at
// most once per kPersistPeriodMs otherwise. On boot the last record is
// loaded and the mission carries on from it.
//
// Limits until the RTC is added:
//   * Mission time is restored from the last save, so a reset loses up to
//     kPersistPeriodMs plus the boot time (~0.3 s). **CHECK:** once the RTC
//     is on the board, take mission time from it instead (see
//     missionTimeMs()).
//   * The packet count is bumped by kPacketCountResetMargin after a reset,
//     so it can skip a few numbers but never repeats or goes backwards.
//
// Starting a new mission: the record persists until the RST command clears
// it (main.cpp only allows that before launch). **CHECK:** add "send RST"
// to the mission operations checklist, right after the judge powers the
// container on and before CAL/ARM; otherwise mission time and packet count
// continue from the last bench test.

#include <stdint.h>

#include "mission_manager.hpp"

struct FlightRecord {
  FlightState state          = IDLE;
  bool        landed         = false;
  uint8_t     mech_state     = 0;      // ReleaseMechanism bits.
  uint32_t    packet_count   = 0;
  uint32_t    command_count  = 0;
  uint32_t    mission_time_ms = 0;     // Mission time at the last save.
  float       max_altitude_m = 0.0f;   // Needed for the 90 % release after a reset.
  // UTC from the ST command (F3): the UTC time of day and the mission time
  // at which it was set. UTC "now" is derived from the mission clock.
  bool        utc_set        = false;
  uint32_t    utc_seconds_at_set = 0;
  uint32_t    mission_ms_at_utc_set = 0;
};

class FlightStore {
public:
  // Loads the saved record. Returns true if one was found (i.e. this boot is
  // a continuation after a reset), false for a fresh start.
  bool begin(uint32_t now_ms);

  FlightRecord& record() { return rec_; }
  const FlightRecord& record() const { return rec_; }

  // Mission time in ms: time since the mission record started, continuing
  // across resets.
  uint32_t missionTimeMs(uint32_t now_ms) const;

  // UTC seconds since midnight, if the ST command has set it.
  bool utcSecondsOfDay(uint32_t now_ms, uint32_t& out) const;
  void setUtc(uint32_t now_ms, uint32_t seconds_of_day);

  // Saves now. Use after state changes and commands.
  void saveNow(uint32_t now_ms);
  // Saves only if kPersistPeriodMs has passed since the last save.
  void saveIfDue(uint32_t now_ms);

  // RST: starts a fresh mission record (time 0, counts 0, state IDLE).
  void clear(uint32_t now_ms);

private:
  FlightRecord rec_;
  // Mission time = base_ms_ + (now_ms - epoch_ms_).
  uint32_t base_ms_  = 0;
  uint32_t epoch_ms_ = 0;
  uint32_t last_save_ms_ = 0;
};
