#pragma once
// Radio payload definitions shared by all three boards.
//
// WIRE FORMAT (team decision, 2026-11-21): telemetry is ASCII CSV text.
//   * Each ESP-NOW packet carries exactly one telemetry line, already in the
//     judges' column format, with no line terminator (container: see
//     software/firmware/container/include/container_telemetry.hpp).
//   * The ground-station radio bridge forwards each received payload to the
//     GUI over USB unchanged, followed by "\r\n" (Serial.println). It must
//     not decode, reformat or re-order fields.
//   * Commands go the other way as ASCII "CMD,<team>,<OP>[,<value>]".
// Binary structs must not go over the air: the bridge and the GUI split
// lines on '\r' / '\n', and a binary packet can contain those bytes.
// The old binary ContainerPacket is commented out below for that reason.
#include <stdint.h>
#include <stddef.h>

enum class OpState : uint8_t { LAUNCH_PAD, ASCENT, APOGEE, PQ_RELEASE };
enum class Mode    : char    { FLIGHT = 'F', SIMULATION = 'S' };

#pragma pack(push, 1)
// COMMENTED OUT, NOT DELETED (2026-11-21): the team chose ASCII CSV as the
// over-the-air format, so the container no longer sends a binary packet
// (wire-format rules at the top of firmware_lib/inc/protocol.hpp). Binary
// packets can contain '\r'/'\n' bytes, which break the line framing used
// by the radio bridge and the GUI. The container builds its CSV line in
// software/firmware/container/include/container_telemetry.hpp instead.
// Kept here, commented, in case the decision is revisited. Do not re-enable
// without changing the bridge/GUI framing first.
// struct ContainerPacket {
//   uint16_t team_id;
//   uint32_t mission_time_ms;
//   uint32_t packet_count;
//   uint16_t command_count;
//   Mode     mode;
//   float    altitude_m;
//   uint32_t pressure_pa;
//   float    temperature_c;
//   float    battery_v;
//   uint16_t battery_ma;
//   uint8_t  mech_state;
//   OpState  state;
//   char     cmd_echo[12];
// };

// **CHECK:** the PQ wire format is still to be decided. Under the decision
// above the PQ should also send an ASCII CSV line, which would retire this
// struct and the PQ functions in telemetry.hpp as well.
struct PQPacket{ 
  uint16_t team_id;
  uint32_t mission_time_ms;
  uint32_t packet_count;
  uint16_t command_count;
  char     cmd_echo[12];
  Mode     mode;
  float    altitude_m;
  uint32_t pressure_pa;
  float    temperature_c;
  float    battery_v;
  uint16_t battery_ma;
  //rot-rate xyz
  float    rot_rate_x_dps;
  float    rot_rate_y_dps;
  float    rot_rate_z_dps;
  //accel xyz
  float    accel_x_mps2;
  float    accel_y_mps2;
  float    accel_z_mps2;
  //mag xyz
  float    mag_x_mG;
  float    mag_y_mG;
  float    mag_z_mG;
  //gnss data
  double   gnss_latitude_deg;
  double   gnss_longitude_deg;
  double   gnss_altitude_m;
  uint32_t gnss_time_s;
  uint16_t gnss_sats;
  float    solar_1_v;
  float    solar_2_v;
  uint8_t  mech_state;
  //32 or 64?
  char     image_stabilization[32];
  char     science_exp[32];
};
#pragma pack(pop)


