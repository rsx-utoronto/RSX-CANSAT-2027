#pragma once
#include <stdint.h>
#include <stddef.h>

enum class OpState : uint8_t { LAUNCH_PAD, ASCENT, APOGEE, PQ_RELEASE };
enum class Mode    : char    { FLIGHT = 'F', SIMULATION = 'S' };

#pragma pack(push, 1)
struct ContainerPacket {
  uint16_t team_id;
  uint32_t mission_time_ms;
  uint32_t packet_count;
  uint16_t command_count;
  char     mode;
  float    altitude_m;
  uint32_t pressure_pa;
  float    temperature_c;
  float    battery_v;
  uint16_t battery_ma;
  uint8_t  mech_state;
  OpState  state;
  char     cmd_echo[12];
};

struct PQPacket{

};
#pragma pack(pop)

