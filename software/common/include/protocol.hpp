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
  //'PQ' is placed before similar variables
  uint16_t PQteam_id;
  uint32_t PQmission_time_ms;
  uint32_t gnss_time_s;
  uint32_t PQpacket_count;
  uint16_t PQcommand_count;
  char     PQcmd_echo[12];
  char     PQmode;
  float    PQaltitude_m;
  uint32_t PQpressure_pa;
  float    PQtemperature_c;
  float    PQbattery_v;
  uint16_t PQbattery_ma;
  float    rot_rate_x_dps;
  float    rot_rate_y_dps;
  float    rot_rate_z_dps;
  //rot-rate xyz
  float    accel_x_mps2;
  float    accel_y_mps2;
  float    accel_z_mps2;
  //accel xyz
  float    mag_x_mG;
  float    mag_y_mG;
  float    mag_z_mG;
  //mag xyz
  double   gnss_latitude_deg;
  double   gnss_longitude_deg;
  double   gnss_altitude_m;
  uint16_t gnss_sats;
  float    solar_1_v;
  float    solar_2_v;
  uint8_t  mech_state;
  //IMAGE_STABILIZATION unclear data type
  //SCIENCE_EXP unclear data type
  //Please double-check var names, types, and units
};
#pragma pack(pop)

