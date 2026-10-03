#pragma once
#include "protocol.hpp"
#include <stdint.h>

struct ContainerSensorData {
  float    altitude_m;
  uint32_t pressure_pa;
  float    temperature_c;
  float    battery_v;
  uint16_t battery_ma;
  uint8_t  mech_state;
};
struct PQSensorData {
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
  float    solar_1_v;
  float    solar_2_v;
};

class ContainerSensorInterface {
public:
  virtual ~ContainerSensorInterface() = default;
};
