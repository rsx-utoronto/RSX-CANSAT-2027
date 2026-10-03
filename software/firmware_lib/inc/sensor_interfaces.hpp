#pragma once
#include <stdint.h>
#include <stddef.h>

class IBarometer{
public:
  virtual bool  begin() = 0;
  virtual float readPressurePa() = 0;
  virtual float readTemperatureC() = 0;
  virtual ~IBarometer() = default;
};

class IPowerMonitor {
public:
  virtual bool  begin() = 0;
  virtual float readVolts() = 0;
  virtual float readMilliamps() = 0;
  virtual ~IPowerMonitor() = default;
};

class IMU {

};

