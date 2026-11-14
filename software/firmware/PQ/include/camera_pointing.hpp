#pragma once

// CameraPointing: the fast camera loop. Works out where the camera must
// point, relative to the body, so the picture holds still (C8, PQ15).
//
//   picture heading = body heading + camera angle theta
//   theta_cmd       = target picture heading - body heading - rate_z * L
//
// L (kCamLeadS) makes the command lead the body's spin, to make up for the
// servo and loop delay. The target picture heading is the heading at
// lock-on plus the mission manager's commanded offset, so the 90 deg turn
// (C11) is just a new offset.
//
// It only computes the angle. CameraDrive moves the camera.
// Plain C++ with no Arduino.h, so it compiles on a laptop for tests.
//
// TODO: call this from control_task (not written yet), passing
// MissionManager::targetHeadingOffsetDeg() to setTargetOffsetDeg().
// TODO: if mech can only give +/-90 deg of camera travel, add the plan's
// re-centring: after a turn, move the body target 90 deg slowly so the
// camera returns to the middle of its travel. saturated() shows when it's
// needed.

#include "config.hpp"

// Tunables, defaulting to config.hpp. Tests can pass different values.
struct CameraPointingParams {
  float lead_s         = FirmwareConfig::kCamLeadS;
  float travel_min_deg = FirmwareConfig::kCamTravelMinDeg;
  float travel_max_deg = FirmwareConfig::kCamTravelMaxDeg;
};

class CameraPointing {
public:
  using Params = CameraPointingParams;

  explicit CameraPointing(const Params& params = Params()) : p_(params) {}

  // Release and damping phases: hold the camera at the centre of its travel.
  void park();

  // Lock-on: the picture heading right now becomes the target. Call when
  // the body has stopped spinning fast (the plan's "Lock" phase).
  void lock(float body_heading_deg);

  // Commanded turn, from MissionManager::targetHeadingOffsetDeg().
  void setTargetOffsetDeg(float offset_deg) { offset_deg_ = offset_deg; }

  // Call every control cycle. Returns the camera angle command in degrees,
  // relative to the body, inside the travel limits.
  float update(float body_heading_deg, float rate_z_dps);

  bool  locked() const      { return locked_; }
  float commandDeg() const  { return command_deg_; }
  // True when the command hit a travel limit, so the picture is no longer
  // held. The panel loop should bring the body back toward the target.
  bool  saturated() const   { return saturated_; }
  float targetPictureHeadingDeg() const;

private:
  Params p_;
  bool  locked_      = false;
  float lock_deg_    = 0.0f;
  float offset_deg_  = 0.0f;
  float command_deg_ = 0.0f;
  bool  saturated_   = false;
};
