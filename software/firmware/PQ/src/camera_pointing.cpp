#include "camera_pointing.hpp"

#include <math.h>

#include "heading_estimator.hpp"  // wrap180(), wrap360()

void CameraPointing::park() {
  locked_ = false;
  command_deg_ = 0.0f;
  saturated_ = false;
}

void CameraPointing::lock(float body_heading_deg) {
  // The camera may not be centred at lock-on, so lock to where the picture
  // actually points: body heading plus the current camera angle. Any
  // commanded offset already set is kept, and applies on top of this.
  lock_deg_ = wrap360(body_heading_deg + command_deg_ - offset_deg_);
  locked_ = true;
}

float CameraPointing::targetPictureHeadingDeg() const {
  return wrap360(lock_deg_ + offset_deg_);
}

float CameraPointing::update(float body_heading_deg, float rate_z_dps) {
  if (!locked_ || !isfinite(body_heading_deg)) return command_deg_;
  if (!isfinite(rate_z_dps)) rate_z_dps = 0.0f;

  // Where the body will be once the camera has actually moved.
  const float predicted_body_deg = body_heading_deg + rate_z_dps * p_.lead_s;
  const float wanted = wrap180(targetPictureHeadingDeg() - predicted_body_deg);

  // The same direction can be reached as wanted, wanted - 360 or wanted + 360.
  // Pick the one inside the travel limits that is closest to the current
  // command, so the camera never swings the long way round past +/-180.
  float best = wanted;
  float best_cost = INFINITY;
  bool  reachable = false;
  const float candidates[3] = {wanted, wanted - 360.0f, wanted + 360.0f};
  for (float candidate : candidates) {
    const bool inside = candidate >= p_.travel_min_deg && candidate <= p_.travel_max_deg;
    const float cost = fabsf(candidate - command_deg_);
    // Prefer any reachable angle over an unreachable one, then the closest.
    if ((inside && !reachable) || (inside == reachable && cost < best_cost)) {
      best = candidate;
      best_cost = cost;
      reachable = reachable || inside;
    }
  }

  // If no version is reachable, hold at the nearest limit.
  saturated_ = !reachable;
  if (best < p_.travel_min_deg) best = p_.travel_min_deg;
  if (best > p_.travel_max_deg) best = p_.travel_max_deg;

  command_deg_ = best;
  return command_deg_;
}
