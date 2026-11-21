#pragma once

// CameraDrive: moves the camera to the angle CameraPointing asks for.
//
// The camera turns on a positional hobby servo. The servo's own
// potentiometer and controller hold the angle, so this only sends a pulse
// width: no encoder, no position loop.
//
// The servo driver header stays in the .cpp, so this header pulls in no
// Arduino code.
//
// TODO: if a servo with position feedback is chosen (analog feedback wire
// or a serial bus servo), read it back here, so get_measuredDeg() reports
// the real angle.

class CameraDrive {
public:
  // Attaches the servo and centres the camera. Returns false if the servo
  // can't be attached (e.g. the pin isn't set yet); the caller may continue.
  bool begin();

  // Camera angle relative to the body, in degrees. Clamped to the travel
  // limits in config.hpp.
  void setTargetDeg(float deg);

  // Call every control cycle: writes the target to the servo.
  void update();

  float get_targetDeg() const { return target_deg_; }
  // Measured camera angle. A plain servo can't report it, so this is NAN.
  float get_measuredDeg() const;
  bool  get_health() const { return healthy_; }

private:
  float target_deg_ = 0.0f;
  bool  healthy_    = false;
};
