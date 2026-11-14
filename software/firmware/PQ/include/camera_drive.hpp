#pragma once

// CameraDrive: moves the camera to the angle CameraPointing asks for.
// FirmwareConfig::kCameraDrive picks the hardware:
//
//   Option A, CameraDriveType::Servo
//     Positional hobby servo. Its own potentiometer and controller hold the
//     angle, so this only sends a pulse width. No encoder, no position loop.
//
//   Option B, CameraDriveType::DcMotor
//     DC gearmotor through an H-bridge, with an AS5600 absolute encoder on
//     the camera shaft. This class closes the position loop itself (PID).
//
// Callers see the same functions for both options, so switching hardware is
// a one-line change in config.hpp. Driver headers stay in the .cpp, so this
// header pulls in no Arduino code.
//
// TODO: Option B needs the shared I2C bus (Wire) started before begin(),
// and update() called at kCamMotorLoopHz (200 Hz), faster than the 100 Hz
// control loop. Give it its own timing if Option B is chosen.
// TODO: if a servo with position feedback is chosen (analog feedback wire
// or a serial bus servo), read it back here, so get_measuredDeg() works
// for Option A too.

class CameraDrive {
public:
  // Starts the selected hardware and centres the camera. Returns false if
  // something is missing (e.g. pins not set yet); the caller may continue.
  bool begin();

  // Camera angle relative to the body, in degrees. Clamped to the travel
  // limits in config.hpp.
  void setTargetDeg(float deg);

  // Call regularly. Option A: writes the servo. Option B: reads the
  // encoder and runs one step of the position loop.
  void update(float dt_s);

  float get_targetDeg() const { return target_deg_; }
  // Measured camera angle. Option A can't measure it, so this is NAN there.
  float get_measuredDeg() const { return measured_deg_; }
  bool  get_health() const { return healthy_; }

private:
  float target_deg_   = 0.0f;
  float measured_deg_ = 0.0f;
  bool  healthy_      = false;

  // Option B position loop state.
  float integral_     = 0.0f;
  float prev_measured_deg_ = 0.0f;
  bool  have_prev_    = false;
};
