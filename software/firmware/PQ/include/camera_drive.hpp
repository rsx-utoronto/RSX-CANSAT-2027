#pragma once

// CameraDrive: moves the camera to the angle CameraPointing asks for.
//
// The camera turns on a DC gearmotor through an H-bridge. A motor doesn't
// hold an angle by itself, so this class reads an AS5600 absolute encoder
// on the camera shaft and closes the position loop (PID). Because the
// encoder measures the real angle, get_measuredDeg() can be sent in the
// IMAGE_STABILIZATION telemetry to show the judges the camera is held.
//
// The driver headers stay in the .cpp, so this header pulls in no Arduino
// code.
//
// TODO: the shared I2C bus (Wire) must be started before begin(), by the
// PQ sensor code.
// TODO: call update() at kCamMotorLoopHz (200 Hz), faster than the 100 Hz
// control loop. Give it its own timing in control_task.

class CameraDrive {
public:
  // Starts the encoder and the motor, with the motor stopped. Returns false
  // if either is missing (e.g. pins not set yet); the caller may continue.
  bool begin();

  // Camera angle relative to the body, in degrees. Clamped to the travel
  // limits in config.hpp.
  void setTargetDeg(float deg);

  // Call regularly: reads the encoder and runs one step of the position
  // loop. dt_s is the time since the last call.
  void update(float dt_s);

  float get_targetDeg() const { return target_deg_; }
  // Measured camera angle from the encoder, relative to the body, in
  // (-180, 180]. NAN until the first good encoder reading.
  float get_measuredDeg() const { return measured_deg_; }
  bool  get_health() const { return healthy_; }

private:
  float target_deg_   = 0.0f;
  float measured_deg_ = __builtin_nanf("");
  bool  healthy_      = false;

  // Position loop state.
  float integral_          = 0.0f;
  float prev_measured_deg_ = 0.0f;
  bool  have_prev_         = false;
};
