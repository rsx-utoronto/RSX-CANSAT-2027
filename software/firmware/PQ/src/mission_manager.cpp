#include "mission_manager.hpp"

#include <math.h>

#include "config.hpp"

using namespace FirmwareConfig;

void MissionManager::begin(uint32_t now_ms, bool panels_already_out) {
    // Start the stopwatch: everything after power on is timed from here.
    power_on_ms = now_ms;

    // After a mid-flight reset millis() restarts at 0, so the timer would
    // run again. If the detectors show the panels are already out, there
    // is nothing left to do.
    panels_requested = panels_already_out;
    panels_due = false;
}

void MissionManager::update(uint32_t now_ms) {
    // Non-blocking timer: compare elapsed time instead of using delay(), so
    // telemetry, commands and the camera loop keep running. Unsigned
    // subtraction stays correct when millis() wraps around.
    if (!panels_requested && now_ms - power_on_ms >= kPanelDeployDelayMs) {
        panels_requested = true;  // Fire once only.
        panels_due = true;
    }
}

void MissionManager::requestBoomDeploy() {
    boom_due = true;
}

void MissionManager::requestCameraRotate(float degrees) {
    if (!isfinite(degrees)) return;  // Ignore a corrupt command.

    // Commands add up: +90 then +90 points 180 degrees from the start.
    // fmodf keeps the total in (-360, 360); the if moves it to [0, 360).
    heading_offset_deg = fmodf(heading_offset_deg + degrees, 360.0f);
    if (heading_offset_deg < 0.0f) heading_offset_deg += 360.0f;
}

SimModeStatus MissionManager::getSimStatus() { return sim_status; }

OperatingMode MissionManager::getOpMode() { return op_mode; }

void MissionManager::setSimStatus(SimModeStatus status) { sim_status = status; }

void MissionManager::setOpMode(OperatingMode mode) { op_mode = mode; }
