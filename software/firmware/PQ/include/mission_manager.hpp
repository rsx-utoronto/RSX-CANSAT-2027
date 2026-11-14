#pragma once

// MissionManager: decides WHEN the PocketQube's mechanisms should act.
//
// The PQ's mission is not a chain of states like the container's. Its
// events are independent:
//   * Power on      = release from the container. Recording starts (PQ16).
//   * +5 s          = deploy both solar panels (C6, PQ8). A timer.
//   * Ground command = deploy the boom (C9, PQ11). Any time.
//   * Ground command = rotate the camera view by N degrees (C11, PQ17, PQ18).
//
// This class only decides. It never moves a servo. main.cpp asks
// panelsDue() / boomDue(), acts, then calls the matching clear function.
// The camera control loop reads targetHeadingOffsetDeg().
//
// **CHECK:** nothing calls this class yet. main.cpp must call begin() once
// in setup() and update() every loop, and the command handler must call
// requestBoomDeploy() / requestCameraRotate().

#include <stdint.h>

enum SimModeStatus {
    SIM_OFF = 0,
    SIM_EN = 1,
    SIM_ON = 2
};

// **CHECK:** OperatingMode duplicates Mode in firmware_lib/inc/protocol.hpp.
// Merge them into one enum before telemetry uses both.
enum OperatingMode {
    OPMODE_FLIGHT = 0,
    OPMODE_SIM = 1
};

inline const char* op_mode_to_string(OperatingMode mode, int full){
    if(full == 1){
        if(mode == OPMODE_FLIGHT){
            return "FLIGHT";
        } else {
            return "SIM";
        }
    } else {
        if(mode == OPMODE_FLIGHT){
            return "F";
        } else {
            return "S";
        }
    }
}

class MissionManager {

private:
    SimModeStatus sim_status = SimModeStatus::SIM_OFF;
    OperatingMode op_mode = OperatingMode::OPMODE_FLIGHT;

    // Solar panel timer. power_on_ms is when the stopwatch started;
    // panels_requested makes sure the timer only fires once.
    uint32_t power_on_ms = 0;
    bool panels_requested = false;

    // "Do this now" flags, read and cleared by main.cpp.
    bool panels_due = false;
    bool boom_due = false;

    // Total commanded camera rotation, in degrees, kept in [0, 360).
    // Positive is clockwise looking down the nadir (+Z) axis.
    float heading_offset_deg = 0.0f;

public:
    // Call once in setup(). Power on is the moment of release, so this
    // starts the solar panel timer.
    //   now_ms              : current millis()
    //   panels_already_out  : true if the panel deployment detectors (PQ32)
    //                         already show both panels out, e.g. after a
    //                         mid-flight reset. The timer is then skipped.
    void begin(uint32_t now_ms, bool panels_already_out);

    // Call every loop. Checks the solar panel timer. Never blocks.
    void update(uint32_t now_ms);

    // Ground commands (MEC). Safe to call at any time.
    void requestBoomDeploy();
    void requestCameraRotate(float degrees);  // + clockwise, - counter-clockwise

    // Read by main.cpp: true means "act now". Clear after acting.
    bool panelsDue() const { return panels_due; }
    bool boomDue()   const { return boom_due; }
    void clearPanelsDue() { panels_due = false; }
    void clearBoomDue()   { boom_due = false; }

    // Read by the camera control loop: where the camera should point,
    // relative to its heading at power on.
    float targetHeadingOffsetDeg() const { return heading_offset_deg; }

    SimModeStatus getSimStatus();

    OperatingMode getOpMode();

    void setSimStatus(SimModeStatus status);

    void setOpMode(OperatingMode mode);
};
