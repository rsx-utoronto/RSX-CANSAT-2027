#pragma once

// MissionManager: the container's flight-state machine and operating mode.
//
// Plain C++ with no Arduino dependencies, so software/tests can drive it
// with synthetic flight profiles on a PC (see test_container_logic.py).

#include <stdint.h>

#include "sensor_readings.hpp"

enum SimModeStatus : uint8_t {
    SIM_OFF = 0,  // Flight mode, real barometer.
    SIM_EN  = 1,  // SIM ENABLE received; waiting for SIM ACTIVATE.
    SIM_ON  = 2   // Simulation active; SIMP pressures replace the barometer.
};

enum FlightState : uint8_t {
    LAUNCH_PAD = 0,
    ASCENT     = 1,
    APOGEE     = 2,
    PQ_RELEASE = 3,
    IDLE       = 4   // Internal only: powered on but not armed (see below).
};

enum OperatingMode : uint8_t {
    OPMODE_FLIGHT = 0,
    OPMODE_SIM    = 1
};

// 'F' or 'S' for the MODE telemetry field.
inline char op_mode_to_char(OperatingMode mode) {
    return mode == OPMODE_SIM ? 'S' : 'F';
}

// Internal state name, for debug output.
inline const char* flight_state_to_string(FlightState state) {
    switch (state) {
        case LAUNCH_PAD: return "LAUNCH_PAD";
        case ASCENT:     return "ASCENT";
        case APOGEE:     return "APOGEE";
        case PQ_RELEASE: return "PQ_RELEASE";
        case IDLE:       return "IDLE";
    }
    return "UNKNOWN";
}

// State name for the STATE telemetry field. The guide only defines
// LAUNCH_PAD ("waiting for command or launch"), ASCENT, APOGEE and
// PQ_RELEASE. IDLE is our internal "not armed yet" state, which is still
// "waiting for command" on the pad, so it is reported as LAUNCH_PAD.
inline const char* telemetry_state_name(FlightState state) {
    return state == IDLE ? "LAUNCH_PAD" : flight_state_to_string(state);
}

// Result of a SIM command, so the caller can react (e.g. reset the altitude
// filter when simulation turns on or off).
enum class SimCommandResult : uint8_t {
    Rejected,   // Unknown argument, or ACTIVATE without a prior ENABLE.
    Enabled,    // SIM ENABLE accepted; simulation not active yet.
    Activated,  // SIM ACTIVATE accepted; simulation now active.
    Disabled    // Back to flight mode.
};

class MissionManager {

private:
    FlightState fl_state = FlightState::IDLE;
    SimModeStatus sim_status = SimModeStatus::SIM_OFF;
    OperatingMode op_mode = OperatingMode::OPMODE_FLIGHT;

    // True once landing has been detected after PQ_RELEASE. Telemetry stops
    // when this is set (CTR16). Not a FlightState, because the guide's state
    // list has no "landed" state.
    bool landed = false;

    // Number of consecutive samples the next transition's condition has
    // held. A transition fires once this reaches kStateConfirmSamples.
    uint8_t confirm_count = 0;

    // Start time of the current "speed is near zero" stretch, for landing
    // detection. Only meaningful while still_tracking is true.
    uint32_t still_since_ms = 0;
    bool     still_tracking = false;

    // Time of the last altitude sample used, so the same sample isn't
    // counted twice when the loop runs faster than the sensors.
    uint32_t last_sample_ms = 0;
    bool     have_sample = false;

public:
    FlightState getFlState() const { return fl_state; }

    SimModeStatus getSimStatus() const { return sim_status; }

    OperatingMode getOpMode() const { return op_mode; }

    bool isLanded() const { return landed; }

    // True while SIMP pressures should replace the barometer.
    bool simActive() const { return sim_status == SIM_ON; }

    void setFlState(FlightState state);

    // ARM command: enables launch detection (IDLE -> LAUNCH_PAD). Returns
    // false if already armed or in flight, in which case nothing changes.
    bool arm();

    // SIM command. arg is "ENABLE", "ACTIVATE" or "DISABLE". Simulation only
    // starts after ENABLE followed by ACTIVATE (F6), so a single stray
    // command can't switch the flight software to fake pressure.
    SimCommandResult simCommand(const char* arg);

    // Restores state saved before a processor reset (F8). Simulation mode is
    // deliberately NOT restored: a reset always comes back in flight mode.
    void restore(FlightState state, bool was_landed);

    // Moves the flight state forward from the latest sensor readings. Call
    // once per loop after SensorManager::update(). Returns the new state.
    //
    //   IDLE        -> (none)      Not armed. Leave it with arm().
    //   LAUNCH_PAD  -> ASCENT      altitude > kLaunchAltitudeM and climbing
    //                              faster than kLaunchSpeedMps
    //   ASCENT      -> APOGEE      altitude has fallen kApogeeDropM below max
    //   APOGEE      -> PQ_RELEASE  altitude <= kReleaseFraction (90 %) of max
    //                              altitude (requirement C3)
    //   PQ_RELEASE  -> (none)      final state; landing is tracked separately
    //                              in isLanded()
    //
    // States only move forward, and each transition must hold for
    // kStateConfirmSamples new samples in a row.
    FlightState updateState(const SensorReadings& readings);
};
