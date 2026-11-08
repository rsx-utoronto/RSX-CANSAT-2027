#pragma once

#include <stdint.h>

#include "sensor_manager.hpp"

enum SimModeStatus {
    SIM_OFF = 0,
    SIM_EN = 1,
    SIM_ON = 2
};

// **CHECK:** FlightState and OperatingMode duplicate OpState and Mode in
// firmware_lib/inc/protocol.hpp, and OpState has no IDLE. Merge them into one
// set of enums before telemetry uses both.
enum FlightState {
    LAUNCH_PAD = 0,
    ASCENT = 1,
    APOGEE = 2,
    PQ_RELEASE = 3,
    IDLE = 4
};

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

inline const char* flight_state_to_string (FlightState state){
    static const char* states[] = {
        "LAUNCH_PAD",
        "ASCENT",
        "APOGEE",
        "PQ_RELEASE",
        "IDLE"
    };

    return states[state];
}

class MissionManager {

private:
    FlightState fl_state = FlightState::IDLE;
    SimModeStatus sim_status = SimModeStatus::SIM_OFF;
    OperatingMode op_mode = OperatingMode::OPMODE_FLIGHT;

    // Number of consecutive updates the next transition's condition has
    // held. A transition fires once this reaches kStateConfirmSamples.
    uint8_t confirm_count = 0;

    // Time of the last altitude sample used, so the same sample isn't
    // counted twice when the loop runs faster than the sensors.
    uint32_t last_sample_ms = 0;

public:
    FlightState getFlState();

    SimModeStatus getSimStatus();

    OperatingMode getOpMode();

    void setFlState(FlightState state);

    void setSimStatus(SimModeStatus status);

    void setOpMode(OperatingMode mode);

    // Moves the flight state forward from the latest sensor readings. Call
    // once per loop after SensorManager::update(). Returns the new state.
    //
    //   IDLE        -> (none)      Not armed. Leave it only through
    //                              setFlState(LAUNCH_PAD), e.g. on a command.
    //   LAUNCH_PAD  -> ASCENT      altitude > kLaunchAltitudeM and climbing
    //                              faster than kLaunchSpeedMps
    //   ASCENT      -> APOGEE      altitude has fallen kApogeeDropM below max
    //   APOGEE      -> PQ_RELEASE  altitude < kReleaseAltitudeM
    //   PQ_RELEASE  -> (none)      final automatic state
    //
    // States only move forward, and each transition must hold for
    // kStateConfirmSamples new samples in a row.
    //
    // **CHECK:** the flight state is not saved to flash. After a mid-flight
    // reset it starts at IDLE again and never advances, so the PQ would not
    // be released. Persist fl_state before flight.
    //
    // **CHECK:** IDLE has no automatic exit. Decide which command arms the
    // container (calls setFlState(LAUNCH_PAD)) and make sure it is wired up.
    FlightState updateState(const SensorReadings& readings);
};
