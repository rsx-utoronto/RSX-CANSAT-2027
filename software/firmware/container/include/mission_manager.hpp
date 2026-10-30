//test
#pragma once

enum SimModeStatus {
    SIM_OFF = 0,
    SIM_EN = 1,
    SIM_ON = 2
};

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

public:
    FlightState getFlState();

    SimModeStatus getSimStatus();

    OperatingMode getOpMode();

    void setFlState(FlightState state);

    void setSimStatus(SimModeStatus status);

    void setOpMode(OperatingMode mode);
};