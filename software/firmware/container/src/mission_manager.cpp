#include "mission_manager.hpp"

#include "config.hpp"

using namespace FirmwareConfig;

FlightState MissionManager::getFlState() { return fl_state; }

SimModeStatus MissionManager::getSimStatus() { return sim_status; }

OperatingMode MissionManager::getOpMode() { return op_mode; }

void MissionManager::setFlState(FlightState state) {
    fl_state = state;
    confirm_count = 0;  // Start confirming the next transition from scratch.
}

void MissionManager::setSimStatus(SimModeStatus status) { sim_status = status; }

void MissionManager::setOpMode(OperatingMode mode) { op_mode = mode; }

FlightState MissionManager::updateState(const SensorReadings& readings) {
    // Only count each altitude sample once. The main loop can run many times
    // between barometer samples (or SIMP packets in simulation mode).
    if (readings.sample_time_ms == last_sample_ms) return fl_state;
    last_sample_ms = readings.sample_time_ms;

    // Work out whether the condition for leaving the current state holds,
    // and which state it leads to.
    bool condition = false;
    FlightState next = fl_state;

    switch (fl_state) {
        case LAUNCH_PAD:
            condition = readings.altitude_m > kLaunchAltitudeM &&
                        readings.vertical_speed_mps > kLaunchSpeedMps;
            next = ASCENT;
            break;

        case ASCENT:
            // Apogee is detected once altitude has clearly dropped below the
            // peak, not when the speed first reaches zero, which noise can
            // briefly cause during the climb.
            condition = readings.max_altitude_m - readings.altitude_m > kApogeeDropM;
            next = APOGEE;
            break;

        case APOGEE:
            condition = readings.altitude_m < kReleaseAltitudeM;
            next = PQ_RELEASE;
            break;

        case PQ_RELEASE:
        case IDLE:
            // No automatic transitions out of these states.
            return fl_state;
    }

    // Debounce: the condition must hold for several new samples in a row.
    if (!condition) {
        confirm_count = 0;
        return fl_state;
    }
    if (++confirm_count >= kStateConfirmSamples) {
        setFlState(next);
    }
    return fl_state;
}
