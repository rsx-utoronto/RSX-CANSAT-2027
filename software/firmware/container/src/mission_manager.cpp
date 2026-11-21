#include "mission_manager.hpp"

#include <math.h>
#include <string.h>

#include "config.hpp"

using namespace FirmwareConfig;

void MissionManager::setFlState(FlightState state) {
    fl_state = state;
    confirm_count = 0;  // Start confirming the next transition from scratch.
}

bool MissionManager::arm() {
    if (fl_state != IDLE) return false;
    setFlState(LAUNCH_PAD);
    return true;
}

SimCommandResult MissionManager::simCommand(const char* arg) {
    if (arg == nullptr) return SimCommandResult::Rejected;

    if (strcmp(arg, "ENABLE") == 0) {
        // ENABLE only arms simulation; it doesn't start it.
        if (sim_status == SIM_OFF) sim_status = SIM_EN;
        return SimCommandResult::Enabled;
    }
    if (strcmp(arg, "ACTIVATE") == 0) {
        if (sim_status == SIM_OFF) return SimCommandResult::Rejected;  // Needs ENABLE first.
        sim_status = SIM_ON;
        op_mode = OPMODE_SIM;
        return SimCommandResult::Activated;
    }
    if (strcmp(arg, "DISABLE") == 0) {
        sim_status = SIM_OFF;
        op_mode = OPMODE_FLIGHT;
        return SimCommandResult::Disabled;
    }
    return SimCommandResult::Rejected;
}

void MissionManager::restore(FlightState state, bool was_landed) {
    setFlState(state);
    landed = was_landed;
    sim_status = SIM_OFF;
    op_mode = OPMODE_FLIGHT;
}



FlightState MissionManager::updateState(const SensorReadings& readings) {
    // Only count each altitude sample once. The main loop can run many times
    // between barometer samples (or SIMP packets in simulation mode).
    if (have_sample && readings.sample_time_ms == last_sample_ms) return fl_state;
    have_sample = true;
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
            // C3: release at 90 % of peak altitude on the way down.
            condition = readings.altitude_m <= kReleaseFraction * readings.max_altitude_m;
            next = PQ_RELEASE;
            break;

        case PQ_RELEASE:
            // Landing: vertical speed near zero for kLandedConfirmMs.
            if (!landed) {
                if (fabsf(readings.vertical_speed_mps) < kLandedSpeedMps) {
                    if (!still_tracking) {
                        still_tracking = true;
                        still_since_ms = readings.sample_time_ms;
                    } else if (readings.sample_time_ms - still_since_ms >= kLandedConfirmMs) {
                        landed = true;
                    }
                } else {
                    still_tracking = false;
                }
            }
            return fl_state;

        case IDLE:
            // Not armed: no automatic transitions.
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
