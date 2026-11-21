#pragma once

#include <stdint.h>

#include "firmware_boot.hpp"

// Container configuration: hardware assignments, radio settings and mission
// thresholds. This header is plain C++ (no Arduino includes) so the host
// tests in software/tests can compile the mission logic against it.

namespace FirmwareConfig {
constexpr FirmwareBoot::NodeRole kNodeRole = FirmwareBoot::NodeRole::Container;

// ---------------------------------------------------------------------------
// Team and radio
// ---------------------------------------------------------------------------
// **CHECK:** placeholder. Set to the team number the competition assigns.
// It appears in the telemetry ID ("1000C") and commands addressed to any
// other team number are ignored (requirement X5).
constexpr uint16_t kTeamId = 1000;

// **CHECK:** the competition assigns each team an ESP-NOW channel (X2).
constexpr uint8_t kEspNowChannel = 1;

// **CHECK:** MAC of OUR ground-station radio, printed by its boot banner.
// Must never be the broadcast address FF:FF:FF:FF:FF:FF (X6); EspNowLink
// refuses it. All zeros means "not set yet" and nothing will be delivered.
constexpr uint8_t kGroundStationMac[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// ---------------------------------------------------------------------------
// Telemetry timing (CTR15, CTR16)
// ---------------------------------------------------------------------------
constexpr uint32_t kTelemetryFastPeriodMs = 250;   // 4 Hz from power-on.
constexpr uint32_t kTelemetrySlowPeriodMs = 1000;  // 1 Hz after release.
constexpr uint32_t kSlowRateDelayMs       = 5000;  // Switch 5 s after release.

// ---------------------------------------------------------------------------
// Persistence (F1, F2, F8)
// ---------------------------------------------------------------------------
// How often the flight record is saved to flash during normal running. It is
// also saved immediately on every state change and command that changes it.
constexpr uint32_t kPersistPeriodMs = 1000;
// After a reset, the restored packet count is bumped by this much. Packets
// sent after the last save are not on record, and this margin keeps the
// count from ever going backwards (at 4 Hz, one second is 4 packets).
constexpr uint32_t kPacketCountResetMargin = 8;

// ---------------------------------------------------------------------------
// Sensor bus
// ---------------------------------------------------------------------------
// **CHECK:** the container board schematic does not exist yet, so the I2C
// pins are unknown. -1 tells Wire.begin() to use the ESP32 defaults
// (SDA = GPIO21, SCL = GPIO22), which is what the bench tests used. Set the
// real pins once the board is drawn.
constexpr int      kI2cSdaPin  = -1;
constexpr int      kI2cSclPin  = -1;
constexpr uint32_t kI2cClockHz = 400000;  // Both BMP581 and INA236 support fast mode.

// BMP581 barometer. Its address is set by its SDO pin; the bench test found
// the breakout at 0x47 and fell back to 0x46, so we try both.
// **CHECK:** confirm how SDO is wired on the container board. Both addresses
// are tried, but only the right one should answer.
constexpr uint8_t kBaroAddrPrimary   = 0x47;
constexpr uint8_t kBaroAddrSecondary = 0x46;

// INA236 voltage/current monitor.
// **CHECK:** the address is set by the A0 pin (GND=0x40, VS=0x41, SDA=0x42,
// SCL=0x43). 0x40 is a guess; confirm once the container board is drawn.
constexpr uint8_t kPowerMonitorAddr = 0x40;
// Shunt resistor on the voltage/current monitor block (electrical/blocks).
// **CHECK:** this is the value in the reusable block. Confirm the container
// board uses the same 0.1 ohm part, or every current reading is scaled wrong.
constexpr float kShuntOhms = 0.1f;

// ---------------------------------------------------------------------------
// Release mechanism (CTR10, CTR11, CTR12)
// ---------------------------------------------------------------------------
// **CHECK:** the release actuator and both sensors are not designed yet.
// -1 means "not fitted": the firmware still tracks the mechanism state but
// drives and reads no pins. See release_mechanism.hpp.
constexpr int kReleaseDeployedSensorPin = -1;  // CTR11: mechanism fully deployed.
constexpr int kPqClearedSensorPin       = -1;  // CTR12: PQ cleared the rail.

// ---------------------------------------------------------------------------
// Altitude processing
// ---------------------------------------------------------------------------
// Number of samples in the altitude moving-average window. At the BMP581's
// 10 Hz output rate, 5 samples smooth over roughly half a second.
constexpr uint8_t kAltitudeWindow = 5;
// Weight given to each new vertical-speed estimate (0..1). Lower values give
// a smoother but slower-reacting speed.
constexpr float kVerticalSpeedAlpha = 0.3f;
// Minimum time between barometer samples, in milliseconds (matches 10 Hz ODR).
constexpr uint32_t kSensorPeriodMs = 100;

// ---------------------------------------------------------------------------
// Flight-state detection (MissionManager::updateState)
// ---------------------------------------------------------------------------
// **CHECK:** the launch/apogee/landing thresholds are first guesses. All
// altitudes are metres above the launch pad. Tune them against
// simulation-mode runs with the judges' pressure profile.
constexpr float kLaunchAltitudeM = 20.0f;   // Above this and climbing -> ASCENT.
constexpr float kLaunchSpeedMps  = 5.0f;    // Minimum climb rate for ASCENT.
constexpr float kApogeeDropM     = 5.0f;    // Fall this far below max -> APOGEE.
// C3: release the PocketQube when the container has descended to 90 % of
// the peak altitude. This one comes straight from the guide.
constexpr float kReleaseFraction = 0.9f;
// Landing (CTR16: stop transmitting when landed): vertical speed stays
// below kLandedSpeedMps for kLandedConfirmMs.
constexpr float    kLandedSpeedMps  = 1.0f;
constexpr uint32_t kLandedConfirmMs = 5000;
// A transition only happens after its condition holds for this many
// consecutive samples, so a single noisy reading can't change the state.
constexpr uint8_t kStateConfirmSamples = 3;
}  // namespace FirmwareConfig
