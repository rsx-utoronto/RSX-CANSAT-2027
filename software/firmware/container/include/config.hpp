#pragma once

#include <stdint.h>

#include "firmware_boot.hpp"

namespace FirmwareConfig {
constexpr FirmwareBoot::NodeRole kNodeRole = FirmwareBoot::NodeRole::Container;

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
// **CHECK:** every threshold below is a placeholder until the 2027 mission
// profile and competition guide are final. All altitudes are metres above
// the launch pad. Tune them against simulation-mode test runs.
constexpr float kLaunchAltitudeM  = 20.0f;   // Above this and climbing -> ASCENT.
constexpr float kLaunchSpeedMps   = 5.0f;    // Minimum climb rate for ASCENT.
constexpr float kApogeeDropM      = 5.0f;    // Fall this far below max -> APOGEE.
// **CHECK:** if the flight peaks below this altitude, PQ_RELEASE follows
// straight after APOGEE. Decide whether that is the behaviour we want.
constexpr float kReleaseAltitudeM = 400.0f;  // Descend below this -> PQ_RELEASE.
// A transition only happens after its condition holds for this many
// consecutive updates, so a single noisy reading can't change the state.
constexpr uint8_t kStateConfirmSamples = 3;
}  // namespace FirmwareConfig
