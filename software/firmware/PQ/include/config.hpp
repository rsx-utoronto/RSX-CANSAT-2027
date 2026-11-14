#pragma once

#include <stdint.h>

#include "firmware_boot.hpp"

namespace FirmwareConfig {
constexpr FirmwareBoot::NodeRole kNodeRole = FirmwareBoot::NodeRole::PocketQube;

// ---------------------------------------------------------------------------
// Mission timing
// ---------------------------------------------------------------------------
// Solar panels deploy this long after power on (C6: five seconds after the
// PocketQube powers up).
constexpr uint32_t kPanelDeployDelayMs = 5000;
}  // namespace FirmwareConfig
