#pragma once
/** @file
 * Role-specific inactive startup reporting, shared by all three app_main entry points.
 */

namespace cansat::esp_idf {

enum class AppRole { container, pocketqube, ground_radio };

// Logs only: does not create tasks or initialize any application peripherals.
// This is an inactive scaffold, not an operational arming/safety interlock.
void report_inactive_startup(AppRole role);

}  // namespace cansat::esp_idf
