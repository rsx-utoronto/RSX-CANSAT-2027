#include "cansat/app_boot.hpp"

extern "C" void app_main(void) {
    // No ESP-NOW peers/channel or command UART are configured. Console logs only.
    cansat::esp_idf::report_inactive_startup(cansat::esp_idf::AppRole::ground_radio);
    // The existing GUI/legacy bridge protocol is not connected here yet.
}
