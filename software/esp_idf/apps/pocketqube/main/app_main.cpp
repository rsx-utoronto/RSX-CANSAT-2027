#include "cansat/app_boot.hpp"

extern "C" void app_main(void) {
    // Power-on mission intents are not dispatched by this inactive scaffold.
    cansat::esp_idf::report_inactive_startup(cansat::esp_idf::AppRole::pocketqube);
    // Camera processing remains a separate experiment, not a flight dependency.
}
