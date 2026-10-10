#include "cansat/app_boot.hpp"

extern "C" void app_main(void) {
    // No mission update without explicit configuration and real, qualified input.
    cansat::esp_idf::report_inactive_startup(cansat::esp_idf::AppRole::container);
    // Returning ends the SDK main task; this scaffold creates no flight tasks.
}
