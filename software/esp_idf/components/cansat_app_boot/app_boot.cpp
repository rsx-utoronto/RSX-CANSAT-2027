#include "cansat/app_boot.hpp"

#include "esp_log.h"

namespace cansat::esp_idf {
namespace {
const char* role_name(AppRole role) {
    switch (role) {
        case AppRole::container: return "container";
        case AppRole::pocketqube: return "pocketqube";
        case AppRole::ground_radio: return "ground_radio";
    }
    return "unknown";
}
}  // namespace

void report_inactive_startup(AppRole role) {
    ESP_LOGW("cansat", "role=%s profile=devkitc_v4 target=esp32", role_name(role));
    ESP_LOGW("cansat", "%s", "state=INACTIVE configuration=unconfigured_do_not_flash");
    ESP_LOGW("cansat", "%s", "No application tasks, sensors, radio, or actuators started.");
}

}  // namespace cansat::esp_idf
