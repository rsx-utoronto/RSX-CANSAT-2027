#include <cstdarg>
#include <cstdio>
#include <iostream>
#include <string>
#include <vector>

extern "C" void app_main(void);

namespace {
std::vector<std::string> lines;
bool format_error = false;
}  // namespace

void cansat_test_log(const char* tag, const char* format, ...) {
    char buffer[256]{};
    va_list args;
    va_start(args, format);
    const int length = std::vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    if (length < 0 || static_cast<std::size_t>(length) >= sizeof(buffer)) {
        format_error = true;
        return;
    }
    lines.emplace_back(std::string(tag) + ": " + buffer);
}

int main() {
    // Call the production entry point, not a duplicate host-only boot function.
    app_main();
    const std::vector<std::string> expected{
        "cansat: role=" CANSAT_EXPECTED_ROLE " profile=devkitc_v4 target=esp32",
        "cansat: state=INACTIVE configuration=unconfigured_do_not_flash",
        "cansat: No application tasks, sensors, radio, or actuators started.",
    };
    if (format_error || lines != expected) {
        std::cerr << "Unexpected startup log for " CANSAT_EXPECTED_ROLE "\n";
        for (const auto& line : lines) std::cerr << line << '\n';
        return 1;
    }
    for (const auto& line : lines) std::cout << line << '\n';
    std::cout << "HOST_SMOKE: actual app_main returned; private logging substitute; not an SDK/hardware test\n";
    return 0;
}
