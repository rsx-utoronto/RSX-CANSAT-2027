#include <iostream>
#include <vector>
#include <string>
#include <cmath>

// Mirrors what we'll actually do in common/include/altitude.hpp
inline double pressureToAltitude(double pressure_pa, double ref_pa) {
    return 44330.0 * (1.0 - std::pow(pressure_pa / ref_pa, 0.1903));
}

int main() {
    std::cout << "=== CanSat toolchain check ===\n\n";

    // 1. Which C++ standard did the compiler actually use?
    std::cout << "C++ standard: ";
    if (__cplusplus >= 201703L)      std::cout << "C++17 or newer  [OK]\n";
    else if (__cplusplus >= 201402L) std::cout << "C++14  [OK, but prefer C++17]\n";
    else                             std::cout << "older than C++14  [PROBLEM]\n";

    // 2. Which compiler?
#if defined(__clang__)
    std::cout << "Compiler:     Clang " << __clang_major__ << "." << __clang_minor__ << "\n";
#elif defined(__GNUC__)
    std::cout << "Compiler:     GCC " << __GNUC__ << "." << __GNUC_MINOR__ << "\n";
#else
    std::cout << "Compiler:     unknown\n";
#endif

    // 3. Architecture — Apple Silicon should report arm64
#if defined(__aarch64__) || defined(__arm64__)
    std::cout << "Architecture: arm64 (Apple Silicon / ARM)\n";
#elif defined(__x86_64__)
    std::cout << "Architecture: x86_64 (Intel / AMD)\n";
#endif

    // 4. Do modern language features work?
    std::vector<std::string> parts = {"telemetry", "works"};
    auto joined = parts[0] + " " + parts[1];
    std::cout << "Features:     " << joined << "  [OK]\n";

    // 5. Does floating point math behave?
    double alt = pressureToAltitude(95000.0, 101325.0);
    std::cout << "Math check:   95000 Pa -> " << alt << " m ";
    if (alt > 500.0 && alt < 600.0) std::cout << "[OK]\n";
    else                            std::cout << "[PROBLEM]\n";

    std::cout << "\nIf every line says OK, you are ready to build.\n";
    return 0;
}
