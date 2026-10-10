// Private, versioned stdin protocol for flight_replay.py. Never a firmware task.
#include <cansat/flight_detection.hpp>
#include <charconv>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace cansat::mission;
namespace {
std::vector<std::string> tokens(const std::string& line) {
    if (line.size() > 1024) throw std::runtime_error("protocol line too long");
    std::istringstream stream(line);
    std::vector<std::string> out;
    for (std::string word; stream >> word;) out.push_back(word);
    return out;
}
std::uint64_t integer(const std::string& value) {
    std::uint64_t result = 0;
    const auto parsed = std::from_chars(value.data(), value.data() + value.size(), result);
    if (parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size())
        throw std::runtime_error("invalid unsigned integer");
    return result;
}
double number(const std::string& value) {
    char* end = nullptr;
    const double result = std::strtod(value.c_str(), &end);
    if (end == value.c_str() || end != value.c_str() + value.size())
        throw std::runtime_error("invalid numeric token");
    return result;  // NaN/Inf are intentional sensor-fault inputs; config rejects them.
}
const char* phase(DetectionPhase value) {
    switch (value) {
    case DetectionPhase::waiting_for_launch: return "waiting_for_launch";
    case DetectionPhase::ascending: return "ascending";
    case DetectionPhase::descending: return "descending";
    case DetectionPhase::landed: return "landed";
    }
    throw std::runtime_error("unknown detector phase");
}
const char* event(FlightEvent value) {
    switch (value) {
    case FlightEvent::none: return "none";
    case FlightEvent::launch: return "launch";
    case FlightEvent::apogee: return "apogee";
    case FlightEvent::landed: return "landed";
    }
    throw std::runtime_error("unknown detector event");
}
const char* status(AltitudeStatus value) {
    switch (value) {
    case AltitudeStatus::invalid_configuration: return "invalid_configuration";
    case AltitudeStatus::clock_regression: return "clock_regression";
    case AltitudeStatus::missing: return "missing";
    case AltitudeStatus::nonfinite: return "nonfinite";
    case AltitudeStatus::future: return "future";
    case AltitudeStatus::stale: return "stale";
    case AltitudeStatus::out_of_order: return "out_of_order";
    case AltitudeStatus::conflicting_duplicate: return "conflicting_duplicate";
    case AltitudeStatus::duplicate: return "duplicate";
    case AltitudeStatus::seeded: return "seeded";
    case AltitudeStatus::gap_reseeded: return "gap_reseeded";
    case AltitudeStatus::implausible_rate: return "implausible_rate";
    case AltitudeStatus::accepted: return "accepted";
    }
    throw std::runtime_error("unknown detector status");
}
void optional_number(const std::optional<double>& value) {
    if (value) std::cout << *value;
}
}  // namespace

int main(int argc, char** argv) {
    try {
        if (argc != 2 || std::string(argv[1]) != "--protocol-v1")
            throw std::runtime_error("use flight_replay.py; backend requires --protocol-v1");
        std::cout.imbue(std::locale::classic());
        std::cout << std::setprecision(17);
        std::string line;
        if (!std::getline(std::cin, line)) throw std::runtime_error("missing config");
        const auto t = tokens(line);
        if (t.size() != 17 || t[0] != "CONFIG" || (t[1] != "on_pad" && t[1] != "descending_after_release"))
            throw std::runtime_error("invalid config protocol");
        FlightDetectionConfig c;
        c.max_sample_age_ms = integer(t[2]); c.max_sample_gap_ms = integer(t[3]);
        c.max_abs_vertical_speed_mps = number(t[4]); c.launch_altitude_m = number(t[5]);
        c.launch_min_climb_mps = number(t[6]); c.launch_confirm_ms = integer(t[7]);
        c.apogee_min_drop_m = number(t[8]); c.apogee_min_descent_mps = number(t[9]);
        c.apogee_confirm_ms = integer(t[10]); c.landing_altitude_min_m = number(t[11]);
        c.landing_altitude_max_m = number(t[12]); c.landing_max_abs_speed_mps = number(t[13]);
        c.landing_max_span_m = number(t[14]); c.landing_confirm_ms = integer(t[15]);
        const auto count = integer(t[16]);
        if (count > std::numeric_limits<std::size_t>::max()) throw std::runtime_error("sample count overflow");
        c.min_confirm_samples = static_cast<std::size_t>(count);
        AltitudeFlightDetector detector(c, t[1] == "on_pad" ? DetectorStart::on_pad : DetectorStart::descending_after_release);
        if (!detector.configuration_valid()) throw std::runtime_error("invalid detector configuration");
        std::cout << "row,phase,status,event,qualified_altitude_m,vertical_speed_mps,peak_altitude_m\n";
        std::size_t row = 0;
        while (std::getline(std::cin, line)) {
            const auto fields = tokens(line);
            if (++row > 100000 || fields.size() != 5 || fields[0] != "S")
                throw std::runtime_error("invalid sample protocol or row limit exceeded");
            DetectionInput input;
            input.now_ms = integer(fields[1]);
            if ((fields[2] == "-") != (fields[3] == "-")) throw std::runtime_error("incomplete missing sample");
            if (fields[2] != "-") input.altitude = Altitude{number(fields[3]), integer(fields[2])};
            if (fields[4] != "0" && fields[4] != "1") throw std::runtime_error("invalid ARM level");
            input.armed = fields[4] == "1";
            const auto out = detector.update(input);
            std::cout << row << ',' << phase(out.phase) << ',' << status(out.status) << ',' << event(out.event) << ',';
            if (out.qualified_altitude) std::cout << out.qualified_altitude->agl_m;
            std::cout << ','; optional_number(out.vertical_speed_mps);
            std::cout << ','; optional_number(out.peak_altitude_m);
            std::cout << '\n';
        }
        if (std::cin.bad() || row == 0) throw std::runtime_error("empty trace or input read error");
        if (!std::cout) throw std::runtime_error("output write error");
    } catch (const std::exception& error) {
        std::cerr << "flight replay backend: " << error.what() << '\n';
        return 2;
    }
}
