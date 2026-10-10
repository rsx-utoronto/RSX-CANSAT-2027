// Canonical typed payload serialization. Validation is shared by encode/decode;
// private readers/writers never interpret sensor freshness or command authority.
#include <cansat/mission_payloads.hpp>
#include <algorithm>
#include <cstdio>
#include <limits>

namespace cansat::com::payload {
namespace {
enum class Schema : std::uint8_t { container = 1, pocket = 2, command = 3 };
struct Writer {
    Bytes bytes{};
    bool ok = true;
    void put(std::uint64_t value, unsigned width) {
        if (!ok || bytes.size + width > max_payload_bytes) { ok = false; return; }
        for (unsigned i = 0; i < width; ++i) bytes.data[bytes.size++] = static_cast<std::uint8_t>(value >> (8 * i));
    }
    explicit Writer(Schema schema) { put(version,1); put(static_cast<unsigned>(schema),1); }
    template<std::size_t N> void text(const Text<N>& t) {
        put(t.size,1); for (char c : t.data) put(static_cast<unsigned char>(c),1);
    }
};
struct Reader {
    const std::uint8_t* data;
    std::size_t size, offset = 0;
    bool ok;
    Reader(const std::uint8_t* p, std::size_t n, Schema schema) : data(p), size(n), ok(p && n <= max_payload_bytes) {
        if (get(1) != version || get(1) != static_cast<unsigned>(schema)) ok = false;
    }
    std::uint64_t get(unsigned width) {
        if (!ok || offset + width > size) { ok = false; return 0; }
        std::uint64_t value = 0;
        for (unsigned i = 0; i < width; ++i) value |= std::uint64_t{data[offset++]} << (8 * i);
        return value;
    }
    std::int32_t i32() {
        const auto x = static_cast<std::uint32_t>(get(4));
        return x <= INT32_MAX ? static_cast<std::int32_t>(x) : -1 - static_cast<std::int32_t>(~x);
    }
    std::int64_t i64() {
        const auto x = get(8);
        return x <= INT64_MAX ? static_cast<std::int64_t>(x) : -1 - static_cast<std::int64_t>(~x);
    }
    template<std::size_t N> void text(Text<N>& t) {
        t.size = static_cast<std::uint8_t>(get(1));
        for (auto& c : t.data) c = static_cast<char>(get(1));
    }
    bool done() const { return ok && offset == size; }
};
template<std::size_t N> bool validText(const Text<N>& t) {
    if (t.size > N) return false;
    for (std::size_t i = 0; i < N; ++i) {
        const auto c = static_cast<unsigned char>(t.data[i]);
        if (i < t.size ? (c < 0x21 || c > 0x7e || c == ',') : c != 0) return false;
    }
    return true;
}
bool validEcho(const Text<24>& t) {
    if (!validText(t)) return false;
    if (t.size && (t.data[0] < 'A' || t.data[0] > 'Z')) return false;
    for (unsigned i = 0; i < t.size; ++i) {
        const auto c = t.data[i];
        if (!(c >= 'A' && c <= 'Z') && !(c >= '0' && c <= '9') &&
            c != '_' && c != '+' && c != '-' && c != '.' && c != ':') return false;
    }
    return true;
}
template<class T> bool canonical(std::uint32_t mask, Field field, T value) { return (mask & bit(field)) || value == 0; }
bool commonValid(const CommonTelemetry& c, std::uint32_t fields, std::uint8_t mechanisms) {
    return (c.mode == Mode::flight || c.mode == Mode::simulation) && !(c.valid & ~fields) &&
        canonical(c.valid,Field::altitude,c.altitude_mm) && canonical(c.valid,Field::pressure,c.pressure_pa) &&
        canonical(c.valid,Field::temperature,c.temperature_milli_c) &&
        canonical(c.valid,Field::battery_voltage,c.battery_mv) && canonical(c.valid,Field::battery_current,c.battery_current_ma) &&
        (!(c.valid & bit(Field::pressure)) || c.pressure_pa > 0) &&
        (!(c.valid & bit(Field::temperature)) || c.temperature_milli_c >= -273150) &&
        !(c.mechanism_known & ~mechanisms) && !(c.mechanism_state & ~c.mechanism_known) && validEcho(c.command_echo);
}
bool valid(const ContainerTelemetry& c) {
    return commonValid(c.common,container_valid_bits,3) && static_cast<unsigned>(c.state) <= 3;
}
bool extensionValid(const Extension& e) {
    return e.size <= extension_budget && (e.size ? e.profile != 0 && e.version != 0 : e.profile == 0 && e.version == 0);
}
bool valid(const PocketTelemetry& p) {
    const auto mask = p.common.valid;
    if (!commonValid(p.common,pocket_valid_bits,15) || !validText(p.gnss_time) ||
        !extensionValid(p.stabilization) || !extensionValid(p.science) ||
        p.stabilization.size + p.science.size > extension_budget) return false;
    for (unsigned i = 0; i < 3; ++i) {
        if (!canonical(mask,static_cast<Field>(5+i),p.gyro_mdeg_s[i]) ||
            !canonical(mask,static_cast<Field>(8+i),p.accel_mm_s2[i]) ||
            !canonical(mask,static_cast<Field>(11+i),p.mag_milligauss[i])) return false;
    }
    if (!canonical(mask,Field::latitude,p.latitude_nanodeg) || !canonical(mask,Field::longitude,p.longitude_nanodeg) ||
        !canonical(mask,Field::gnss_altitude,p.gnss_altitude_mm) || !canonical(mask,Field::gnss_satellites,p.gnss_satellites) ||
        !canonical(mask,Field::solar_1,p.solar_mv[0]) || !canonical(mask,Field::solar_2,p.solar_mv[1])) return false;
    if (((mask & bit(Field::latitude)) != 0) != ((mask & bit(Field::longitude)) != 0)) return false;
    if (p.latitude_nanodeg < -90000000000LL || p.latitude_nanodeg > 90000000000LL ||
        p.longitude_nanodeg < -180000000000LL || p.longitude_nanodeg > 180000000000LL) return false;
    return (mask & bit(Field::gnss_time)) ? p.gnss_time.size != 0 : p.gnss_time.size == 0;
}
void writeCommon(Writer& w, const CommonTelemetry& c) {
    w.put(c.mission_time_ms,8); w.put(c.packet_count,4); w.put(c.command_count,4);
    w.put(static_cast<unsigned>(c.mode),1); w.put(c.valid,4);
    w.put(static_cast<std::uint32_t>(c.altitude_mm),4); w.put(c.pressure_pa,4);
    w.put(static_cast<std::uint32_t>(c.temperature_milli_c),4); w.put(c.battery_mv,4);
    w.put(static_cast<std::uint32_t>(c.battery_current_ma),4);
    w.put(c.mechanism_state,1); w.put(c.mechanism_known,1); w.text(c.command_echo);
}
void readCommon(Reader& r, CommonTelemetry& c) {
    c.mission_time_ms = r.get(8); c.packet_count = static_cast<std::uint32_t>(r.get(4));
    c.command_count = static_cast<std::uint32_t>(r.get(4)); c.mode = static_cast<Mode>(r.get(1));
    c.valid = static_cast<std::uint32_t>(r.get(4)); c.altitude_mm = r.i32();
    c.pressure_pa = static_cast<std::uint32_t>(r.get(4)); c.temperature_milli_c = r.i32();
    c.battery_mv = static_cast<std::uint32_t>(r.get(4)); c.battery_current_ma = r.i32();
    c.mechanism_state = static_cast<std::uint8_t>(r.get(1)); c.mechanism_known = static_cast<std::uint8_t>(r.get(1));
    r.text(c.command_echo);
}
void writeExtension(Writer& w, const Extension& e) {
    w.put(e.profile,2); w.put(e.version,1); w.put(e.size,1);
    for (unsigned i = 0; i < e.size; ++i) w.put(e.data[i],1);
}
void readExtension(Reader& r, Extension& e) {
    e.profile = static_cast<std::uint16_t>(r.get(2)); e.version = static_cast<std::uint8_t>(r.get(1));
    e.size = static_cast<std::uint8_t>(r.get(1));
    if (e.size > extension_budget) { r.ok = false; return; }
    for (unsigned i = 0; i < e.size; ++i) e.data[i] = static_cast<std::uint8_t>(r.get(1));
}
}  // namespace
bool encodeContainer(const ContainerTelemetry& c, Bytes& output) {
    if (!valid(c)) return false;
    Writer w(Schema::container); writeCommon(w,c.common); w.put(static_cast<unsigned>(c.state),1);
    if (!w.ok || w.bytes.size != container_bytes) return false;
    output = w.bytes; return true;
}
bool decodeContainer(const std::uint8_t* data, std::size_t size, ContainerTelemetry& output) {
    if (size != container_bytes) return false;
    Reader r(data,size,Schema::container); ContainerTelemetry c;
    readCommon(r,c.common); c.state = static_cast<ContainerState>(r.get(1));
    if (!r.done() || !valid(c)) return false;
    output = c; return true;
}
bool encodePocket(const PocketTelemetry& p, Bytes& output) {
    if (!valid(p)) return false;
    Writer w(Schema::pocket); writeCommon(w,p.common);
    for (auto values : {p.gyro_mdeg_s,p.accel_mm_s2,p.mag_milligauss})
        for (auto value : values) w.put(static_cast<std::uint32_t>(value),4);
    w.put(static_cast<std::uint64_t>(p.latitude_nanodeg),8); w.put(static_cast<std::uint64_t>(p.longitude_nanodeg),8);
    w.put(static_cast<std::uint32_t>(p.gnss_altitude_mm),4); w.put(p.gnss_satellites,2); w.text(p.gnss_time);
    for (auto value : p.solar_mv) w.put(value,4);
    writeExtension(w,p.stabilization); writeExtension(w,p.science);
    if (!w.ok || w.bytes.size != pocket_base_bytes + p.stabilization.size + p.science.size) return false;
    output = w.bytes; return true;
}
bool decodePocket(const std::uint8_t* data, std::size_t size, PocketTelemetry& output) {
    if (size < pocket_base_bytes || size > max_payload_bytes) return false;
    Reader r(data,size,Schema::pocket); PocketTelemetry p; readCommon(r,p.common);
    for (auto* values : {&p.gyro_mdeg_s,&p.accel_mm_s2,&p.mag_milligauss}) for (auto& value : *values) value = r.i32();
    p.latitude_nanodeg = r.i64(); p.longitude_nanodeg = r.i64(); p.gnss_altitude_mm = r.i32();
    p.gnss_satellites = static_cast<std::uint16_t>(r.get(2)); r.text(p.gnss_time);
    for (auto& value : p.solar_mv) value = static_cast<std::uint32_t>(r.get(4));
    readExtension(r,p.stabilization); readExtension(r,p.science);
    if (!r.done() || !valid(p)) return false;
    output = p; return true;
}
bool encodeCommand(NodeId destination, const Command& c, Bytes& output) {
    if (destination != NodeId::container && destination != NodeId::pocketqube) return false;
    const bool container = destination == NodeId::container;
    Writer w(Schema::command);
    if (std::holds_alternative<Calibrate>(c)) { if (!container) return false; w.put(static_cast<unsigned>(Op::calibrate),1); }
    else if (std::holds_alternative<Arm>(c)) { if (!container) return false; w.put(static_cast<unsigned>(Op::arm),1); }
    else if (const auto* x = std::get_if<SetUtc>(&c)) {
        if (!container) return false;
        w.put(static_cast<unsigned>(Op::set_utc),1); w.put(x->unix_ms,8);
    } else if (const auto* x = std::get_if<Simulation>(&c)) {
        if (x->action != SimAction::enable && x->action != SimAction::activate && x->action != SimAction::disable) return false;
        w.put(static_cast<unsigned>(Op::simulation),1); w.put(static_cast<unsigned>(x->action),1);
    } else if (const auto* x = std::get_if<SimulatedPressure>(&c)) {
        if (!x->pa) return false;
        w.put(static_cast<unsigned>(Op::simulated_pressure),1); w.put(x->pa,4);
    } else if (const auto* x = std::get_if<Deploy>(&c)) {
        if (container ? x->mechanism != Mechanism::container_release :
            (x->mechanism != Mechanism::solar_1 && x->mechanism != Mechanism::solar_2 && x->mechanism != Mechanism::boom)) return false;
        w.put(static_cast<unsigned>(Op::deploy),1); w.put(static_cast<unsigned>(x->mechanism),1);
    } else if (const auto* x = std::get_if<Recording>(&c)) {
        if (container || (x->action != RecordAction::start && x->action != RecordAction::stop)) return false;
        w.put(static_cast<unsigned>(Op::recording),1); w.put(static_cast<unsigned>(x->action),1);
    } else if (const auto* x = std::get_if<RotateView>(&c)) {
        // Team-defined single-turn wire bound, not a measured mechanical travel limit.
        if (container || x->delta_mdeg == 0 || x->delta_mdeg < -360000 || x->delta_mdeg > 360000) return false;
        w.put(static_cast<unsigned>(Op::rotate_view),1); w.put(static_cast<std::uint32_t>(x->delta_mdeg),4);
    } else return false;
    if (!w.ok) return false;
    output = w.bytes; return true;
}
bool decodeCommand(NodeId destination, const std::uint8_t* data, std::size_t size, Command& output) {
    Reader r(data,size,Schema::command); Command c;
    switch (static_cast<Op>(r.get(1))) {
    case Op::calibrate: c = Calibrate{}; break;
    case Op::arm: c = Arm{}; break;
    case Op::set_utc: c = SetUtc{r.get(8)}; break;
    case Op::simulation: c = Simulation{static_cast<SimAction>(r.get(1))}; break;
    case Op::simulated_pressure: c = SimulatedPressure{static_cast<std::uint32_t>(r.get(4))}; break;
    case Op::deploy: c = Deploy{static_cast<Mechanism>(r.get(1))}; break;
    case Op::recording: c = Recording{static_cast<RecordAction>(r.get(1))}; break;
    case Op::rotate_view: c = RotateView{r.i32()}; break;
    default: return false;
    }
    Bytes canonical;
    if (!r.done() || !encodeCommand(destination,c,canonical)) return false;
    output = c; return true;
}
bool commandEcho(NodeId destination, const Command& c, Text<24>& output) {
    Bytes encoded;
    if (!encodeCommand(destination,c,encoded)) return false;
    char text[25]{};
    int size = -1;
    if (std::holds_alternative<Calibrate>(c)) size = std::snprintf(text,sizeof text,"CAL");
    else if (std::holds_alternative<Arm>(c)) size = std::snprintf(text,sizeof text,"ARM");
    else if (const auto* x = std::get_if<SetUtc>(&c)) size = std::snprintf(text,sizeof text,"ST%llu",static_cast<unsigned long long>(x->unix_ms));
    else if (const auto* x = std::get_if<Simulation>(&c)) size = std::snprintf(text,sizeof text,"SIM%s",x->action == SimAction::enable ? "ENABLE" : x->action == SimAction::activate ? "ACTIVATE" : "DISABLE");
    else if (const auto* x = std::get_if<SimulatedPressure>(&c)) size = std::snprintf(text,sizeof text,"SIMP%lu",static_cast<unsigned long>(x->pa));
    else if (const auto* x = std::get_if<Deploy>(&c)) {
        const char* name = x->mechanism == Mechanism::container_release ? "RELEASE" : x->mechanism == Mechanism::solar_1 ? "SOLAR1" : x->mechanism == Mechanism::solar_2 ? "SOLAR2" : "BOOM";
        size = std::snprintf(text,sizeof text,"MEC_%s",name);
    } else if (const auto* x = std::get_if<Recording>(&c)) size = std::snprintf(text,sizeof text,"RECORD%s",x->action == RecordAction::start ? "ON" : "OFF");
    else if (const auto* x = std::get_if<RotateView>(&c)) size = std::snprintf(text,sizeof text,"VIEW%ld",static_cast<long>(x->delta_mdeg));
    return size >= 0 && size <= 24 && setText(output,std::string_view(text,static_cast<std::size_t>(size)));
}
}  // namespace cansat::com::payload
