#include "flight_store.hpp"

#include <Preferences.h>

#include "config.hpp"

using namespace FirmwareConfig;

namespace {

constexpr const char* kNvsNamespace = "flight";
constexpr const char* kNvsKey       = "record";

// Stored next to the record. Bump it whenever FlightRecord changes layout,
// so an old blob from a previous firmware is discarded instead of being
// misread.
constexpr uint8_t kRecordVersion = 1;

struct StoredBlob {
  uint8_t      version;
  FlightRecord record;
};

}  // namespace

bool FlightStore::begin(uint32_t now_ms) {
  rec_ = FlightRecord{};
  base_ms_ = 0;
  epoch_ms_ = now_ms;
  last_save_ms_ = now_ms;

  Preferences prefs;
  if (!prefs.begin(kNvsNamespace, /*readOnly=*/true)) return false;
  StoredBlob blob{};
  const size_t got = prefs.getBytes(kNvsKey, &blob, sizeof(blob));
  prefs.end();
  if (got != sizeof(blob) || blob.version != kRecordVersion) return false;

  rec_ = blob.record;
  // Continue mission time from the last save; time between that save and
  // the reset is lost (see header).
  base_ms_ = rec_.mission_time_ms;
  // Packets sent after the last save aren't on record, so skip ahead
  // rather than risk sending a packet number twice.
  rec_.packet_count += kPacketCountResetMargin;
  return true;
}

uint32_t FlightStore::missionTimeMs(uint32_t now_ms) const {
  return base_ms_ + (now_ms - epoch_ms_);
}

bool FlightStore::utcSecondsOfDay(uint32_t now_ms, uint32_t& out) const {
  if (!rec_.utc_set) return false;
  const uint32_t elapsed_s = (missionTimeMs(now_ms) - rec_.mission_ms_at_utc_set) / 1000u;
  out = (rec_.utc_seconds_at_set + elapsed_s) % 86400u;
  return true;
}

void FlightStore::setUtc(uint32_t now_ms, uint32_t seconds_of_day) {
  rec_.utc_set = true;
  rec_.utc_seconds_at_set = seconds_of_day;
  rec_.mission_ms_at_utc_set = missionTimeMs(now_ms);
}

void FlightStore::saveNow(uint32_t now_ms) {
  rec_.mission_time_ms = missionTimeMs(now_ms);
  StoredBlob blob{kRecordVersion, rec_};
  Preferences prefs;
  if (prefs.begin(kNvsNamespace, /*readOnly=*/false)) {
    // NVS spreads writes across its flash pages (wear levelling), so one
    // small write per second for a few hours of testing and flight is fine.
    prefs.putBytes(kNvsKey, &blob, sizeof(blob));
    prefs.end();
  }
  last_save_ms_ = now_ms;
}

void FlightStore::saveIfDue(uint32_t now_ms) {
  if (now_ms - last_save_ms_ >= kPersistPeriodMs) saveNow(now_ms);
}

void FlightStore::clear(uint32_t now_ms) {
  rec_ = FlightRecord{};
  base_ms_ = 0;
  epoch_ms_ = now_ms;
  saveNow(now_ms);
}
