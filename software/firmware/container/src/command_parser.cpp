#include "command_parser.hpp"

#include <string.h>

namespace {

struct OpName {
  const char* name;
  CommandOp op;
};

constexpr OpName kOps[] = {
    {"CX", CommandOp::CX},     {"ST", CommandOp::ST},   {"SIM", CommandOp::SIM},
    {"SIMP", CommandOp::SIMP}, {"CAL", CommandOp::CAL}, {"ARM", CommandOp::ARM},
    {"MEC", CommandOp::MEC},   {"RST", CommandOp::RST},
};

bool isTrailingJunk(char c) { return c == '\r' || c == '\n' || c == ' ' || c == '\t'; }

// Copies [begin, end) into dst as a NUL-terminated string. Fails if it
// doesn't fit, rather than silently truncating a command argument.
bool copyField(const char* begin, const char* end, char* dst, size_t size) {
  const size_t n = static_cast<size_t>(end - begin);
  if (n >= size) return false;
  memcpy(dst, begin, n);
  dst[n] = '\0';
  return true;
}

}  // namespace

ParseResult parseCommand(const char* text, size_t len, uint16_t team_id,
                         ParsedCommand& out) {
  out = ParsedCommand{};
  if (text == nullptr) return ParseResult::NotACommand;

  // Ignore trailing CR/LF/spaces from serial terminals.
  while (len > 0 && isTrailingJunk(text[len - 1])) --len;
  const char* p = text;
  const char* end = text + len;

  if (len < 4 || memcmp(p, "CMD,", 4) != 0) return ParseResult::NotACommand;
  p += 4;

  // Team number field.
  const char* team_end = static_cast<const char*>(memchr(p, ',', static_cast<size_t>(end - p)));
  if (team_end == nullptr || team_end == p) return ParseResult::Malformed;
  char team_text[8];
  uint32_t team = 0;
  if (!copyField(p, team_end, team_text, sizeof(team_text)) || !parseUint32(team_text, team)) {
    return ParseResult::Malformed;
  }
  if (team != team_id) return ParseResult::WrongTeam;
  p = team_end + 1;

  // Op field, then everything after the next comma is the value.
  const char* op_end = static_cast<const char*>(memchr(p, ',', static_cast<size_t>(end - p)));
  const char* value_begin = end;
  if (op_end == nullptr) {
    op_end = end;
  } else {
    value_begin = op_end + 1;
  }
  if (op_end == p) return ParseResult::Malformed;
  if (!copyField(p, op_end, out.op_text, sizeof(out.op_text))) return ParseResult::Malformed;
  if (!copyField(value_begin, end, out.value, sizeof(out.value))) return ParseResult::Malformed;

  for (const OpName& entry : kOps) {
    if (strcmp(out.op_text, entry.name) == 0) {
      out.op = entry.op;
      break;
    }
  }
  return ParseResult::Ok;
}

void buildCommandEcho(const ParsedCommand& cmd, char* out, size_t size) {
  if (out == nullptr || size == 0) return;
  size_t n = 0;
  const char* const parts[] = {cmd.op_text, cmd.value};
  for (const char* src : parts) {
    for (; *src != '\0' && n + 1 < size; ++src) {
      // Commas would break the CSV line; control characters would break the
      // ground station's line framing.
      if (*src == ',' || static_cast<unsigned char>(*src) < 0x20) continue;
      out[n++] = *src;
    }
  }
  out[n] = '\0';
}

bool parseUint32(const char* text, uint32_t& out) {
  if (text == nullptr || *text == '\0') return false;
  uint64_t value = 0;
  for (const char* c = text; *c != '\0'; ++c) {
    if (*c < '0' || *c > '9') return false;
    value = value * 10u + static_cast<uint64_t>(*c - '0');
    if (value > 0xFFFFFFFFull) return false;
  }
  out = static_cast<uint32_t>(value);
  return true;
}

bool parseTimeOfDay(const char* text, uint32_t& seconds_out) {
  // Exactly "hh:mm:ss".
  if (text == nullptr || strlen(text) != 8 || text[2] != ':' || text[5] != ':') return false;
  char part[3] = {0, 0, 0};
  uint32_t fields[3];
  for (int i = 0; i < 3; ++i) {
    part[0] = text[i * 3];
    part[1] = text[i * 3 + 1];
    if (!parseUint32(part, fields[i])) return false;
  }
  if (fields[0] > 23 || fields[1] > 59 || fields[2] > 59) return false;
  seconds_out = fields[0] * 3600u + fields[1] * 60u + fields[2];
  return true;
}
