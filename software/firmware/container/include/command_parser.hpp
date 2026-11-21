#pragma once

// Parsing of ground-station commands. Plain C++ (no Arduino) so the host
// tests can check it.
//
// Wire format (kept from the 2026 ground-station GUI, gui-new/code/command):
//
//   CMD,<team id>,<OP>[,<value>]
//
// Examples:
//   CMD,1000,CX,ON          telemetry on        (CX,OFF turns it off)
//   CMD,1000,ST,13:45:07    set UTC time hh:mm:ss (F3)
//   CMD,1000,SIM,ENABLE     simulation: ENABLE, then ACTIVATE; or DISABLE (F6)
//   CMD,1000,SIMP,101325    simulated pressure in Pa, sent at 1 Hz (F4, F5)
//   CMD,1000,CAL            zero the altitude at the current pressure
//   CMD,1000,ARM            enable launch detection (IDLE -> LAUNCH_PAD)
//   CMD,1000,MEC,REL:1      manual mechanism control (REL = release the PQ)
//   CMD,1000,RST            start a new mission record (pad only, see main.cpp)
//
// Commands for another team number are ignored (X5). Trailing CR/LF and
// spaces are tolerated, since the same parser also reads the USB serial port.

#include <stddef.h>
#include <stdint.h>

enum class CommandOp : uint8_t {
  Unknown,  // Well-formed, our team, but an op we don't support.
  CX,
  ST,
  SIM,
  SIMP,
  CAL,
  ARM,
  MEC,
  RST
};

enum class ParseResult : uint8_t {
  Ok,           // out is filled in (op may still be Unknown).
  NotACommand,  // Doesn't start with "CMD,".
  WrongTeam,    // Addressed to another team: ignore silently.
  Malformed     // Missing fields, bad team number, or too long.
};

struct ParsedCommand {
  CommandOp op = CommandOp::Unknown;
  char op_text[8] = "";   // The op exactly as received, e.g. "SIMP".
  char value[40]  = "";   // Everything after the op's comma, may be empty.
};

// Parses one command line. text does not need to be NUL-terminated.
ParseResult parseCommand(const char* text, size_t len, uint16_t team_id,
                         ParsedCommand& out);

// Builds the CMD_ECHO telemetry text: op and value joined with nothing in
// between and with commas removed, because a comma would add a CSV column.
// e.g. ("CX", "ON") -> "CXON", ("SIMP", "101325") -> "SIMP101325".
void buildCommandEcho(const ParsedCommand& cmd, char* out, size_t size);

// Parses a whole decimal number with no sign or junk, e.g. a SIMP value.
bool parseUint32(const char* text, uint32_t& out);

// Parses "hh:mm:ss" (24 h) into seconds since midnight.
bool parseTimeOfDay(const char* text, uint32_t& seconds_out);
