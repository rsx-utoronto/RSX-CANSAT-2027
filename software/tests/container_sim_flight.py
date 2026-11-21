"""Hardware-in-the-loop simulation flight for the container (needs a board).

Drives a container ESP32 over its USB serial port exactly like the ground
station would in simulation mode (F4, F5, F6), then checks the telemetry it
sends back. It never flashes the board.

What it does:
  1. RST            start a fresh mission record
  2. SIM ENABLE, SIM ACTIVATE
  3. first SIMP, then CAL (zero altitude at the profile's ground pressure)
  4. ARM
  5. one SIMP per second from the pressure profile
  6. checks the telemetry lines and saves them to a CSV file

Checks:
  * states appear in order LAUNCH_PAD -> ASCENT -> APOGEE -> PQ_RELEASE
  * PQ released at <= 90 % of peak altitude (C3), and not far below it
  * MECH_STATE shows the release, MODE is 'S'
  * telemetry at ~4 Hz before release, ~1 Hz from 5 s after it (CTR15/16)
  * telemetry stops after landing (CTR16)
  * packet count strictly increases (F1)

Usage (needs `pip install pyserial`):
  python software/tests/container_sim_flight.py --port COM5
  python software/tests/container_sim_flight.py --port COM5 --profile judges.csv
  python software/tests/container_sim_flight.py --dry-run     # no board: print the plan

The profile file is one pressure in Pa per line (a header line or extra CSV
columns are fine; the last number on each line is used). Without --profile a
synthetic flight is used: climb to 700 m, descend at 10 m/s, sit on the ground.
"""

import argparse
import csv
import math
import re
import sys
import time
from pathlib import Path

GROUND_PA = 101325.0
TELEMETRY_COLUMNS = ["ID", "MISSION_TIME", "PACKET_COUNT", "COMMAND_COUNT", "MODE",
                     "ALTITUDE", "PRESSURE", "TEMPERATURE", "BATT_V", "BATT_I",
                     "MECH_STATE", "STATE", "CMD_ECHO"]
STATE_ORDER = ["LAUNCH_PAD", "ASCENT", "APOGEE", "PQ_RELEASE"]


def altitude_to_pressure(alt_m, ground_pa=GROUND_PA):
    """Inverse of the barometric formula used in SensorManager."""
    return ground_pa * (1.0 - alt_m / 44330.0) ** 5.255


def synthetic_profile():
    """Pressures at 1 Hz: 5 s pad, ~12 s climb to 700 m, 10 m/s descent, 15 s still."""
    alts = [0.0] * 5
    alts += [700.0 * math.sin(math.pi / 2 * t / 12) for t in range(1, 13)]
    alts += [max(0.0, 700.0 - 10.0 * t) for t in range(1, 71)]
    alts += [0.0] * 15
    return [round(altitude_to_pressure(a)) for a in alts]


def load_profile(path):
    pressures = []
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        numbers = re.findall(r"\d+(?:\.\d+)?", line)
        if numbers:
            pressures.append(round(float(numbers[-1])))
    if not pressures:
        raise SystemExit(f"No pressures found in {path}")
    return pressures


def cmd(team, op, value=None):
    return f"CMD,{team},{op}" + (f",{value}" if value is not None else "")


def build_plan(team, pressures):
    """(delay_s, command) pairs. SIMP values are sent one second apart."""
    plan = [(0.5, cmd(team, "RST")), (0.5, cmd(team, "SIM", "ENABLE")),
            (0.5, cmd(team, "SIM", "ACTIVATE")), (1.0, cmd(team, "SIMP", pressures[0])),
            (1.0, cmd(team, "CAL")), (0.5, cmd(team, "ARM"))]
    plan += [(1.0, cmd(team, "SIMP", p)) for p in pressures]
    return plan


def parse_telemetry(line, team):
    if not line.startswith(f"{team}C,"):
        return None
    fields = line.split(",")
    if len(fields) != len(TELEMETRY_COLUMNS):
        return None
    return dict(zip(TELEMETRY_COLUMNS, fields))


def check_flight(rows, landing_quiet_s):
    """Returns a list of failure messages (empty = pass)."""
    failures = []
    if not rows:
        return ["no telemetry received"]

    # States in order (each may repeat, none may go backwards).
    seen = [STATE_ORDER.index(r["STATE"]) for r in rows if r["STATE"] in STATE_ORDER]
    if seen != sorted(seen):
        failures.append("STATE went backwards")
    missing = [s for i, s in enumerate(STATE_ORDER) if i not in seen]
    if missing:
        failures.append(f"states never reported: {missing}")

    alts = [float(r["ALTITUDE"]) for r in rows]
    peak = max(alts)
    release = next((r for r in rows if r["STATE"] == "PQ_RELEASE"), None)
    if release:
        rel_alt = float(release["ALTITUDE"])
        print(f"peak {peak:.1f} m, PQ_RELEASE first reported at {rel_alt:.1f} m "
              f"({100 * rel_alt / peak:.1f} % of peak)")
        if rel_alt > 0.9 * peak + 0.1:
            failures.append(f"released above 90 % of peak ({rel_alt} m)")
        if rel_alt < 0.8 * peak:
            failures.append(f"released far below 90 % of peak ({rel_alt} m)")
        if int(release["MECH_STATE"], 16) & 0x01 == 0:
            failures.append("MECH_STATE bit 0 not set at release")

    if any(r["MODE"] != "S" for r in rows):
        failures.append("MODE not 'S' during the simulation")

    counts = [int(r["PACKET_COUNT"]) for r in rows]
    if any(b <= a for a, b in zip(counts, counts[1:])):
        failures.append("PACKET_COUNT not strictly increasing")

    # Rates from mission time deltas.
    times = [float(r["MISSION_TIME"]) for r in rows]
    release_t = float(release["MISSION_TIME"]) if release else None
    fast = [b - a for a, b in zip(times, times[1:]) if release_t is None or b < release_t]
    slow = [b - a for a, b in zip(times, times[1:]) if release_t and a > release_t + 6]
    if fast:
        fast_avg = sum(fast) / len(fast)
        print(f"pre-release interval {fast_avg * 1000:.0f} ms")
        if not 0.2 <= fast_avg <= 0.3:
            failures.append(f"pre-release rate not ~4 Hz ({fast_avg:.3f} s)")
    if slow:
        slow_avg = sum(slow) / len(slow)
        print(f"post-release interval {slow_avg * 1000:.0f} ms")
        if not 0.9 <= slow_avg <= 1.1:
            failures.append(f"post-release rate not ~1 Hz ({slow_avg:.3f} s)")
    elif release:
        failures.append("no telemetry later than 6 s after release")

    if landing_quiet_s is not None and landing_quiet_s < 3.0:
        failures.append(f"telemetry did not stop after landing (quiet only {landing_quiet_s:.1f} s)")
    return failures


def run(args):
    import serial  # pyserial; imported here so --dry-run works without it

    pressures = load_profile(args.profile) if args.profile else synthetic_profile()
    plan = build_plan(args.team, pressures)

    port = serial.Serial()
    port.port = args.port
    port.baudrate = 115200
    port.timeout = 0.05
    # Opening the port with DTR/RTS asserted resets a DevKit; avoid that so
    # the test doesn't depend on boot timing.
    port.dtr = False
    port.rts = False
    port.open()

    rows = []
    recording = False  # Only telemetry after SIM ACTIVATE is checked.
    last_rx = time.monotonic()

    def pump(until):
        nonlocal last_rx
        while time.monotonic() < until:
            raw = port.readline()
            if not raw:
                continue
            line = raw.decode("ascii", errors="replace").strip()
            row = parse_telemetry(line, args.team)
            if row:
                last_rx = time.monotonic()
                if recording:
                    rows.append(row)
            elif line.startswith("#"):
                print(line)

    pump(time.monotonic() + 1.0)
    for delay, command in plan:
        port.write((command + "\n").encode("ascii"))
        if command.endswith(",SIM,ACTIVATE"):
            recording = True
        pump(time.monotonic() + delay)
    # Wait for landing detection and check the link goes quiet.
    pump(time.monotonic() + 15.0)
    quiet = time.monotonic() - last_rx
    port.close()

    out = Path(args.out)
    with out.open("w", newline="", encoding="ascii") as f:
        writer = csv.DictWriter(f, fieldnames=TELEMETRY_COLUMNS, lineterminator="\r")
        writer.writeheader()
        writer.writerows(rows)
    print(f"{len(rows)} telemetry lines saved to {out}")

    failures = check_flight(rows, quiet)
    for failure in failures:
        print("FAIL:", failure)
    print("PASS" if not failures else f"{len(failures)} check(s) failed")
    return 0 if not failures else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", help="container USB serial port, e.g. COM5 or /dev/ttyUSB0")
    parser.add_argument("--team", type=int, default=1000, help="team ID in config.hpp")
    parser.add_argument("--profile", help="pressure profile file (one Pa value per line)")
    parser.add_argument("--out", default="container_sim_flight.csv", help="telemetry CSV to write")
    parser.add_argument("--dry-run", action="store_true", help="print the command plan only")
    args = parser.parse_args()

    if args.dry_run:
        pressures = load_profile(args.profile) if args.profile else synthetic_profile()
        plan = build_plan(args.team, pressures)
        for delay, command in plan:
            print(f"{command}    (then wait {delay:.1f} s)")
        print(f"{len(plan)} commands, ~{sum(d for d, _ in plan):.0f} s")
        return 0
    if not args.port:
        parser.error("--port is required unless --dry-run")
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
