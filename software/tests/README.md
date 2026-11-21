# Software tests

Three levels, from fastest to most realistic. Run everything from the
repository root. Nothing here flashes a board.

| Level | What | Needs |
|---|---|---|
| 1. Host tests | Mission logic, telemetry format, command parser, boot banners | Python 3 + a C++17 compiler (`c++`) |
| 2. Firmware build | All three ESP32 roles compile | PlatformIO |
| 3. Simulation flight | A real container board flies a simulated mission over USB | Board + `pip install pyserial` |

## 1. Host tests (run these before every commit)

```sh
python -m unittest discover -s software/tests -p 'test_*.py' -v
```

- `test_boot_skeleton.py` checks the PlatformIO setup and the boot-only
  PQ and ground-station skeletons.
- `test_container_logic.py` compiles the container's pure-logic sources
  (`mission_manager.cpp`, `container_telemetry.cpp`, `command_parser.cpp`)
  together with `container_logic_harness.cpp` and runs each case. It covers:
  - a full synthetic flight: arm, launch, apogee, release at 90 % of peak
    (C3), landing;
  - debounce (one noisy sample can't change state) and restore after a reset;
  - the two-step SIM ENABLE / SIM ACTIVATE (F6);
  - the exact CSV line and column count, the 4 Hz / 1 Hz / stop schedule
    (CTR15, CTR16), and that the longest line fits in 250 bytes;
  - command parsing, wrong-team rejection (X5) and CMD_ECHO without commas.

To add a case, write a function in `container_logic_harness.cpp` and add
it to the `cases()` table. The Python side picks it up automatically.

Sensor drivers, flash storage and the radio need real hardware, so they
are covered by level 3, not here.

## 2. Firmware build

```sh
pio run -e ground_station -e pq -e container
```

## 3. Simulation flight on a real board

This is the same procedure the judges use if launch is weathered out
(F4, F5, F6), driven from a script over USB instead of the ground station.

1. Flash the container build to a labelled board (only when you mean to:
   `pio run -e container -t upload --upload-port PORT`).
2. Check the plan without a board:
   `python software/tests/container_sim_flight.py --dry-run`
3. Run it:
   `python software/tests/container_sim_flight.py --port PORT`
   Add `--profile FILE` to use a pressure profile (one Pa value per line,
   e.g. the judges' file). Add `--team N` if `kTeamId` isn't 1000.
4. The script sends `RST`, `SIM ENABLE`, `SIM ACTIVATE`, one `SIMP`, `CAL`,
   `ARM`, then one `SIMP` per second. It prints the board's `# ...` debug
   lines and finally `PASS` or a list of failed checks.
5. The received telemetry is saved as `container_sim_flight.csv`, in the
   same format as the judges' file, for review or for the PFR.

What it checks: states in order, release at <= 90 % of peak, MECH_STATE
bits, MODE 'S', ~4 Hz before and ~1 Hz after release, telemetry stops after
landing, packet count always increasing.

### Manual checks with a serial monitor

`pio device monitor --port PORT --baud 115200`, then type commands, e.g.
`CMD,1000,CAL` then `CMD,1000,ARM`. Telemetry lines start with `1000C,`,
debug lines with `# `. Useful for checks the script can't do:

- **Reset recovery (F1, F2, F8):** in the middle of a simulation run, press
  EN/reset. After reboot the `# resumed mission record` line should show
  the same state; packet count continues (it jumps by up to 8), and mission
  time continues (it loses about 1 s).
- **Vacuum chamber test:** in flight mode, pumping down should raise
  ALTITUDE and step through the states; venting should trigger PQ_RELEASE.
- **Radio:** with the ground-station MAC and channel set in `config.hpp`,
  the boot output must not show `radio not up`.
