# CLAUDE.md

Requirement IDs (C3, CTR16, F8, …) refer to the *CanSat Competition Guide 2027,
Rev 1.2e*.

## Mission in one paragraph

Two CanSats ride in one rocket (Aerotech J425, ~10 s delay to ejection). Each CanSat
is a **container** (Ø131 mm × 190 mm) holding a 2P **PocketQube (PQ)**. The judge
powers the container on; it transmits telemetry at **4 Hz**. At apogee the rocket
ejects both containers, which descend at **10 ± 3 m/s**. When the container descends
to **90 % of peak altitude** it releases the PQ (C3). **5 s after release** the
container drops to **1 Hz** telemetry and **stops transmitting when it lands**
(CTR16). The PQ is unpowered while stowed (limit switches), powers on at release,
transmits at 4 Hz, deploys two solar panels after 5 s, descends at 3–5 m/s, records
nadir video stabilised to ±10°, and on command deploys a 220 mm magnetometer boom and
rotates the camera view. Radio is **ESP-NOW** only (backup XBee optional).

## Repository layout

| Path | What it is |
|---|---|
| `platformio.ini` | **Single root PlatformIO project.** Envs: `ground_station` (default), `pq`, `container`. |
| `software/firmware/{container,PQ,ground}/` | Per-role `include/` + `src/`. Each env compiles only its own `src/`. |
| `software/firmware_lib/` | Shared library (symlinked into every env): `inc/` headers, `src/` sources, `library.json` deps. |
| `software/firmware_lib/inc/protocol.hpp` | Binary `PQPacket` (and the retired `ContainerPacket`; the container now sends ASCII CSV). |
| `software/firmware_lib/inc/espnow_link.hpp` | ESP-NOW transport (implemented in `src/espnow_link.cpp`). |
| `software/firmware_lib/inc/drivers/` | Sensor drivers: `bmp581` (barometer), `ina236` (voltage/current). |
| `software/common/src/telemetry.cpp` | Incomplete serializer; **not part of any build**. |
| `software/tests/` | Host tests, container logic harness, hardware simulation-flight script. See `software/tests/README.md`. |
| `software/Initial_testing/` | Old experiments. Not built. Leave alone. |
| `ground_station/gui-2026/` | **Preserved** last-year GUI. Do not modify. |
| `ground_station/gui-new/` | GUI under active development (Python 3.13, PyQt6 + pyqtgraph). |
| `ground_station/gs_radio_firmware/` | Old radio-bridge prototype. Not in any build. Reference only. |
| `electrical/` | KiCad 10 boards, blocks, `test/` bench sketches (BMP581, LSM6DSO, ADC, ESP-NOW range). |

## Build and test

Run from the repository root. On Windows, `pio` lives at
`~/.platformio/penv/Scripts/pio.exe` if it is not on `PATH`.

```sh
pio run -e container            # build one role
pio run -e ground_station -e pq -e container   # build all three
python -m unittest discover -s software/tests -p 'test_*.py' -v
```

- `firmware_lib` is shared: after changing it, build **all three** envs.
- **Never upload/flash** unless the user explicitly asks and names the port.
  Uploading replaces the board's firmware.
- The GUI has its own tests: `python -m unittest discover -s tests` from
  `ground_station/gui-new` (see `ground_station/README.md` for venv setup).

### Constraints from the host tests

- `test_boot_skeleton.py` asserts that the **PQ and ground-station** `main.cpp` files
  stay boot-only: no `Wire.`/`SPI.`/`WiFi.`/`EspNowLink::`, `pinMode`, `xTaskCreate`
  or `while (`. Each is compiled with only `firmware_boot.cpp`. Wiring real logic into
  either one **requires moving that role out of `BOOT_ONLY_ROLES` in the same change**,
  as was done for the container.
- `test_container_logic.py` compiles `mission_manager.cpp`, `container_telemetry.cpp`
  and `command_parser.cpp` on the PC with `-Wall -Wextra -Werror`. Keep those files
  (and the headers they include, including `config.hpp`) free of Arduino headers.
  Hardware code goes in other modules.

## Architecture

- **Topology (X3–X6):** container ⇄ ground station, PQ ⇄ ground station. Flight nodes
  never talk to each other. Peer with the GS MAC explicitly, filter received packets by
  MAC, **never use the broadcast address**. ESP-NOW payload max is 250 bytes. Team
  channel is assigned by the competition (X2).
- **Wire format** is team-defined. The **container sends ASCII CSV lines** (exactly the
  judges' column format, no terminator, one line per ESP-NOW packet). The ground radio
  bridge frames packets as `
`-terminated text, and a binary packet can contain `
`.
  The PQ format is still to be decided (`PQPacket` in `protocol.hpp` is binary).
- **Commands** use the existing GUI format `CMD,<team>,<OP>[,<value>]`, as ASCII over
  ESP-NOW. The container also accepts them on USB serial, for bench testing.
- **ESP-NOW receive callbacks run in the Wi-Fi task**: copy bytes, set a flag, return.
  Never block or do I2C inside them.

### Container firmware (most developed role)

`container/src/main.cpp` is a thin, non-blocking loop: commands → sensors → state →
persist → telemetry. The modules (headers in `container/include/`):

| Module | Role | Host-testable |
|---|---|---|
| `sensor_manager` | BMP581 + INA236 → `SensorReadings` (`sensor_readings.hpp`): smoothed altitude, max, vertical speed, Pa, °C, V, mA. SIMP substitution. Saves the CAL reference (NVS `sensors`). | no |
| `mission_manager` | `IDLE →(ARM) LAUNCH_PAD → ASCENT → APOGEE → PQ_RELEASE`, release at 90 % of peak (C3), landing flag, SIM ENABLE/ACTIVATE (F6). | yes |
| `container_telemetry` | CSV line formatting, and the rate schedule: 4 Hz, 1 Hz from 5 s after release, off after landing. | yes |
| `command_parser` | Parses `CMD,…` lines and builds CMD_ECHO. | yes |
| `flight_store` | State, packet count, mission time, max altitude, mech state and UTC in NVS (`flight`). Saved on events and once a second. `RST` clears it. | no |
| `release_mechanism` | MECH_STATE bits. Actuator and sensors are **stubs** until the hardware exists. | no |

`IDLE` is internal ("not armed") and is reported as `LAUNCH_PAD` in telemetry. Safety
rules in `main.cpp`: `CAL` only on the pad; `SIM` can't be turned on after launch;
`RST` only on the pad, after landing, or during simulation.

## Requirements that drive software (keep these in mind)

**Container telemetry (CTR14/15/16, CSV order on the ground):**
`ID(team#+C), MISSION_TIME (s, 1 ms res), PACKET_COUNT, COMMAND_COUNT, MODE (F/S),
ALTITUDE (0.1 m), PRESSURE (1 Pa), TEMPERATURE (0.1 C), BATT_V, BATT_I (1 mA),
MECH_STATE (hex: bit0 mechanism activated, bit1 PQ released), STATE, CMD_ECHO`
(no commas in any field). CTR21 asks for battery voltage to 0.01 V.

**STATE values the judges expect:** `LAUNCH_PAD`, `ASCENT`, `APOGEE`, `PQ_RELEASE`.
Our internal `IDLE` (not armed) is reported as `LAUNCH_PAD`.

**Minimum commands (CTR28):** `SIM` (ENABLE then ACTIVATE, both required before
simulation mode, F6), `SIMP` (pressure value, sent at 1 Hz), `CAL` (zero altitude),
`ARM` (enable launch detection, so this is what moves `IDLE → LAUNCH_PAD`), `MEC`
(manual mechanism control). Plus setting the time to UTC within 1 s (F3).
`CMD_ECHO` example strings: `CXON`, `SP101325`.

**Flight software (F1–F8):** packet count and mission time must **survive processor
resets** (F1, F2). Zero-altitude calibration and other configuration state must
survive resets (F8). In simulation mode only pressure/altitude are replaced; every
other field stays a real reading.

**Hardware facts with software impact:** the container audio beacon and the PQ beacon
must **not** be software controlled (CTR26, PQ36). The container needs sensors for
"release mechanism fully deployed" (CTR11) and "PQ cleared the rail" (CTR12). The
container records video of the release (CTR13). Batteries: no LiPo (E1).

**PQ telemetry/sensors** are in guide pages 25–26 (IMU, mag in mGauss, GNSS, solar
voltages to 0.01 V, MECH_STATE bits 0–3, IMAGE_STABILIZATION, SCIENCE_EXP).
`PQPacket` in `protocol.hpp` mirrors it.

## Known gaps / decisions pending

Search the code for `**CHECK:**`. Each one marks something that must be verified
before flight. The most important ones:

1. **Release hardware:** `release_mechanism.cpp` drives nothing yet, and while no
   CTR12 sensor is fitted, "released" is assumed to follow "activated".
2. **Placeholders in `config.hpp`:** team ID, ESP-NOW channel and ground-station MAC
   (until the MAC is set, telemetry goes to USB serial only), I2C pins, INA236 address.
   The container board isn't drawn yet. The INA236 driver is untested on hardware.
3. **Mission time across resets** loses up to ~1 s until the RTC is added (planned).
   Mission time counts from the last `RST`, so `RST` belongs in the launch checklist.
4. **Thresholds** (launch, apogee, landing) are first guesses. Tune them with the
   judges' simulation profile (`software/tests/container_sim_flight.py`).
5. **MEC** only implements `REL…`. Agree the full format with the GUI.
6. The PQ and ground roles are still boot-only skeletons. Nothing has been run on a
   board yet.

## Working conventions

- **Plan first.** For non-trivial changes, explain the plan and list the files you will
  touch, then wait for approval. Don't commit, push or stage unless asked.
- **Comment the why**, and use units in names (`altitude_m`, `pressure_pa`,
  `battery_ma`). Internally use **Pa**, not hPa (the Adafruit BMP5xx library reports
  hPa; `Bmp581` converts it).
- Mark anything that needs hardware or mission confirmation with a `**CHECK:**`
  comment, so the team can find it with one search.
- **Never block in flight code.** No `while` waits for sensors or serial. A missing
  sensor reports through health flags and the code keeps running. Rate-limit
  with `millis()` deltas (unsigned subtraction handles wraparound).
- Put tunables and hardware assignments in the role's `config.hpp`
  (`FirmwareConfig::k…`), not as magic numbers.
- Flash writes (NVS) only on rare events (CAL, state changes), never every loop.
- Keep `gui-2026/`, `gs_radio_firmware/` and `Initial_testing/` unchanged. Electrical:
  KiCad 10 only; never modify existing parts in `electrical/lib/`.

## Key dates (Appendix H)

PDR due **2027-01-28**, CDR due **2027-04-01**, environmental test doc due
**2027-05-21**, FRR **2027-06-11**, launch **2027-06-12**, PFR **2027-06-13**.
