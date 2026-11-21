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
| `software/firmware_lib/inc/protocol.hpp` | Packed binary `ContainerPacket` / `PQPacket` structs sent over ESP-NOW. |
| `software/firmware_lib/inc/espnow_link.hpp` | ESP-NOW transport API (**`src/espnow_link.cpp` is still empty**). |
| `software/firmware_lib/inc/drivers/` | Sensor drivers: `bmp581` (barometer), `ina236` (voltage/current). |
| `software/common/src/telemetry.cpp` | Incomplete serializer; **not part of any build**. |
| `software/tests/test_boot_skeleton.py` | Host-only checks (C++17 compiler + Python). |
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
python -m unittest discover -s software/tests -p 'test_boot_skeleton.py' -v
```

- `firmware_lib` is shared: after changing it, build **all three** envs.
- **Never upload/flash** unless the user explicitly asks and names the port.
  Uploading replaces the board's firmware.
- The GUI has its own tests: `python -m unittest discover -s tests` from
  `ground_station/gui-new` (see `ground_station/README.md` for venv setup).

### Constraint from the host tests

`test_boot_skeleton.py` asserts that each role's `src/main.cpp` does **not** call
`Wire.`/`SPI.`/`WiFi.`/`EspNowLink::`, `pinMode`, `xTaskCreate`, or use `while (`,
and it compiles `main.cpp` with only `firmware_boot.cpp`. The `main.cpp` files are
deliberately still boot-only. Wiring real logic into `main.cpp` **requires updating
that test in the same change**. Say so in your plan.

## Architecture

- **Topology (X3–X6):** container ⇄ ground station, PQ ⇄ ground station. Flight nodes
  never talk to each other. Peer with the GS MAC explicitly, filter received packets by
  MAC, **never use the broadcast address**. ESP-NOW payload max is 250 bytes. Team
  channel is assigned by the competition (X2).
- **Wire format** is team-defined (binary is fine). The **ground station** converts it to
  the CSV formats in the guide (GS13). Keep `protocol.hpp` in sync on both ends.
- **ESP-NOW receive callbacks run in the Wi-Fi task**: copy bytes, set a flag, return.
  Never block or do I2C inside them.

### Container firmware (most developed role)

- `SensorManager` (`container/include/sensor_manager.hpp`) owns the BMP581 and the
  INA236. It produces a `SensorReadings` snapshot: smoothed altitude above the pad,
  max altitude, vertical speed, pressure (Pa), temperature, battery V/mA, and health
  flags. It handles simulation pressure (SIMP) and saves the CAL ground reference to
  NVS (`Preferences`, namespace `sensors`).
- `MissionManager` (`container/include/mission_manager.hpp`) owns the flight state
  machine. `updateState(readings)` advances `LAUNCH_PAD → ASCENT → APOGEE →
  PQ_RELEASE`, forward only, with each step confirmed over `kStateConfirmSamples`
  samples.
- `config.hpp` holds pins, addresses and thresholds. Unconfirmed values are marked.
- Neither class is called from `main.cpp` yet.

## Requirements that drive software (keep these in mind)

**Container telemetry (CTR14/15/16, CSV order on the ground):**
`ID(team#+C), MISSION_TIME (s, 1 ms res), PACKET_COUNT, COMMAND_COUNT, MODE (F/S),
ALTITUDE (0.1 m), PRESSURE (1 Pa), TEMPERATURE (0.1 C), BATT_V, BATT_I (1 mA),
MECH_STATE (hex: bit0 mechanism activated, bit1 PQ released), STATE, CMD_ECHO`
(no commas in any field). CTR21 asks for battery voltage to 0.01 V.

**STATE values the judges expect:** `LAUNCH_PAD`, `ASCENT`, `APOGEE`, `PQ_RELEASE`.
Our firmware also has an internal `IDLE` (not armed). Decide how it is reported.
`protocol.hpp`'s `OpState` has no `IDLE`.

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

1. **Release logic does not match C3 yet.** `MissionManager` releases below a fixed
   `kReleaseAltitudeM` (400 m placeholder); the guide requires release at **90 % of
   peak altitude** (`altitude <= 0.9 * max_altitude` during descent).
2. Flight state, packet count and mission time are not yet saved across resets (F1,
   F2, F8). Only the CAL ground reference is saved.
3. `ARM` command handling, CTR16 rate switching (4 Hz → 1 Hz 5 s after release) and
   landing detection (stop transmitting) are not implemented.
4. `FlightState`/`OperatingMode` (container) duplicate `OpState`/`Mode`
   (`protocol.hpp`). Merge them before telemetry uses both.
5. Container I2C pins and the INA236 address are unknown until the container board is
   drawn (`electrical/boards/container/` is empty). The INA236 driver is untested.
6. `espnow_link.cpp` is empty. The PQ and ground roles are boot-only skeletons.

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
