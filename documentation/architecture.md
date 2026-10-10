# CanSat 2027 software skeleton

Start with the [software specification](software-specification.md) for the complete
implemented contracts, API index, wire tables and explicit integration gates. This
guide retains design rationale, setup and detailed replay/detector instructions.

## Confirmed decisions

The Container, PocketQube and ground-radio bridge remain separate firmware
applications. ESP32-WROOM-32UE is the user's preferred module family for the
majority of controller nodes, using ESP32-DevKitC V4 development boards. Per-target
assignments remain unconfirmed. Platform-specific bus, GPIO,
clock, storage and ESP-NOW code must stay outside the portable libraries.

ESP-IDF is the selected firmware framework for all three embedded targets. No
bench hardware is available yet. Full module SKUs, pins and radio assignments
remain unset. ESP-IDF 6.1.0 (release tag `v6.1`) is pinned and installed locally;
the historical Arduino smoke test is not the new firmware baseline.

The 2027 ground station will evolve from the existing Python and PyQt6 application.
The 2026 copy is preserved. Fixed wide-angle camera software stabilization remains
an isolated experiment; its processor and camera are undecided.

## Implemented slices

The root CMake build is a **host test build**, not flight firmware. It contains:

- `cansat_core`: timestamped, optional sensor readings and an injected register bus.
- `cansat_drivers`: a read-only INA236 adapter with explicit address and shunt value.
- `cansat_communications`: bounded ASCII framing, binary radio envelope, NodeCom
  scheduling/retries and typed mission telemetry/command codecs; no runtime wiring.
- `cansat_mission`: independent Container and PocketQube state machines with
  caller-supplied monotonic time, configurable altitude-based event qualification,
  and deployment feedback.
- `sensor_diagnostics`: explicitly selected register fixtures, not hardware readings.
- Native and Python cross-language tests without Qt or a connected board.
- Offline CSV flight replay and explicit parameter-grid comparison, using the
  same compiled C++ detector rather than a separate Python detector.
- An ESP-IDF register-bus adapter and reusable component registrations; its
  error paths are tested with a private SDK substitute, not a physical bus.
- Separate inactive ESP-IDF applications for Container, PocketQube and the
  ground-radio bridge, using the DevKitC V4 `esp32` build profile. Their actual
  entry points run in host smoke tests and all three projects build with the
  pinned real SDK. Hardware execution is still unverified.

The Python framing implementation is in the new `cansat2027` package inside the
2027 GUI source. It is **not connected to the existing serial manager yet**. The
current GUI/bridge CR-versus-LF and destination-prefix mismatch remains in the
old runtime until the integration phase. The original repository files remain
unchanged; the new skeleton files evolve as decisions are confirmed.

Existing packet declarations and the empty shared telemetry implementation remain
unchanged. Do not treat the host serial envelope as a finalized RF packet format
or competition CSV schema. Consolidate old and new shared-code locations in a
separate reviewed migration, rather than maintaining competing implementations.

## Run the native checks

On this checkout, with CMake, a C++17 compiler, and Python 3.10 or newer:

```sh
cmake -S /Users/remiz/CLionProjects/RSX-CANSAT-2027 -B /private/tmp/rsx-cansat-native-20261010 -DCMAKE_BUILD_TYPE=Debug
cmake --build /private/tmp/rsx-cansat-native-20261010 --parallel 2
ctest --test-dir /private/tmp/rsx-cansat-native-20261010 --output-on-failure
/private/tmp/rsx-cansat-native-20261010/sensor_diagnostics --fixture
/private/tmp/rsx-cansat-native-20261010/mission_tests
/private/tmp/rsx-cansat-native-20261010/flight_detection_tests
```

The diagnostic output identifies `source=fixture`. Running without that explicit
option fails instead of silently substituting simulated data. The legacy GUI
requires its own Python environment and dependencies; these native tests do not
validate it.

## INA236 adapter scope

Register definitions and conversion factors follow the
[TI INA236 datasheet, SBOSA81D](https://www.ti.com/lit/ds/symlink/ina236.pdf?ts=1750631723231),
sections 7.5, 7.6 and 8.1. It probes manufacturer/device IDs, checks continuous
dual-channel conversion mode and conversion-ready status, and reads big-endian
bus/shunt registers. Current is derived from signed shunt voltage and the explicit
resistor value, not from the uninitialized current-calibration register.

This bring-up adapter does not configure the device, identify board wiring, verify
the resistor, or guarantee simultaneous voltage/current conversion timestamps.
The timestamp denotes acquisition start. Reading status acknowledges the device
conversion-ready flag; one owner must manage this instance and its status reads.
The platform adapter must enforce finite bus timeouts and reject short transfers.
Hardware accuracy, freshness under load, address wiring, and hot-plug recovery
still require bench tests. The 0x40 address and 20 mOhm shunt in fixtures are test
inputs, not a selected board configuration.

## Module boundaries

Sensor acquisition feeds validated samples into mission logic. Transport consumes
telemetry snapshots, not sensor drivers. Mission logic must not block on camera
work or radio acknowledgements. Actuator commands and physical deployment feedback
are distinct states. A command echo is not proof that a mechanism moved.

The vision benchmark contract records inputs and measurements only. No camera
pipeline, stabilization performance, hardware target, or organizer approval is
claimed. The flight applications and radio have an `esp32` skeleton build profile,
but no approved flight pin map, memory profile or active firmware tasks.

## ESP-IDF integration boundary

`software/lib/platform/boards.json` records `module_policy.preferred_family` as
`ESP32-WROOM-32UE` and the corresponding ESP-IDF target as `esp32`. This is a
majority-use preference, not a finalized assignment to every node. The preferred
carrier is `ESP32-DevKitC V4`. The configuration still says
`unconfigured_do_not_flash`: it does not choose a full memory SKU,
pin map, radio channel, antenna assembly or power circuit. The existing module
boundaries and current sensor/mission interfaces do not require a different SoC;
actual resource, bus and timing margins still need a real target build and bench test.

The [DevKitC V4 guide](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32/esp32-devkitc/user_guide.html)
lists multiple module variants, including WROOM-32UE. The board name therefore
does not resolve its module's flash/PSRAM suffix. The user's board clarification
does not imply that physical boards are already available for testing.

The [Espressif module datasheet](https://documentation.espressif.com/esp32-wroom-32e_esp32-wroom-32ue_datasheet_en.html)
lists 4/8/16 MB flash options; R2 variants have 2 MB PSRAM, while the listed
non-R2 variants do not. UE uses an external antenna connector. GPIO16 is reserved
by PSRAM on the R2 variants. Confirm the full suffix before allocating memory or pins.

Vision remains the possible exception, not a proven need for another chip.
[Espressif's camera driver notes](https://components.espressif.com/components/espressif/esp32-camera/versions/2.1.8/readme?language=en)
require PSRAM except for low-resolution JPEG cases and warn about RGB/YUV memory
traffic, especially with Wi-Fi. A PSRAM-equipped ESP32-S3 is a candidate to benchmark
if the preferred module misses the camera/warp/recording budget; this is an
engineering inference, not a throughput guarantee or finalized processor selection.

Component registrations are under
`/Users/remiz/CLionProjects/RSX-CANSAT-2027/software/esp_idf/components`.
The seven components are `cansat_core`, `cansat_drivers`,
`cansat_communications`, `cansat_mission`, `cansat_esp_idf`, `cansat_app_boot`,
and `cansat_espnow`. The last is a separately registered radio adapter; it does
not pull the I2C adapter or sensor drivers into the ground-radio project.
The three app projects add that directory to `EXTRA_COMPONENT_DIRS` before
including `project.cmake`, then build only `main` and its dependency closure.
The platform component depends on `esp_driver_i2c` and C++17.

`EspIdfRegisterBus` borrows one existing, synchronously configured I2C device
handle and its explicit address. It does not allocate a bus, infer pins, register
callbacks, or own the handle. The application must keep that handle alive,
ensure its address matches, and use single-task ownership. Asynchronous device
callbacks are incompatible with this adapter's stack buffers.

Each read uses a repeated-start transaction with an explicit timeout of 1 to
1000 milliseconds. Invalid input is rejected before calling the SDK. Failed
transactions do not publish partial bytes. Up to 256 bytes can be read; this is
an adapter bound, not a sensor-specific register limit. See Espressif's
[I2C documentation](https://docs.espressif.com/projects/esp-idf/en/v6.1/esp32/api-reference/peripherals/i2c.html)
and [v6.1 API header](https://github.com/espressif/esp-idf/blob/v6.1/components/esp_driver_i2c/include/driver/i2c_master.h).

The SDK is deliberately not installed on the global shell PATH. The project-local
build runner activates the pinned environment only in child processes. All three
apps now configure, compile and link with real ESP-IDF; flashing, device boot,
sensor I/O and electrical behavior remain unverified. Host SDK substitutes stay
private to native test executables and never enter the firmware component paths.

## DevKitC V4 application skeletons

Each directory below is an independent ESP-IDF project with its own `main`
component and C-linkage `app_main`. The root CMake project remains host-only.

| Role | ESP-IDF project directory | Declared project dependencies |
| --- | --- | --- |
| Container | `/Users/remiz/CLionProjects/RSX-CANSAT-2027/software/esp_idf/apps/container` | Boot reporting, mission, drivers, ESP-IDF bus adapter, communications, ESP-NOW adapter |
| PocketQube | `/Users/remiz/CLionProjects/RSX-CANSAT-2027/software/esp_idf/apps/pocketqube` | Same portable/platform components; no camera dependency |
| Ground-radio bridge | `/Users/remiz/CLionProjects/RSX-CANSAT-2027/software/esp_idf/apps/ground_radio` | Boot reporting, communications and ESP-NOW adapter; no flight/sensor logic |

The shared `devkitc_v4.cmake` profile selects `IDF_TARGET=esp32`, rejects a
conflicting cache or environment target, and reports an actionable error for
missing/incomplete `IDF_PATH`. Each app also declares `REQUIRED_IDF_TARGETS esp32`.
An alternate SoC must get a reviewed profile, not silently reuse this board label.
This layout follows Espressif's
[component build model](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/build-system.html).
The pinned SDK is ESP-IDF 6.1.0, tag `v6.1`, commit
`fff9895c82d744c7237be8847347bdd1b07c6643`. Direct CMake invocation checks the
release version; the build-only runner also checks the exact commit, origin, clean
tracked SDK files, recursive submodule pins and tool-manifest SHA-256.

Every entry point calls `report_inactive_startup(AppRole)` and returns. It logs the
role/profile and `state=INACTIVE configuration=unconfigured_do_not_flash`. It does
not instantiate/update mission logic, create application tasks, initialize I2C,
read sensors, configure Wi-Fi/ESP-NOW, dispatch actuator intents, start a camera,
or parse/forward ground commands. ESP-IDF permits `app_main` to return, cleaning up
its main task while system tasks continue; see the
[startup guide](https://docs.espressif.com/projects/esp-idf/en/v5.4.2/esp32/api-guides/startup.html).
SDK boot/console initialization is not disabled. Console diagnostics are not
telemetry and must not be fed directly into the ground-station serial protocol.

The status banner is a static description of this scaffold, not hardware
detection, a runtime read of `boards.json`, or an arming/safety interlock. The
`unconfigured_do_not_flash` policy does not technically prevent manual flashing.
No GPIO, flash-size, PSRAM, partition, radio-peer/channel or flight-threshold
defaults were selected. The SDK's generated defaults are **not** an approved
board configuration. Full module SKU, peripherals, power/antenna assembly and
pin map still require review before any hardware execution.

The isolated installation is under
`/Users/remiz/CLionProjects/RSX-CANSAT-2027/software/esp_idf/.local` (ignored by Git):
SDK source in `esp-idf-v6.1`, tools and Python venv in `tools-v6.1`, final per-app
builds/configs in `build-ninja`. Initial bootstrap builds using the pre-existing
host CMake are retained separately in `build`; the acceptance builds use local
CMake **4.0.3**, Ninja **1.12.1**, and Xtensa GCC **15.2.0**
(`esp-15.2.0_20251204`). SDK, tool
archives and generated binaries are not included in the source/rollback archive.
No shell profile, global Python packages or system SDK installation was changed.

Re-run all three build-only targets without activating the shell globally:

```sh
python3 /Users/remiz/CLionProjects/RSX-CANSAT-2027/tools/esp_idf_build.py check
python3 /Users/remiz/CLionProjects/RSX-CANSAT-2027/tools/esp_idf_build.py build all --jobs 2
```

Replace `all` with `container`, `pocketqube` or `ground_radio` for one app. The
runner accepts only `check`/`build`, rejects serial-port/flash/monitor arguments,
sets `IDF_TARGET=esp32`, disables the unused component manager and ccache, and
uses one separate `SDKCONFIG` under each build directory. The SDK itself prints
suggested flash commands after a successful build; these are **not executed**.
The static inactive firmware banner does not technically prevent someone from
manually using other tools to flash, so hardware review remains mandatory.

For a fresh macOS/Linux checkout, obtain the exact SDK tag recursively, verify its
commit against `sdk.lock.json`, then use Espressif's `install.sh esp32` with
`IDF_TOOLS_PATH` set to the project-local `tools-v6.1` directory. Explicitly install
`cmake ninja` using that SDK's `tools/idf_tools.py`; these are optional on macOS and
otherwise a global CMake can be reused unintentionally. Put a Python **3.12**
interpreter first on the install command's PATH; this host used the existing
read-only bundled Python 3.12.14 at
`/Users/remiz/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3`.
The installer creates its own `python_env/idf6.1_py3.12_env`; it does not install
packages into that base interpreter. Use a different installed Python 3.12 path
on other hosts, not an assumed Codex runtime location.

`python-requirements.lock.txt` records all 66 observed Python package versions for
this Python/macOS environment and can be supplied as `PIP_CONSTRAINT` on a
same-platform reinstall. It is not a wheel-hash lock or proof of cross-platform
reproducibility. Set `PIP_CACHE_DIR` inside `.local`, `PIP_CONFIG_FILE=/dev/null`
and `PYTHONNOUSERSITE=1` during installation. SDK tools are selected and
SHA-256-checked by Espressif's pinned `tools/tools.json`; its own hash is recorded
in `sdk.lock.json`. No managed third-party component dependencies were added.
See the [official v6.1 release](https://github.com/espressif/esp-idf/releases/tag/v6.1)
for SDK acquisition and installation instructions. No reinstall or update occurs
automatically when running the build wrapper.

The host build now runs **19 CTest suites**. Three compile and call the actual
`app_main` plus boot reporter with a private logging substitute, checking exact
role-specific output and return. Eight Python contract tests exercise all three
projects: registration/source paths, dependency closure, explicit target, wrong
cache/environment target, absent/empty/incomplete SDK paths, and wrong SDK version. The positive
registration tests use an explicitly labeled CMake recorder, **not** ESP-IDF.
They do not test SDK macro compatibility, Kconfig, Xtensa code generation, linking,
flash capacity, RTOS scheduling, power-on behavior or electrical safety. Missing
SDK tests use real CMake configure and require it to stop before compiler setup.

Eight additional runner tests reject changed SDK revisions/origins/manifests,
missing/changed submodules and unsupported flash/monitor/port/job inputs. Real
SDK build evidence is separate from these host substitutes: generated Xtensa ELF,
application binary, bootloader and partition-table outputs are hashed and retained.
The pre-commit payload-slice application binaries were 140320 bytes each and parsed with
`esptool image-info`; their ELF headers identify 32-bit little-endian Xtensa.
The SDK-generated configs currently select **2 MB flash and no PSRAM**. These
values are build defaults, not a measured module capacity or approved pin/memory
profile. Do not flash them as flight firmware. The SDK application version is
derived from the Git revision visible at build time; a later source commit does
not retroactively change those artifacts. Use the saved source and binary SHA-256
manifests for each build's provenance, rather than treating its filename as a release.
Unused portable libraries may be compiled but discarded by the linker; building
them does not imply an active mission loop or runtime integration.

Next integration gate: reviewed hardware profiles, reset recovery and explicit
clock/sample acquisition/configuration/actuator-dispatch boundaries before flight
entry points become active. Never replace missing hardware data with
the replay fixtures at firmware startup.

## Deterministic mission core

`ContainerMission::update` and `PocketMission::update` consume explicit input
snapshots and return state plus **one-shot intents**. They perform no I/O, sleep,
RTOS scheduling or physical actuation. One owner must call each instance; outputs
must be handed to a reliable actuator dispatcher before the next update. A lost
intent is not retried by this model. No acknowledgement, retry, actuator timeout,
fault escalation or persisted reset recovery is implemented yet.

The following is the initial behavioral mapping to the
[mission overview](https://cansatcompetition.com/mission.html) and the guide
linked below, not a complete compliance matrix:

| Mission behavior | Model and deterministic evidence |
| --- | --- |
| Container starts at 4 Hz; ARM gates launch detection | `container_nominal`: unarmed launch ignored, armed qualified launch enters ascent |
| Release at 90% of peak on descent | Qualified apogee gates automatic release; 901 m versus 900 m with a 1000 m peak tests the boundary |
| Mechanism deployment and PQ clearance are separate evidence | Release intent does not confirm either sensor; both feedback values latch independently |
| Container drops to 1 Hz five seconds after release | First qualified `pq_clear` observation anchors the timer, not the command or mechanism switch; 4999/5000 ms tested |
| Container stops transmission at landing | Qualified landing selects 0 Hz and a terminal state |
| PQ powers on at release, records and sends telemetry at 4 Hz | Explicit `power_on_at_ms` event enters descending and emits recording intent; recording feedback is separate |
| Panels deploy five seconds after PQ release/power-on | Each panel gets an independent intent at 5000 ms; confirmation and already-deployed suppression tested |
| Boom and camera rotation are commanded | No autonomous boom or 30-second camera rotation; sequenced relative-angle commands reject duplicate/older IDs |

The mission state machines consume **already-qualified input events**. The
altitude detector below now provides one possible upstream event source; it is
coupled to both mission models in a synthetic flight test, not wired to firmware.
No production altitude, acceleration, dwell-time or descent-rate thresholds have
been selected.
Altitude must be finite, not from the future, within the explicitly configured
maximum age, and not older than the last accepted altitude sample. Equal sample
timestamps allow reusing an unchanged observation while it remains fresh; callers
must not attach different measurements to the same timestamp. A regressing update
clock rejects the entire input without state changes. Test ages of 100/1000 ms
are fixtures, not approved flight configuration.

Qualified switch feedback is latched; debounce, polarity, stuck-switch checks and
physical sensor validation belong upstream. Container manual release is an
explicit ground override without an ARM prerequisite. PQ automatic-panel policy
is an explicit constructor option, allowing a bench mode with only manual panel
commands. These software policies are not physical interlocks. After qualified
landing, new mechanism/rotation intents are suppressed as a conservative project
policy; PQ telemetry remains 4 Hz rather than inheriting the Container cutoff.

The PQ power event and all updates share one monotonic millisecond timeline.
The first update after power-on requests recording; it cannot record retroactively.
A delayed first update still computes the panel deadline from the supplied event
timestamp. Replayed power events cannot reset that deadline. The application must
run the core promptly and supply the correct release/power-on anchor. Camera
rotation is an abstract view-offset intent, not a tested physical servo or
software-stabilization implementation. The mission's 30-second ground command
must be scheduled by the ground operator/application, not invented as an onboard
automatic action. These internal phases do not yet map to legacy `OpState` or
competition telemetry fields.

`deterministic_mission` tests nominal trajectories, exact timing/altitude bounds,
missing confirmations, invalid/stale/out-of-order altitude, backward time,
duplicate commands, future/replayed power events, finite angle normalization,
manual overrides and landing priority. The current 19-suite build includes `altitude_flight_detection`, `csv_flight_replay`, and `preferred_module_policy`.
All tests use explicit fixture inputs without a wall clock or connected hardware.
They do not prove real-flight detection, end-to-end command delivery, sensor accuracy,
real-time scheduling or competition compliance.

## Altitude-based flight-event detectors

`AltitudeFlightDetector::update` is a portable, allocation-free runtime model in
`software/lib/mission`. It emits `FlightEvent::launch`, `apogee`, and `landed`
once each as its phase advances. All configuration must be supplied explicitly;
the zero-initialized configuration is invalid and produces no events.
Configuration checks enforce finite, positive rates/durations, consistent altitude
bounds and at least two confirmation samples. No sensor-specific defaults are
hidden in this library or in the board configuration.

| Event | Required evidence over a continuous sample-time window |
| --- | --- |
| Launch | ARM level, altitude at/above launch threshold, minimum positive climb rate, dwell duration and minimum sample count |
| Apogee | Previously detected launch, configured drop from tracked peak, minimum negative vertical rate, dwell duration and sample count |
| Landing | Descending phase, altitude inside explicit landing band, low absolute vertical rate, bounded altitude span, dwell duration and sample count |

Vertical rate is a two-sample finite difference, **not** a Kalman filter or sensor
fusion estimate. A configurable absolute rate bound rejects implausible jumps
before they can update the peak or reach the mission model. This cannot reject
every physically plausible glitch. Confirmation uses acquisition timestamps,
not caller frequency or time since the first seed. Threshold comparisons are
inclusive. A sample gap greater than the configured limit reseeds the rate
estimate without forwarding that sample or carrying the old confirmation window.

Missing, nonfinite, future, stale, reordered or conflicting same-timestamp data,
implausible rates, and a regressing caller clock invalidate rate/confirmation
history. The flight phase and established peak are preserved; time is not rewound.
An identical fresh duplicate is a no-op, not another vote. After a fault, one
new sample seeds the rate estimate and subsequent valid samples must complete a
new confirmation window. Loss of ARM cancels pending launch confirmation, even on
a duplicate sample; it does not erase a flight already detected. Landing is terminal.

Use `DetectorStart::on_pad` for the Container. The PocketQube explicitly starts
with `DetectorStart::descending_after_release` when powered at release, so it
does not need to observe an earlier launch/apogee. This is **not** a reset-recovery
mechanism: constructing a new detector loses its history. Both instances have
independent configuration/state and share no mutable global state.

The input `Altitude::agl_m` must be supplied in metres relative to the agreed
ground/reference datum. Pressure conversion, filtering, calibration and reference
transfer/persistence are not implemented. In particular, do not zero the PQ's
barometer at its in-air power-on and call that ground level. A landing site above
or below the reference may require a different explicit band; site and pressure
data must validate that policy. An altitude-only detector cannot distinguish a
sensor frozen near ground from a real landing if its acquisition timestamp keeps
advancing. Health checks and independent evidence remain integration work.

Pass only `DetectionOutput::qualified_altitude` and its one-shot `event` into
`ContainerMission` in the same update. Do not bypass rejected samples by forwarding
raw altitude instead. Seed, duplicate and rejected updates carry no qualified
altitude. For the powered PQ, map `event == FlightEvent::landed` into
`PocketInput::landed`. The application must latch command/ARM state, use one
owner per instance, and promptly consume event outputs; no task, radio, GPIO or
physical actuator is exercised by this slice.

The fixture configuration in `test_flight_detection.cpp` uses 10 m / 2 m/s /
2 s for launch, a 5 m drop / 1 m/s descent / 2 s for apogee, and a [-3, 3] m
band / 0.5 m/s / 0.5 m span / 3 s for landing, with three samples minimum,
1000 ms age/gap bounds and a 150 m/s jump bound. These are **synthetic test
parameters, not recommended flight settings or competition-mandated thresholds**.
The coupled trace detects launch at 4000 ms, apogee at 10000 ms and landing at
17000 ms; it exercises release, confirmed-clear timing, panel timing and both
telemetry policies without injected launch/apogee/landing events. Additional
tests cover noise, rebound, slow drift, bounce, dropouts, duplicates, rate spikes,
exact bounds, invalid configurations and timer arithmetic near `uint64_t` maximum.

Before flight, tune with recorded/calibrated pressure-altitude traces and fault
injection, measure missed/false events and detection latency, and verify the
apogee confirmation delay leaves time for release near 90% of peak. This model
may confirm apogee only after passing that altitude; it does not guarantee exact
physical release altitude. No field data or hardware validation is available yet.

## CSV replay and explicit tuning comparisons

The host-only `tools/flight_replay.py` CLI has `replay` and `tune` commands. It
parses files with Python's standard CSV/JSON libraries and calls the compiled
`flight_replay_backend`, which invokes `AltitudeFlightDetector::update`. There is
no duplicate Python detector and no hardware or new dependency requirement.
This tool replays detector decisions only, not the full mission/actuator/telemetry
runtime. The separate coupled native mission test remains the integration fixture.

### Input contract

One UTF-8 CSV represents one flight, in original acquisition/delivery order:

```csv
now_ms,sampled_at_ms,altitude_m,armed,expected_event
0,0,0,1,none
1000,1000,5,1,launch
```

- `now_ms`: caller time as unsigned 64-bit decimal milliseconds.
- `sampled_at_ms`: acquisition time on the same clock. Blank only when altitude
  is also blank, representing a missing sample.
- `altitude_m`: metres using the detector's agreed ground datum. Decimal/scientific
  numbers and explicit `nan`/`inf` sensor faults are accepted. Numeric overflow,
  guessed units and nonnumeric cells are rejected.
- `armed`: literal `0` or `1`, as a command-state level.
- Optional `expected_event`: `none`, `launch`, `apogee`, or `landed`. Fully label
  every row, or omit/blank every label. Partial labels are rejected rather than
  interpreted as negative examples. Each named event may appear once per trace.

Header order, standard CSV quoting, CRLF and a UTF-8 BOM are supported. Unknown or
duplicate columns, incomplete rows and ambiguous values fail explicitly. No
sorting, interpolation, deduplication or timestamp repair occurs; the C++ detector
sees faults and duplicates in their original order. Ground truth labels refer to
`now_ms`, not a guessed physical event time between samples. Delayed acquisition
is preserved separately in `sampled_at_ms`.

Config JSON schema 1 requires `source`, explicit `start` (`on_pad` or
`descending_after_release`) and every field of `FlightDetectionConfig` under
`parameters`. Unknown keys, duplicate JSON keys, booleans used as numbers and
nonfinite config values are rejected. The native detector performs its own
cross-field validity check. Grid JSON schema 1 supplies nonempty value lists
under `parameters`; `tune` compares the baseline and every unique Cartesian
combination. It neither ranks a winner nor updates a flight configuration.

### Run the checked-in synthetic example

After the root native build, run this command once with a new output directory:

```sh
python3 -B /Users/remiz/CLionProjects/RSX-CANSAT-2027/tools/flight_replay.py tune \
  --backend /private/tmp/rsx-cansat-native-20261010/flight_replay_backend \
  --trace /Users/remiz/CLionProjects/RSX-CANSAT-2027/software/tests/replay/fixtures/synthetic_flight.csv \
  --config /Users/remiz/CLionProjects/RSX-CANSAT-2027/software/tests/replay/fixtures/synthetic_config.json \
  --grid /Users/remiz/CLionProjects/RSX-CANSAT-2027/software/tests/replay/fixtures/synthetic_grid.json \
  --match-window-ms 3000 \
  --output-dir /private/tmp/cansat-flight-tuning-example
```

For one configuration, change `tune` to `replay` and omit `--grid`. Existing
output directories are never overwritten; choose a new directory on each run.
Exit 0 means replay/report generation succeeded, **not** that the detector met
flight requirements. Input, configuration and backend errors return exit 2.

Outputs are `baseline_samples.csv`, one samples CSV per candidate,
`comparison.csv`, and a completion manifest `summary.json`. Each sample records
input timestamps/altitude, detector phase/status/event, qualified altitude,
vertical speed and peak. The JSON records full evaluated parameters, status
counts, event times, label metrics, and SHA-256 provenance for the trace,
configuration, grid, Python driver, native binary and output CSVs. Backend or
validation failures publish no output directory. A final filesystem write failure
may leave a partial directory; `summary.json` is published last. Verify its CSV
hashes before consuming results. Inputs are never modified.

Labeled traces require an explicit `--match-window-ms`. Signed event latency is
`detected_now_ms - labeled_now_ms`; a negative value means early detection. Match
uses the inclusive absolute timing window for the same event type. An out-of-window
detection counts both as a missed expected event and a false emitted event;
absence of both contributes neither. Unlabeled traces have **null** metrics and
blank comparison cells, not invented zero errors. This is event-level matching,
not sample-level classification accuracy or a safety score.

The example reuses the native 18-row altitude trajectory but deliberately labels
launch/apogee/landing at 1000/6000/14000 ms as manually authored synthetic reference
markers. Baseline detections are 4000/10000/17000 ms. Varying the apogee drop from
5 m to 1 m or 10 m shifts its detected time to 9000 or 11000 ms. These fixtures
demonstrate latency measurement, not true hardware timing or an optimal setting.
A smaller threshold can reduce this trace's delay while increasing false events
on other data. Tune on varied recorded flights/fault traces and evaluate on held-out
traces before proposing any production values.

Limits: 20 MiB CSV, 100000 rows, 64 unique configurations including baseline,
250000 total row evaluations, 64 KiB JSON files, and a 30-second timeout per native
run. Twenty replay tests cover parsing, native parity, timing metrics, fault
preservation, configuration grids, determinism, provenance and failure handling.
The native backend's `--protocol-v1` stdin format is private to this Python tool.

## Node COM skeleton

The selected topology is a three-node star, with a reserved local vision boundary:

```text
Container mission <-> NodeCom <-> ESP-NOW ----\
                                               GroundRadio <-> USB <-> gui-new
PocketQube mission <-> NodeCom <-> ESP-NOW ----/
         |
         +-- future local vision command/status interface (no transport selected)
```

The radio transport and portable core exist now. The arrows to mission dispatch,
USB routing and GUI represent **planned integration**, not a working end-to-end
radio system. All three `app_main` entry points remain byte-identical, reporting
INACTIVE and returning; none instantiate NodeCom or call the radio adapter.

### Layer ownership

- `/Users/remiz/CLionProjects/RSX-CANSAT-2027/software/lib/communications/include/cansat/radio_protocol.hpp`:
  fixed-capacity messages and explicit wire encoding, with no packed-struct copies.
- `/Users/remiz/CLionProjects/RSX-CANSAT-2027/software/lib/communications/include/cansat/radio_transport.hpp`:
  nonblocking submission, peer-tagged receives and asynchronous MAC completions.
- `/Users/remiz/CLionProjects/RSX-CANSAT-2027/software/lib/communications/include/cansat/node_com.hpp`:
  single-owner scheduling, routing checks, command receipts and retries. No SDK,
  sensor, GPIO, Qt or implicit clock dependency; fixed queues, no heap allocation.
- `/Users/remiz/CLionProjects/RSX-CANSAT-2027/software/lib/platform/esp_idf/include/cansat/esp_now_transport.hpp`:
  the process-lifetime `EspNowTransport` singleton, compiled by `cansat_espnow`.
- `/Users/remiz/CLionProjects/RSX-CANSAT-2027/software/tests/native/fake_radio.hpp`:
  test-only deterministic transport with explicitly injected delivery/failure.

The user selected compact binary radio packets, readable USB records and a
separate competition CSV exporter. The [mission guide](https://cansatcompetition.com/assets/docs/competition/CanSat_Mission_Guide_2027_r1.2c.pdf)
requires ESP-NOW, assigned channels, team-specific unicast routing and no broadcast;
it permits binary or ASCII on air. Exported telemetry uses its prescribed CSV
layout and CR terminator. USB framing is not that export format.

### Wire contract and deliberate limits

`/Users/remiz/CLionProjects/RSX-CANSAT-2027/software/lib/communications/protocol.json`
defines experimental version 1: a 26-byte header, up to 220 payload bytes, and
4-byte CRC-32/ISO-HDLC. All multi-byte fields are little endian. The complete
application datagram is at most 250 bytes, including header and CRC. This is a
conservative interoperability policy, not ESP-NOW v2's absolute maximum.
[Espressif documents](https://docs.espressif.com/projects/esp-idf/en/v6.1/esp32/api-reference/network/esp_now.html)
the v1/v2 size distinction, callback execution context and the fact that MAC
success does not prove application reception.

Header fields identify kind, team, sender/destination, both boot sessions, message
ID, correlation ID and payload length. CRC detects corruption, not forgery.
The transport remains payload-agnostic. Typed mission telemetry/command codecs
now sit above it, as described below; sensor conversions, command execution and
vision/science profile contents remain separate. Do not serialize the legacy
`ContainerPacket` or `PQPacket` memory directly. Result payloads remain typed.

The existing `encodeSerialFrame`/`decodeSerialFrame` and Python equivalents are
unchanged. Their 250-byte **ASCII body** cap is independent of the radio budget;
full PocketQube USB records need a separate worst-case size calculation. The
legacy GUI's 57600/CR and Arduino bridge's 115200/LF mismatch remains unfixed until
that reviewed migration. Unknown legacy prefixes, unbounded RX copies, truncated
lines and volatile-only handoffs must not be carried into the new bridge.

### Service contract

One task owns NodeCom; all calls use the same injected monotonic clock. The
application supplies `ContainerOutput::telemetry_hz` or
`PocketOutput::telemetry_hz` through `setTelemetryRate(0/1/4)`. The service does not
duplicate flight-state logic. `publishTelemetry` replaces the previous snapshot;
missed transmit periods are skipped, not replayed as an old backlog. Received
telemetry is a bounded FIFO with drop accounting, not persistent storage.

Ground can track one command per flight peer. Retries retain the same identity;
`max_attempts` counts queued radio submissions. Busy/rejected submissions cannot
extend the command TTL. A timeout reports UNKNOWN, not proof that nothing happened.
Receipt, application acceptance, rejection, completion and failure are distinct.
The application alone calls `reportCommandResult`; completion must be backed by
its real mechanism/sensor evidence, not radio delivery. Reordered receipts cannot
regress accepted command state. Every queue and per-tick RX budget is bounded.

The receiver queues each new ordered command once. Eight receipt slots retain
results; unfinished commands are not evicted. A monotonic watermark prevents old
identities from executing after terminal-cache eviction. Out-of-order new commands
are rejected as stale; ground stop-and-wait per peer makes that intentional.
Same identity with different bytes is rejected. These are within-session rules,
not a persistent exactly-once guarantee.

Self and authenticated peer boot/session IDs are explicit configuration inputs;
zero is unknown. No session discovery or durable store is implemented yet. A new
receiver session rejects commands targeting its previous boot. Never reuse a
session with a reset message counter, automatically replay a relative rotation
after reset, or treat transport IDs as persistent competition packet counters.
Recovery must define durable counters, mission time, calibration and command
reconciliation before flight integration. Do not select flash-per-packet writes
without a power-loss and wear design.

### ESP-IDF adapter boundary

Construction and invalid/default configuration have no SDK side effects.
`start` is explicit and unused by the apps. A future caller must exclusively own
ESP-NOW, start STA Wi-Fi, and set its reviewed channel first. The adapter checks
that channel, validates star peers/non-broadcast MACs, requires nonzero explicit
PMK/LMKs, and configures encrypted unicast peers. No example keys, peer MACs or
assigned channel are selected in production. The channel's numeric range check
is not regulatory authorization; Wi-Fi country configuration remains the caller's
responsibility. Keys must come from private provisioning, not this repository.

SDK callbacks copy length-checked bytes/metadata into static FreeRTOS queues.
There is one outstanding send, released when its completion is consumed. Source
and destination MACs are checked before RX enqueue; the portable layer then checks
team, role, version, session and message bounds. No callback parses commands,
runs mission code, allocates buffers, writes a serial port or actuates a mechanism.

This first adapter has no stop/reconfigure or radio-recovery lifecycle. A missing
or mismatched SDK completion leaves TX busy rather than risking completion
misassociation; command TTLs still report UNKNOWN. A later supervised recovery
policy must reconcile outstanding commands before resetting radio state. Keep the
adapter and future service storage static, not on a small application task stack.
SDK-double tests are sequential contract tests, not proof of multicore timing,
encryption, RF range, coexistence, device boot or flight readiness. The real SDK
build validates API/ABI compilation; unused adapter code may be removed by linking.

### Checked first slice and next work

The host suite adds codec/boundary and deterministic NodeCom scenarios, independent
Python struct/zlib golden bytes, and an adapter SDK-double contract. The real
v6.1 component is included in all three build dependency closures; app startup
stays inactive. Test fixtures use invented identifiers, keys and timings solely
as test inputs, not approved flight settings.

Typed payload schemas are now implemented below. Next: review unresolved export
policies, implement the bounded USB stream parser and gui-new adapter/exporter,
then integrate session/recovery state and mission command dispatch. A wired vision transport and backup radio remain out of
scope. No different ESP32 is required by this architecture alone; resource/range
margins and any vision processor choice remain measurement-dependent.

## Mission telemetry and command payload schemas

The portable API is
`/Users/remiz/CLionProjects/RSX-CANSAT-2027/software/lib/communications/include/cansat/mission_payloads.hpp`.
Its machine-readable contract is
`/Users/remiz/CLionProjects/RSX-CANSAT-2027/software/lib/communications/mission_payloads.json`.
Both are experimental version 1, independent of the existing radio-envelope
version. Native codecs are compiled by the same `cansat_communications` component
for host tests and all three ESP-IDF projects. No application entry point changed.

### Encoded sizes and representation

| Payload | Schema tag | Encoded payload | Including 30-byte radio envelope |
| --- | --- | --- | --- |
| Container | 1 | 71 bytes | 101 bytes |
| PocketQube base | 2 | 177 bytes | 207 bytes |
| PocketQube with both extensions | 2 | Up to 220 bytes | Up to 250 bytes |
| Ground command | 3 | 3 to 11 bytes | 33 to 41 bytes |

All payloads begin with one version byte and one schema byte. Multi-byte numeric
values have explicit little-endian encoding, including portable signed decoding;
there is no struct-memory serialization. Container and PocketQube share a 70-byte
prefix. The JSON contract lists every byte offset, width, unit and validity bit.

Telemetry uses uint64 elapsed milliseconds and uint32 application-owned counters.
Sensor values use scaled integers: millimetres, pascals, milli-degrees Celsius,
millivolts, signed milliamps, milli-degrees/second, millimetres/second-squared,
milligauss and nanodegrees. GNSS altitude retains its receiver datum; the codec
does not silently relabel it AGL. Receiver-provided GNSS time is bounded ASCII
(up to 32 bytes), not an invented UTC conversion. Greater encoded precision does
not establish sensor accuracy or guarantee lossless conversion from every device.
Driver adapters must range-check and explicitly quantize floating measurements.

Validity bits distinguish a real zero from an unavailable value. Missing numeric
fields must contain zero; missing GNSS time must be empty. Unknown bits, invalid
text/padding, a half-valid coordinate pair, nonpositive valid pressure and values
below absolute-zero temperature are rejected. Latitude/longitude bounds are
physical coordinate bounds. Other encoded integer bounds are representational,
not selected flight thresholds. Sensor status, age and calibration decide validity
in the acquisition/application layer, not in this codec.

Mechanisms have separate `mechanism_state` and `mechanism_known` masks so unknown
feedback is not confused with a confirmed stowed/off condition. State bits cannot
be asserted unless known. Command echo is a bounded 24-byte, comma-free ASCII
token with canonical zero padding. It is supplied by the application after
processing, never inferred from MAC delivery or a raw transport receipt.

PocketQube carries two ordered extension blocks: stabilization, then science.
Each block has profile ID, profile version and data length; their data shares
**43 bytes total**, not 43 bytes each. All-zero headers mean absent. Nonempty data
requires nonzero profile/version. Unknown profiles can be retained as opaque
bounded bytes but not interpreted or flattened into CSV. No real profile IDs,
field meanings or example values have been selected for either experiment.

### Mission-only command catalog

| Typed command | Target | Body |
| --- | --- | --- |
| `Calibrate` / CAL | Container | None |
| `Arm` / ARM | Container | None |
| `SetUtc` / ST | Container | uint64 Unix UTC milliseconds |
| `Simulation` / SIM | Either flight node | ENABLE, ACTIVATE or DISABLE enum |
| `SimulatedPressure` / SIMP | Either flight node | Positive uint32 pascals |
| `Deploy` / MEC | Container or PocketQube | One role-appropriate deployment enum |
| `Recording` / RECORD | PocketQube | START or STOP enum |
| `RotateView` / VIEW | PocketQube | Nonzero signed milli-degrees within one turn |

MEC names the Container release or one of PocketQube's two panels/boom; it does
not carry raw GPIO numbers, servo IDs or PWM values. RECORD and VIEW are
team-defined mission operations, not generic debug access. A positive view angle
uses the right-hand convention about nadir +Z; actual camera/control-frame mapping
must be bench-verified. The +/-360-degree wire bound is a team design choice,
not a measured mechanism limit. `commandEcho` formats compact tokens; VIEW echoes
milli-degrees, ST milliseconds, and SIMP pascals.

`SetUtc` sets a separate UTC reference, **not** elapsed `mission_time_ms`.
SIM ACTIVATE decoding does not enable simulation: a future authorized application
dispatcher must enforce ENABLE before ACTIVATE and accept injected pressure only
in the active simulation state. It must also enforce calibration/arming timing,
mission mechanism policy, command identity and physical feedback. The codecs do
not change mission state, operate actuators or prove command completion. Ground
must not resend a relative rotation under a new identity after an uncertain result.

The existing `NodeCom` intentionally remains payload-agnostic. Callers must bind
payload schema to the already validated outer message kind and source/destination
before processing it. Native integration tests demonstrate a typed command through
the fake transport and back into a typed request; this is not runtime app wiring.

### Requirement mapping and unresolved export policy

The [mission guide Rev. 1.2c](https://cansatcompetition.com/assets/docs/competition/CanSat_Mission_Guide_2027_r1.2c.pdf)
is the field/command reference. The JSON records the required per-node CSV order,
unit conversions, filenames and CR terminator; **no exporter is implemented here**.
Wire field order is independent of CSV field order. The legacy serial/GUI mismatch
and gui-2026 remain untouched.

The guide gives differing voltage precision in CTR21 versus its telemetry table,
and differing solar precision in PQ31 versus its table; PQ27 uses amperes whereas
the telemetry table uses milliamps. The wire retains finer mV values and signed mA.
Final CSV rounding, unavailable-reading representation and stabilization/science
columns require review rather than silently emitting misleading zeroes. Invalid
fields must remain marked invalid all the way to that exporter.

`ContainerState` is the four published wire states, not the internal mission-enum
ordinal. Release-requested must not become PQ_RELEASE before qualified clearance.
A landed Container stops telemetry; any persisted last-state/export mapping still
belongs to reviewed integration. The schema does not implement mission-time or
counter persistence and does not substitute message IDs for packet counts.

### Verification and remaining work

Native tests cover roundtrips, signed extremes, invalid/missing values, malformed
text, unsupported node commands, truncated/trailing bytes, bounded extensions and
maximum-size radio wrapping. Independent Python `struct` vectors check C++ bytes
and JSON offsets. Deterministic malformed-input cases also run with address and
undefined-behavior sanitizers. These are software checks, not hardware validation.

The user confirmed explicit validity flags with precise scaled-integer units,
versioned bounded extension slots while their fields remain undefined, and
mission-only commands on 2026-10-10. Other detailed wire choices remain experimental
engineering decisions. No hardware profile, peer/channel, production retry values,
keys, flight thresholds, command dispatcher or sensor conversion is selected here.

Next choices: bounded USB records plus gui-new parsing/export policy, or an
authorized mission-command dispatcher with SIM gating and reset reconciliation.

## Deferred integration work

1. Supply real traces for the implemented CSV replay tool, or choose the calibrated
   pressure-to-altitude pipeline, reset recovery, or a two-node GUI simulator.
   The pinned ESP-IDF build gate now passes for all three inactive `esp32` apps.
   Bench-test the first sensor only once a board and breakout are available.
2. Add BMP581 using Bosch's SensorAPI, followed by LSM6DSO, MMC5983MA and MAX-M10S
   adapters. Keep calibration, units, axis transforms and sample age explicit.
3. Review the implemented mission schemas and unresolved CSV/extension policy;
   add the bounded serial stream parser and integrate bridge/gui-new node models.
4. Implement simulation-mode pressure injection and mode/command handshakes;
   persist mission state, counters, mission time and calibration across resets.
   Add the reliable actuator dispatcher and timeout/fault reporting before flight
   integration. Mission updates alone cannot guarantee delivery of actuator intent.
5. Benchmark vision independently before choosing its processor or lens.

The requirements baseline is the
[2027 Mission Guide Rev. 1.2c](https://cansatcompetition.com/assets/docs/competition/CanSat_Mission_Guide_2027_r1.2c.pdf).
The behavioral mapping above must expand into a requirement-ID traceability
matrix with integration and hardware evidence before any compliance claim.
