# CanSat 2027 software specification

## Scope and status

This is the specification of the implemented software skeleton through mission
telemetry and command payloads. It is the entry point for maintaining the portable
libraries, inactive ESP-IDF applications and host tools. The implementation is
build-tested, not flight-ready. No application currently starts sensor acquisition,
radio traffic, mission control, command execution or camera recording.

The selected baseline is ESP-IDF on ESP32-DevKitC V4, with ESP32-WROOM-32UE preferred
for most nodes. The Container, PocketQube and ground-radio bridge are independent
applications. Python/PyQt6 `gui-new` is the future ground UI; `gui-2026` and the
original Arduino/PlatformIO runtime remain historical and unchanged. The fixed
wide-angle camera/software-stabilization experiment is isolated, with no selected
processor or camera.

Requirements below describe **implemented behavior** unless marked **integration
requirement** or **open decision**. “Must” in an integration requirement is a gate
for future work, not a claim that the current skeleton already implements it.
The [architecture and build guide](architecture.md) contains mission rationale,
research links, setup instructions and detailed detector/replay behavior. This
specification, public header contracts and machine-readable contracts are intended
to be reviewed together, not treated as independent competing implementations.

## Contract sources and change policy

| Contract | Location and purpose |
| --- | --- |
| Radio envelope | [protocol.json](../software/lib/communications/protocol.json): offsets, version, node/kind values and service policies |
| Mission payloads | [mission_payloads.json](../software/lib/communications/mission_payloads.json): every field offset, unit, validity bit, command argument and CSV mapping |
| Board policy | [boards.json](../software/lib/platform/boards.json): preferred family/carrier and unconfigured per-role hardware profiles |
| SDK pin | [sdk.lock.json](../software/esp_idf/sdk.lock.json) and [Python package versions](../software/esp_idf/python-requirements.lock.txt) |
| Vision boundary | [benchmark_contract.json](../software/experiments/vision/benchmark_contract.json): inputs, measurements and isolation constraints, not a camera implementation |
| Wire examples | [mission_payload_vectors.json](../software/tests/native/fixtures/mission_payload_vectors.json) and independent Python/native golden tests |

The envelope version and payload version are independent, currently both 1.
Changing an existing field width, order, unit, meaning or validation rule requires
an explicit compatibility review, version decision and updated golden vectors.
Unknown versions, schema tags, command opcodes and invalid lengths are rejected;
there is no negotiated downgrade. Unknown extension profiles can be retained as
opaque bounded bytes, not interpreted as known fields. Golden examples use test
identifiers/readings and are never provisioning or flight settings.

## Module ownership and implemented coverage

| Module | Responsibility | Ownership and exclusions |
| --- | --- | --- |
| `cansat_core` | Register-read interface and timestamped optional samples | Caller owns bus and clock; no board setup |
| `cansat_drivers` | INA236 identity/status/read-only conversion | One owner; explicit address/shunt; no sensor configuration writes |
| `cansat_mission` | Container/PocketQube state and altitude event detection | One owner per object; monotonic time; emits intents, never actuates |
| `cansat_communications` | Complete ASCII frames, binary envelope, NodeCom, typed payloads | No SDK or Qt; fixed radio storage; ASCII strings may allocate |
| `cansat_esp_idf` | Synchronous I2C adapter | Borrowed device lifetime and explicit finite timeout |
| `cansat_espnow` | Encrypted-unicast ESP-NOW transport adapter | Explicit start, static callback queues, externally configured STA Wi-Fi |
| `cansat_app_boot` | Role-specific inactive startup logging | No application task or peripheral setup |
| `cansat2027` Python package | Equivalent whole-frame ASCII codec | Not yet wired to the GUI serial manager |
| Host tools | Fixture diagnostic, CSV replay/tuning, pinned SDK build runner | Explicit inputs; no hardware fallback or flashing |

Firmware components use C++17. Host tests use CMake 3.20 or newer, a C++17 compiler
and Python 3.10 or newer without Qt. The local SDK runner uses the separate pinned
Python 3.12 environment. `tests/native/esp_idf_stubs` and test doubles are host-only;
they must not enter firmware include paths. Ground radio does not depend on flight
mission logic, sensors or the I2C adapter. Vision processing must remain off the
mission-control and telemetry critical path.

## Sensor and bus contracts

`RegisterBus::read(address, reg, destination, length)` is a synchronous exact-read
operation. True means all requested bytes were transferred in device wire order;
NACK, timeout and short reads return false. Platform implementations enforce
finite timeouts. A driver must ignore output bytes on failure; the ESP-IDF adapter
specifically preserves destination bytes on every failed read.

`EspIdfRegisterBus` borrows a synchronous I2C device handle configured with the same
7-bit address. Valid lengths are 1..256 bytes and timeouts 1..1000 ms. Null output,
invalid address/handle/size/timeout or an address mismatch fails before SDK I/O.
It uses a repeated start and a temporary buffer. `lastError()` exposes the latest
validation/SDK result, initially `ESP_ERR_INVALID_STATE`. No callbacks, pin choices,
device creation/destruction or concurrent callers are supported by this adapter.

`Ina236` requires an address in 0x08..0x77 and a finite positive shunt resistance in
ohms. `probe()` clears prior identification, checks manufacturer/device IDs and
returns a typed error. `read()` requires successful identification, continuous
shunt-plus-bus mode and conversion-ready status. It returns volts and signed amps,
derived from shunt voltage and the explicit resistor. Invalid samples carry no
value; zero is never substituted for a failed measurement. `observed_at_us` is
caller-supplied acquisition-start time in microseconds, not hardware conversion
time. Reading status acknowledges conversion-ready; one owner must manage it.

No BMP581, LSM6DSO, MMC5983MA or MAX-M10S hardware driver is implemented in this
slice. Their acquisition, calibration, axis mapping, age/health checks and
conversion into wire units remain integration work. INA236 fixtures do not establish
the installed address, shunt resistor, electrical compatibility or sensor accuracy.

## Time domains and mission behavior

| Time or identity | Meaning | Forbidden substitution |
| --- | --- | --- |
| Sensor `observed_at_us` | Acquisition start, caller monotonic microseconds | Conversion completion or UTC |
| Mission/detector `now_ms` | Caller monotonic milliseconds | Wall-clock time that can jump |
| Altitude `sampled_at_ms` | Acquisition on the same millisecond timeline | Delivery time of an old sample |
| Wire `mission_time_ms` | Application-owned elapsed mission time | `SetUtc::unix_ms` |
| UTC reference | Separate Unix UTC milliseconds | Rewinding detector time |
| Radio session/message IDs | Volatile transport identity and ordering | Competition packet/command counters |
| Rotation sequence | Strictly increasing identity within the mission runtime | Unreviewed retry under a new identity |

The mission models are deterministic and perform no I/O or blocking waits.
Regressing update time rejects the input without changing mission state. Qualified
feedback is distinct from a request and latches where specified. One-shot intents
must be consumed during that update; there is no actuator dispatcher, timeout,
retry, fault escalation or persisted recovery in these models.

| Container state or input | Implemented transition or output |
| --- | --- |
| Launch pad | 4 Hz telemetry; ARM latches; qualified launch plus valid altitude enters ascent |
| Ascent | Track accepted altitude peak; qualified apogee enters apogee phase |
| Apogee | Fresh altitude at or below 90% of a positive tracked peak requests release once |
| Manual release command | Ground override without ARM prerequisite; one intent while not landed/clear |
| Mechanism feedback | Latches independently; not proof that the PocketQube cleared |
| First qualified PQ-clear | Anchors the release timer and enters released unless already landed |
| Five seconds after clear | Container telemetry changes from 4 Hz to 1 Hz |
| Qualified landing away from pad | Terminal landed state, 0 Hz telemetry, no new release intent |

| PocketQube state or input | Implemented transition or output |
| --- | --- |
| No power event | Off model, 0 Hz telemetry; not physical power control |
| First valid power event | Descending model, 4 Hz, one recording-start intent |
| Five seconds after power event | Independent panel intents if automatic policy is enabled and not already requested/confirmed |
| Explicit panel/boom command | Independent one-shot deployment intent, never confirmation |
| Increasing finite rotation command | Relative view offset normalized into [0,360) degrees; duplicate/older sequence ignored |
| Qualified landing | Suppress new deployment/rotation intents; telemetry remains 4 Hz |
| Recording feedback | Latest optional feedback updates confirmation; absence does not imply success |

Internal Container phases are **not wire enum ordinals**. `release_requested` is
not published as `PQ_RELEASE` until clearance is qualified. The Container's landed
phase has no new published CSV state; transmission stops. Release/rotation intents
are software policy, not electrical interlocks. The mission's future ground-side
rotation scheduling must not be silently replaced by an onboard timer.

## Detector and replay contracts

`AltitudeFlightDetector` uses consecutive finite AGL-metre observations and
sample-time differences. Launch requires ARM, minimum altitude/climb and continuous
dwell/sample count. Apogee requires prior launch, a drop from tracked peak,
descent rate and dwell/count. Landing requires descending phase, an explicit
altitude band, low absolute speed, bounded altitude span and dwell/count.
Threshold comparisons are inclusive; all configuration is explicit. Zero defaults
are invalid. Detector thresholds in fixtures are not flight recommendations.

Missing/nonfinite/future/stale/reordered/conflicting samples, implausible rates and
clock regression clear rate/confirmation continuity while preserving phase/peak.
A fresh identical duplicate adds no vote. A gap above the configured maximum
reseeds; a seed is not qualified altitude. Events are one-shot. Container starts
`on_pad`; PQ starts `descending_after_release` on the release/power timeline.
Constructing a detector after a reset loses history and is not recovery.

The application must pass `qualified_altitude` and its event together to mission
logic, never substitute raw rejected altitude. Pressure-to-altitude conversion,
ground-reference transfer and health checks are absent. An altitude-only detector
cannot distinguish a frozen sensor from landing when its timestamp keeps advancing.

`flight_replay.py replay|tune` validates UTF-8 CSV/JSON, runs the same compiled C++
detector and writes new report directories. Required CSV columns are `now_ms`,
`sampled_at_ms`, `altitude_m`, `armed`; optional `expected_event` supplies labels.
The [architecture input contract](architecture.md#input-contract) defines missing
samples, fault values, configuration/grid constraints and exact commands.
No input reordering, hidden parameter optimization or Python detector exists.

Limits: 20 MiB CSV, 100000 rows, 64 unique configurations, 250000 evaluated rows,
64 KiB JSON and 30 seconds per backend run. Reports contain per-sample outputs,
comparison metrics and hashes; `summary.json` is published last. Exit 0 means
processing succeeded, not flight qualification. Input/config/backend errors exit 2.
Labels need an explicit matching window; unlabeled metrics remain null. A final
filesystem write failure can leave a partial directory, which is not a completed
report without the manifest and matching hashes.

## Radio envelope and service

Only ground-to-Container and ground-to-PocketQube links are permitted. No broadcast
or flight-to-flight routing exists. The 250-byte application datagram limit includes
26 header bytes, 1..220 payload bytes and 4 CRC bytes. Integer fields are explicit
little endian. CRC-32/ISO-HDLC covers header and payload, with polynomial
0xedb88320, initial 0xffffffff and final complement; its stored value is little
endian. CRC detects corruption, not authentication or authority.

Header team is 1..9999; sender session and message ID are nonzero. Telemetry has
zero destination session/correlation and a flight source. Commands have nonzero
destination session, zero correlation and ground source. Results return to ground,
carry both sessions, a nonzero command correlation and exactly one result byte.
Envelope parsing validates exact length, magic/version, CRC, roles and kind rules.
NodeCom additionally compares team, transport-supplied peer, configured sender
session and its own destination session. Payload role/schema binding remains the
application's responsibility, not an automatic NodeCom operation.

`NodeCom` borrows a nonblocking transport and is single-owner. It copies submitted
payloads. Configuration requires a nonzero session, positive retry interval,
TTL at least the retry interval and 1..8 queued attempts maximum. Unknown peer
sessions prevent command submission and reject incoming traffic from that peer.
Every call shares a monotonic millisecond clock; regression is rejected.

`tick()` consumes at most eight completions and eight received datagrams, expires
command TTLs, then attempts at most one outgoing message. Priority is reply FIFO,
round-robin pending commands, then latest telemetry. Transport-busy/rejected calls
do not count as queued attempts and do not extend TTL. Flight telemetry is
0/1/4 Hz as supplied by mission policy; late periods never trigger catch-up bursts.

| Storage | Bound and overflow behavior |
| --- | --- |
| In-flight transport send | One; released by matching MAC completion |
| Pending ground command | One per flight peer |
| Incoming commands | Four; full -> BUSY reply, no admission |
| Command receipts | Eight; only unused/terminal entries may be reused |
| Outgoing replies | Eight; full -> drop/account, cached result remains available for retry |
| Incoming telemetry | Eight; full -> drop/account |
| Ground result events | Sixteen; full -> drop/account; not a durable audit log |
| Latest outgoing telemetry | One replaceable snapshot |

Receipt identity is retained during retries. A duplicate with different payload
bytes is malformed. A monotonic receive watermark rejects old commands after
terminal-cache eviction; unfinished receipts cannot be evicted. Out-of-order new
commands are stale. This is session-local duplicate suppression, not exactly-once
execution across resets. Message/token counters do not wrap to reused identities;
exhaustion requires reviewed session/service replacement. Diagnostic counters are
unsigned and may wrap; they are not persisted telemetry counters.

| Result byte | Meaning and transition |
| --- | --- |
| 1 RECEIVED | Command admitted to the receiver queue, not executed |
| 2 ACCEPTED | Application accepted it for processing; still nonterminal |
| 3 REJECTED | Application rejection from RECEIVED; terminal |
| 4 COMPLETED | Application completion from ACCEPTED; terminal |
| 5 FAILED | Application failure from ACCEPTED; terminal |
| 6 BUSY | Admission capacity unavailable; terminal to current ground tracking |
| 7 STALE | Command identity is not admissible; terminal |
| 8 UNKNOWN | Outcome uncertain, including local TTL expiry; terminal |

`reportCommandResult` permits RECEIVED -> ACCEPTED/REJECTED and ACCEPTED ->
COMPLETED/FAILED only. Reordered nonterminal results cannot regress progress.
After ACCEPTED, contradictory REJECTED/BUSY/STALE results are discarded.
MAC completion is separate from all these statuses. UNKNOWN never authorizes
blind re-execution under a fresh ID, especially for a relative rotation.

## ESP-NOW adapter lifecycle

`EspNowTransport::instance()` is a process-lifetime singleton; construction performs
no SDK setup. One task owns its public operations. Callbacks only validate and copy
into static FreeRTOS queues (eight RX records, one completion); they never decode
commands, execute mission logic, write serial records or operate mechanisms.

`start()` is explicit and absent from all current apps. A future caller must own
ESP-NOW exclusively and already have STA Wi-Fi running on a reviewed/assigned
channel with the correct country policy. Ground config requires two distinct flight
peers; a flight node one ground peer. MACs must be nonzero unicast and not self;
PMK and every LMK must be explicitly nonzero. Values come from private provisioning,
never examples in this repository. Numeric channel bounds alone do not authorize
transmission. Peer configuration requests encrypted unicast.

SDK startup failure returns an error and cleans up initialized ESP-NOW state where
applicable. An active instance refuses restart/reconfiguration. There is no stop,
channel-change or recovery API. A missing/mismatched send completion leaves the
adapter busy, avoiding completion misassociation; command TTL can still become
UNKNOWN. `receive` and `completion` are nonblocking pops that preserve output on
failure. Sequential SDK-double tests do not prove multicore timing, encryption,
RF range, coexistence or hardware behavior.

## Payload semantics and integration obligations

The user confirmed explicit validity flags, precise scaled-integer units, bounded
versioned stabilization/science extensions, and mission-only commands. Detailed
wire choices remain experimental engineering decisions. A payload always begins
with version and schema bytes; types must never be transmitted with `sizeof` or
native struct copies. Encoders/decoders preserve their output on failure and reject
unknown enums/tags/versions, invalid values, truncated data and trailing bytes.

Container payload is exactly 71 bytes. PQ is 177 bytes plus at most 43 extension
data bytes **shared** across stabilization and science. Each extension adds a
profile u16, version u8 and data-length u8 header already included in the base size.
An absent extension has all-zero header; nonempty data requires nonzero profile and
version. Actual profile IDs, fields and interpretations remain undefined.

Invalid numeric fields must store zero; a valid zero has its validity bit set.
Unknown bits are rejected. Mechanism state may assert only known bits. Latitude
and longitude validity travel together; coordinate bounds are +/-90 and +/-180
degrees. Valid pressure is positive and valid temperature at least -273.150 C.
Other integer limits describe representation, not approved flight operating ranges.
Acquisition decides validity from calibration, health and age. Quantization and
range checking from physical readings into integers are not implemented here.

Text is bounded printable ASCII excluding comma/whitespace/control, with explicit
byte count and zero padding. Command echo holds at most 24 bytes and uses uppercase
letters, digits, underscore, sign, dot or colon, beginning uppercase when nonempty.
GNSS time holds at most 32 receiver-supplied bytes without an inferred epoch/time
scale. Oversized text is rejected, never truncated. GNSS altitude retains its
receiver datum rather than being silently relabeled AGL.

Command codecs validate target, operation and argument syntax only. Future dispatch
must authenticate authority, enforce mission-state policy, gate SIM ENABLE before
ACTIVATE and SIMP only during active simulation, deduplicate identities and
reconcile resets. Decode success never arms, calibrates or moves hardware.
`SetUtc` establishes a separate UTC reference, not elapsed mission time. VIEW is
a nonzero relative rotation within +/-360000 milli-degrees, with right-hand sign
about nadir +Z; this is a wire bound, not tested mechanical travel. Physical camera
frame mapping remains a bench gate. RECORD/VIEW are team mission extensions, not
raw servo/GPIO access. `commandEcho` formats a token; the application publishes it
only after processing, not on MAC receipt.

## USB and CSV separation

The implemented host envelope is exactly `C:<payload>\n` or `P:<payload>\n`, with
1..250 ASCII bytes in 0x20..0x7e and total length payload+3. C++ returns nullopt and
Python raises ValueError for malformed complete frames. No incremental receive
buffer, timeout/resynchronization policy or large typed-telemetry serialization
exists yet. The legacy GUI/bridge baud, CR/LF and prefix mismatch remains unmodified.
A complete PQ record needs a separate worst-case readable-size budget.

Competition CSV export is **not implemented**. Its field order, identifier,
filename templates and CR terminator are recorded in `mission_payloads.json`; RF,
USB and CSV are different contracts. Wire scaled units retain precision for later
conversion. Final missing-value representation, rounding and extension-column
layout need explicit review. Invalid readings must not become apparent real zeros.
Guide voltage/current/solar precision discrepancies remain recorded in the JSON.
Elapsed time, packet count and command count are application-owned and not updated
or persisted by any codec. The proposed unique-processed-command count semantics
still need scoring-policy confirmation before flight integration.

## Open decisions and release gates

| Decision or work item | Responsible layer | Required acceptance before activation |
| --- | --- | --- |
| Full module SKU, flash/PSRAM, pins, power and antenna | Hardware profile | Board/BOM review and measured electrical/resource margins |
| Sensor acquisition and pressure/AGL reference | Drivers/acquisition | Bench calibration, timeout/fault tests, explicit unit/axis/sample-age checks |
| Detector flight thresholds | Mission integration | Representative held-out traces, false/missed events and latency against release timing |
| Mission time/counters/calibration persistence | Recovery | Power-loss/reset fault tests, wear budget and reconciliation policy |
| Peer sessions, keys and assigned channel | Provisioning/COM | Private provisioning, boot identity, no stale command acceptance after reset |
| SIM/command dispatcher and physical feedback | Application | State/authorization gating, actuator outcome and no duplicate non-idempotent action |
| Readable USB records and GUI migration | Bridge/gui-new | Bounded stream parser, resynchronization, per-node model and legacy migration tests |
| `INVALID_READING_EXPORT_POLICY` | CSV exporter | Organizer-compatible representation that does not falsify measurements |
| `GUIDE_PRECISION_CLARIFICATION` | CSV exporter | Reviewed voltage/current/solar conversion and rounding fixtures |
| `STABILIZATION_CSV_COLUMNS`, `SCIENCE_CSV_COLUMNS` | Experiment/export | Assigned versioned profiles, bounded payloads and agreed export order |
| Camera/processor and stabilization implementation | Vision experiment | Measured resolution, residual rotation, crop, latency, frame loss, memory, CPU and power |
| RF and deployed firmware qualification | Hardware integration | Actual boot, range, interference, fault recovery and flight-relevant timing |

These are explicit unfinished integration gates, not hidden defaults. No alternative
ESP32 is required by the present portable API alone; vision resource needs remain
benchmark-dependent. Do not use synthetic fixtures as startup fallbacks or store
production secrets in this repository.

## Verification and reproduction

Run from any checkout root, replacing the build directory if needed:

```sh
ROOT=$(git rev-parse --show-toplevel)
BUILD=/tmp/rsx-cansat-native
cmake -S "$ROOT" -B "$BUILD" -DCMAKE_BUILD_TYPE=Debug
cmake --build "$BUILD" --parallel 2
ctest --test-dir "$BUILD" --output-on-failure
"$BUILD/sensor_diagnostics" --fixture
python3 "$ROOT/tools/esp_idf_build.py" check
python3 "$ROOT/tools/esp_idf_build.py" build all --jobs 2
```

The SDK commands require the isolated installation described in the architecture
guide; they never install it, accept a serial port or run a flash/monitor action.
The root CMake build is host-only. The three SDK projects target `esp32`, use the
pinned v6.1 commit and compile the portable sources plus real platform adapters.
Every `app_main` only calls `report_inactive_startup` and returns. SDK console/boot
initialization still occurs; its output is not telemetry. Generated 2 MB/no-PSRAM
settings are SDK defaults, not measured or approved module capacities.

For an address/undefined-behavior sanitizer run on the verified Clang/GCC host:

```sh
SAN=/tmp/rsx-cansat-sanitized
cmake -S "$ROOT" -B "$SAN" -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_FLAGS='-fsanitize=address,undefined -fno-omit-frame-pointer' \
  -DCMAKE_EXE_LINKER_FLAGS='-fsanitize=address,undefined'
cmake --build "$SAN" --parallel 2
ctest --test-dir "$SAN" --output-on-failure
```

| Check group | Evidence and boundary |
| --- | --- |
| Sensor/core and diagnostic | Exact fixture conversions/failures; diagnostic refuses implicit fixture use |
| Mission/detector/replay | Deterministic state, thresholds, faults, replay parity and report provenance |
| Radio core and wire golden | Routing, sessions, queue limits, retry/status semantics, independent Python CRC/layout |
| Payload/native and golden | Typed roundtrips, 1566 native checks, six Python cases, signed/boundary/malformed input |
| SDK doubles | I2C atomic-output/timeout contract and sequential ESP-NOW lifecycle/queue contract |
| App/build policies | Inactive entry-point output, component boundaries, board target and SDK runner rejection cases |
| Real SDK builds | All three applications compile/link; Xtensa ELF and image validation, no device execution |

The root build registers 19 CTest suites at this specification baseline. Golden
vectors and schema offsets must change together; all existing tests must still
pass. Native and sanitizer success is not a hardware timing or flight-compliance
certificate. SDK warnings about upstream private includes or shell completion
must be distinguished from compiler/build failure. Unused library code can be
removed by the linker; successful compilation is not runtime integration.

Source delivery excludes `.idea`, `.local` SDK/tools/builds, generated host builds,
Python caches, recordings and private keys. The source transaction also retains a
modified archive, reconstructing patch, literal verification ledger and rollback
script outside the repository. Rollback operates on a separate target copy, needs
its sibling pristine baseline archive, preserves ignored SDK state and refuses to
clobber later conflicting edits. No remote push or device deployment is implicit
in committing this specification.

## Public code contract index

Public headers carry module descriptions plus ownership, units, failure and side-
effect contracts. Implementation comments explain validation, scheduling and
callback boundaries rather than duplicating individual statements.

| Header or entry point | Contract to preserve |
| --- | --- |
| `register_bus.hpp`, `sample.hpp` | Exact synchronous reads; absent values; acquisition-start timestamp |
| `ina236.hpp` | Borrowed bus, explicit configuration, probe-first, read-only status acknowledgment |
| `mission.hpp` | Qualified inputs, one-shot intents, latched feedback, volatile state |
| `flight_detection.hpp` | Sample-time continuity, phase-preserving faults, qualified-output forwarding |
| `radio_protocol.hpp` | Explicit layout and CRC; role/session/correlation rules |
| `radio_transport.hpp` | Nonblocking copy-on-submit, token completion and queue ownership |
| `node_com.hpp` | Borrowed transport, one owner, bounded tick, progress versus execution |
| `mission_payloads.hpp` | Units, canonical absence/padding, exact lengths, target checks, no dispatch |
| `serial_frame.hpp` and Python `serial_frame.py` | Complete frames only, strict ASCII and matching errors |
| `esp_idf_register_bus.hpp` | Borrowed synchronous handle, finite timeout, unchanged output on failure |
| `esp_now_transport.hpp` | Explicit singleton startup, single caller owner, callback isolation |
| `app_boot.hpp` and role `app_main.cpp` | Inactive logging only, not a physical safety interlock |
| `flight_replay.py` and backend | Validated reports using native detector, no implicit optimization |
| `esp_idf_build.py` | Provenance validation and child-process build only, no installer or flashing |

## Wire field reference

These tables restate the JSON contracts for code review. JSON offsets and independent
golden tests remain the byte-layout checks; values below are not native struct offsets.

### Envelope header

| Field | Offset | Bytes |
| --- | --- | --- |
| `magic_CS` | 0 | 2 |
| `version` | 2 | 1 |
| `kind` | 3 | 1 |
| `team` | 4 | 2 |
| `source` | 6 | 1 |
| `destination` | 7 | 1 |
| `sender_session` | 8 | 4 |
| `destination_session` | 12 | 4 |
| `message_id` | 16 | 4 |
| `correlation_id` | 20 | 4 |
| `payload_length` | 24 | 2 |

### Common telemetry prefix

| Field | Offset | Bytes | Encoding and unit | Validity bit |
| --- | --- | --- | --- | --- |
| `version` | 0 | 1 | u8 | — |
| `schema` | 1 | 1 | u8 | — |
| `mission_time_ms` | 2 | 8 | u64le / ms | — |
| `packet_count` | 10 | 4 | u32le | — |
| `command_count` | 14 | 4 | u32le | — |
| `mode` | 18 | 1 | u8 ASCII F/S | — |
| `valid` | 19 | 4 | u32le bitmask | — |
| `altitude_mm` | 23 | 4 | i32le / mm AGL | 0 |
| `pressure_pa` | 27 | 4 | u32le / Pa | 1 |
| `temperature_milli_c` | 31 | 4 | i32le / 0.001 degC | 2 |
| `battery_mv` | 35 | 4 | u32le / mV | 3 |
| `battery_current_ma` | 39 | 4 | i32le / mA; signed | 4 |
| `mechanism_state` | 43 | 1 | u8 bitmask | — |
| `mechanism_known` | 44 | 1 | u8 bitmask | — |
| `command_echo_length` | 45 | 1 | u8; 0..24 | — |
| `command_echo` | 46 | 24 | ASCII; zero-padded | — |

### Container tail

| Field | Offset | Bytes | Encoding and unit | Validity bit |
| --- | --- | --- | --- | --- |
| `state` | 70 | 1 | u8 | — |

### PocketQube fixed tail

| Field | Offset | Bytes | Encoding and unit | Validity bit |
| --- | --- | --- | --- | --- |
| `gyro_mdeg_s_x` | 70 | 4 | i32le / 0.001 deg/s | 5 |
| `gyro_mdeg_s_y` | 74 | 4 | i32le / 0.001 deg/s | 6 |
| `gyro_mdeg_s_z` | 78 | 4 | i32le / 0.001 deg/s | 7 |
| `accel_mm_s2_x` | 82 | 4 | i32le / 0.001 m/s^2 | 8 |
| `accel_mm_s2_y` | 86 | 4 | i32le / 0.001 m/s^2 | 9 |
| `accel_mm_s2_z` | 90 | 4 | i32le / 0.001 m/s^2 | 10 |
| `mag_milligauss_x` | 94 | 4 | i32le / mG | 11 |
| `mag_milligauss_y` | 98 | 4 | i32le / mG | 12 |
| `mag_milligauss_z` | 102 | 4 | i32le / mG | 13 |
| `latitude_nanodeg` | 106 | 8 | i64le / 1e-9 degree | 14 |
| `longitude_nanodeg` | 114 | 8 | i64le / 1e-9 degree | 15 |
| `gnss_altitude_mm` | 122 | 4 | i32le / mm in receiver-provided altitude datum | 16 |
| `gnss_satellites` | 126 | 2 | u16le / count | 17 |
| `gnss_time_length` | 128 | 1 | u8; 0..32 | — |
| `gnss_time` | 129 | 32 | ASCII; zero-padded / receiver-provided text; no conversion inferred | 18 |
| `solar_mv_1` | 161 | 4 | u32le / mV | 19 |
| `solar_mv_2` | 165 | 4 | u32le / mV | 20 |

### Command wire reference

All command arguments begin at offset 3 after version=1, schema=3 and opcode.
C means Container and P means PocketQube. Unlisted opcodes/enum values are invalid.

| Opcode | Name and C++ type | Target | Payload bytes | Argument |
| --- | --- | --- | --- | --- |
| 1 | CAL / `Calibrate` | C | 3 | None |
| 2 | ARM / `Arm` | C | 3 | None |
| 3 | ST / `SetUtc` | C | 11 | unix_ms: u64le |
| 4 | SIM / `Simulation` | C/P | 4 | action: u8 {"enable":1,"activate":2,"disable":3} |
| 5 | SIMP / `SimulatedPressure` | C/P | 7 | pa: u32le |
| 6 | MEC / `Deploy` | C/P | 4 | mechanism: u8 {"container_release":1,"solar_1":2,"solar_2":3,"boom":4} |
| 7 | RECORD / `Recording` | P | 4 | action: u8 {"start":1,"stop":2} |
| 8 | VIEW / `RotateView` | P | 7 | delta_mdeg: i32le |

MEC targets: C accepts container_release=1 only; P accepts solar_1=2, solar_2=3,
boom=4 only. SIMP requires positive Pa. ST uses u64 Unix milliseconds. VIEW uses
i32 milli-degrees, excludes zero and is bounded to [-360000,360000].

Container wire state values: LAUNCH_PAD=0, ASCENT=1, APOGEE=2, PQ_RELEASE=3.
Container mechanism bits are release mechanism=0 and PQ clear=1. PocketQube
mechanism bits are solar panel 1=0, solar panel 2=1, boom=2 and recording=3.
Each state bit must be accompanied by its corresponding known bit.

PQ extension blocks start at offset 169; science follows the variable-size
stabilization block. Each header is profile u16le, version u8, length u8, followed
by that many opaque bytes. Two empty headers occupy the final 8 base bytes.

### CSV column reference

Export remains unimplemented. The exact mapped order, including unresolved experiment
columns, is recorded here for the future exporter rather than inferred from wire order.

**Container**

```text
ID,MISSION_TIME,PACKET_COUNT,COMMAND_COUNT,MODE,ALTITUDE,PRESSURE,TEMPERATURE,BATT_V,BATT_I,MECH_STATE,STATE,CMD_ECHO
```

**PocketQube**

```text
ID,MODE,MISSION_TIME,PACKET_COUNT,COMMAND_COUNT,ALTITUDE,TEMPERATURE,PRESSURE,VOLTAGE,CURRENT,GNSS_TIME,GNSS_ALTITUDE,GNSS_LATITUDE,GNSS_LONGITUDE,GNSS_SATS,ROT_RATE_X,ROT_RATE_Y,ROT_RATE_Z,ACCEL_X,ACCEL_Y,ACCEL_Z,MAG_X,MAG_Y,MAG_Z,SOLAR_1,SOLAR_2,MECH_STATE,CMD_ECHO,IMAGE_STABILIZATION,SCIENCE_EXP
```

Filename templates are `Flight_<TEAM_ID>C.csv` and `Flight_<TEAM_ID>P.csv`; records
use a CR terminator. Resolve the open export decisions before emitting competition files.
