# RSX-CANSAT-2027

This repository contains all files created by the Aerial Team of Robotics for Space eXploration (RSX) at the University of Toronto, supporting our entry in the 2027 CanSat Competition.

![rsx_logo](https://github.com/adam-kabbara/RSX-CANSAT-2026/raw/main/competition_documentation/team_assets/rsx_logo_light_mode.png)
![uoft_engineering_logo](https://github.com/adam-kabbara/RSX-CANSAT-2026/raw/main/competition_documentation/team_assets/uoft_engineering_logo_light_mode.png)

<p align="center">
  <img src="https://github.com/adam-kabbara/RSX-CANSAT-2026/raw/main/competition_documentation/team_assets/cansat_aas_logo.png" alt="cansat_aas_logo">
</p>

## Competition Team Members:
| Name | Role |
|---|---|
| Yechun(Vincent) Jia | Software Team Lead |
| Yulin Lin | Electrical Team Lead |
| Nathan Xia | Mechanical Team Co-Lead |
| Jingshu Zhu | Mechanical Team Co-Lead |
| Parth Bhasin | Software Team Junior Lead |
| Remi Zhang | Software Team Member |
| More... |  |

## ESP32 firmware: one PlatformIO project

Open the repository root in VS Code with **PlatformIO IDE** and **C/C++**
installed. The root PlatformIO configuration is the entry point for all three
boot-only firmware environments:

| Environment | Role | Source directory, relative to the repository root |
|---|---|---|
| `ground_station` (default) | Ground-station ESP32 | `software/firmware/ground/src` |
| `pq` | PocketQube ESP32 | `software/firmware/PQ/src` |
| `container` | Container ESP32 | `software/firmware/container/src` |

All environments use the pinned `platformio/espressif32@8.0.0` platform,
Arduino-ESP32 3.3.12, and the `esp32dev` profile (4 MB flash, no PSRAM).
This covers our ESP32-DevKitC V4 boards with ESP32-WROOM-32E or
ESP32-WROOM-32UE modules. The UE variant uses an external antenna connector;
connect the appropriate antenna before future radio testing.

The role builds share `software/firmware_lib` through a local symlink dependency;
editing that library does not require copying it into each project. Role-specific
source filters and include paths keep the three application entry points separate.
The existing library manifest and its dependency declarations are retained.

### Build and choose an environment

Run commands from the repository root. In VS Code, use the PlatformIO terminal
and **Project Tasks** to select the desired environment. Outside that terminal,
`pio` may be available as `~/.platformio/penv/bin/pio` on macOS/Linux or through
`platformio` on your PATH. The first build needs network access to install the
pinned platform, framework, toolchain, and library dependencies.

```sh
# Default environment: ground_station only.
pio run

# Build an individual role.
pio run -e ground_station
pio run -e pq
pio run -e container

# Build all three roles explicitly; this does not upload anything.
pio run -e ground_station -e pq -e container
```

### Upload and monitor: explicit board selection

**Uploading replaces the selected board's existing application.** Label the board
with its role first, identify its port with `pio device list`, and replace `PORT`
below with that exact port (for example, a `/dev/cu.*` port on macOS or `COM4` on
Windows). Ports are deliberately not committed to the configuration.

```sh
pio device list

# Use exactly one role per upload; choose the command matching the labelled board.
pio run -e ground_station -t upload --upload-port PORT
pio run -e pq -t upload --upload-port PORT
pio run -e container -t upload --upload-port PORT

# Open the matching serial port; all roles use 115200 baud.
pio device monitor --port PORT --baud 115200
```

After opening the monitor, press **EN/reset** to see the one-time application
banner. For a ground-station board it has this form (the MAC is illustrative):

```text
ROLE=GROUND_STATION
MAC=AA:BB:CC:DD:EE:FF
MODE=BOOT_ONLY
```

The other role labels are `PQ` and `CONTAINER`. If reading the station MAC fails,
the middle line is `MAC=UNAVAILABLE (error=ERROR_CODE)`; startup still completes.
The MAC is read through `esp_read_mac(..., ESP_MAC_WIFI_STA)` without starting
Wi-Fi. Startup has only a bounded 250 ms settling delay; each idle loop yields
for 10 ms. There is no terminal-connection wait or recurring application output.
ESP32 ROM/bootloader messages may appear before this application banner.

### Inactive hardware boundary and preserved prototypes

These are **boot-only skeletons**, not deployed mission or communication firmware.
They do not initialize Wi-Fi/ESP-NOW, access sensors, drive actuators, or connect
to the desktop GUI. No peer MACs, radio channels, or actuator pins are configured.
The unresolved protocol, sensor, and mission interfaces are intentionally not used.

The original radio-bridge prototype remains unchanged under
`ground_station/gs_radio_firmware`; it is not part of any root-project build.
Its old empty PlatformIO configuration and the two empty per-flight-role
configurations have been removed. Existing experimental PlatformIO projects,
the shared library's historical standalone configuration, and both Python GUI
copies are unchanged. Incomplete `software/common` telemetry code is also excluded.
The old GUI/bridge baud-rate and framing mismatches remain unresolved.

### Host checks and physical acceptance

```sh
python3 -m unittest discover -s software/tests -p 'test_boot_skeleton.py' -v
```

These checks compile the real startup helper and all three real entry points
against small host fakes using a C++17 compiler (`c++`, or the `CXX` executable).
They verify role isolation, boot banners, MAC-read errors, and yielding idle loops.
They are not hardware tests. Physical acceptance requires an explicitly selected
board/port, a successful upload, and observing that board's role and actual MAC
at 115200 baud. Building or running host checks never flashes a board.
