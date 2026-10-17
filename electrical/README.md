## Layout

| Folder | Contents |
|---|---|
| `lib/` | Shared `RSX_Aerial` KiCad library: symbols, footprints (`RSX_Aerial.pretty/`), STEP models (`3d/`) |
| `boards/` | One folder per KiCad project |
| `blocks/` | Reusable schematic sheets (INA236, MAX-M10S, ESP32 ) |
| `test/` | Antenna range tests, ESP-NOW Long Range tests |
| `docs/` | Documentation |

## Opening a project

1. Install KiCad 10.
2. Add the shared library once, in **Preferences → Manage Symbol Libraries** and **Manage Footprint Libraries**:
   - Symbols: `electrical/lib/RSX_Aerial.kicad_sym`
   - Footprints: `electrical/lib/RSX_Aerial.pretty`
3. Open the `.kicad_pro` file in the board's folder under `boards/`.

