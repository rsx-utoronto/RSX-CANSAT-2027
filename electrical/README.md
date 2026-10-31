# Electrical

| Folder | Contents |
|---|---|
| `lib/` | The Aerial KiCad library: symbols, footprints (`RSX_Aerial.pretty/`), 3D models (`3d/`) |
| `boards/` | One folder per board, each holding its KiCad project |
| `blocks/` | Reusable schematic sheets, one circuit each (INA236 current sensor, IMU) |
| `test/` | Sensor and radio tests (PlatformIO projects) |
| `docs/` | Documentation |

## 1. Install KiCad 10 first

Everyone uses KiCad 10. Install it from [kicad.org/download](https://www.kicad.org/download/) **before opening any file in this repo**.

- The boards here are already in KiCad 10 format, and KiCad 9 can't open them.

## 2. Use the Aerial library

We now have our own library in `lib/`. Every part comes from the Aerial library or from KiCad's built-in libraries. **Don't use the old RSX library any more.**

**First, remove the old RSX library:** in **Preferences → Manage Symbol Libraries** and **Manage Footprint Libraries**, delete every `rsx_*` entry. Keep KiCad's own libraries.

**Then load the Aerial library in one of two ways:**

### Option A: add it to your global libraries (recommended)

1. **Preferences → Manage Symbol Libraries** → **Global Libraries** → click the folder icon, go to `electrical/lib/` in your copy of this repo, and select all eight `RSX_Aerial*.kicad_sym` files. 
2. **Preferences → Manage Footprint Libraries** → **Global Libraries** → add `RSX_Aerial.pretty` and `RSX_Aerial_Graphics.pretty`.

You should end up with KiCad's libraries plus the `RSX_Aerial…` entries.

### Option B: use the board's library tables (no setup)

If a board's folder has a `sym-lib-table` and `fp-lib-table` (copied in when the board was made, see [Starting a new board](#starting-a-new-board)), opening its `.kicad_pro` loads the Aerial library automatically, with nothing to set up in KiCad.

However, use option A if you edit blocks(.sch) and doesn't have .pro open.

## 3. Pull before you start

Run `git pull` every time before you open KiCad. The library changes whenever someone adds a part. On an old copy you'll be missing their parts, and your push can clash with their changes.

## Opening a board or block

- **Board:** open the `.kicad_pro` in `boards/<board-name>/`.
- **Block:** `blocks/` isn't a KiCad project. Open the `.kicad_sch` directly in the Schematic Editor (**File → Open**). This needs option A.

## Starting a new board

1. Create the KiCad project directly in `boards/<board-name>/`, not in a subfolder.
2. Copy `lib/templates/sym-lib-table` and `lib/templates/fp-lib-table` into that folder, next to the `.kicad_pro`.

Do this even if you use option A, so the board also works for people using option B. 

## Need a part that isn't in the library?

1. Find the part on Digi-Key and download its KiCad files from the product page (look for **EDA Models**), or from SnapEDA.
2. Send the files and the Digi-Key link to @persfi in the team chat. They will add it to the library and tell you when it's pushed.
3. `git pull`, then use the part.

**Don't change an existing part.** Add to the library only, never change it.
