# Ground station

- `gui-2026/`: preserved 2026 ground station GUI.
- `gui-new/`: identical starting copy for new development.

Both copies came from `RSX-CANSAT-2026-OB/Software/ground_station_source`
at commit `d729e36950e02e5e3ee17dd2069e5615ee320b05`, including the local
`bin/launch.sh` and two `tests/test_*.py` files. The source repository is unchanged.
Virtual environments, generated telemetry/logs, downloaded map tiles, and local
agent working files are excluded.

## Run (PowerShell)

From either GUI directory, using Python 3.13:

```powershell
py -3.13 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt -r requirements.in
.\.venv\Scripts\python.exe code/main.py --omit-screen-resolution
```

Both requirement files are needed because the original lock file omits `pygame`.
Display, map, and graph settings are in `code/config.toml`. Offline map imagery
must be supplied separately under `code/media/tiles/`; `code/tiles.py` contains
the existing tile download utility. Session output is written to `code/output/`.

Run the existing checks from either GUI directory:

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests
```
