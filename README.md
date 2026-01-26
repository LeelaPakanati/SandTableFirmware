# Sisyphus Table Controller

ESP32 firmware for a Sisyphus-style polar-coordinate sand table. It drives Theta/Rho steppers, streams pattern files from SD, serves a Web UI/API, supports OTA updates, and provides motion tuning and diagnostics.

## Features
- Dual-core task split: real-time motion on Core 1, UI/network/file streaming on Core 0.
- S-curve motion planning with synchronized Theta/Rho timing and lookahead step generation.
- Web UI with live position visualization, playlist control, and tuning controls.
- SD card pattern storage, uploads, and optional PNG previews.
- OTA firmware updates and runtime telemetry.

## Hardware
- ESP32 dev board
- 2x (or 3x) TMC2209 stepper drivers (UART)
- 2x stepper motors (Theta + Rho)
- SD card module
- 12V/24V PSU sized for motors and LEDs

## Quick Start
1. Open the project in PlatformIO.
2. Adjust WiFi credentials in `src/main.cpp` if desired.
3. Build and flash:
   ```bash
   pio run -t upload
   ```
4. Upload filesystem assets if needed:
   ```bash
   pio run -t uploadfs
   ```
5. Connect to the device IP and open the Web UI.

## Configuration
Key settings in `src/main.cpp`:
- `AP_SSID`, `AP_PWD`: WiFi AP fallback
- Static IP defaults (`100.76.149.200`)
- OTA hostname/password

Pin defaults live in `lib/PolarControl/src/PolarControl.hpp`:
- Rho Step: 33
- Rho Dir: 25
- Theta Step: 32
- Theta Dir: 22
- UART RX: 27
- UART TX: 26

## Web UI and API
Core endpoints (see `lib/WebServer/src/SisyphusWebServer.cpp`):
- `GET /` UI
- `GET /api/status` current state and telemetry
- `GET /api/stream` SSE position stream
- `POST /api/pattern/start` start a pattern
- `POST /api/pattern/stop|pause|resume` control playback
- `GET /api/files` list files
- `POST /api/files/upload` upload `.thr` and optional `.png`
- `POST /api/files/delete` delete a pattern
- `GET|POST /api/led/brightness` LED control
- `GET|POST /api/speed` speed control
- `GET /api/tuning/*` driver and motion tuning

## Pattern Format (.thr)
Text file of polar coordinates in radians and normalized radius:
```
# comments with # or //
0.0 0.5
0.1 0.5
...
6.28 0.5
```
- `theta`: radians
- `rho`: normalized 0.0 to 1.0 (scaled by max radius)
- Separators: space, comma, or tab

File parsing ignores empty/comment lines. Lines longer than 127 characters are skipped and reported to the error log.

## File Storage Layout
- Preferred: `/patterns/name/name.thr` and `/patterns/name/name.png`
- Fallback: `/patterns/name.thr` and `/patterns/name.png`

The Web UI accepts `.thr` plus optional `.png` uploads with the same base name.

## Motion Planning
- S-curve profiles per axis with synchronized segment durations.
- Lookahead step generation into a queue to avoid underruns.
- PREPARING state is used when a file is loading and the first coord has not arrived yet.

## Logging and Diagnostics
- Serial logs include queue depth, underruns, timing stats, and state changes.
- Error log is stored in memory and exposed via `GET /api/errors`.

## Tests
Native motion planner tests:
```bash
pio run -e native
./run_all_tests.sh
```

## Security and Operational Notes
- Web endpoints are unauthenticated and CORS is open. Use trusted networks.
- WiFi and OTA credentials are hard-coded by default; update before deployment.

## Directory Map
```
/        - firmware root
lib/     - PolarControl, WebServer, SDCard, Playlist, LEDController
src/     - main firmware entry
scripts/ - helper scripts for upload/status
test/    - native motion planner tests and pattern files
```
