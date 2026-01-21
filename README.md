# Sisyphus Table Controller

This project is an ESP32-based firmware for controlling a [Sisyphus Table](https://sisyphus-industries.com/) (or similar polar-coordinate sand art tables). It drives two motors (Theta and Rho) to move a magnet through sand, creating intricate patterns.

## Features

- **Polar Motion Control:** Precise control of Theta (angle) and Rho (radius) using stepper motors.
- **Web Interface:** Full-featured responsive Web UI for control and configuration.
- **WiFi Connectivity:** Connects to existing WiFi or creates its own Access Point (`SisyphusTable`).
- **SD Card Support:** Reads pattern files (`.thr`) from an SD card.
- **Playlist Management:** Create, save, and loop playlists of patterns.
- **LED Control:** Control brightness and potentially color of LED lighting.
- **OTA Updates:** Upload new firmware wirelessly.
- **Motion Tuning:** Adjust acceleration, velocity, and jerk settings via the Web UI.

## Hardware Requirements

- **Microcontroller:** ESP32 Dev Board (or compatible).
- **Stepper Drivers:** 2x or 3x TMC2209 (UART control supported).
    - Theta Motor
    - Rho Motor
    - (Optional) Rho Counter-balance or LED dedicated driver?
- **Motors:** 2x Stepper Motors.
- **Storage:** SD Card Module.
- **Power Supply:** Sufficient 12V/24V PSU for motors and LEDs.

## Getting Started

### Prerequisites

- [PlatformIO](https://platformio.org/) (VS Code Extension recommended).
- Git.

### Installation

1.  Clone this repository:
    ```bash
    git clone https://github.com/yourusername/SisyphusTable.git
    cd SisyphusTable
    ```
2.  Open the project in PlatformIO.
3.  Configure your WiFi credentials in `src/main.cpp` (or use the WiFiManager portal on first boot):
    ```cpp
    #define AP_SSID "SisyphusTable"
    #define AP_PWD "sandpatterns"
    ```
4.  Build and Upload:
    ```bash
    pio run -t upload
    ```
5.  Upload Filesystem (if needed for Web UI assets):
    ```bash
    pio run -t uploadfs
    ```

### Pin Configuration

Default pinout (check `lib/PolarControl/src/PolarControl.hpp`):

| Function | Pin |
|----------|-----|
| Rho Step | 33  |
| Rho Dir  | 25  |
| Theta Step| 32 |
| Theta Dir| 22  |
| UART RX  | 27  |
| UART TX  | 26  |

## Usage

1.  **Power on** the table.
2.  Connect to the WiFi network `SisyphusTable` (password: `sandpatterns`) or the network you configured.
3.  Navigate to `http://100.76.149.200` (static IP) or the device's assigned IP.
4.  **Web Interface:**
    -   **Upload:** Upload `.thr` pattern files.
    -   **Play:** Select a pattern to start drawing.
    -   **Playlist:** Queue multiple patterns.
    -   **Settings:** Tune motor parameters and LED brightness.

## Pattern Format (.thr)

The system accepts `.thr` text files. Each line represents a point in polar coordinates:

```text
theta rho
```

-   `theta`: Angle in radians. The system automatically normalizes theta to the range `[-PI, PI]` between patterns to ensure the most efficient movement to the next starting point.
-   `rho`: Normalized radius (0.0 to 1.0). 0.0 is center, 1.0 is edge.

Example:
```text
0.0 0.5
0.1 0.5
...
6.28 0.5
```

## UI & Visualization

The web interface features a real-time position viewer. 
- **PNG Overlays:** To provide a preview for a pattern, upload a square PNG (recommended 800x800) alongside the `.thr` file.
- **Coordinates:** The viewer maps the table's polar coordinates to a 1:1 Cartesian space.


## License

[MIT](LICENSE)
