# Architecture Documentation

This document outlines the software architecture of the Sisyphus Table firmware.

## System Overview

The system is built on the ESP32 platform using the Arduino framework within PlatformIO. It utilizes the dual-core architecture of the ESP32 to separate real-time motion control from user interface and networking tasks.

### Core Allocation

-   **Core 0 (System & I/O):** Handles WiFi, Web Server, OTA updates, and **SD Card File Reading**.
-   **Core 1 (Real-time Control):** Dedicated to the `MotorTask`, ensuring smooth, jitter-free stepper motor pulses and motion planning.

## Key Components

### 1. Main Controller (`src/main.cpp`)
-   **Entry Point:** Initializes hardware, sets up WiFi, and creates FreeRTOS tasks.
-   **Tasks:**
    -   `motorTask`: Runs the motion control loop on Core 1.
    -   `webTask`: Handles web server requests and OTA on Core 0.

### 2. PolarControl (`lib/PolarControl`)
This library manages the kinematics and motor driving.
-   **`PolarControl` Class:** The central interface for movement. Uses `std::unique_ptr` for safe management of position generators.
-   **`MotionPlanner`:** Handles trajectory generation using independent 7-phase S-curves for Theta and Rho axes. Synchronizes axis movement by time-stretching the faster axis to match the slower one. Uses a lookahead buffer and a high-frequency ISR for precise step generation.
-   **`PolarUtils`:** TODO
-   **Kinematics:** 
    -   Positive Theta movement denotes clockwise rotation.
    -   Theta is normalized between patterns to the nearest equivalent value to prevent unnecessary "unwinding" rotations.

### 3. WebServer (`lib/WebServer`)
Provides the user interface and API.
-   **`SisyphusWebServer` Class:** Wraps `ESPAsyncWebServer`.
-   **`JsonHelpers`:** Extracted logic for building standardized JSON responses for status, file lists, and system info.
-   **API Endpoints:**
    -   `/api/status`: Returns current state (position, playing file, etc.).
    -   `/api/upload`: Handles `.thr` file and optional preview image uploads.
    -   `/api/playlist/*`: Manages the playlist.
    -   `/api/tuning/*`: Updates motor settings in real-time.

### 4. SDCard (`lib/SDCard`)
Handles file I/O operations.
-   Stores `.thr` pattern files in a structured `/patterns/name/name.thr` or flat `/patterns/name.thr` format.
-   Reads files in chunks to stream to the motion planner.

### 5. Playlist (`lib/Playlist`)
-   Manages queues of patterns.
-   Handles looping, shuffling, and transitions.
-   Supports automatic clearing patterns between tracks.

### 6. LEDController (`lib/LEDController`)
-   Manages the LED strip lighting via PWM.

## Data Flow

1.  **Pattern Loading (Async):**
    -   User selects a file via Web UI.
    -   `SisyphusWebServer` calls `loadAndRunFile`.
    -   A `CMD_LOAD` message is sent to the `FileReadTask` queue (Core 0).
    -   `FileReadTask` opens the file and begins streaming coordinates into `m_coordQueue`.

2.  **Motion Execution:**
    -   `MotorTask` (Core 1) continuously polls `PolarControl::processNextMove()`.
    -   `PolarControl` consumes coordinates from `m_coordQueue` (fast operation) and feeds them to `MotionPlanner`.
    -   `MotionPlanner` calculates S-curve trajectories and generates step pulses via a 10kHz ISR.

3.  **Status Feedback:**
    -   `MotorTask` updates positions.
    -   `SisyphusWebServer` broadcasts normalized Cartesian coordinates via SSE (Server-Sent Events) at ~15Hz for the real-time UI viewer.

## Directory Structure

```
/
├── lib/
│   ├── PolarControl/    # Motion logic, Kinematics, & Utilities
│   ├── WebServer/       # UI, API, & JSON Helpers
│   ├── SDCard/          # Storage handling
│   ├── Playlist/        # Sequence management
│   └── LEDController/   # Lighting
├── src/
│   └── main.cpp         # Entry point & Task setup
├── platformio.ini       # Build configuration (C++17)
└── ARCHITECTURE.md      # This file
```
