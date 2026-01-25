# MotionPlanner Redesign

## Status Update (2026-01-23)
- **Phase 1 (S-Curve Validation):** COMPLETED
- **Phase 2 (MotionPlanner Implementation):** COMPLETED
- **Phase 3 (Verification):** COMPLETED
- **Phase 4 (Optimization - Pipeline Generation):** COMPLETED (Fixed underruns)

## Optimization: Pipeline Generation Architecture

To resolve step queue underruns caused by main loop latency (e.g., SD card reads, WiFi), the MotionPlanner was refactored to decouple **Step Generation** from **Segment Execution**.

### Old Architecture (Synchronous)
- **Generation:** `fillStepQueue()` only generated steps for the *currently executing* segment (`m_segmentTail`).
- **Constraint:** It could not generate steps for the *next* segment until the current one finished execution.
- **Failure Mode:** At segment boundaries, if the main loop was blocked, the step queue would drain completely (underrun) before the planner could switch to the next segment and start generating.

### New Architecture (Lookahead Pipeline)
- **Decoupled Pointers:**
    - `m_segmentTail`: The segment currently being executed by the ISR (motors moving).
    - `m_genSegmentIdx`: The segment currently being generated into the buffer.
- **Lookahead:** The `process()` loop now generates steps for future segments up to a 200ms horizon (`STEP_QUEUE_HORIZON_US`), crossing segment boundaries seamlessly.
- **Deterministic Timing:** Future segment start times are calculated deterministically (`PrevStart + PrevDuration`), allowing steps to be generated with precise timestamps long before execution begins.
- **Buffer:** `STEP_QUEUE_SIZE` increased to 2048 events to accommodate the 200ms horizon at full speed.

### Performance Profiling
Real-time diagnostics are now available on the Serial console:
- **MaxProc:** Maximum time (us) spent in the `process()` loop.
- **MaxInt:** Maximum interval (us) between `process()` calls. (Critical: Must stay < 200ms).
- **AvgGen:** Average time (us) to generate a batch of steps.
- **Q:** Queue depth. Should remain consistently high (~200ms worth of steps).
- **UR:** Underrun count. Should remain 0.

## Core Design (Retained)

### Independent Axis Profiles
- Theta and Rho axes are treated independently.
- Each axis gets its own S-curve profile.
- Durations are synchronized by stretching the faster axis.

### Data Structures
```cpp
struct Segment {
    // ... target info ...
    AxisProfile theta, rho;
    bool executing;          // Owned by ISR/Execution logic
    bool generationComplete; // Owned by Pipeline Generator
};
```

### Locking & Safety
- **Mutex Protection:** `process()`, `setSpeed()`, and settings updates are protected by a recursive mutex to prevent race conditions.
- **Atomic Operations:** Step queue indices and position counters use atomic operations or careful single-writer/single-reader patterns.

## Performance Optimization Summary (January 2026)

The following optimizations were implemented to resolve step queue underruns and ensure smooth motion at high speeds (up to 60k loops/sec).

### 1. Streaming Flash Cache (The "Stutter Killer")
**Problem:** SD card read latency spikes (>100ms) were blocking the motor task, causing the step queue to drain. Full-file copies caused long startup delays for large files and couldn't handle files larger than flash.
**Solution:** Implemented `StreamingFileCache` - a chunked caching system using ping-pong buffers on flash.

**How it works:**
1. Opens source file on SD card
2. Copies first 16KB chunk to `/cache0.thr` on LittleFS
3. Playback starts immediately (fast startup)
4. While reading from chunk 0, pre-copies next chunk to `/cache1.thr`
5. When chunk 0 exhausted, switches to chunk 1 and starts filling chunk 0
6. Repeats indefinitely - handles arbitrarily large files

**Benefits:**
- Fast startup: Only wait for first 16KB chunk, not entire file
- Large file support: Files bigger than flash work via chunk recycling
- Deterministic reads: All reads come from flash, not SD
- Background loading: `maintainCache()` pre-loads next chunk while reading

**Code:**
- `StreamingFileCache.hpp`: New class handling chunked SD→Flash→Reader pipeline
- `PolarControl::fileReadTask`: Uses `StreamingFileCache` instead of full copy
- `platformio.ini`: Configured `board_build.filesystem = littlefs`

### 2. Pipeline Generation Architecture
**Problem:** Step generation was synchronous with execution, causing gaps at segment boundaries if the CPU was busy.
**Solution:** Decoupled generation from execution with a lookahead buffer.
**Code:**
- `MotionPlanner::process`: Loops to generate steps for *future* segments up to `STEP_QUEUE_HORIZON_US` (200ms).
- `MotionPlanner::fillStepQueue`: Updates to accept arbitrary segment indices and start times.
- `MotionPlanner::m_genSegmentIdx`: Tracks generation progress independently of `m_segmentTail`.

### 3. Buffer Expansion
**Problem:** Short segments exhausted the buffer faster than it could be refilled.
**Solution:** Increased buffer sizes to ride out system latency.
**Code:**
- `MotionPlanner.hpp`: `SEGMENT_BUFFER_SIZE` increased to **32** (was 8).
- `MotionPlanner.hpp`: `STEP_QUEUE_SIZE` increased to **2048** (was 512).

### 4. Robust Line Parsing
**Problem:** Unsafe string construction caused "Invalid line format" errors and garbage data.
**Solution:** Replaced unsafe `String` constructor with manual byte-by-byte appending.
**Code:**
- `PosGen.hpp`: Rewrote `readBufferedLine` to safely construct strings from the raw read buffer.

### 5. Profiling Infrastructure
**Problem:** Invisible bottlenecks.
**Solution:** Added real-time telemetry to the serial output.
**Code:**
- `Profiler.hpp`: Lightweight atomic profiler class.
- `MotionPlanner::process`: Instruments `MaxProc` (process time), `MaxInt` (interval), `AvgGen` (generation cost).
- `main.cpp`: Prints diagnostics (`Q`, `UR`, `Loop/s`) every second.

### 6. User Experience: PREPARING State
**Problem:** Copying large files to Flash takes a few seconds, during which the UI would show "IDLE" or "RUNNING" without motion.
**Solution:** Added a `PREPARING` state to indicate the file is being cached.
**Code:**
- `PolarControl.hpp`: Added `PREPARING` to `State_t`.
- `PolarControl.cpp`: Transitions to `PREPARING` during load, and to `RUNNING` only when the first coordinate is read from Flash.
- `JsonHelpers.hpp`: Exposes "PREPARING" string to the Web UI.

## Discarded Approaches

### Starvation Pausing (Reverted)
**Concept:** If the step queue empties, pause the motion timer (`esp_timer_stop`) and resume when data arrives.
**Outcome:** Successfully prevented "Underrun" errors but caused visible stuttering during SD card latency spikes.
**Decision:** Reverted in favor of **Flash Caching**. It is better to wait upfront (Preparing) and run smoothly than to stutter during execution.
