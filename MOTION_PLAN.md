# MotionPlanner Redesign Plan

## Status Update (2026-01-23)
- **Phase 1 (S-Curve Validation):** COMPLETED
- **Phase 2 (MotionPlanner Implementation):** COMPLETED
- **Phase 3 (Verification):** COMPLETED. All 24 pattern files pass verification with perfect step-accurate positioning and verified speed scaling.
- **Phase 4 (Optimization):** COMPLETED. Float optimization for ESP32 FPU (~3x faster step generation).

## Overview

Rewrite the MotionPlanner for the polar Sisyphus table to treat theta and rho as **completely independent axes** (no Cartesian
calculations, no junction deviation). Each axis gets its own S-curve profile, synchronized only by time.

## Implementation Phases

### Phase 1: Validate S-Curve Implementation [COMPLETED]
Before building the MotionPlanner, validate and fix the existing SCurve class.

**Validation checklist:**
- [X] Verify 7-phase profile calculations are mathematically correct
- [X] Test edge cases: zero distance, vStart > vMax, vEnd > vMax, very short distances
- [X] Verify `getPosition()`, `getVelocity()`, `getAcceleration()` return correct values
- [X] Check that profiles respect jerk/accel/velocity limits
- [X] Test the iterative velocity reduction loop

### Phase 2: Implement MotionPlanner [COMPLETED]
Build the new MotionPlanner with independent axis control.

### Phase 3: Verification & Robustness [COMPLETED]
- [X] **Desktop Test Harness Improvements**: 
    - Implemented `waitForTarget` to sync validation with the ISR step executor, eliminating race conditions.
    - Aligned test constants (`STEPS_PER_RAD_T`) with implementation defaults.
- [X] **Accuracy Fixes**:
    - Implemented strict step clamping in `fillStepQueue` to prevent rounding errors from overshooting targets.
    - Ensured `m_executed` step counters are updated atomically for thread safety between ISR and main loop.
- [X] **Speed Scaling Validation**:
    - Added benchmarking to pattern tests to verify that `setSpeedMultiplier` correctly affects execution time ratios.
    - Verified all 24 patterns pass at multiple speed settings.

## Key Design Decisions

### 1. Independent Axis Profiles with Time Synchronization
- Calculate S-curve for theta using theta limits (rad/s, rad/s², rad/s³)
- Calculate S-curve for rho using rho limits (mm/s, mm/s², mm/s³)
- Compare durations, stretch the faster axis to match the slower one
- Time-stretching preserves S-curve smoothness (jerk scales as 1/t³, so stretching reduces jerk)

### 2. Speed Setting (1-10)
- **Scale velocity only** (not acceleration/jerk)
- Rationale: Consistent motion character across speed settings; scaling accel/jerk would change the "feel"
- **On speed change**: Recalculate all segments not yet in the step queue
- The currently executing segment continues as planned (steps already queued)
- All pending segments get recalculated with new velocity limits
- This allows real-time speed adjustment during playback

### 3. Step Execution
- Pre-compute ~20ms worth of steps into a queue (512 events)
- ISR at **10kHz** (100μs period) checks queue and executes steps by timestamp.
- ISR must be tight (~5-10μs execution) to fit within period.

## Data Structures

```cpp
struct AxisProfile {
    int32_t startSteps;        // Starting position in steps
    int32_t targetSteps;       // Absolute target in steps
    int32_t deltaSteps;        // Signed delta from start
    double deltaUnits;         // Delta in physical units (rad or mm)
    int8_t direction;          // +1 or -1
    SCurve::Profile profile;   // 7-phase S-curve (double precision)
    SCurve::ProfileF profileF; // Float version for FPU-optimized evaluation
    float timeScaleF;          // Ratio: duration / profile.totalTime (float)
};

struct Segment {
    double targetTheta, targetRho;  // Target positions
    AxisProfile theta, rho;         // Per-axis profiles
    double duration;                // Synchronized duration (same for both)
    double thetaEntryVel, thetaExitVel;  // Velocity continuity
    double rhoEntryVel, rhoExitVel;
    bool calculated, executing;
};

struct StepEvent {
    uint32_t executeTime;      // Microsecond timestamp
    uint8_t stepMask;          // bit 0 = theta, bit 1 = rho
    uint8_t dirMask;           // direction bits
};
```

## Position Tracking (3 distinct values)

| Position | Updated By | Purpose |
|----------|------------|---------|
| Queued (`m_queuedTSteps`, `m_queuedRSteps`) | Main loop | Starting point for recalculate |
| Executed (`m_executedTSteps`, `m_executedRSteps`) | ISR | Actual motor position (std::atomic) |
| Target (in segments) | addSegment() | Goal for each segment |

## Algorithm Flow

### addSegment(theta, rho)
1. Calculate deltas from previous target
2. Add to ring buffer (N=32)
3. Mark as needing calculation

### recalculate()
Two-pass algorithm with **independent axis velocity management**:

#### Pass 1: Forward Pass (entry velocities)
For each segment from tail to head:
- Set entry velocity from previous segment's exit velocity (per-axis)
- Initial exit velocity = max (will be refined in backward pass)

#### Pass 2: Backward Pass (exit velocities)
For each segment from head back to tail:
- **Per-axis direction check**: If next segment reverses direction on that axis → exit vel = 0
- **End-of-pattern**: If this is the last segment → exit vel = 0 for both axes
- **Velocity propagation**: If exit vel was reduced, ensure entry vel is achievable
- Given distance and exit_vel, calculate max achievable entry_vel
- If current entry_vel > max achievable, reduce it and propagate backward

#### Final: Profile Calculation
For each segment:
- Calculate S-curve for theta with (entryVel, exitVel, limits)
- Calculate S-curve for rho with (entryVel, exitVel, limits)
- Synchronize: `syncDuration = max(thetaTime, rhoTime)`
- Calculate `timeScale = syncDuration / axisTime` for each axis

**Key insight**: Each axis independently manages velocity continuity. Theta only decelerates to 0 when theta reverses (or
end-of-pattern). Rho only decelerates to 0 when rho reverses. This allows smooth motion when one axis continues while the other
reverses.

### process() (main loop, ~50Hz)
1. Fill step queue up to 50ms horizon
2. For each sample time:
   - Get position from stretched S-curve: `profileTime = syncTime / timeScale`
   - Convert to target steps (clamped to segment target)
   - Generate step events to reach target
3. Check for segment completion, advance if needed

### stepTimerISR() (10kHz)
1. Check if head event's timestamp <= now
2. Set direction pins
3. Pulse step pins (theta and/or rho)
4. Update executed step counters (Atomic)
5. Advance queue head

### setSpeedMultiplier(mult)
1. Update `m_speedMultiplier`
2. Mark all non-executing segments as needing recalculation (`calculated = false`)
3. Call `recalculate()` to apply new speed immediately

## Interface (compatible with existing PolarControl usage)

```cpp
class MotionPlanner {
public:
    void init(int stepsPerMmR, int stepsPerRadT, double maxRho,
              double rMaxVel, double rMaxAccel, double rMaxJerk,
              double tMaxVel, double tMaxAccel, double tMaxJerk);

    bool addSegment(double theta, double rho);
    void recalculate();
    void start();
    void stop();
    void process();

    bool hasSpace() const;
    bool isRunning() const;
    bool isIdle() const;

    void getCurrentPosition(double& theta, double& rho) const;
    void resetTheta();
    void setSpeedMultiplier(double mult);
    void setEndOfPattern(bool ending);

    uint32_t getCompletedCount() const;
    void resetCompletedCount();
};
```

## Verification Plan

### Desktop Test Harness (Primary Verification)

Create a desktop build target that runs the motion planning code on Linux (not ESP32) for thorough validation.

**Validation checks:**
- [X] Velocity never exceeds scaled max for either axis
- [X] Acceleration never exceeds max for either axis
- [X] Jerk never exceeds max for either axis
- [X] Both axes complete each segment at the same time (within tolerance)
- [X] Step timing is consistent with velocity profile
- [X] No step queue underruns

### Diagnostics
Real-time telemetry has been added to the serial output (1Hz) to monitor planner health without a logic analyzer:

```
[MOTOR Core1] Position: ... | Q: 128 | UR: 0
```

- **Q (Queue Depth):** Number of steps currently in the ISR ring buffer (max 512). Should stay high (>100) during motion.
- **UR (Underruns):** Cumulative count of ISR buffer underruns. If this increments during motion, the CPU is too slow to generate steps in time.

## Design Decisions (Confirmed)

1. **S-curve validation first**: Validate and fix SCurve.cpp before building MotionPlanner
2. **Short segment handling**: Enforce minimum duration (10ms)
3. **Independent axis velocity continuity**: Each axis manages its own velocity
4. **Step-accurate clamping**: Step generation is clamped to segment targets to prevent cumulative drift.
5. **Atomic position tracking**: ISR updates positions atomically to prevent read-during-write glitches in the main loop.

## Implementation Notes & Bug Fixes

### 1. Cumulative Step Drift (The "Rounding" Bug)
**Issue:** Pattern tests initially reported massive Rho offsets (e.g., target 45,000 steps vs actual 113,698).
**Cause:** In `fillStepQueue`, calculating target steps using `round(frac * deltaSteps)` could occasionally result in a value 1 step beyond the target due to floating-point precision limits at segment boundaries. Because the next segment started from the *actual* target, these extra "overshoot" steps accumulated over hundreds of segments.
**Fix:** Added explicit clamping logic in the step generator. Before generating steps, the target step count is now strictly capped by the segment's defined target steps based on the direction of travel.

### 2. Test Harness Race Conditions
**Issue:** Verification frequently failed with "mismatch" errors even when the motion was mathematically correct.
**Cause:** The test runner was comparing the ISR's "executed" position against the "target" immediately after the generator finished queuing steps. Because the step queue maintains a ~50ms lookahead horizon, the motors were naturally "behind" the generator.
**Fix:** Created a `waitForTarget` helper in `test_main.cpp`. The test now simulates time and calls `process()` until the ISR actually reaches the target step count or times out.

### 3. Thread Safety & Atomicity
**Issue:** Potential for lost steps or corrupted position readings.
**Cause:** Position counters (`m_executedTSteps`, etc.) were marked `volatile`, but increments/decrements are not atomic operations. A read from the main loop could occur mid-update from the ISR.
**Fix:** Migrated all shared position counters to `std::atomic<int32_t>`.

### 4. Constant Alignment
**Issue:** Massive Theta errors during pattern verification.
**Cause:** The test harness was hardcoded to 7639 steps/rad (based on an older configuration), while the MotionPlanner implementation was defaulting to 3000 steps/rad.
**Fix:** Synchronized physical constants between the test harness and implementation.

### 5. Float Optimization for ESP32 FPU
**Issue:** Step generation was CPU-intensive due to `double` math being software-emulated.
**Cause:** The ESP32's FPU only supports single-precision `float`; all `double` operations are emulated in software (~3x slower).
**Fix:** Added `SCurve::ProfileF` (float version) and `getPositionF()` for real-time evaluation. The initial profile calculation uses `double` for precision, then converts to `float` for step generation in `fillStepQueue()`. This provides ~3x speedup in the main loop while maintaining accuracy.

### 6. Stop/Start Bug
**Issue:** Stopping a pattern mid-way caused the next pattern to fail or start incorrectly.
**Cause:** `stop()` only cleared the step queue but left segments in the buffer and didn't sync position tracking.
**Fix:** Updated `stop()` to clear segment buffer, reset endOfPattern flag, and sync queued positions with executed positions.

### 7. Motion Settings While Running
**Issue:** Changing motion settings via `setMotionSettings()` would reset positions to zero, breaking any running pattern.
**Cause:** It called `init()` which does a full reset.
**Fix:** Added `setMotionLimits()` that updates velocity/accel/jerk limits without resetting positions, and recalculates pending segments with the new limits.