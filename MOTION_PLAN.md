# MotionPlanner Redesign Plan

  ## Status Update (2026-01-22)
  - **Phase 1 (S-Curve Validation):** COMPLETED
  - **Phase 2 (MotionPlanner Implementation):** COMPLETED
  - **Phase 3 (Verification):** Desktop tests passing. Full pattern playback verified with `shell.thr`.

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
  - [X] Test the iterative velocity reduction loop (line 100: `vCruise -= 0.5` looks suspicious)

  **Known concerns in SCurve.cpp:**
  - Line 100: `vCruise -= 0.5` - hardcoded step size may not work for all unit scales
  - Reduced jerk profile calculation may have issues with very small velocity deltas

  ### Phase 2: Implement MotionPlanner [COMPLETED]
  Build the new MotionPlanner with independent axis control.

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
  - ISR at **25-50kHz** (20-40μs period) checks queue and executes steps by timestamp
  - Higher frequency chosen for better timing precision at high step rates
  - ISR must be tight (~5-10μs execution) to fit within period

  ## Data Structures

  ```cpp
  struct AxisProfile {
  int32_t targetSteps;       // Absolute target in steps
  int32_t deltaSteps;        // Signed delta from start
  double deltaUnits;         // Delta in physical units (rad or mm)
  int8_t direction;          // +1 or -1
  SCurve::Profile profile;   // 7-phase S-curve
  double syncDuration;       // Duration after synchronization
  double timeScale;          // Ratio: syncDuration / profile.totalTime
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
  | Executed (`m_executedTSteps`, `m_executedRSteps`) | ISR | Actual motor position |
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
  1. Fill step queue up to 20ms horizon
  2. For each sample time:
  - Get position from stretched S-curve: `profileTime = syncTime / timeScale`
  - Convert to target steps
  - Generate step events to reach target
  3. Check for segment completion, advance if needed

  ### stepTimerISR() (25-50kHz)
  1. Check if head event's timestamp <= now
  2. Set direction pins
  3. Pulse step pins (theta and/or rho)
  4. Update executed step counters
  5. Advance queue head

  Note: At 25kHz (40μs period), ISR must complete in <30μs to leave headroom

  ### setSpeedMultiplier(mult)
  1. Update `m_speedMultiplier`
  2. Mark all non-executing segments as needing recalculation (`calculated = false`)
  3. Call `recalculate()` to apply new speed immediately

  ### Direction Change Detection (for backward pass)
  ```cpp
  bool thetaReverses = (seg.theta.direction != 0) &&
  (nextSeg.theta.direction != 0) &&
  (seg.theta.direction != nextSeg.theta.direction);
  bool rhoReverses = (seg.rho.direction != 0) &&
  (nextSeg.rho.direction != 0) &&
  (seg.rho.direction != nextSeg.rho.direction);
  ```
  Note: direction=0 (no motion) doesn't count as a reversal.

  ### Max Achievable Entry Velocity Calculation
  Given: distance `d`, exit velocity `vEnd`, max accel `aMax`, max jerk `jMax`
  Find: maximum entry velocity `vStart` that can decelerate to `vEnd` within distance `d`

  This is the inverse of the S-curve distance calculation - solved iteratively or via closed-form approximation.

  ## Files to Create/Modify

  | File | Action |
  |------|--------|
  | `lib/PolarControl/src/MotionPlanner.hpp` | Create - data structures & class declaration |
  | `lib/PolarControl/src/MotionPlanner.cpp` | Create - full implementation |
  | `lib/PolarControl/src/PolarControl.cpp` | Verify - interface compatibility |

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

  **Components:**
  1. **ESP32 Mock Layer** - Stubs for: `esp_timer`, GPIO functions, FreeRTOS primitives
  2. **THR File Reader** - Parse theta/rho pattern files
  3. **Test Runner** - Feed positions to MotionPlanner, simulate time progression
  4. **Profile Logger** - Output per-segment: durations, velocities, accelerations, step counts
  5. **Limit Validator** - Assert that no profile exceeds configured limits
  6. **Step Visualizer** (optional) - Output step timing data for plotting

  **Test harness output for each segment:**
  ```
  Segment 42: theta=1.234rad rho=150.5mm
  Theta: deltaSteps=1234, duration=0.45s, maxVel=0.8rad/s, maxAccel=1.5rad/s²
  Rho:   deltaSteps=567,  duration=0.45s, maxVel=25mm/s, maxAccel=18mm/s²
  Sync:  duration=0.45s (theta was limiting axis)
  Steps: theta=[t0:+1, t40us:+1, ...] rho=[t0:+1, t80us:+1, ...]
  PASS: All limits respected
  ```

  **Validation checks:**
  - [X] Velocity never exceeds scaled max for either axis
  - [X] Acceleration never exceeds max for either axis
  - [X] Jerk never exceeds max for either axis
  - [X] Both axes complete each segment at the same time (within tolerance)
  - [X] Step timing is consistent with velocity profile
  - [X] No step queue underruns

  **Build approach: PlatformIO native environment**

  Add `[env:native]` to `platformio.ini` for desktop builds. This compiles the motion planning code for Linux, enabling local testing
  without ESP32 hardware.

  **Files to create:**
  | File | Purpose |
  |------|---------|
  | `test/test_motion/test_main.cpp` | Test runner entry point |
  | `test/test_motion/esp32_mock.hpp` | ESP32 API stubs (timers, GPIO, FreeRTOS) |
  | `test/test_motion/thr_reader.hpp` | Pattern file parser (.thr: `theta rho`, rho 0-1 scaled) |
  | `test/test_motion/profile_validator.hpp` | Limit checking and assertion |

  **platformio.ini addition:**
  ```ini
  [env:native]
  platform = native
  build_flags = -std=c++17 -DNATIVE_BUILD
  build_src_filter = -<main.cpp> +<../test/test_motion/>
  lib_ignore = WiFiManager, ESPAsyncWebServer, AsyncTCP, TMC2209
  ```

  **Usage:**
  ```bash
  # Build and run desktop tests
  pio test -e native

  # Or build and run manually
  pio run -e native
  .pio/build/native/program patterns/some_pattern.thr
  ```

  **Single Command Execution:**
  A helper script has been created to build the environment, run unit tests, and validate all available pattern files in one go:
  ```bash
  ./run_all_tests.sh
  ```

  ### On-Device Testing (Secondary)
  1. **Logic analyzer**: Verify step pulse timing matches expectations
  2. **End-to-end**: Run patterns and observe smooth motion
  3. **Stress test**: Run clearing patterns at various speeds
  
  ### Diagnostics
  Real-time telemetry has been added to the serial output (1Hz) to monitor planner health without a logic analyzer:
  
  ```
  [MOTOR Core1] Position: ... | Q: 128 | UR: 0
  ```
  
  - **Q (Queue Depth):** Number of steps currently in the ISR ring buffer (max 512). Should stay high (>100) during motion.
  - **UR (Underruns):** Cumulative count of ISR buffer underruns. If this increments during motion, the CPU is too slow to generate steps in time.

  ## Design Decisions (Confirmed)

  1. **S-curve validation first**: Validate and fix SCurve.cpp before building MotionPlanner
  2. **Short segment handling**: Enforce minimum duration (not skip or coalesce)
  - If calculated duration < MIN_SEGMENT_DURATION (e.g., 10ms), extend to minimum
  - This ensures consistent step generation even for tiny movements
  3. **Segment buffer size**: 32 segments (can adjust if needed during testing)
  4. **Independent axis velocity continuity**: Each axis manages its own velocity
  - Theta only decelerates to 0 when theta direction reverses or end-of-pattern
  - Rho only decelerates to 0 when rho direction reverses or end-of-pattern
  - Allows smooth motion when one axis continues while the other reverses
  5. **Speed changes trigger recalculation**: `setSpeedMultiplier()` marks non-executing segments for recalculation

  ## Project Documentation

  **MOTION_PLAN.md** - Create in project root to track:
  - Implementation TODOs and status
  - Notes on what was tried and whether it worked
  - Failed approaches (and why they failed)
  - Design decisions and rationale
  - Update this doc after every task done

  ## Test Pattern Files

  **Source:** `./thrs/` (24 patterns including Fractals, Spirals, Sierpinski, etc.)
  **Move to:** `test/test_motion/patterns/` when creating test harness


  If you need specific details from before exiting plan mode (like exact code snippets, error messages, or content you generated),
  read the full transcript at:
  /home/leela/.claude/projects/-home-leela-code-sisyphus-SisyphusTable/12721d96-f068-4dfd-95ab-e67b85c02de0.jsonl
