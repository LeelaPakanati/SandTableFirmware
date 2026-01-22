#pragma once
#include <cstdint>
#include <atomic>
#include "SCurve.hpp"

#ifndef NATIVE_BUILD
#include <Arduino.h>
#endif

// IRAM_ATTR is ESP32-specific; define as empty for native builds
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

// Pin definitions for step/dir
#ifndef R_STEP_PIN
#define R_STEP_PIN 33
#define R_DIR_PIN 25
#define T_STEP_PIN 32
#define T_DIR_PIN 22
#endif

// Configuration constants
static constexpr int SEGMENT_BUFFER_SIZE = 32;
static constexpr int STEP_QUEUE_SIZE = 512;
static constexpr double MIN_SEGMENT_DURATION = 0.010;  // 10ms minimum
static constexpr uint32_t STEP_TIMER_PERIOD_US = 40;   // 25kHz ISR
static constexpr uint32_t STEP_QUEUE_HORIZON_US = 20000;  // 20ms lookahead

// Per-axis motion profile
struct AxisProfile {
    int32_t startSteps;        // Starting position in steps
    int32_t targetSteps;       // Absolute target in steps
    int32_t deltaSteps;        // Signed delta from start
    double deltaUnits;         // Delta in physical units (rad or mm)
    int8_t direction;          // +1 or -1 (0 if no motion)
    SCurve::Profile profile;   // 7-phase S-curve
    double syncDuration;       // Duration after synchronization
    double timeScale;          // Ratio: syncDuration / profile.totalTime
};

// A motion segment with synchronized theta/rho profiles
struct Segment {
    double targetTheta;        // Target theta position (radians, unwrapped)
    double targetRho;          // Target rho position (mm)

    AxisProfile theta;         // Theta axis profile
    AxisProfile rho;           // Rho axis profile

    double duration;           // Synchronized duration (same for both axes)

    // Velocity continuity
    double thetaEntryVel;      // Entry velocity for theta (rad/s)
    double thetaExitVel;       // Exit velocity for theta (rad/s)
    double rhoEntryVel;        // Entry velocity for rho (mm/s)
    double rhoExitVel;         // Exit velocity for rho (mm/s)

    bool calculated;           // True if profile has been calculated
    bool executing;            // True if currently being executed
};

// Step event for the ISR queue
struct StepEvent {
    uint32_t executeTime;      // Microsecond timestamp (relative to segment start)
    uint8_t stepMask;          // bit 0 = theta, bit 1 = rho
    uint8_t dirMask;           // direction bits: bit 0 = theta dir, bit 1 = rho dir
};

// Motion planner with independent axis control and S-curve profiles
class MotionPlanner {
public:
    MotionPlanner();
    ~MotionPlanner();

    // Initialize with physical parameters
    // stepsPerMmR: steps per mm for rho axis
    // stepsPerRadT: steps per radian for theta axis
    // maxRho: maximum rho position in mm
    // rMaxVel/rMaxAccel/rMaxJerk: rho limits (mm/s, mm/s², mm/s³)
    // tMaxVel/tMaxAccel/tMaxJerk: theta limits (rad/s, rad/s², rad/s³)
    void init(int stepsPerMmR, int stepsPerRadT, double maxRho,
              double rMaxVel, double rMaxAccel, double rMaxJerk,
              double tMaxVel, double tMaxAccel, double tMaxJerk);

    // Add a segment to the buffer (returns false if buffer full)
    bool addSegment(double theta, double rho);

    // Recalculate profiles for all pending segments
    void recalculate();

    // Start motion execution
    void start();

    // Stop motion execution (decelerate to stop)
    void stop();

    // Main processing loop - call frequently (~50Hz or faster)
    // Fills the step queue and manages segment transitions
    void process();

    // Check if there's space for more segments
    bool hasSpace() const;

    // Check if motion is currently running
    bool isRunning() const;

    // Check if planner is idle (no more segments to execute)
    bool isIdle() const;

    // Get current position in physical units
    void getCurrentPosition(double& theta, double& rho) const;

    // Reset theta to zero (current position becomes new origin)
    void resetTheta();

    // Set speed multiplier (0.1 to 1.0, scales velocity only)
    void setSpeedMultiplier(double mult);

    // Signal end of pattern (causes deceleration to stop)
    void setEndOfPattern(bool ending);

    // Get count of completed segments
    uint32_t getCompletedCount() const { return m_completedCount; }

    // Reset completed count
    void resetCompletedCount() { m_completedCount = 0; }

    // Get diagnostic info
    void getDiagnostics(uint32_t& queueDepth, uint32_t& underruns) const;

private:
    // Physical parameters
    int m_stepsPerMmR;
    int m_stepsPerRadT;
    double m_maxRho;

    // Motion limits (base values before speed scaling)
    double m_rMaxVel, m_rMaxAccel, m_rMaxJerk;
    double m_tMaxVel, m_tMaxAccel, m_tMaxJerk;

    // Speed multiplier (applied to velocity only)
    double m_speedMultiplier;

    // Segment ring buffer
    Segment m_segments[SEGMENT_BUFFER_SIZE];
    volatile int m_segmentHead;      // Next segment to add
    volatile int m_segmentTail;      // Next segment to execute
    int m_segmentCount;              // Number of segments in buffer

    // Position tracking (in steps)
    // These are the three distinct position values mentioned in the plan:

    // 1. Queued position - where the last queued segment ends
    //    Used as starting point for new segment calculations
    std::atomic<int32_t> m_queuedTSteps;
    std::atomic<int32_t> m_queuedRSteps;

    // 2. Executed position - actual motor position (updated by ISR)
    volatile int32_t m_executedTSteps;
    volatile int32_t m_executedRSteps;

    // Target positions in physical units (for the last added segment)
    double m_targetTheta;
    double m_targetRho;

    // Step event queue (circular buffer)
    StepEvent m_stepQueue[STEP_QUEUE_SIZE];
    volatile int m_stepQueueHead;    // Next position to write
    volatile int m_stepQueueTail;    // Next position to read (ISR)
    std::atomic<uint32_t> m_underrunCount{0}; // Track queue underruns

    // Timing
    uint32_t m_segmentStartTime;     // Microseconds when current segment started
    double m_segmentElapsed;         // Time elapsed in current segment

    // State
    bool m_running;
    bool m_endOfPattern;
    uint32_t m_completedCount;

    // Timer handle (ESP32 specific)
    void* m_timerHandle;

    // Internal methods
    void calculateSegmentProfile(Segment& seg);
    void fillStepQueue();
    int getStepQueueSpace() const;
    bool queueStepEvent(uint32_t time, uint8_t stepMask, uint8_t dirMask);

    // Convert physical units to steps
    int32_t thetaToSteps(double theta) const;
    int32_t rhoToSteps(double rho) const;

    // Convert steps to physical units
    double stepsToTheta(int32_t steps) const;
    double stepsToRho(int32_t steps) const;

    // ISR callback (static for C compatibility)
    static void IRAM_ATTR stepTimerISR(void* arg);
    void IRAM_ATTR handleStepTimer();
};
