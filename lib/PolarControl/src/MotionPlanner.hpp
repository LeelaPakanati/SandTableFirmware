#pragma once
#include <cstdint>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <esp_timer.h>
#include "SCurve.hpp"

// Motion planner with lookahead buffer and S-curve profiles

struct PlannerBlock {
    // Target position in steps (absolute)
    int32_t rTargetSteps;
    int32_t tTargetSteps;

    // Delta steps for this block
    int32_t rDeltaSteps;
    int32_t tDeltaSteps;

    // Delta in physical units
    double dRho;      // mm
    double dTheta;    // radians

    // Cartesian distance of this segment (for speed calculations)
    double distance;  // mm

    // Effective limits for this segment (constrained by both axes)
    double maxVelocity;   // mm/s (effective)
    double maxAccel;      // mm/s² (effective)
    double maxJerk;       // mm/s³ (effective)

    // Velocity profile
    double entrySpeed;   // mm/s at block start
    double cruiseSpeed;  // mm/s max during block
    double exitSpeed;    // mm/s at block end

    // S-curve profile for this block
    SCurve::Profile profile;

    // Block state
    bool planned;
    bool executing;
};

class MotionPlanner {
public:
    MotionPlanner();
    ~MotionPlanner();

    // Initialize planner with physical parameters
    void init(
        int stepsPerMmR,
        int stepsPerRadT,
        double maxRho,
        // Rho limits (mm units)
        double rMaxVelocity,   // mm/s
        double rMaxAccel,      // mm/s²
        double rMaxJerk,       // mm/s³
        // Theta limits (radian units)
        double tMaxVelocity,   // rad/s
        double tMaxAccel,      // rad/s²
        double tMaxJerk        // rad/s³
    );

    // Add a segment to the planner queue
    // Returns false if queue is full
    bool addSegment(double theta, double rho);

    // Set speed multiplier (0.1 to 1.0)
    void setSpeedMultiplier(double mult);

    // Check if planner has space for more segments
    bool hasSpace() const;

    // Check if planner is currently executing motion
    bool isRunning() const;

    // Check if planner is idle (no blocks, not moving)
    bool isIdle() const;

    // Start execution (call after adding initial segments)
    void start();

    // Stop execution immediately
    void stop();

    // Get current position
    void getCurrentPosition(double& theta, double& rho) const;

    // Reset theta to zero (current position becomes new origin)
    void resetTheta();

    // Get number of completed segments since start/reset
    uint32_t getCompletedCount() const { return m_completedCount; }
    void resetCompletedCount() { m_completedCount = 0; }

    // Process function - call from main loop to feed the planner
    void process();

private:
    static constexpr int LOOKAHEAD_SIZE = 16;
    static constexpr double JUNCTION_DEVIATION = 0.5;  // mm - allowable corner cutting

    // Block buffer (ring buffer)
    PlannerBlock m_blocks[LOOKAHEAD_SIZE];
    volatile int m_head = 0;      // Next block to execute
    volatile int m_tail = 0;      // Next slot to fill
    volatile int m_count = 0;     // Number of blocks in queue
    volatile uint32_t m_completedCount = 0; // Number of blocks completed

    // Physical parameters
    int m_stepsPerMmR = 100;
    int m_stepsPerRadT = 150;
    double m_maxRho = 450.0;

    // Rho limits (mm units)
    double m_rMaxVelocity = 30.0;
    double m_rMaxAccel = 20.0;
    double m_rMaxJerk = 100.0;

    // Theta limits (radian units)
    double m_tMaxVelocity = 1.0;
    double m_tMaxAccel = 2.0;
    double m_tMaxJerk = 10.0;

    double m_speedMult = 1.0;

    // Current position tracking
    volatile int32_t m_currentRSteps = 0;
    volatile int32_t m_currentTSteps = 0;
    double m_lastTheta = 0;
    double m_lastRho = 0;

    // Previous segment for junction calculation
    double m_prevDirX = 0;
    double m_prevDirY = 0;
    bool m_hasPrevDir = false;

    // Execution state
    volatile bool m_running = false;
    volatile double m_currentVelocity = 0;

    // Timer for step generation
    esp_timer_handle_t m_stepTimer = nullptr;

    // Current block execution state
    volatile int m_executingBlock = -1;
    volatile double m_blockStartTime = 0;
    volatile int32_t m_blockStartR = 0;
    volatile int32_t m_blockStartT = 0;

    // Bresenham state for coordinated stepping
    volatile int32_t m_bresenhamError = 0;
    volatile int32_t m_bresenhamDeltaMajor = 0;
    volatile int32_t m_bresenhamDeltaMinor = 0;
    volatile bool m_rIsMajor = true;
    volatile int32_t m_stepsRemaining = 0;
    volatile int32_t m_rDir = 1;
    volatile int32_t m_tDir = 1;

    // Step timing
    volatile uint64_t m_lastStepTime = 0;
    volatile uint32_t m_stepInterval = 1000;  // microseconds

    // Flag for ISR to signal main loop
    volatile bool m_needNextBlock = false;

    // Mutex for thread safety
    SemaphoreHandle_t m_mutex = nullptr;

    // GPIO pins (set during init)
    int m_rStepPin = 33;
    int m_rDirPin = 25;
    int m_tStepPin = 32;
    int m_tDirPin = 22;

    // Private methods
    void recalculate();
    double calculateJunctionSpeed(int blockIdx);
    void planBlock(int blockIdx);
    void startNextBlock();

    // Timer ISR
    static void IRAM_ATTR stepTimerISR(void* arg);
    void IRAM_ATTR generateSteps();

    // Helpers
    int nextIndex(int idx) const { return (idx + 1) % LOOKAHEAD_SIZE; }
    int prevIndex(int idx) const { return (idx - 1 + LOOKAHEAD_SIZE) % LOOKAHEAD_SIZE; }
};
