// Desktop test harness for MotionPlanner
// Build with: pio run -e native
// Run with: .pio/build/native/program [pattern.thr]

#include <iostream>
#include <iomanip>
#include <string>
#include <cmath>
#include <vector>
#include <chrono>

#include "esp32_mock.hpp"
#include "thr_reader.hpp"
#include "profile_validator.hpp"
#include "SCurve.hpp"
#include "MotionPlanner.hpp"

// Test configuration
static constexpr double R_MAX = 450.0;           // mm
static constexpr int STEPS_PER_MM_R = 100;       // steps per mm for rho
static constexpr int STEPS_PER_RAD_T = 3000;     // steps per rad for theta (aligned with MotionPlanner default)

int getQueueSpace(const MotionPlanner& p) {
    uint32_t depth, underruns;
    p.getDiagnostics(depth, underruns);
    return STEP_QUEUE_SIZE - 1 - depth;
}

// Motion limits
static constexpr double R_MAX_VEL = 10.0;        // mm/s
static constexpr double R_MAX_ACCEL = 20.0;      // mm/s²
static constexpr double R_MAX_JERK = 100.0;      // mm/s³
static constexpr double T_MAX_VEL = 0.25;         // rad/s
static constexpr double T_MAX_ACCEL = 1.0;       // rad/s²
static constexpr double T_MAX_JERK = 10.0;       // rad/s³

// ============================================================================
// Test: SCurve basic functionality
// ============================================================================

bool testSCurveBasic() {
    std::cout << "\n=== Test: SCurve Basic ===" << std::endl;

    ProfileValidator validator;
    SCurve::Profile profile;
    bool allPassed = true;

    // Test 1: Simple move from rest to rest
    std::cout << "\n1. Simple move (100mm, 0->0):" << std::endl;
    SCurve::calculate(100.0, 0.0, 0.0, R_MAX_VEL, R_MAX_ACCEL, R_MAX_JERK, profile);
    auto result = validator.validate(profile, 100.0, R_MAX_VEL, R_MAX_ACCEL, R_MAX_JERK);
    validator.printValidation(result);
    allPassed &= result.passed;

    // Test 2: Move with entry velocity
    std::cout << "\n2. Move with entry velocity (100mm, 15->0):" << std::endl;
    SCurve::calculate(100.0, 15.0, 0.0, R_MAX_VEL, R_MAX_ACCEL, R_MAX_JERK, profile);
    result = validator.validate(profile, 100.0, R_MAX_VEL, R_MAX_ACCEL, R_MAX_JERK);
    validator.printValidation(result);
    allPassed &= result.passed;

    // Test 3: Move with exit velocity
    std::cout << "\n3. Move with exit velocity (100mm, 0->15):" << std::endl;
    SCurve::calculate(100.0, 0.0, 15.0, R_MAX_VEL, R_MAX_ACCEL, R_MAX_JERK, profile);
    result = validator.validate(profile, 100.0, R_MAX_VEL, R_MAX_ACCEL, R_MAX_JERK);
    validator.printValidation(result);
    allPassed &= result.passed;

    // Test 4: Short move (can't reach max velocity)
    std::cout << "\n4. Short move (10mm, 0->0):" << std::endl;
    SCurve::calculate(10.0, 0.0, 0.0, R_MAX_VEL, R_MAX_ACCEL, R_MAX_JERK, profile);
    result = validator.validate(profile, 10.0, R_MAX_VEL, R_MAX_ACCEL, R_MAX_JERK);
    validator.printValidation(result);
    allPassed &= result.passed;

    // Test 5: Very short move
    std::cout << "\n5. Very short move (1mm, 0->0):" << std::endl;
    SCurve::calculate(1.0, 0.0, 0.0, R_MAX_VEL, R_MAX_ACCEL, R_MAX_JERK, profile);
    result = validator.validate(profile, 1.0, R_MAX_VEL, R_MAX_ACCEL, R_MAX_JERK);
    validator.printValidation(result);
    allPassed &= result.passed;

    // Test 6: Zero distance
    std::cout << "\n6. Zero distance (0mm):" << std::endl;
    SCurve::calculate(0.0, 0.0, 0.0, R_MAX_VEL, R_MAX_ACCEL, R_MAX_JERK, profile);
    result = validator.validate(profile, 0.0, R_MAX_VEL, R_MAX_ACCEL, R_MAX_JERK);
    validator.printValidation(result);
    allPassed &= result.passed;

    // Test 7: Entry velocity exceeds max
    std::cout << "\n7. Entry vel > max (100mm, 50->0, max=30):" << std::endl;
    SCurve::calculate(100.0, 50.0, 0.0, R_MAX_VEL, R_MAX_ACCEL, R_MAX_JERK, profile);
    result = validator.validate(profile, 100.0, R_MAX_VEL, R_MAX_ACCEL, R_MAX_JERK);
    validator.printValidation(result);
    allPassed &= result.passed;

    // Test 8: Theta axis parameters
    std::cout << "\n8. Theta axis (PI rad, 0->0):" << std::endl;
    SCurve::calculate(M_PI, 0.0, 0.0, T_MAX_VEL, T_MAX_ACCEL, T_MAX_JERK, profile);
    result = validator.validate(profile, M_PI, T_MAX_VEL, T_MAX_ACCEL, T_MAX_JERK);
    validator.printValidation(result);
    allPassed &= result.passed;

    return allPassed;
}

// ============================================================================
// Test: SCurve deceleration distance calculation
// ============================================================================

bool testDecelDistance() {
    std::cout << "\n=== Test: Deceleration Distance ===" << std::endl;

    bool allPassed = true;

    // Test various velocity combinations
    struct TestCase {
        double vStart, vEnd;
        const char* desc;
    };

    TestCase cases[] = {
        {30.0, 0.0, "Full stop from max"},
        {30.0, 15.0, "Half decel"},
        {15.0, 0.0, "Half to stop"},
        {5.0, 0.0, "Slow to stop"},
        {30.0, 29.0, "Small decel"},
    };

    for (const auto& tc : cases) {
        double dist = SCurve::decelerationDistance(tc.vStart, tc.vEnd, R_MAX_ACCEL, R_MAX_JERK);

        // Verify by calculating profile
        SCurve::Profile profile;
        SCurve::calculate(dist, tc.vStart, tc.vEnd, R_MAX_VEL, R_MAX_ACCEL, R_MAX_JERK, profile);

        double actualDist = profile.totalDistance;
        double error = std::abs(actualDist - dist);
        bool passed = error < dist * 0.02 + 0.1;  // 2% + 0.1mm tolerance

        std::cout << tc.desc << " (" << tc.vStart << " -> " << tc.vEnd << "): "
                  << "dist=" << dist << "mm, actual=" << actualDist << "mm"
                  << (passed ? " PASS" : " FAIL") << std::endl;

        allPassed &= passed;
    }

    return allPassed;
}

// ============================================================================
// Test: Max achievable entry velocity
// ============================================================================

bool testMaxEntryVel() {
    std::cout << "\n=== Test: Max Achievable Entry Velocity ===" << std::endl;

    bool allPassed = true;

    struct TestCase {
        double distance, vEnd;
        const char* desc;
    };

    TestCase cases[] = {
        {100.0, 0.0, "100mm to stop"},
        {50.0, 0.0, "50mm to stop"},
        {10.0, 0.0, "10mm to stop"},
        {100.0, 15.0, "100mm to 15mm/s"},
        {50.0, 15.0, "50mm to 15mm/s"},
    };

    for (const auto& tc : cases) {
        double maxEntry = SCurve::maxAchievableEntryVelocity(
            tc.distance, tc.vEnd, R_MAX_VEL, R_MAX_ACCEL, R_MAX_JERK);

        // Verify by calculating decel distance
        double decelDist = SCurve::decelerationDistance(maxEntry, tc.vEnd, R_MAX_ACCEL, R_MAX_JERK);
        bool passed = decelDist <= tc.distance * 1.02 + 0.1;

        std::cout << tc.desc << ": maxEntry=" << maxEntry << " mm/s"
                  << ", decelDist=" << decelDist << "mm"
                  << (passed ? " PASS" : " FAIL") << std::endl;

        allPassed &= passed;
    }

    return allPassed;
}

// ============================================================================
// Test: MotionPlanner with synthetic data
// ============================================================================

bool testMotionPlannerBasic() {
    std::cout << "\n=== Test: MotionPlanner Basic ===" << std::endl;
    resetMock();

    MotionPlanner planner;
    planner.init(STEPS_PER_MM_R, STEPS_PER_RAD_T, R_MAX,
                 R_MAX_VEL, R_MAX_ACCEL, R_MAX_JERK,
                 T_MAX_VEL, T_MAX_ACCEL, T_MAX_JERK);

    bool allPassed = true;

    // Add some segments
    std::cout << "\nAdding segments..." << std::endl;

    // Simple spiral outward
    double theta = 0;
    double rho = 0;
    int segCount = 0;

    for (int i = 0; i < 10; i++) {
        theta += M_PI / 10;  // 18 degrees
        rho = std::min(R_MAX, rho + 20.0);

        if (!planner.addSegment(theta, rho)) {
            std::cout << "Buffer full at segment " << i << std::endl;
            break;
        }
        segCount++;
    }
    std::cout << "Added " << segCount << " segments" << std::endl;

    // Mark end of pattern and recalculate
    planner.setEndOfPattern(true);
    planner.recalculate();

    // Start the planner
    planner.start();

    // Simulate time progression - use larger time steps for faster completion
    std::cout << "\nSimulating execution..." << std::endl;
    setMicros(0);

    int iterations = 0;
    int maxIterations = 10000;  // Reduced iterations, larger time steps
    uint32_t lastCompleted = 0;

    while (planner.isRunning() && iterations < maxIterations) {
        planner.process();
        advanceMicros(50000);  // 50ms steps for faster simulation
        iterations++;

        uint32_t completed = planner.getCompletedCount();
        if (completed != lastCompleted) {
            std::cout << "  Segment completed at t=" << (g_mockMicros.load() / 1000000.0) << "s"
                      << ", total=" << completed << "/" << segCount << std::endl;
            lastCompleted = completed;
        }
    }

    if (iterations >= maxIterations) {
        std::cout << "FAIL: Did not complete in time (after " << iterations << " iterations)" << std::endl;
        allPassed = false;
    } else {
        std::cout << "Completed in " << iterations << " iterations ("
                  << (g_mockMicros.load() / 1000000.0) << " seconds)" << std::endl;
    }

    std::cout << "Segments completed: " << planner.getCompletedCount() << "/" << segCount << std::endl;
    return allPassed && (planner.getCompletedCount() == (uint32_t)segCount);
}

// ============================================================================
// Test: Direction reversal handling
// ============================================================================

bool testDirectionReversal() {
    std::cout << "\n=== Test: Direction Reversal ===" << std::endl;
    resetMock();

    MotionPlanner planner;
    planner.init(STEPS_PER_MM_R, STEPS_PER_RAD_T, R_MAX,
                 R_MAX_VEL, R_MAX_ACCEL, R_MAX_JERK,
                 T_MAX_VEL, T_MAX_ACCEL, T_MAX_JERK);

    // Pattern with direction reversals
    // theta: forward, forward, reverse, reverse
    // rho: outward, outward, outward, inward

    planner.addSegment(M_PI / 4, 100);   // theta+, rho+
    planner.addSegment(M_PI / 2, 200);   // theta+, rho+
    planner.addSegment(M_PI / 4, 300);   // theta-, rho+  <- theta reversal
    planner.addSegment(0, 200);          // theta-, rho-  <- rho reversal

    planner.setEndOfPattern(true);
    planner.recalculate();
    planner.start();

    setMicros(0);
    int iterations = 0;
    int maxIterations = 50000;

    while (planner.isRunning() && iterations < maxIterations) {
        planner.process();
        advanceMicros(20000);
        iterations++;
    }

    bool passed = iterations < maxIterations && planner.getCompletedCount() == 4;
    std::cout << "Completed: " << planner.getCompletedCount() << "/4 segments"
              << (passed ? " PASS" : " FAIL") << std::endl;

    return passed;
}

// ============================================================================
// Test: Speed multiplier
// ============================================================================

bool testSpeedMultiplier() {
    std::cout << "\n=== Test: Speed Multiplier ===" << std::endl;
    resetMock();

    MotionPlanner planner;
    planner.init(STEPS_PER_MM_R, STEPS_PER_RAD_T, R_MAX,
                 R_MAX_VEL, R_MAX_ACCEL, R_MAX_JERK,
                 T_MAX_VEL, T_MAX_ACCEL, T_MAX_JERK);

    // Add segments
    for (int i = 0; i < 5; i++) {
        planner.addSegment((i + 1) * M_PI / 5, (i + 1) * 50);
    }
    planner.setEndOfPattern(true);
    planner.recalculate();

    // Run at full speed
    planner.setSpeedMultiplier(1.0);
    planner.start();

    setMicros(0);
    int iterations = 0;
    int maxIterations = 50000;
    while (planner.isRunning() && iterations < maxIterations) {
        planner.process();
        advanceMicros(20000);
        iterations++;
    }
    double fullSpeedTime = g_mockMicros.load() / 1000000.0;
    planner.stop();

    if (iterations >= maxIterations) {
        std::cout << "FAIL: Full speed run timeout" << std::endl;
        return false;
    }

    // Reset and run at half speed
    planner.init(STEPS_PER_MM_R, STEPS_PER_RAD_T, R_MAX,
                 R_MAX_VEL, R_MAX_ACCEL, R_MAX_JERK,
                 T_MAX_VEL, T_MAX_ACCEL, T_MAX_JERK);

    for (int i = 0; i < 5; i++) {
        planner.addSegment((i + 1) * M_PI / 5, (i + 1) * 50);
    }
    planner.setEndOfPattern(true);
    planner.setSpeedMultiplier(0.5);
    planner.recalculate();
    planner.start();

    setMicros(0);
    iterations = 0;
    while (planner.isRunning() && iterations < maxIterations) {
        planner.process();
        advanceMicros(20000);
        iterations++;
    }
    double halfSpeedTime = g_mockMicros.load() / 1000000.0;

    if (iterations >= maxIterations) {
        std::cout << "FAIL: Half speed run timeout" << std::endl;
        return false;
    }

    std::cout << "Full speed time: " << fullSpeedTime << "s" << std::endl;
    std::cout << "Half speed time: " << halfSpeedTime << "s" << std::endl;

    // Half speed should take roughly twice as long (with some tolerance)
    double ratio = halfSpeedTime / fullSpeedTime;
    bool passed = ratio > 1.5 && ratio < 2.5;
    std::cout << "Time ratio: " << ratio << (passed ? " PASS" : " FAIL") << std::endl;

    return passed;
}

// ============================================================================
// Test: Pattern file
// ============================================================================

// Helper to wait for ISR to reach a specific target with tolerance
bool waitForTarget(const MotionPlanner& planner, int32_t targetT, int32_t targetR,
                   int maxIters = 10000, int32_t toleranceT = 0, int32_t toleranceR = 0) {
    int iters = 0;
    while (iters < maxIters) {
        double curT, curR;
        planner.getCurrentPosition(curT, curR);

        // Use exact same scaling as internal to avoid rounding diffs
        int32_t sT = (int32_t)round(curT * STEPS_PER_RAD_T);
        int32_t sR = (int32_t)round(curR * STEPS_PER_MM_R);

        int32_t errT = std::abs(sT - targetT);
        int32_t errR = std::abs(sR - targetR);

        if (errT <= toleranceT && errR <= toleranceR) {
            return true;
        }

        const_cast<MotionPlanner&>(planner).process();
        advanceMicros(1000);
        iters++;
    }

    double curT, curR;
    planner.getCurrentPosition(curT, curR);
    int32_t sT = (int32_t)round(curT * STEPS_PER_RAD_T);
    int32_t sR = (int32_t)round(curR * STEPS_PER_MM_R);
    std::cout << "\nDEBUG: waitForTarget timeout! Target=(" << targetT << "," << targetR
              << ") Got=(" << sT << "," << sR << ") err=(" << (sT-targetT) << "," << (sR-targetR) << ")" << std::endl;

    return false;
}

// Helper to run a loaded pattern and return the total time
double runPatternFileInternal(MotionPlanner& planner, ThrReader& reader,
                             const std::vector<std::pair<int32_t, int32_t>>& expectedStepTargets) {
    planner.start();

    int iterations = 0;
    int maxIterations = 2000000;
    uint32_t totalSegments = expectedStepTargets.size();

    // Run the pattern to completion
    while ((planner.isRunning() || !planner.isIdle()) && iterations < maxIterations) {
        planner.process();
        advanceMicros(10000); // 10ms steps
        iterations++;
    }

    if (planner.getCompletedCount() < totalSegments) {
        std::cout << "DEBUG: segment count mismatch: completed=" << planner.getCompletedCount()
                  << " total=" << totalSegments << std::endl;
        return -1.0;
    }

    // Verify final position matches expected final target
    // For small patterns, use tight tolerance; for large patterns, allow proportional error
    int32_t toleranceT = std::max(3, (int)(totalSegments / 1000));  // 0.1% or 3 steps minimum
    int32_t toleranceR = std::max(2, (int)(totalSegments / 2000));  // 0.05% or 2 steps minimum

    if (!expectedStepTargets.empty()) {
        int32_t finalTargetT = expectedStepTargets.back().first;
        int32_t finalTargetR = expectedStepTargets.back().second;

        // Wait for ISR to finish executing remaining steps
        if (!waitForTarget(planner, finalTargetT, finalTargetR, 50000, toleranceT, toleranceR)) {
            double curT, curR;
            planner.getCurrentPosition(curT, curR);
            int32_t actualT = (int32_t)round(curT * STEPS_PER_RAD_T);
            int32_t actualR = (int32_t)round(curR * STEPS_PER_MM_R);
            std::cout << "DEBUG: Final position mismatch! Target=(" << finalTargetT << "," << finalTargetR
                      << ") Got=(" << actualT << "," << actualR
                      << ") err=(" << (actualT - finalTargetT) << "," << (actualR - finalTargetR)
                      << ") tolerance=(" << toleranceT << "," << toleranceR << ")" << std::endl;
            return -1.0;
        }
    }

    return g_mockMicros.load() / 1000000.0;
}

bool testPatternFile(const std::string& filepath) {
    std::cout << "\n=== Test: Pattern File ===" << std::endl;
    std::cout << "Loading: " << filepath << std::endl;
    resetMock();

    ThrReader reader;
    reader.setMaxRho(R_MAX);

    if (!reader.load(filepath)) {
        std::cout << "Failed to load file" << std::endl;
        return false;
    }

    // Capture all target steps first
    std::vector<std::pair<int32_t, int32_t>> expectedStepTargets;
    reader.reset();
    double theta, rho;
    while (reader.getNextPosition(theta, rho)) {
        int32_t targetT = (int32_t)(theta * STEPS_PER_RAD_T);
        int32_t targetR = (int32_t)(rho * STEPS_PER_MM_R);
        expectedStepTargets.push_back({targetT, targetR});
    }

    auto runAtSpeed = [&](double speedMult) -> double {
        resetMock();
        MotionPlanner planner;
        planner.init(STEPS_PER_MM_R, STEPS_PER_RAD_T, R_MAX,
                     R_MAX_VEL, R_MAX_ACCEL, R_MAX_JERK,
                     T_MAX_VEL, T_MAX_ACCEL, T_MAX_JERK);
        planner.setSpeedMultiplier(speedMult);
        
        reader.reset();
        bool plannerStarted = false;
        int added = 0;
        while (reader.getNextPosition(theta, rho)) {
            while (!planner.addSegment(theta, rho)) {
                if (!plannerStarted) {
                    planner.recalculate();
                    planner.start();
                    plannerStarted = true;
                }
                planner.process();
                advanceMicros(10000);
            }
            added++;
            
            // Periodically recalculate to keep motion smooth
            if (added % 8 == 0) {
                planner.recalculate();
            }
        }
        planner.setEndOfPattern(true);
        planner.recalculate();
        
        if (!plannerStarted) {
            planner.start();
        }
        
        return runPatternFileInternal(planner, reader, expectedStepTargets);
    };

    std::cout << "Running at full speed (1.0)..." << std::endl;
    double time10 = runAtSpeed(1.0);
    if (time10 < 0) {
        std::cout << "FAIL: Full speed run failed" << std::endl;
        return false;
    }
    std::cout << "Time: " << time10 << "s" << std::endl;

    std::cout << "Running at half speed (0.5)..." << std::endl;
    double time05 = runAtSpeed(0.5);
    if (time05 < 0) {
        std::cout << "FAIL: Half speed run failed" << std::endl;
        return false;
    }
    std::cout << "Time: " << time05 << "s" << std::endl;

    double ratio = time05 / time10;
    bool ratioPassed = ratio > 1.2; // Should be significantly slower
    if (ratioPassed) {
        std::cout << "PASS" << std::endl;
    } else {
        std::cout << "FAIL" << std::endl;
    }

    return ratioPassed;
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char* argv[]) {
    std::cout << "========================================" << std::endl;
    std::cout << "MotionPlanner Desktop Test Harness" << std::endl;
    std::cout << "========================================" << std::endl;

    bool allPassed = true;

    // Run S-curve tests
    allPassed &= testSCurveBasic();
    allPassed &= testDecelDistance();
    allPassed &= testMaxEntryVel();

    // Run MotionPlanner tests
    allPassed &= testMotionPlannerBasic();
    allPassed &= testDirectionReversal();
    allPassed &= testSpeedMultiplier();

    // If a pattern file was provided, test it
    if (argc > 1) {
        allPassed &= testPatternFile(argv[1]);
    }

    std::cout << "\n========================================" << std::endl;
    if (allPassed) {
        std::cout << "ALL TESTS PASSED" << std::endl;
    } else {
        std::cout << "SOME TESTS FAILED" << std::endl;
    }
    std::cout << "========================================" << std::endl;

    return allPassed ? 0 : 1;
}
