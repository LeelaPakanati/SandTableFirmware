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
static constexpr int STEPS_PER_RAD_T = 3000;     // steps per rad for theta

// Motion limits
static constexpr double R_MAX_VEL = 30.0;        // mm/s
static constexpr double R_MAX_ACCEL = 20.0;      // mm/s²
static constexpr double R_MAX_JERK = 100.0;      // mm/s³
static constexpr double T_MAX_VEL = 1.0;         // rad/s
static constexpr double T_MAX_ACCEL = 2.0;       // rad/s²
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

bool testPatternFile(const std::string& filepath) {
    std::cout << "\n=== Test: Pattern File ===" << std::endl;
    std::cout << "Loading: " << filepath << std::endl;

    ThrReader reader;
    reader.setMaxRho(R_MAX);

    if (!reader.load(filepath)) {
        std::cout << "Failed to load file" << std::endl;
        return false;
    }

    MotionPlanner planner;
    planner.init(STEPS_PER_MM_R, STEPS_PER_RAD_T, R_MAX,
                 R_MAX_VEL, R_MAX_ACCEL, R_MAX_JERK,
                 T_MAX_VEL, T_MAX_ACCEL, T_MAX_JERK);

    // Feed segments to planner
    int segmentsAdded = 0;
    double theta, rho;
    bool started = false;

    while (reader.getNextPosition(theta, rho)) {
        // Keep buffer moderately full
        if (!planner.hasSpace()) {
            planner.recalculate();
            
            if (!started) {
                planner.start();
                started = true;
                setMicros(0); // Reset time for simulation tracking
            }

            int waitIters = 0;
            while (!planner.hasSpace() && waitIters < 100000) { // Increased timeout
                // Process to make room
                planner.process();
                advanceMicros(1000);
                waitIters++;
            }
            
            if (waitIters >= 100000) {
                std::cout << "FAIL: Buffer clear timeout" << std::endl;
                return false;
            }
        }

        if (planner.addSegment(theta, rho)) {
            segmentsAdded++;
        }

        // Mark end of pattern when we've added all segments
        if (reader.currentIndex() >= reader.size()) {
            planner.setEndOfPattern(true);
        }
    }

    std::cout << "Added " << segmentsAdded << " segments" << std::endl;

    // Start and run to completion
    planner.setEndOfPattern(true);
    planner.recalculate();
    
    if (!started) {
        planner.start();
        started = true;
        setMicros(0);
    }
    int iterations = 0;
    int maxIterations = 1000000;

    while (planner.isRunning() && iterations < maxIterations) {
        planner.process();
        advanceMicros(20000);
        iterations++;

        if (iterations % 1000 == 0) {
            double t, r;
            planner.getCurrentPosition(t, r);
            int progress = (planner.getCompletedCount() * 100) / segmentsAdded;
            std::cout << "\r  Progress: " << progress << "% (" << planner.getCompletedCount()
                      << "/" << segmentsAdded << ")" << std::flush;
        }
    }

    std::cout << std::endl;

    bool passed = planner.getCompletedCount() == (uint32_t)segmentsAdded;
    double totalTime = g_mockMicros.load() / 1000000.0;

    std::cout << "Completed: " << planner.getCompletedCount() << "/" << segmentsAdded
              << " segments in " << totalTime << " seconds"
              << (passed ? " PASS" : " FAIL") << std::endl;

    return passed;
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
