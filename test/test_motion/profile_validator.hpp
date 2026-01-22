#pragma once

#include "SCurve.hpp"
#include <cmath>
#include <iostream>
#include <iomanip>
#include <string>

struct ValidationResult {
    bool passed;
    double maxVelocity;
    double maxAccel;
    double maxJerk;
    double finalPosition;
    double finalVelocity;
    std::string errorMessage;
};

class ProfileValidator {
public:
    ProfileValidator() = default;

    // Validate an S-curve profile against limits
    ValidationResult validate(const SCurve::Profile& profile,
                              double expectedDistance,
                              double vMax, double aMax, double jMax,
                              double tolerance = 0.01) {
        ValidationResult result;
        result.passed = true;
        result.maxVelocity = 0;
        result.maxAccel = 0;
        result.maxJerk = jMax;  // Jerk is applied discretely at phase boundaries
        result.finalPosition = 0;
        result.finalVelocity = 0;

        if (profile.totalTime <= 0 && expectedDistance > 0.0001) {
            result.passed = false;
            result.errorMessage = "Zero duration for non-zero distance";
            return result;
        }

        // Sample profile at many points
        int numSamples = std::max(100, (int)(profile.totalTime * 10000));
        double dt = profile.totalTime / numSamples;

        double prevVel = profile.v[0];
        double prevAccel = 0;
        double prevPos = 0;

        for (int i = 0; i <= numSamples; i++) {
            double t = i * dt;

            double pos = SCurve::getPosition(profile, t);
            double vel = SCurve::getVelocity(profile, t);
            double accel = SCurve::getAcceleration(profile, t);

            // Track maximums
            result.maxVelocity = std::max(result.maxVelocity, std::abs(vel));
            result.maxAccel = std::max(result.maxAccel, std::abs(accel));

            // Estimate jerk from acceleration change
            if (i > 0 && dt > 0) {
                double jerk = (accel - prevAccel) / dt;
                result.maxJerk = std::max(result.maxJerk, std::abs(jerk));
            }

            prevVel = vel;
            prevAccel = accel;
            prevPos = pos;
        }

        result.finalPosition = prevPos;
        result.finalVelocity = prevVel;

        // Check velocity limit
        if (result.maxVelocity > vMax * (1.0 + tolerance)) {
            result.passed = false;
            result.errorMessage += "Velocity exceeded: " + std::to_string(result.maxVelocity) +
                                   " > " + std::to_string(vMax) + ". ";
        }

        // Check acceleration limit
        if (result.maxAccel > aMax * (1.0 + tolerance)) {
            result.passed = false;
            result.errorMessage += "Acceleration exceeded: " + std::to_string(result.maxAccel) +
                                   " > " + std::to_string(aMax) + ". ";
        }

        // Check jerk limit (with more tolerance due to estimation)
        if (result.maxJerk > jMax * (1.0 + tolerance * 2)) {
            result.passed = false;
            result.errorMessage += "Jerk exceeded: " + std::to_string(result.maxJerk) +
                                   " > " + std::to_string(jMax) + ". ";
        }

        // Check final position
        if (std::abs(result.finalPosition - expectedDistance) > expectedDistance * tolerance + 0.001) {
            result.passed = false;
            result.errorMessage += "Position error: " + std::to_string(result.finalPosition) +
                                   " != " + std::to_string(expectedDistance) + ". ";
        }

        return result;
    }

    // Print detailed profile info
    void printProfile(const SCurve::Profile& profile, const std::string& label) {
        std::cout << std::fixed << std::setprecision(4);
        std::cout << "\n=== " << label << " ===" << std::endl;
        std::cout << "Total time: " << profile.totalTime << "s" << std::endl;
        std::cout << "Total distance: " << profile.totalDistance << std::endl;
        std::cout << "\nPhase durations:" << std::endl;

        const char* phaseNames[] = {
            "1 (jerk+)", "2 (const accel)", "3 (jerk-)",
            "4 (cruise)",
            "5 (jerk-)", "6 (const decel)", "7 (jerk+)"
        };

        for (int i = 0; i < 7; i++) {
            if (profile.t[i] > 0.0001) {
                std::cout << "  Phase " << phaseNames[i] << ": "
                          << profile.t[i] << "s, ends at t=" << profile.tEnd[i]
                          << ", pos=" << profile.posEnd[i] << std::endl;
            }
        }

        std::cout << "\nVelocities: ";
        for (int i = 0; i < 8; i++) {
            std::cout << profile.v[i];
            if (i < 7) std::cout << " -> ";
        }
        std::cout << std::endl;

        std::cout << "Accelerations: ";
        for (int i = 0; i < 8; i++) {
            std::cout << profile.a[i];
            if (i < 7) std::cout << " -> ";
        }
        std::cout << std::endl;
    }

    // Print validation result
    void printValidation(const ValidationResult& result) {
        if (result.passed) {
            std::cout << "  PASS";
        } else {
            std::cout << "  FAIL: " << result.errorMessage;
        }
        std::cout << std::endl;
        std::cout << "  Max vel: " << result.maxVelocity
                  << ", Max accel: " << result.maxAccel
                  << ", Max jerk: " << result.maxJerk << std::endl;
    }
};
