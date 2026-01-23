#pragma once

#include "SCurve.hpp"
#include <cmath>
#include <iostream>
#include <iomanip>
#include <string>

struct ValidationResult {
    bool passed;
    float maxVelocity;
    float maxAccel;
    float maxJerk;
    float finalPosition;
    float finalVelocity;
    std::string errorMessage;
};

class ProfileValidator {
public:
    ProfileValidator() = default;

    // Validate an S-curve profile against limits
    ValidationResult validate(const SCurve::Profile& profile,
                              float expectedDistance,
                              float vMax, float aMax, float jMax,
                              float tolerance = 0.01f) {
        ValidationResult result;
        result.passed = true;
        result.maxVelocity = 0.0f;
        result.maxAccel = 0.0f;
        result.maxJerk = jMax;  // Jerk is applied discretely at phase boundaries
        result.finalPosition = 0.0f;
        result.finalVelocity = 0.0f;

        if (profile.totalTime <= 0.0f && expectedDistance > 0.0001f) {
            result.passed = false;
            result.errorMessage = "Zero duration for non-zero distance";
            return result;
        }

        // Sample profile at many points
        int numSamples = std::max(100, (int)(profile.totalTime * 10000.0f));
        float dt = profile.totalTime / numSamples;

        float prevVel = profile.v[0];
        float prevAccel = 0.0f;
        float prevPos = 0.0f;

        for (int i = 0; i <= numSamples; i++) {
            float t = i * dt;

            float pos = SCurve::getPosition(profile, t);
            float vel = SCurve::getVelocity(profile, t);
            float accel = SCurve::getAcceleration(profile, t);

            // Track maximums
            result.maxVelocity = std::max(result.maxVelocity, fabsf(vel));
            result.maxAccel = std::max(result.maxAccel, fabsf(accel));

            // Estimate jerk from acceleration change
            if (i > 0 && dt > 0.0f) {
                float jerk = (accel - prevAccel) / dt;
                result.maxJerk = std::max(result.maxJerk, fabsf(jerk));
            }

            prevVel = vel;
            prevAccel = accel;
            prevPos = pos;
        }

        result.finalPosition = prevPos;
        result.finalVelocity = prevVel;

        // Check velocity limit
        if (result.maxVelocity > vMax * (1.0f + tolerance)) {
            result.passed = false;
            result.errorMessage += "Velocity exceeded: " + std::to_string(result.maxVelocity) +
                                   " > " + std::to_string(vMax) + ". ";
        }

        // Check acceleration limit
        if (result.maxAccel > aMax * (1.0f + tolerance)) {
            result.passed = false;
            result.errorMessage += "Acceleration exceeded: " + std::to_string(result.maxAccel) +
                                   " > " + std::to_string(aMax) + ". ";
        }

        // Check jerk limit (with more tolerance due to estimation)
        if (result.maxJerk > jMax * (1.0f + tolerance * 2.0f)) {
            result.passed = false;
            result.errorMessage += "Jerk exceeded: " + std::to_string(result.maxJerk) +
                                   " > " + std::to_string(jMax) + ". ";
        }

        // Check final position
        if (fabsf(result.finalPosition - expectedDistance) > expectedDistance * tolerance + 0.001f) {
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
            if (profile.t[i] > 0.0001f) {
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
