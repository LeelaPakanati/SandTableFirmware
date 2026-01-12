#pragma once
#include <cmath>

// Motion profile base structure
struct MotionProfile {
    double startVel;
    double maxVel;
    double endVel;
    double accel;
    double decel;
    double distance;
    double accelDist;
    double cruiseDist;
    double decelDist;
    double accelTime;
    double cruiseTime;
    double decelTime;
    double totalTime;

    void reset() {
        startVel = maxVel = endVel = 0.0;
        accel = decel = distance = 0.0;
        accelDist = cruiseDist = decelDist = 0.0;
        accelTime = cruiseTime = decelTime = totalTime = 0.0;
    }
};

// Enhanced motion profile with S-curve (jerk-limited) support
struct SMotionProfile : MotionProfile {
    double jerk;           // Max jerk (m/s³ or rad/s³)
    double t_j1, t_j2;     // Jerk phase times for acceleration
    double t_j3, t_j4;     // Jerk phase times for deceleration
    double t_a, t_v, t_d;  // Acceleration, constant velocity, deceleration times
    bool useScurve;        // Whether to use S-curve or standard trapezoidal

    void reset() {
        MotionProfile::reset();
        jerk = 0.0;
        t_j1 = t_j2 = t_j3 = t_j4 = 0.0;
        t_a = t_v = t_d = 0.0;
        useScurve = false;
    }
};

/**
 * MotionPlanner class provides advanced motion planning algorithms
 * for smooth trajectory generation with jerk-limited S-curves.
 */
class MotionPlanner {
public:
    /**
     * Calculate a jerk-limited S-curve motion profile.
     *
     * S-curve profiles have 7 phases:
     * 1. Jerk up (acceleration increases)
     * 2. Constant acceleration
     * 3. Jerk down (acceleration decreases to 0)
     * 4. Constant velocity (cruise)
     * 5. Jerk down (deceleration increases)
     * 6. Constant deceleration
     * 7. Jerk up (deceleration decreases to 0)
     *
     * @param distance Total distance to travel
     * @param startVel Starting velocity
     * @param endVel Ending velocity
     * @param maxVel Maximum velocity
     * @param maxAccel Maximum acceleration
     * @param maxJerk Maximum jerk
     * @param profile Output profile structure
     * @return true if profile calculated successfully
     */
    static bool calculateSCurveProfile(
        double distance,
        double startVel,
        double endVel,
        double maxVel,
        double maxAccel,
        double maxJerk,
        SMotionProfile &profile);

private:
    /**
     * Solve for time parameters in jerk-limited profile.
     * Internal helper for S-curve calculation.
     */
    static bool solveJerkLimitedTime(
        double distance,
        double v0,
        double v1,
        double vMax,
        double aMax,
        double jMax,
        double &t_j1,
        double &t_a,
        double &t_v,
        double &t_j2,
        double &t_d);
};
