#include "MotionPlanner.hpp"
#include <algorithm>

bool MotionPlanner::calculateSCurveProfile(
    double distance,
    double startVel,
    double endVel,
    double maxVel,
    double maxAccel,
    double maxJerk,
    SMotionProfile &profile) {

    profile.reset();

    // Validate inputs
    if (distance <= 0.0 || maxVel <= 0.0 || maxAccel <= 0.0 || maxJerk <= 0.0) {
        return false;
    }

    profile.distance = distance;
    profile.startVel = startVel;
    profile.endVel = endVel;
    profile.maxVel = maxVel;
    profile.accel = maxAccel;
    profile.decel = maxAccel;
    profile.jerk = maxJerk;

    // Calculate jerk time (time to reach max acceleration from zero)
    double t_j = maxAccel / maxJerk;

    // Simplified S-curve: assume symmetric acceleration and deceleration
    // Full implementation would handle asymmetric cases

    // Time to reach max velocity from start velocity with S-curve
    double dv_accel = maxVel - startVel;
    double dv_decel = maxVel - endVel;

    // Check if we can reach max velocity
    // Distance during acceleration with S-curve (simplified):
    // s = v0*t + 0.5*a*t^2 + (1/6)*j*t^3

    // For now, use simplified trapezoidal approximation
    // Full S-curve math is complex and requires iterative solving

    // Phase durations (simplified)
    profile.t_j1 = std::min(t_j, sqrt(dv_accel / maxJerk));
    profile.t_j2 = profile.t_j1;

    double a_reached = maxJerk * profile.t_j1;
    double t_const_accel = (dv_accel - a_reached * profile.t_j1) / a_reached;
    profile.t_a = std::max(0.0, t_const_accel);

    profile.t_j3 = profile.t_j1;
    profile.t_j4 = profile.t_j1;
    profile.t_d = profile.t_a;

    // Calculate distances for each phase
    double s_accel = startVel * (2 * profile.t_j1 + profile.t_a) +
                     0.5 * a_reached * profile.t_a * profile.t_a +
                     (1.0 / 6.0) * maxJerk * profile.t_j1 * profile.t_j1 * profile.t_j1;

    double s_decel = endVel * (2 * profile.t_j3 + profile.t_d) +
                     0.5 * a_reached * profile.t_d * profile.t_d +
                     (1.0 / 6.0) * maxJerk * profile.t_j3 * profile.t_j3 * profile.t_j3;

    // Cruise distance
    profile.accelDist = s_accel;
    profile.decelDist = s_decel;
    profile.cruiseDist = std::max(0.0, distance - s_accel - s_decel);

    // Cruise time
    profile.t_v = profile.cruiseDist / maxVel;

    // Total time
    profile.accelTime = 2 * profile.t_j1 + profile.t_a;
    profile.cruiseTime = profile.t_v;
    profile.decelTime = 2 * profile.t_j3 + profile.t_d;
    profile.totalTime = profile.accelTime + profile.cruiseTime + profile.decelTime;

    profile.useScurve = true;

    return true;
}

bool MotionPlanner::solveJerkLimitedTime(
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
    double &t_d) {

    // Full analytical solution for jerk-limited profiles
    // This is a simplified placeholder
    // Production code would implement the complete solution

    t_j1 = aMax / jMax;
    t_a = (vMax - v0) / aMax - t_j1;
    t_j2 = t_j1;
    t_d = t_a;
    t_v = distance / vMax - (t_j1 + t_a);

    return true;
}
