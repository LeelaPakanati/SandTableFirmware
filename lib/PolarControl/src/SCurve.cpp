#include "SCurve.hpp"
#include <algorithm>

// Distance during jerk phase: d = v0*t + 0.5*a0*t² + (1/6)*j*t³
double SCurve::jerkPhaseDistance(double v0, double a0, double j, double t) {
    return v0 * t + 0.5 * a0 * t * t + (1.0 / 6.0) * j * t * t * t;
}

// Distance during constant acceleration: d = v0*t + 0.5*a*t²
double SCurve::constAccelDistance(double v0, double a, double t) {
    return v0 * t + 0.5 * a * t * t;
}

// Velocity after jerk phase: v = v0 + a0*t + 0.5*j*t²
double SCurve::jerkPhaseVelocity(double v0, double a0, double j, double t) {
    return v0 + a0 * t + 0.5 * j * t * t;
}

bool SCurve::calculate(
    double distance,
    double vStart,
    double vEnd,
    double vMax,
    double aMax,
    double jMax,
    Profile& p
) {
    // Store constraints
    p.jerk = jMax;
    p.maxAccel = aMax;
    p.maxVelocity = vMax;

    // Time to reach max acceleration with jerk limit
    double tJerk = aMax / jMax;

    // Velocity change during one jerk phase (phases 1 or 3)
    double vJerk = 0.5 * jMax * tJerk * tJerk;

    // Velocity change during accel ramp (phases 1+2+3) if we reach max accel
    // This is the minimum velocity change if we use full acceleration
    double vAccelMin = 2.0 * vJerk;  // Just the jerk phases, no const accel

    // Calculate required velocity change for accel and decel
    double vCruise = vMax;

    // Check if we can reach cruise velocity
    double vAccelNeeded = vCruise - vStart;
    double vDecelNeeded = vCruise - vEnd;

    // If we can't reach cruise, find the peak velocity
    // This is a simplified calculation - find max achievable velocity
    if (vAccelNeeded + vDecelNeeded > 0) {
        // Distance needed for symmetric accel/decel with full jerk phases
        double dAccelFull = vStart * (2.0 * tJerk) + 2.0 * jerkPhaseDistance(0, 0, jMax, tJerk);
        double dDecelFull = vEnd * (2.0 * tJerk) + 2.0 * jerkPhaseDistance(0, 0, jMax, tJerk);

        // Start with max velocity and reduce if needed
        while (vCruise > std::max(vStart, vEnd)) {
            // Calculate distances for acceleration and deceleration phases
            double accelDist = 0;
            double decelDist = 0;

            // Acceleration phase distances
            double deltaVAccel = vCruise - vStart;
            if (deltaVAccel > 0) {
                if (deltaVAccel <= vAccelMin) {
                    // Can't reach max accel, reduced jerk profile
                    double tJ = sqrt(deltaVAccel / jMax);
                    accelDist = 2.0 * jerkPhaseDistance(vStart, 0, jMax, tJ);
                    accelDist += (vStart + vJerk) * 0;  // No const accel phase
                } else {
                    // Full profile with const accel phase
                    double vConstAccel = deltaVAccel - vAccelMin;
                    double tConstAccel = vConstAccel / aMax;
                    accelDist = jerkPhaseDistance(vStart, 0, jMax, tJerk);  // Phase 1
                    accelDist += constAccelDistance(vStart + vJerk, aMax, tConstAccel);  // Phase 2
                    accelDist += jerkPhaseDistance(vStart + vJerk + vConstAccel, aMax, -jMax, tJerk);  // Phase 3
                }
            }

            // Deceleration phase distances
            double deltaVDecel = vCruise - vEnd;
            if (deltaVDecel > 0) {
                if (deltaVDecel <= vAccelMin) {
                    double tJ = sqrt(deltaVDecel / jMax);
                    decelDist = 2.0 * jerkPhaseDistance(vCruise, 0, -jMax, tJ);
                } else {
                    double vConstDecel = deltaVDecel - vAccelMin;
                    double tConstDecel = vConstDecel / aMax;
                    decelDist = jerkPhaseDistance(vCruise, 0, -jMax, tJerk);  // Phase 5
                    decelDist += constAccelDistance(vCruise - vJerk, -aMax, tConstDecel);  // Phase 6
                    decelDist += jerkPhaseDistance(vEnd + vJerk, -aMax, jMax, tJerk);  // Phase 7
                }
            }

            if (accelDist + decelDist <= distance) {
                break;  // Found achievable cruise velocity
            }

            vCruise -= 0.5;  // Reduce and try again
            if (vCruise < 0.1) {
                vCruise = std::max(vStart, vEnd);
                break;
            }
        }
    }

    // Now calculate actual profile with determined cruise velocity
    // Initialize all phases to zero
    for (int i = 0; i < 7; i++) {
        p.t[i] = 0;
        p.tEnd[i] = 0;
    }
    for (int i = 0; i < 8; i++) {
        p.v[i] = 0;
        p.a[i] = 0;
    }

    p.v[0] = vStart;
    p.a[0] = 0;

    double pos = 0;
    double currentTime = 0;
    int phase = 0;

    // Phase 1: Jerk+ (accelerating)
    double deltaVAccel = vCruise - vStart;
    if (deltaVAccel > 0.001) {
        if (deltaVAccel <= vAccelMin) {
            // Reduced jerk - can't reach max accel
            p.t[0] = sqrt(deltaVAccel / jMax);
            p.t[1] = 0;
            p.t[2] = p.t[0];
        } else {
            p.t[0] = tJerk;
            p.t[1] = (deltaVAccel - vAccelMin) / aMax;
            p.t[2] = tJerk;
        }
    }

    // Calculate velocities and positions through accel phases
    // Phase 1
    p.v[1] = jerkPhaseVelocity(p.v[0], 0, jMax, p.t[0]);
    p.a[1] = jMax * p.t[0];
    pos += jerkPhaseDistance(p.v[0], 0, jMax, p.t[0]);
    currentTime += p.t[0];
    p.tEnd[0] = currentTime;

    // Phase 2
    p.v[2] = p.v[1] + p.a[1] * p.t[1];
    p.a[2] = p.a[1];
    pos += constAccelDistance(p.v[1], p.a[1], p.t[1]);
    currentTime += p.t[1];
    p.tEnd[1] = currentTime;

    // Phase 3
    p.v[3] = jerkPhaseVelocity(p.v[2], p.a[2], -jMax, p.t[2]);
    p.a[3] = 0;  // Should be zero at end of accel
    pos += jerkPhaseDistance(p.v[2], p.a[2], -jMax, p.t[2]);
    currentTime += p.t[2];
    p.tEnd[2] = currentTime;

    // Phase 4: Cruise
    double cruiseDist = distance - pos;

    // Calculate decel distance
    double decelDist = 0;
    double deltaVDecel = vCruise - vEnd;
    double t5, t6, t7;
    if (deltaVDecel > 0.001) {
        if (deltaVDecel <= vAccelMin) {
            t5 = sqrt(deltaVDecel / jMax);
            t6 = 0;
            t7 = t5;
        } else {
            t5 = tJerk;
            t6 = (deltaVDecel - vAccelMin) / aMax;
            t7 = tJerk;
        }

        // Estimate decel distance
        double v5 = jerkPhaseVelocity(vCruise, 0, -jMax, t5);
        decelDist += jerkPhaseDistance(vCruise, 0, -jMax, t5);
        double a5 = -jMax * t5;
        double v6 = v5 + a5 * t6;
        decelDist += constAccelDistance(v5, a5, t6);
        decelDist += jerkPhaseDistance(v6, a5, jMax, t7);
    } else {
        t5 = t6 = t7 = 0;
    }

    cruiseDist -= decelDist;
    if (cruiseDist < 0) cruiseDist = 0;

    p.t[3] = (vCruise > 0.001) ? cruiseDist / vCruise : 0;
    p.v[4] = vCruise;
    p.a[4] = 0;
    pos += cruiseDist;
    currentTime += p.t[3];
    p.tEnd[3] = currentTime;

    // Phase 5-7: Deceleration
    p.t[4] = t5;
    p.t[5] = t6;
    p.t[6] = t7;

    // Phase 5
    p.v[5] = jerkPhaseVelocity(p.v[4], 0, -jMax, p.t[4]);
    p.a[5] = -jMax * p.t[4];
    currentTime += p.t[4];
    p.tEnd[4] = currentTime;

    // Phase 6
    p.v[6] = p.v[5] + p.a[5] * p.t[5];
    p.a[6] = p.a[5];
    currentTime += p.t[5];
    p.tEnd[5] = currentTime;

    // Phase 7
    p.v[7] = vEnd;
    p.a[7] = 0;
    currentTime += p.t[6];
    p.tEnd[6] = currentTime;

    p.totalTime = currentTime;
    p.totalDistance = distance;

    return true;
}

double SCurve::getVelocity(const Profile& p, double t) {
    if (t <= 0) return p.v[0];
    if (t >= p.totalTime) return p.v[7];

    // Find which phase we're in
    int phase = 0;
    double tPhase = t;
    for (int i = 0; i < 7; i++) {
        if (t <= p.tEnd[i]) {
            phase = i;
            tPhase = (i == 0) ? t : t - p.tEnd[i - 1];
            break;
        }
    }

    double v0 = p.v[phase];
    double a0 = p.a[phase];
    double j = 0;

    switch (phase) {
        case 0: j = p.jerk; break;       // Jerk+
        case 1: j = 0; break;            // Const accel
        case 2: j = -p.jerk; break;      // Jerk-
        case 3: j = 0; break;            // Cruise
        case 4: j = -p.jerk; break;      // Jerk-
        case 5: j = 0; break;            // Const decel
        case 6: j = p.jerk; break;       // Jerk+
    }

    return jerkPhaseVelocity(v0, a0, j, tPhase);
}

double SCurve::getPosition(const Profile& p, double t) {
    if (t <= 0) return 0;
    if (t >= p.totalTime) return p.totalDistance;

    double pos = 0;

    // Sum completed phases
    for (int i = 0; i < 7; i++) {
        double tStart = (i == 0) ? 0 : p.tEnd[i - 1];
        if (t <= p.tEnd[i]) {
            // We're in this phase
            double tPhase = t - tStart;
            double v0 = p.v[i];
            double a0 = p.a[i];
            double j = 0;

            switch (i) {
                case 0: j = p.jerk; break;
                case 1: j = 0; break;
                case 2: j = -p.jerk; break;
                case 3: j = 0; break;
                case 4: j = -p.jerk; break;
                case 5: j = 0; break;
                case 6: j = p.jerk; break;
            }

            if (j != 0) {
                pos += jerkPhaseDistance(v0, a0, j, tPhase);
            } else {
                pos += constAccelDistance(v0, a0, tPhase);
            }
            return pos;
        }

        // Add full phase distance
        double tPhase = p.t[i];
        double v0 = p.v[i];
        double a0 = p.a[i];
        double j = 0;

        switch (i) {
            case 0: j = p.jerk; break;
            case 1: j = 0; break;
            case 2: j = -p.jerk; break;
            case 3: j = 0; break;
            case 4: j = -p.jerk; break;
            case 5: j = 0; break;
            case 6: j = p.jerk; break;
        }

        if (j != 0) {
            pos += jerkPhaseDistance(v0, a0, j, tPhase);
        } else {
            pos += constAccelDistance(v0, a0, tPhase);
        }
    }

    return pos;
}

double SCurve::getAcceleration(const Profile& p, double t) {
    if (t <= 0) return p.a[0];
    if (t >= p.totalTime) return 0;

    // Find which phase we're in
    int phase = 0;
    double tPhase = t;
    for (int i = 0; i < 7; i++) {
        if (t <= p.tEnd[i]) {
            phase = i;
            tPhase = (i == 0) ? t : t - p.tEnd[i - 1];
            break;
        }
    }

    double a0 = p.a[phase];
    double j = 0;

    switch (phase) {
        case 0: j = p.jerk; break;
        case 1: j = 0; break;
        case 2: j = -p.jerk; break;
        case 3: j = 0; break;
        case 4: j = -p.jerk; break;
        case 5: j = 0; break;
        case 6: j = p.jerk; break;
    }

    return a0 + j * tPhase;
}
