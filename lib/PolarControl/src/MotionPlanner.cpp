#include "MotionPlanner.hpp"
#include <cmath>
#include <cstdio>
#ifndef NATIVE_BUILD
#include <Arduino.h>
#include <esp_timer.h>
#else
#include "esp32_mock.hpp"
#endif

// Step pulse width in microseconds
static constexpr uint32_t STEP_PULSE_WIDTH_US = 2;

// Direction setup time before step pulse
static constexpr uint32_t DIR_SETUP_TIME_US = 1;

MotionPlanner::MotionPlanner()
    : m_stepsPerMmR(100)
    , m_stepsPerRadT(3000)
    , m_maxRho(450.0)
    , m_rMaxVel(30.0)
    , m_rMaxAccel(20.0)
    , m_rMaxJerk(100.0)
    , m_tMaxVel(1.0)
    , m_tMaxAccel(2.0)
    , m_tMaxJerk(10.0)
    , m_speedMultiplier(1.0)
    , m_segmentHead(0)
    , m_segmentTail(0)
    , m_segmentCount(0)
    , m_queuedTSteps(0)
    , m_queuedRSteps(0)
    , m_executedTSteps(0)
    , m_executedRSteps(0)
    , m_targetTheta(0.0)
    , m_targetRho(0.0)
    , m_stepQueueHead(0)
    , m_stepQueueTail(0)
    , m_segmentStartTime(0)
    , m_segmentElapsed(0.0)
    , m_running(false)
    , m_endOfPattern(false)
    , m_completedCount(0)
    , m_timerHandle(nullptr)
{
    // Initialize segments
    for (int i = 0; i < SEGMENT_BUFFER_SIZE; i++) {
        m_segments[i].calculated = false;
        m_segments[i].executing = false;
    }
}

MotionPlanner::~MotionPlanner() {
    stop();
}

void MotionPlanner::init(int stepsPerMmR, int stepsPerRadT, double maxRho,
                         double rMaxVel, double rMaxAccel, double rMaxJerk,
                         double tMaxVel, double tMaxAccel, double tMaxJerk) {
    m_stepsPerMmR = stepsPerMmR;
    m_stepsPerRadT = stepsPerRadT;
    m_maxRho = maxRho;

    m_rMaxVel = rMaxVel;
    m_rMaxAccel = rMaxAccel;
    m_rMaxJerk = rMaxJerk;

    m_tMaxVel = tMaxVel;
    m_tMaxAccel = tMaxAccel;
    m_tMaxJerk = tMaxJerk;

#ifndef NATIVE_BUILD
    // Setup GPIO pins
    pinMode(T_STEP_PIN, OUTPUT);
    pinMode(T_DIR_PIN, OUTPUT);
    pinMode(R_STEP_PIN, OUTPUT);
    pinMode(R_DIR_PIN, OUTPUT);

    digitalWrite(T_STEP_PIN, LOW);
    digitalWrite(R_STEP_PIN, LOW);
#endif

    // Reset positions
    m_queuedTSteps = 0;
    m_queuedRSteps = 0;
    m_executedTSteps = 0;
    m_executedRSteps = 0;
    m_targetTheta = 0.0;
    m_targetRho = 0.0;

    // Reset buffer
    m_segmentHead = 0;
    m_segmentTail = 0;
    m_segmentCount = 0;
    m_stepQueueHead = 0;
    m_stepQueueTail = 0;
}

bool MotionPlanner::addSegment(double theta, double rho) {
    if (!hasSpace()) {
        return false;
    }

    // Clamp rho to valid range
    rho = std::max(0.0, std::min(rho, m_maxRho));

    Segment& seg = m_segments[m_segmentHead];
    seg.targetTheta = theta;
    seg.targetRho = rho;
    seg.calculated = false;
    seg.executing = false;

    // Reset execution/generation state for new segment
    seg.thetaPhaseIdx = 0;
    seg.rhoPhaseIdx = 0;
    seg.lastGenTimeF = 0.0;

    // Set start positions based on previous segment or queued position
    if (m_segmentCount > 0) {
        int prevIdx = (m_segmentHead - 1 + SEGMENT_BUFFER_SIZE) % SEGMENT_BUFFER_SIZE;
        seg.theta.startSteps = m_segments[prevIdx].theta.targetSteps;
        seg.rho.startSteps = m_segments[prevIdx].rho.targetSteps;
    } else {
        seg.theta.startSteps = m_queuedTSteps.load();
        seg.rho.startSteps = m_queuedRSteps.load();
    }

    // Initialize generation counters to segment start
    seg.lastGenThetaSteps = seg.theta.startSteps;
    seg.lastGenRhoSteps = seg.rho.startSteps;

    // Calculate target steps
    seg.theta.targetSteps = thetaToSteps(theta);
    seg.rho.targetSteps = rhoToSteps(rho);

    // Calculate deltas
    seg.theta.deltaSteps = seg.theta.targetSteps - seg.theta.startSteps;
    seg.rho.deltaSteps = seg.rho.targetSteps - seg.rho.startSteps;

    seg.theta.direction = (seg.theta.deltaSteps > 0) ? 1 : ((seg.theta.deltaSteps < 0) ? -1 : 0);
    seg.rho.direction = (seg.rho.deltaSteps > 0) ? 1 : ((seg.rho.deltaSteps < 0) ? -1 : 0);

    // Convert to physical units
    seg.theta.deltaUnits = stepsToTheta(seg.theta.deltaSteps);
    seg.rho.deltaUnits = stepsToRho(seg.rho.deltaSteps);

    // Update target positions
    m_targetTheta = theta;
    m_targetRho = rho;

    // Advance head
    m_segmentHead = (m_segmentHead + 1) % SEGMENT_BUFFER_SIZE;
    m_segmentCount++;

    // Update queued positions
    m_queuedTSteps.store(seg.theta.targetSteps);
    m_queuedRSteps.store(seg.rho.targetSteps);

    return true;
}

void MotionPlanner::calculateSegmentProfile(Segment& seg) {
    // Apply speed multiplier to velocity limits only
    double tMaxVel = m_tMaxVel * m_speedMultiplier;
    double rMaxVel = m_rMaxVel * m_speedMultiplier;

    // Calculate S-curve for theta axis
    double thetaDist = std::abs(seg.theta.deltaUnits);
    if (thetaDist > 0.0001) {
        SCurve::calculate(
            thetaDist,
            seg.thetaEntryVel,
            seg.thetaExitVel,
            tMaxVel,
            m_tMaxAccel,
            m_tMaxJerk,
            seg.theta.profile
        );
    } else {
        // Zero motion - zero-duration profile
        seg.theta.profile.totalTime = 0;
        seg.theta.profile.totalDistance = 0;
        for (int i = 0; i < 7; i++) {
            seg.theta.profile.t[i] = 0;
            seg.theta.profile.tEnd[i] = 0;
        }
    }

    // Calculate S-curve for rho axis
    double rhoDist = std::abs(seg.rho.deltaUnits);
    if (rhoDist > 0.0001) {
        SCurve::calculate(
            rhoDist,
            seg.rhoEntryVel,
            seg.rhoExitVel,
            rMaxVel,
            m_rMaxAccel,
            m_rMaxJerk,
            seg.rho.profile
        );
    } else {
        // Zero motion - zero-duration profile
        seg.rho.profile.totalTime = 0;
        seg.rho.profile.totalDistance = 0;
        for (int i = 0; i < 7; i++) {
            seg.rho.profile.t[i] = 0;
            seg.rho.profile.tEnd[i] = 0;
        }
    }

    // Synchronize durations - stretch the faster axis to match the slower one
    double thetaTime = seg.theta.profile.totalTime;
    double rhoTime = seg.rho.profile.totalTime;
    double syncDuration = std::max(thetaTime, rhoTime);

    // Enforce minimum segment duration
    syncDuration = std::max(syncDuration, MIN_SEGMENT_DURATION);

    seg.duration = syncDuration;

    // Calculate time scale for each axis (float for FPU optimization)
    // timeScale > 1 means we stretch the profile (slower motion)
    seg.theta.timeScaleF = (thetaTime > 0.0001) ? (float)(syncDuration / thetaTime) : 1.0f;
    seg.rho.timeScaleF = (rhoTime > 0.0001) ? (float)(syncDuration / rhoTime) : 1.0f;

    // Convert profiles to float for real-time FPU evaluation
    SCurve::toFloat(seg.theta.profile, seg.theta.profileF);
    SCurve::toFloat(seg.rho.profile, seg.rho.profileF);

    seg.thetaPhaseIdx = 0;
    seg.rhoPhaseIdx = 0;

    seg.lastGenTimeF = 0;
    seg.lastGenThetaSteps = seg.theta.startSteps;
    seg.lastGenRhoSteps = seg.rho.startSteps;

    seg.calculated = true;
}

void MotionPlanner::recalculate() {
    if (m_segmentCount == 0) return;

    // Apply speed multiplier to velocity limits
    double tMaxVel = m_tMaxVel * m_speedMultiplier;
    double rMaxVel = m_rMaxVel * m_speedMultiplier;

    // =========================================================================
    // PASS 1: Forward pass - Set entry velocities from previous segment's exit
    // =========================================================================

    // Get starting velocity from currently executing segment or 0
    double prevThetaVel = 0.0;
    double prevRhoVel = 0.0;
    int8_t prevThetaDir = 0;
    int8_t prevRhoDir = 0;

    if (m_running && m_segmentCount > 0) {
        Segment& current = m_segments[m_segmentTail];
        if (current.executing && current.calculated) {
            prevThetaVel = current.thetaExitVel;
            prevRhoVel = current.rhoExitVel;
            prevThetaDir = current.theta.direction;
            prevRhoDir = current.rho.direction;
        }
    }

    int idx = m_segmentTail;
    for (int i = 0; i < m_segmentCount; i++) {
        Segment& seg = m_segments[idx];

        if (!seg.executing) {
            // Check for direction reversals - if direction changes, entry vel must be 0
            bool thetaReverses = (prevThetaDir != 0) && (seg.theta.direction != 0) &&
                                 (prevThetaDir != seg.theta.direction);
            bool rhoReverses = (prevRhoDir != 0) && (seg.rho.direction != 0) &&
                               (prevRhoDir != seg.rho.direction);

            seg.thetaEntryVel = thetaReverses ? 0.0 : std::min(prevThetaVel, tMaxVel);
            seg.rhoEntryVel = rhoReverses ? 0.0 : std::min(prevRhoVel, rMaxVel);

            // Set initial exit velocities to max (will be constrained in backward pass)
            seg.thetaExitVel = tMaxVel;
            seg.rhoExitVel = rMaxVel;
        }

        prevThetaVel = seg.thetaExitVel;
        prevRhoVel = seg.rhoExitVel;
        prevThetaDir = seg.theta.direction;
        prevRhoDir = seg.rho.direction;

        idx = (idx + 1) % SEGMENT_BUFFER_SIZE;
    }

    // =========================================================================
    // PASS 2: Backward pass - Constrain exit velocities based on next segment
    // =========================================================================

    // Find the last segment index
    int lastIdx = (m_segmentHead - 1 + SEGMENT_BUFFER_SIZE) % SEGMENT_BUFFER_SIZE;

    // The last segment must exit at 0 if end of pattern
    if (m_endOfPattern) {
        m_segments[lastIdx].thetaExitVel = 0.0;
        m_segments[lastIdx].rhoExitVel = 0.0;
    }

    // Work backwards, propagating constraints
    idx = lastIdx;
    for (int i = m_segmentCount - 1; i >= 0; i--) {
        Segment& seg = m_segments[idx];

        if (seg.executing) {
            // Don't modify executing segment
            idx = (idx - 1 + SEGMENT_BUFFER_SIZE) % SEGMENT_BUFFER_SIZE;
            continue;
        }

        // Get next segment's entry requirements (which become our exit constraints)
        int nextIdx = (idx + 1) % SEGMENT_BUFFER_SIZE;
        if (i < m_segmentCount - 1) {
            Segment& nextSeg = m_segments[nextIdx];

            // Check for direction reversals - if next segment reverses, we must exit at 0
            bool thetaReverses = (seg.theta.direction != 0) && (nextSeg.theta.direction != 0) &&
                                 (seg.theta.direction != nextSeg.theta.direction);
            bool rhoReverses = (seg.rho.direction != 0) && (nextSeg.rho.direction != 0) &&
                               (seg.rho.direction != nextSeg.rho.direction);

            if (thetaReverses) {
                seg.thetaExitVel = 0.0;
            } else {
                // Exit velocity can't exceed next segment's entry velocity
                seg.thetaExitVel = std::min(seg.thetaExitVel, nextSeg.thetaEntryVel);
            }

            if (rhoReverses) {
                seg.rhoExitVel = 0.0;
            } else {
                seg.rhoExitVel = std::min(seg.rhoExitVel, nextSeg.rhoEntryVel);
            }
        }

        // Now check if our entry velocity can achieve the required exit velocity
        // If not, we need to reduce entry velocity and propagate backward

        double thetaDist = std::abs(seg.theta.deltaUnits);
        if (thetaDist > 0.0001) {
            double maxEntry = SCurve::maxAchievableEntryVelocity(
                thetaDist, seg.thetaExitVel, tMaxVel, m_tMaxAccel, m_tMaxJerk);
            if (seg.thetaEntryVel > maxEntry) {
                seg.thetaEntryVel = maxEntry;
            }
        }

        double rhoDist = std::abs(seg.rho.deltaUnits);
        if (rhoDist > 0.0001) {
            double maxEntry = SCurve::maxAchievableEntryVelocity(
                rhoDist, seg.rhoExitVel, rMaxVel, m_rMaxAccel, m_rMaxJerk);
            if (seg.rhoEntryVel > maxEntry) {
                seg.rhoEntryVel = maxEntry;
            }
        }

        // Propagate entry velocity constraint to previous segment's exit
        if (i > 0) {
            int prevIdx = (idx - 1 + SEGMENT_BUFFER_SIZE) % SEGMENT_BUFFER_SIZE;
            Segment& prevSeg = m_segments[prevIdx];
            if (!prevSeg.executing) {
                prevSeg.thetaExitVel = std::min(prevSeg.thetaExitVel, seg.thetaEntryVel);
                prevSeg.rhoExitVel = std::min(prevSeg.rhoExitVel, seg.rhoEntryVel);
            }
        }

        idx = (idx - 1 + SEGMENT_BUFFER_SIZE) % SEGMENT_BUFFER_SIZE;
    }

    // =========================================================================
    // PASS 3: Calculate actual S-curve profiles with final velocities
    // =========================================================================

    // Pass 3: Calculate actual profiles
    idx = m_segmentTail;
    for (int i = 0; i < m_segmentCount; i++) {
        Segment& seg = m_segments[idx];
        // Only calculate if not already executing AND no steps generated yet
        if (!seg.executing && seg.lastGenTimeF < 0.0001) {
            calculateSegmentProfile(seg);
        }
        idx = (idx + 1) % SEGMENT_BUFFER_SIZE;
    }
}

void MotionPlanner::start() {
    if (m_running) return;

    // Create timer if not already created
    if (m_timerHandle == nullptr) {
        esp_timer_create_args_t timerArgs = {
            .callback = stepTimerISR,
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "step_timer",
            .skip_unhandled_events = true
        };
        esp_timer_create(&timerArgs, (esp_timer_handle_t*)&m_timerHandle);
    }

    m_running = true;
    m_segmentStartTime = micros();
    m_segmentElapsed = 0.0;

    // Mark first segment as executing
    if (m_segmentCount > 0) {
        m_segments[m_segmentTail].executing = true;
    }
}

void MotionPlanner::stop() {
    if (m_timerHandle != nullptr) {
        esp_timer_stop((esp_timer_handle_t)m_timerHandle);
    }

    m_running = false;
    m_timerActive = false;

    // Clear step queue
    m_stepQueueHead = 0;
    m_stepQueueTail = 0;

    // Clear segment buffer
    m_segmentHead = 0;
    m_segmentTail = 0;
    m_segmentCount = 0;

    // Reset pattern state
    m_endOfPattern = false;

    // Sync queued positions with executed positions so next pattern
    // starts from current actual position
    m_queuedTSteps.store(m_executedTSteps.load());
    m_queuedRSteps.store(m_executedRSteps.load());

    // Update target positions to match
    m_targetTheta = stepsToTheta(m_executedTSteps);
    m_targetRho = stepsToRho(m_executedRSteps);
}

void MotionPlanner::process() {
    if (!m_running) return;

    // Fill the step queue with upcoming step events
    fillStepQueue();

    // If we have segments but none are executing, start the first one
    if (m_segmentCount > 0 && !m_segments[m_segmentTail].executing) {
        m_segmentStartTime = micros();
        m_segmentElapsed = 0.0;
        m_segments[m_segmentTail].executing = true;
    }

    // Auto-start timer if buffer is sufficiently full
    if (!m_timerActive) {
        int queueDepth = STEP_QUEUE_SIZE - 1 - getStepQueueSpace();
        if (queueDepth > 64) {
            esp_timer_start_periodic((esp_timer_handle_t)m_timerHandle, STEP_TIMER_PERIOD_US);
            m_timerActive = true;
        }
    }

    // Check for segment completion
    if (m_segmentCount > 0) {
        Segment& current = m_segments[m_segmentTail];

        // Calculate elapsed time in current segment
        uint32_t now = micros();
        double elapsed = (now - m_segmentStartTime) / 1000000.0;

        // Check if current segment is complete
        // It's complete if time has elapsed AND we've generated all steps for it
        if (current.executing && elapsed >= current.duration && 
            current.lastGenTimeF >= current.duration - 0.000001) {
            
            if (m_segmentCount < 5 && m_endOfPattern) {
                double curT, curR;
                getCurrentPosition(curT, curR);
                printf("[DEBUG] Segment %u complete. count=%d, target=(%.3f, %.3f), actual=(%.3f, %.3f), duration=%.3f, elapsed=%.3f\n",
                       m_completedCount, m_segmentCount, current.targetTheta, current.targetRho,
                       curT, curR, current.duration, elapsed);
            }

            // Segment complete
            current.executing = false;
            m_completedCount++;

            // Move to next segment
            m_segmentTail = (m_segmentTail + 1) % SEGMENT_BUFFER_SIZE;
            m_segmentCount--;

            // Start next segment
            if (m_segmentCount > 0) {
                m_segmentStartTime = now;
                m_segmentElapsed = 0.0;
                m_segments[m_segmentTail].executing = true;
            }
        }
    }

    // If no more segments, we're idle
    if (m_segmentCount == 0 && m_endOfPattern) {
        stop();
    }
}

void MotionPlanner::fillStepQueue() {
    if (m_segmentCount == 0) return;

    Segment& seg = m_segments[m_segmentTail];
    if (!seg.calculated || !seg.executing) return;

    uint32_t now = micros();

    // All calculations use float for ESP32 FPU optimization
    // The ESP32 FPU only supports single-precision, so double is ~3x slower
    float segDuration = (float)seg.duration;

    // Calculate current wall-clock position in segment time
    float wallTime = (now - m_segmentStartTime) / 1000000.0f;

    // Start generating from where we last generated
    float t = seg.lastGenTimeF;
    float tEnd = std::min(segDuration, wallTime + (STEP_QUEUE_HORIZON_US / 1000000.0f));

    static constexpr float SAMPLE_INTERVAL = STEP_TIMER_PERIOD_US / 1000000.0f;

    int32_t lastThetaSteps = seg.lastGenThetaSteps;
    int32_t lastRhoSteps = seg.lastGenRhoSteps;

    // Cache profile references for faster access
    const SCurve::ProfileF& thetaProf = seg.theta.profileF;
    const SCurve::ProfileF& rhoProf = seg.rho.profileF;
    float thetaTimeScale = seg.theta.timeScaleF;
    float rhoTimeScale = seg.rho.timeScaleF;

    while (t <= tEnd) {
        // Get position from S-curve profile (with time scaling)
        float thetaProfileTime = t / thetaTimeScale;
        float rhoProfileTime = t / rhoTimeScale;

        // Get fractional position (0 to 1) using float-optimized getPosition
        float thetaFrac = (thetaProf.totalDistance > 0.0f)
            ? SCurve::getPositionF(thetaProf, thetaProfileTime, seg.thetaPhaseIdx) / thetaProf.totalDistance
            : 0.0f;
        float rhoFrac = (rhoProf.totalDistance > 0.0f)
            ? SCurve::getPositionF(rhoProf, rhoProfileTime, seg.rhoPhaseIdx) / rhoProf.totalDistance
            : 0.0f;

        // Calculate target steps
        int32_t targetThetaSteps, targetRhoSteps;
        if (t >= segDuration - 0.000001f) {
            targetThetaSteps = seg.theta.targetSteps;
            targetRhoSteps = seg.rho.targetSteps;
        } else {
            targetThetaSteps = seg.theta.startSteps + (int32_t)lroundf(thetaFrac * seg.theta.deltaSteps);
            targetRhoSteps = seg.rho.startSteps + (int32_t)lroundf(rhoFrac * seg.rho.deltaSteps);

            // Safety: clamp to target to prevent overshooting due to rounding
            if (seg.theta.deltaSteps > 0) {
                targetThetaSteps = std::min(targetThetaSteps, seg.theta.targetSteps);
            } else if (seg.theta.deltaSteps < 0) {
                targetThetaSteps = std::max(targetThetaSteps, seg.theta.targetSteps);
            }

            if (seg.rho.deltaSteps > 0) {
                targetRhoSteps = std::min(targetRhoSteps, seg.rho.targetSteps);
            } else if (seg.rho.deltaSteps < 0) {
                targetRhoSteps = std::max(targetRhoSteps, seg.rho.targetSteps);
            }
        }

        // Generate step events for any steps needed
        uint32_t eventTime = m_segmentStartTime + (uint32_t)(t * 1000000.0f);

        while (lastThetaSteps != targetThetaSteps || lastRhoSteps != targetRhoSteps) {
            uint8_t stepMask = 0;
            uint8_t dirMask = 0;

            if (lastThetaSteps != targetThetaSteps) {
                stepMask |= 0x01;
                if (targetThetaSteps > lastThetaSteps) {
                    dirMask |= 0x01;  // Forward
                    lastThetaSteps++;
                } else {
                    lastThetaSteps--;
                }
            }

            if (lastRhoSteps != targetRhoSteps) {
                stepMask |= 0x02;
                if (targetRhoSteps > lastRhoSteps) {
                    dirMask |= 0x02;  // Forward
                    lastRhoSteps++;
                } else {
                    lastRhoSteps--;
                }
            }

            if (stepMask != 0) {
                if (!queueStepEvent(eventTime, stepMask, dirMask)) {
                    // Queue full - roll back steps and exit
                    if (stepMask & 0x01) {
                        if (dirMask & 0x01) lastThetaSteps--;
                        else lastThetaSteps++;
                    }
                    if (stepMask & 0x02) {
                        if (dirMask & 0x02) lastRhoSteps--;
                        else lastRhoSteps++;
                    }

                    seg.lastGenTimeF = t;
                    seg.lastGenThetaSteps = lastThetaSteps;
                    seg.lastGenRhoSteps = lastRhoSteps;
                    return;
                }
            }
        }

        t += SAMPLE_INTERVAL;
    }

    // Finished this batch - save state
    seg.lastGenTimeF = t;
    seg.lastGenThetaSteps = lastThetaSteps;
    seg.lastGenRhoSteps = lastRhoSteps;
}

int MotionPlanner::getStepQueueSpace() const {
    int head = m_stepQueueHead;
    int tail = m_stepQueueTail;

    if (head >= tail) {
        return STEP_QUEUE_SIZE - (head - tail) - 1;
    } else {
        return tail - head - 1;
    }
}

bool MotionPlanner::queueStepEvent(uint32_t time, uint8_t stepMask, uint8_t dirMask) {
    int nextHead = (m_stepQueueHead + 1) % STEP_QUEUE_SIZE;
    if (nextHead == m_stepQueueTail) {
        return false;  // Queue full
    }

    m_stepQueue[m_stepQueueHead].executeTime = time;
    m_stepQueue[m_stepQueueHead].stepMask = stepMask;
    m_stepQueue[m_stepQueueHead].dirMask = dirMask;
    m_stepQueueHead = nextHead;

    return true;
}

bool MotionPlanner::hasSpace() const {
    return m_segmentCount < SEGMENT_BUFFER_SIZE - 1;
}

bool MotionPlanner::isRunning() const {
    return m_running;
}

bool MotionPlanner::isIdle() const {
    bool queueEmpty = (m_stepQueueHead == m_stepQueueTail);
    return !m_running && m_segmentCount == 0 && queueEmpty;
}

void MotionPlanner::getCurrentPosition(double& theta, double& rho) const {
    theta = stepsToTheta(m_executedTSteps);
    rho = stepsToRho(m_executedRSteps);
}

void MotionPlanner::resetTheta() {
    // Reset theta to zero at current position
    m_executedTSteps = 0;
    m_queuedTSteps = 0;
    m_targetTheta = 0.0;

    // Update any pending segments
    for (int i = 0; i < SEGMENT_BUFFER_SIZE; i++) {
        m_segments[i].theta.startSteps = 0;
        m_segments[i].theta.targetSteps = thetaToSteps(m_segments[i].targetTheta);
        m_segments[i].calculated = false;
    }
}

void MotionPlanner::setMotionLimits(double rMaxVel, double rMaxAccel, double rMaxJerk,
                                     double tMaxVel, double tMaxAccel, double tMaxJerk) {
    m_rMaxVel = rMaxVel;
    m_rMaxAccel = rMaxAccel;
    m_rMaxJerk = rMaxJerk;
    m_tMaxVel = tMaxVel;
    m_tMaxAccel = tMaxAccel;
    m_tMaxJerk = tMaxJerk;

    // Mark all non-executing segments as needing recalculation
    int idx = m_segmentTail;
    for (int i = 0; i < m_segmentCount; i++) {
        Segment& seg = m_segments[idx];
        if (!seg.executing) {
            seg.calculated = false;
        }
        idx = (idx + 1) % SEGMENT_BUFFER_SIZE;
    }

    // Recalculate all pending segments with new limits
    if (m_segmentCount > 0) {
        recalculate();
    }
}

void MotionPlanner::setSpeedMultiplier(double mult) {
    double newMult = std::max(0.1, std::min(1.0, mult));

    // Only recalculate if multiplier actually changed
    if (std::abs(newMult - m_speedMultiplier) < 0.001) {
        return;
    }

    m_speedMultiplier = newMult;

    // Mark all non-executing segments as needing recalculation
    int idx = m_segmentTail;
    for (int i = 0; i < m_segmentCount; i++) {
        Segment& seg = m_segments[idx];
        if (!seg.executing) {
            seg.calculated = false;
        }
        idx = (idx + 1) % SEGMENT_BUFFER_SIZE;
    }

    // Recalculate all pending segments with new speed
    recalculate();
}

void MotionPlanner::setEndOfPattern(bool ending) {
    m_endOfPattern = ending;
}

void MotionPlanner::getDiagnostics(uint32_t& queueDepth, uint32_t& underruns) const {
    queueDepth = STEP_QUEUE_SIZE - 1 - getStepQueueSpace();
    underruns = m_underrunCount.load();
}

int32_t MotionPlanner::thetaToSteps(double theta) const {
    return (int32_t)(theta * m_stepsPerRadT);
}

int32_t MotionPlanner::rhoToSteps(double rho) const {
    return (int32_t)(rho * m_stepsPerMmR);
}

double MotionPlanner::stepsToTheta(int32_t steps) const {
    return (double)steps / m_stepsPerRadT;
}

double MotionPlanner::stepsToRho(int32_t steps) const {
    return (double)steps / m_stepsPerMmR;
}

// ISR callback - called at 25kHz (40us period)
void IRAM_ATTR MotionPlanner::stepTimerISR(void* arg) {
    MotionPlanner* planner = static_cast<MotionPlanner*>(arg);
    planner->handleStepTimer();
}

void IRAM_ATTR MotionPlanner::handleStepTimer() {
    // Check if there's a step event ready to execute
    if (m_stepQueueHead == m_stepQueueTail) {
        if (m_running) {
             m_underrunCount++;
        }
        return;  // Queue empty
    }

    uint32_t now = micros();
    StepEvent& event = m_stepQueue[m_stepQueueTail];

    // Check if it's time to execute this event
    // Handle wraparound by checking if we're within a reasonable window
    int32_t timeDiff = (int32_t)(event.executeTime - now);
    if (timeDiff > 0 && timeDiff < 1000000) {
        return;  // Not yet time
    }

    // Set direction pins first
    if (event.stepMask & 0x01) {
        digitalWrite(T_DIR_PIN, (event.dirMask & 0x01) ? HIGH : LOW);
    }
    if (event.stepMask & 0x02) {
        digitalWrite(R_DIR_PIN, (event.dirMask & 0x02) ? HIGH : LOW);
    }

    // Small delay for direction setup (inline)
    for (volatile int i = 0; i < 10; i++) {}

    // Generate step pulses
    if (event.stepMask & 0x01) {
        digitalWrite(T_STEP_PIN, HIGH);
    }
    if (event.stepMask & 0x02) {
        digitalWrite(R_STEP_PIN, HIGH);
    }

    // Brief pulse width delay
    for (volatile int i = 0; i < 20; i++) {}

    // End step pulses
    if (event.stepMask & 0x01) {
        digitalWrite(T_STEP_PIN, LOW);
        // Update executed position
        if (event.dirMask & 0x01) {
            m_executedTSteps++;
        } else {
            m_executedTSteps--;
        }
    }
    if (event.stepMask & 0x02) {
        digitalWrite(R_STEP_PIN, LOW);
        if (event.dirMask & 0x02) {
            m_executedRSteps++;
        } else {
            m_executedRSteps--;
        }
    }

    // Advance queue tail
    m_stepQueueTail = (m_stepQueueTail + 1) % STEP_QUEUE_SIZE;
}
