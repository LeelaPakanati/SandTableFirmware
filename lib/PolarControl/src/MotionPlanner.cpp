#include "MotionPlanner.hpp"
#include <cmath>
#include <cstdio>
#ifndef NATIVE_BUILD
#include <Arduino.h>
#include <esp_timer.h>
#include "Logger.hpp"
#else
#include "esp32_mock.hpp"
#define LOG(fmt, ...) printf(fmt, ##__VA_ARGS__)
#endif

// Step pulse width in microseconds
static constexpr uint32_t STEP_PULSE_WIDTH_US = 2;

// Direction setup time before step pulse
static constexpr uint32_t DIR_SETUP_TIME_US = 1;

MotionPlanner::MotionPlanner()
    : m_stepsPerMmR(100)
    , m_stepsPerRadT(3000)
    , m_maxRho(450.0f)
    , m_rMaxVel(30.0f)
    , m_rMaxAccel(20.0f)
    , m_rMaxJerk(100.0f)
    , m_tMaxVel(1.0f)
    , m_tMaxAccel(2.0f)
    , m_tMaxJerk(10.0f)
    , m_speedMultiplier(1.0f)
    , m_segmentHead(0)
    , m_segmentTail(0)
    , m_genSegmentIdx(0)
    , m_genSegmentStartTime(0)
    , m_queuedTSteps(0)
    , m_queuedRSteps(0)
    , m_executedTSteps(0)
    , m_executedRSteps(0)
    , m_targetTheta(0.0f)
    , m_targetRho(0.0f)
    , m_stepQueueHead(0)
    , m_stepQueueTail(0)
    , m_segmentStartTime(0)
    , m_segmentElapsed(0.0f)
    , m_running(false)
    , m_endOfPattern(false)
    , m_completedCount(0)
    , m_timerHandle(nullptr)
{
    // Initialize segments
    for (int i = 0; i < SEGMENT_BUFFER_SIZE; i++) {
        m_segments[i].calculated = false;
        m_segments[i].executing = false;
        m_segments[i].generationComplete = false;
    }
}

MotionPlanner::~MotionPlanner() {
    stop();
}

void MotionPlanner::init(int stepsPerMmR, int stepsPerRadT, float maxRho,
                         float rMaxVel, float rMaxAccel, float rMaxJerk,
                         float tMaxVel, float tMaxAccel, float tMaxJerk) {
    m_stepsPerMmR = stepsPerMmR;
    m_stepsPerRadT = stepsPerRadT;
    m_maxRho = maxRho;

    m_rMaxVel = rMaxVel;
    m_rMaxAccel = rMaxAccel;
    m_rMaxJerk = rMaxJerk;

    m_tMaxVel = tMaxVel;
    m_tMaxAccel = tMaxAccel;
    m_tMaxJerk = tMaxJerk;
    
    // Setup GPIO pins
    pinMode(T_STEP_PIN, OUTPUT);
    pinMode(T_DIR_PIN, OUTPUT);
    pinMode(R_STEP_PIN, OUTPUT);
    pinMode(R_DIR_PIN, OUTPUT);

    FastGPIO::setLow(T_STEP_PIN);
    FastGPIO::setLow(R_STEP_PIN);

    // Reset positions
    m_queuedTSteps = 0;
    m_queuedRSteps = 0;
    m_executedTSteps = 0;
    m_executedRSteps = 0;
    m_targetTheta = 0.0f;
    m_targetRho = 0.0f;

    // Reset buffer
    m_segmentHead = 0;
    m_segmentTail = 0;
    m_genSegmentIdx = 0;
    m_genSegmentStartTime = 0;
    m_stepQueueHead = 0;
    m_stepQueueTail = 0;
    m_completedCount = 0;
    m_lastUnderrunUs.store(0);
    m_underrunCount.store(0);
    m_consecutiveUnderruns.store(0);
    m_maxConsecutiveUnderruns.store(0);
    m_minQueueDepth = 0xFFFFFFFFu;
    m_maxQueueDepth = 0;
}

bool MotionPlanner::addSegment(float theta, float rho) {
    if (!hasSpace()) {
        return false;
    }

    // Clamp rho to valid range
    rho = std::max(0.0f, std::min(rho, m_maxRho));

    Segment& seg = m_segments[m_segmentHead];
    seg.targetTheta = theta;
    seg.targetRho = rho;
    seg.calculated = false;
    seg.executing = false;
    seg.generationComplete = false;

    // Reset execution/generation state for new segment
    seg.thetaPhaseIdx = 0;
    seg.rhoPhaseIdx = 0;
    seg.lastGenTime = 0.0f;

    // Set start positions based on previous segment or queued position
    if (m_segmentHead != m_segmentTail) {
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

    // Update queued positions
    m_queuedTSteps.store(seg.theta.targetSteps);
    m_queuedRSteps.store(seg.rho.targetSteps);

    return true;
}

            // Reduce by 1% of vMax (scales with units)
            // vCruise -= vMax * 0.01f;
            // ...
    // Note: This logic is in SCurve.cpp, I cannot edit it here easily.
    // I will add prints to MotionPlanner.cpp

void MotionPlanner::calculateSegmentProfile(Segment& seg) {
    // Apply speed multiplier to velocity limits only
    float tMaxVel = m_tMaxVel * m_speedMultiplier;
    float rMaxVel = m_rMaxVel * m_speedMultiplier;
    
    // printf("DEBUG: CalcSeg: mult=%.2f, rMaxVel=%.2f\n", m_speedMultiplier, rMaxVel);

    // Calculate S-curve for theta axis
    float thetaDist = fabsf(seg.theta.deltaUnits);
    if (thetaDist > 0.0001f) {
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
        seg.theta.profile.totalTime = 0.0f;
        seg.theta.profile.totalDistance = 0.0f;
        for (int i = 0; i < 7; i++) {
            seg.theta.profile.t[i] = 0.0f;
            seg.theta.profile.tEnd[i] = 0.0f;
        }
    }

    // Calculate S-curve for rho axis
    float rhoDist = fabsf(seg.rho.deltaUnits);
    if (rhoDist > 0.0001f) {
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
        seg.rho.profile.totalTime = 0.0f;
        seg.rho.profile.totalDistance = 0.0f;
        for (int i = 0; i < 7; i++) {
            seg.rho.profile.t[i] = 0.0f;
            seg.rho.profile.tEnd[i] = 0.0f;
        }
    }

    // Synchronize durations - stretch the faster axis to match the slower one
    float thetaTime = seg.theta.profile.totalTime;
    float rhoTime = seg.rho.profile.totalTime;
    float syncDuration = std::max(thetaTime, rhoTime);

    // Enforce minimum segment duration
    syncDuration = std::max(syncDuration, MIN_SEGMENT_DURATION);

    seg.duration = syncDuration;

    // Calculate time scale for each axis
    // timeScale > 1 means we stretch the profile (slower motion)
    seg.theta.timeScale = (thetaTime > 0.0001f) ? (syncDuration / thetaTime) : 1.0f;
    seg.rho.timeScale = (rhoTime > 0.0001f) ? (syncDuration / rhoTime) : 1.0f;

    seg.thetaPhaseIdx = 0;
    seg.rhoPhaseIdx = 0;

    seg.lastGenTime = 0.0f;
    seg.lastGenThetaSteps = seg.theta.startSteps;
    seg.lastGenRhoSteps = seg.rho.startSteps;

    seg.calculated = true;
}

void MotionPlanner::recalculate() {
    if (m_segmentHead == m_segmentTail) return;

    // Apply speed multiplier to velocity limits
    float tMaxVel = m_tMaxVel * m_speedMultiplier;
    float rMaxVel = m_rMaxVel * m_speedMultiplier;

    // =========================================================================
    // PASS 1: Forward pass - Set entry velocities from previous segment's exit
    // =========================================================================

    // Get starting velocity from currently executing segment or 0
    float prevThetaVel = 0.0f;
    float prevRhoVel = 0.0f;
    int8_t prevThetaDir = 0;
    int8_t prevRhoDir = 0;

    if (m_running && m_segmentHead != m_segmentTail) {
        Segment& current = m_segments[m_segmentTail];
        if (current.executing && current.calculated) {
            prevThetaVel = current.thetaExitVel;
            prevRhoVel = current.rhoExitVel;
            prevThetaDir = current.theta.direction;
            prevRhoDir = current.rho.direction;
        }
    }

    int idx = m_segmentTail;
    while (idx != m_segmentHead) {
        Segment& seg = m_segments[idx];

        if (!seg.executing) {
            // Check for direction reversals - if direction changes, entry vel must be 0
            bool thetaReverses = (prevThetaDir != 0) && (seg.theta.direction != 0) &&
                                 (prevThetaDir != seg.theta.direction);
            bool rhoReverses = (prevRhoDir != 0) && (seg.rho.direction != 0) &&
                               (prevRhoDir != seg.rho.direction);

            seg.thetaEntryVel = thetaReverses ? 0.0f : std::min(prevThetaVel, tMaxVel);
            seg.rhoEntryVel = rhoReverses ? 0.0f : std::min(prevRhoVel, rMaxVel);

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
        m_segments[lastIdx].thetaExitVel = 0.0f;
        m_segments[lastIdx].rhoExitVel = 0.0f;
    }

    // Work backwards, propagating constraints
    idx = lastIdx;
    while (true) {
        Segment& seg = m_segments[idx];

        if (seg.executing) {
            // Don't modify executing segment
            if (idx == m_segmentTail) break; // Reached start
            idx = (idx - 1 + SEGMENT_BUFFER_SIZE) % SEGMENT_BUFFER_SIZE;
            continue;
        }

        // Get next segment's entry requirements (which become our exit constraints)
        int nextIdx = (idx + 1) % SEGMENT_BUFFER_SIZE;
        if (nextIdx != m_segmentHead) {
            Segment& nextSeg = m_segments[nextIdx];

            // Check for direction reversals - if next segment reverses, we must exit at 0
            bool thetaReverses = (seg.theta.direction != 0) && (nextSeg.theta.direction != 0) &&
                                 (seg.theta.direction != nextSeg.theta.direction);
            bool rhoReverses = (seg.rho.direction != 0) && (nextSeg.rho.direction != 0) &&
                               (seg.rho.direction != nextSeg.rho.direction);

            if (thetaReverses) {
                seg.thetaExitVel = 0.0f;
            } else {
                // Exit velocity can't exceed next segment's entry velocity
                seg.thetaExitVel = std::min(seg.thetaExitVel, nextSeg.thetaEntryVel);
            }

            if (rhoReverses) {
                seg.rhoExitVel = 0.0f;
            } else {
                seg.rhoExitVel = std::min(seg.rhoExitVel, nextSeg.rhoEntryVel);
            }
        }

        // Now check if our entry velocity can achieve the required exit velocity
        // If not, we need to reduce entry velocity and propagate backward

        float thetaDist = fabsf(seg.theta.deltaUnits);
        if (thetaDist > 0.0001f) {
            float maxEntry = SCurve::maxAchievableEntryVelocity(
                thetaDist, seg.thetaExitVel, tMaxVel, m_tMaxAccel, m_tMaxJerk);
            if (seg.thetaEntryVel > maxEntry) {
                seg.thetaEntryVel = maxEntry;
            }
        }

        float rhoDist = fabsf(seg.rho.deltaUnits);
        if (rhoDist > 0.0001f) {
            float maxEntry = SCurve::maxAchievableEntryVelocity(
                rhoDist, seg.rhoExitVel, rMaxVel, m_rMaxAccel, m_rMaxJerk);
            if (seg.rhoEntryVel > maxEntry) {
                seg.rhoEntryVel = maxEntry;
            }
        }

        // Propagate entry velocity constraint to previous segment's exit
        if (idx != m_segmentTail) {
            int prevIdx = (idx - 1 + SEGMENT_BUFFER_SIZE) % SEGMENT_BUFFER_SIZE;
            Segment& prevSeg = m_segments[prevIdx];
            if (!prevSeg.executing) {
                prevSeg.thetaExitVel = std::min(prevSeg.thetaExitVel, seg.thetaEntryVel);
                prevSeg.rhoExitVel = std::min(prevSeg.rhoExitVel, seg.rhoEntryVel);
            }
        }
        
        if (idx == m_segmentTail) break; // Finished all
        idx = (idx - 1 + SEGMENT_BUFFER_SIZE) % SEGMENT_BUFFER_SIZE;
    }

    // =========================================================================
    // PASS 3: Calculate actual S-curve profiles with final velocities
    // =========================================================================

    // Pass 3: Calculate actual profiles
    idx = m_segmentTail;
    while (idx != m_segmentHead) {
        Segment& seg = m_segments[idx];
        // Only calculate if not already executing AND no steps generated yet
        if (!seg.executing && seg.lastGenTime < 0.0001f) {
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
    m_segmentElapsed = 0.0f;

    m_startupHoldoff = true;
    
    // Initialize generation state
    m_genSegmentIdx = m_segmentTail;
    m_genSegmentStartTime = m_segmentStartTime;

    // Mark first segment as executing
    if (m_segmentHead != m_segmentTail) {
        m_segments[m_segmentTail].executing = true;
    }
}

void MotionPlanner::stop() {
    if (m_timerHandle != nullptr) {
        esp_timer_stop((esp_timer_handle_t)m_timerHandle);
    }

    m_running = false;
    m_timerActive = false;
    m_startupHoldoff = false;

    // Clear step queue
    m_stepQueueHead = 0;
    m_stepQueueTail = 0;

    // Clear segment buffer
    m_segmentHead = 0;
    m_segmentTail = 0;
    m_genSegmentIdx = 0;
    m_genSegmentStartTime = 0;

    // Reset pattern state
    m_endOfPattern = false;

    // Sync queued positions with executed positions so next pattern
    // starts from current actual position
    m_queuedTSteps.store(m_executedTSteps.load());
    m_queuedRSteps.store(m_executedRSteps.load());

    // Update target positions to match
    m_targetTheta = stepsToTheta(m_executedTSteps);
    m_targetRho = stepsToRho(m_executedRSteps);

    m_lastUnderrunUs.store(0);
    m_consecutiveUnderruns.store(0);
    m_maxConsecutiveUnderruns.store(0);
    m_minQueueDepth = 0xFFFFFFFFu;
    m_maxQueueDepth = 0;
    m_lastQueuedEventTime = 0;
}

void MotionPlanner::process() {
    uint32_t now = micros();
    if (m_lastProcessTime > 0) {
        m_intervalProfiler.addSample(now - m_lastProcessTime);
    }
    m_lastProcessTime = now;

    if (!m_running) return;

    // 1. Generation Loop
    // Fill once per call; fillStepQueue will span segments until horizon or queue full.
    int queueDepth = STEP_QUEUE_SIZE - 1 - getStepQueueSpace();
    if (queueDepth < 0) queueDepth = 0;
    if (m_genSegmentIdx != m_segmentHead) {
        fillStepQueue(STEP_QUEUE_HORIZON_US);
        queueDepth = STEP_QUEUE_SIZE - 1 - getStepQueueSpace();
        if (queueDepth < 0) queueDepth = 0;
    }

    // If we have segments but none are executing, start the first one
    if (m_segmentHead != m_segmentTail && !m_segments[m_segmentTail].executing) {
        // First segment start - use current time
        // Note: m_segmentStartTime was already set in start(), but if we added 
        // segments after start() while idle, we need to update it.
        // If we are recovering from idle, sync gen time too.
        if (m_segmentTail == m_genSegmentIdx) { // Only if we haven't generated ahead
             m_segmentStartTime = micros();
             m_genSegmentStartTime = m_segmentStartTime;
        }
        
        m_segmentElapsed = 0.0f;
        m_segments[m_segmentTail].executing = true;
    }

    if (m_startupHoldoff) {
        bool metStartupConditions = (queueDepth >= (STEP_QUEUE_SIZE - 1));
        if (metStartupConditions &&
            m_lastFillStopReason.load() != static_cast<uint32_t>(FillStopReason::TimeBudget)) {
            m_startupHoldoff = false;
        }
    }

    // Auto-start timer if buffer has any data and startup holdoff is cleared
    if (!m_timerActive && !m_startupHoldoff) {
        if (queueDepth > 0) {
            esp_timer_start_periodic((esp_timer_handle_t)m_timerHandle, STEP_TIMER_PERIOD_US);
            m_timerActive = true;
        }
    }

    uint32_t depth = static_cast<uint32_t>(queueDepth);
    if (m_minQueueDepth == 0xFFFFFFFFu || depth < m_minQueueDepth) {
        m_minQueueDepth = depth;
    }
    if (depth > m_maxQueueDepth) {
        m_maxQueueDepth = depth;
    }

    // 2. Execution Completion Check
    if (m_segmentHead != m_segmentTail) {
        Segment& current = m_segments[m_segmentTail];

        // Calculate elapsed time in current segment
        uint32_t now = micros();
        // Handle timer wraparound for elapsed calculation
        uint32_t diff = now - m_segmentStartTime;
        float elapsed = diff / 1000000.0f;

        // Check if current segment is complete
        // It's complete if time has elapsed AND we've generated all steps for it
        if (current.executing && elapsed >= current.duration && current.generationComplete) {

            // Calculate segment count for debug
            int count = (m_segmentHead - m_segmentTail + SEGMENT_BUFFER_SIZE) % SEGMENT_BUFFER_SIZE;

            if (count < 5 && m_endOfPattern) {
                float curT, curR;
                getCurrentPosition(curT, curR);
            }

            // Segment complete
            current.executing = false;
            m_completedCount++;

            // Move to next segment
            m_segmentTail = (m_segmentTail + 1) % SEGMENT_BUFFER_SIZE;
            
            // Update start time deterministically
            m_segmentStartTime += (uint32_t)(current.duration * 1000000.0f);
            m_segmentElapsed = 0.0f;

            // Start next segment immediately if available
            if (m_segmentHead != m_segmentTail) {
                m_segments[m_segmentTail].executing = true;
                
                // If we fell behind in generation (underrun recovery), 
                // snap generation to execution
                if (m_genSegmentIdx == m_segmentTail && !m_segments[m_segmentTail].generationComplete) {
                     // We are generating the segment we just started executing.
                     // Ensure the timestamps are aligned.
                     m_genSegmentStartTime = m_segmentStartTime;
                }
            }
        }
    }

    // If no more segments, we're idle
    if (m_segmentHead == m_segmentTail && m_endOfPattern) {
        stop();
    }
    
    m_processProfiler.addSample(micros() - now);
}

void MotionPlanner::fillStepQueue(uint32_t horizonUs) {
    uint32_t startUs = micros();
    uint32_t now = startUs;
    static constexpr float SAMPLE_INTERVAL = STEP_TIMER_PERIOD_US / 1000000.0f;
    m_lastFillStopReason.store(static_cast<uint32_t>(FillStopReason::None));

    while (m_genSegmentIdx != m_segmentHead) {
        if ((micros() - startUs) >= STEP_QUEUE_MAX_PROCESS_US) {
            m_fillStopTimeBudgetCount.fetch_add(1);
            m_lastFillStopReason.store(static_cast<uint32_t>(FillStopReason::TimeBudget));
            break;
        }
        Segment& seg = m_segments[m_genSegmentIdx];

        if (!seg.calculated) {
            calculateSegmentProfile(seg);
        }

        uint32_t startTime = m_genSegmentStartTime;
        float segDuration = seg.duration;

        // Calculate current wall-clock position relative to THIS segment's start time
        // If startTime is in the future, wallTime will be negative, which is correct
        int32_t timeDiff = (int32_t)(now - startTime);
        float wallTime = timeDiff / 1000000.0f;

        // Start generating from where we last generated
        float t = seg.lastGenTime;

        // Determine end time for this batch
        // We want to generate up to HORIZON ahead of current real time
        float tLimit = wallTime + (horizonUs / 1000000.0f);
        float tEnd = std::min(segDuration, tLimit);

        // If we are already ahead of the horizon, don't generate anything
        if (t >= tEnd) {
            if (t >= segDuration - 0.000001f) {
                seg.generationComplete = true;
                m_genSegmentStartTime += (uint32_t)(seg.duration * 1000000.0f);
                m_genSegmentIdx = (m_genSegmentIdx + 1) % SEGMENT_BUFFER_SIZE;
                continue;
            }
            uint32_t blankTime = startTime + (uint32_t)(tEnd * 1000000.0f);
            if (blankTime > m_lastQueuedEventTime && getStepQueueSpace() > 0) {
                queueStepEvent(blankTime, 0, 0);
            }
            m_fillStopHorizonCount.fetch_add(1);
            m_lastFillStopReason.store(static_cast<uint32_t>(FillStopReason::Horizon));
            break;
        }

        int32_t lastThetaSteps = seg.lastGenThetaSteps;
        int32_t lastRhoSteps = seg.lastGenRhoSteps;

        // Cache profile references for faster access
        const SCurve::Profile& thetaProf = seg.theta.profile;
        const SCurve::Profile& rhoProf = seg.rho.profile;
        float thetaTimeScale = seg.theta.timeScale;
        float rhoTimeScale = seg.rho.timeScale;

        while (t <= tEnd) {
            if ((micros() - startUs) >= STEP_QUEUE_MAX_PROCESS_US) {
                seg.lastGenTime = t;
                seg.lastGenThetaSteps = lastThetaSteps;
                seg.lastGenRhoSteps = lastRhoSteps;
                m_fillStopTimeBudgetCount.fetch_add(1);
                m_lastFillStopReason.store(static_cast<uint32_t>(FillStopReason::TimeBudget));
                m_genProfiler.addSample(micros() - startUs);
                return;
            }
            // Get position from S-curve profile (with time scaling)
            float thetaProfileTime = t / thetaTimeScale;
            float rhoProfileTime = t / rhoTimeScale;

            // Get fractional position (0 to 1)
            float thetaFrac = (thetaProf.totalDistance > 0.0f)
                ? SCurve::getPosition(thetaProf, thetaProfileTime, seg.thetaPhaseIdx) / thetaProf.totalDistance
                : 0.0f;
            float rhoFrac = (rhoProf.totalDistance > 0.0f)
                ? SCurve::getPosition(rhoProf, rhoProfileTime, seg.rhoPhaseIdx) / rhoProf.totalDistance
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
            // Use startTime (the segment's absolute start) + t
            uint32_t eventTime = startTime + (uint32_t)(t * 1000000.0f);

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

                        seg.lastGenTime = t;
                        seg.lastGenThetaSteps = lastThetaSteps;
                        seg.lastGenRhoSteps = lastRhoSteps;
                        m_fillStopQueueFullCount.fetch_add(1);
                        m_lastFillStopReason.store(static_cast<uint32_t>(FillStopReason::QueueFull));
                        m_genProfiler.addSample(micros() - startUs);
                        return;
                    }
                }
            }

            if (t >= segDuration - 0.000001f) break;
            t += SAMPLE_INTERVAL;
            if (t > segDuration) t = segDuration;
        }

        // Finished this segment batch - save state
        seg.lastGenTime = t;
        seg.lastGenThetaSteps = lastThetaSteps;
        seg.lastGenRhoSteps = lastRhoSteps;

        if (t >= segDuration - 0.000001f) {
            seg.generationComplete = true;
            m_genSegmentStartTime += (uint32_t)(seg.duration * 1000000.0f);
            m_genSegmentIdx = (m_genSegmentIdx + 1) % SEGMENT_BUFFER_SIZE;
            continue;
        }

        // Horizon reached within this segment; stop.
        uint32_t blankTime = startTime + (uint32_t)(tEnd * 1000000.0f);
        if (blankTime > m_lastQueuedEventTime && getStepQueueSpace() > 0) {
            queueStepEvent(blankTime, 0, 0);
        }
        m_fillStopHorizonCount.fetch_add(1);
        m_lastFillStopReason.store(static_cast<uint32_t>(FillStopReason::Horizon));
        break;
    }

    m_genProfiler.addSample(micros() - startUs);
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
    m_lastQueuedEventTime = time;

    return true;
}

bool MotionPlanner::hasSpace() const {
    return ((m_segmentHead + 1) % SEGMENT_BUFFER_SIZE) != m_segmentTail;
}

bool MotionPlanner::isRunning() const {
    return m_running;
}

bool MotionPlanner::isIdle() const {
    bool queueEmpty = (m_stepQueueHead == m_stepQueueTail);
    return !m_running && (m_segmentHead == m_segmentTail) && queueEmpty;
}

void MotionPlanner::getCurrentPosition(float& theta, float& rho) const {
    theta = stepsToTheta(m_executedTSteps);
    rho = stepsToRho(m_executedRSteps);
}

void MotionPlanner::getCurrentVelocity(float& thetaVel, float& rhoVel) const {
    thetaVel = 0.0f;
    rhoVel = 0.0f;

    if (!m_running || m_segmentHead == m_segmentTail) {
        return;
    }

    const Segment& current = m_segments[m_segmentTail];
    if (!current.executing || !current.calculated || current.duration <= 0.0001f) {
        return;
    }

    uint32_t now = micros();
    uint32_t diff = now - m_segmentStartTime;
    float elapsed = diff / 1000000.0f;
    if (elapsed < 0.0f) elapsed = 0.0f;
    if (elapsed > current.duration) elapsed = current.duration;

    if (current.theta.profile.totalDistance > 0.0f && current.theta.timeScale > 0.0f) {
        float t = elapsed / current.theta.timeScale;
        float v = SCurve::getVelocity(current.theta.profile, t) / current.theta.timeScale;
        thetaVel = v * static_cast<float>(current.theta.direction);
    }

    if (current.rho.profile.totalDistance > 0.0f && current.rho.timeScale > 0.0f) {
        float t = elapsed / current.rho.timeScale;
        float v = SCurve::getVelocity(current.rho.profile, t) / current.rho.timeScale;
        rhoVel = v * static_cast<float>(current.rho.direction);
    }
}

void MotionPlanner::resetTheta() {
    // Reset theta to zero at current position
    m_executedTSteps = 0;
    m_queuedTSteps = 0;
    m_targetTheta = 0.0f;

    // Update any pending segments
    for (int i = 0; i < SEGMENT_BUFFER_SIZE; i++) {
        m_segments[i].theta.startSteps = 0;
        m_segments[i].theta.targetSteps = thetaToSteps(m_segments[i].targetTheta);
        m_segments[i].calculated = false;
    }
}

void MotionPlanner::setMotionLimits(float rMaxVel, float rMaxAccel, float rMaxJerk,
                                     float tMaxVel, float tMaxAccel, float tMaxJerk) {
    m_rMaxVel = rMaxVel;
    m_rMaxAccel = rMaxAccel;
    m_rMaxJerk = rMaxJerk;
    m_tMaxVel = tMaxVel;
    m_tMaxAccel = tMaxAccel;
    m_tMaxJerk = tMaxJerk;

    // Mark all non-executing segments as needing recalculation
    int idx = m_segmentTail;
    while (idx != m_segmentHead) {
        Segment& seg = m_segments[idx];
        // Only allow modifying segments that haven't started generating steps
        if (!seg.executing && seg.lastGenTime < 0.0001f) {
            seg.calculated = false;
            seg.generationComplete = false;
        }
        idx = (idx + 1) % SEGMENT_BUFFER_SIZE;
    }

    // Recalculate all pending segments with new limits
    if (m_segmentHead != m_segmentTail) {
        recalculate();
    }
    
    // Reset generation index to tail (it will fast-forward in process() if needed)
    m_genSegmentIdx = m_segmentTail;
    m_genSegmentStartTime = m_segmentStartTime;
}

void MotionPlanner::setSpeedMultiplier(float mult) {
    float newMult = std::max(0.1f, std::min(1.0f, mult));

    // Only recalculate if multiplier actually changed
    if (fabsf(newMult - m_speedMultiplier) < 0.001f) {
        return;
    }

    m_speedMultiplier = newMult;

    // Mark all non-executing segments as needing recalculation
    int idx = m_segmentTail;
    while (idx != m_segmentHead) {
        Segment& seg = m_segments[idx];
        // Only allow modifying segments that haven't started generating steps
        if (!seg.executing && seg.lastGenTime < 0.0001f) {
            seg.calculated = false;
            seg.generationComplete = false;
        }
        idx = (idx + 1) % SEGMENT_BUFFER_SIZE;
    }

    // Recalculate all pending segments with new speed
    recalculate();

    // Reset generation index to tail
    m_genSegmentIdx = m_segmentTail;
    m_genSegmentStartTime = m_segmentStartTime;
}

void MotionPlanner::setEndOfPattern(bool ending) {
    m_endOfPattern = ending;
}

void MotionPlanner::getDiagnostics(uint32_t& queueDepth, uint32_t& underruns) const {
    queueDepth = STEP_QUEUE_SIZE - 1 - getStepQueueSpace();
    underruns = m_underrunCount.load();
}

void MotionPlanner::getProfileData(uint32_t& maxProcessUs, uint32_t& maxIntervalUs, uint32_t& avgGenUs) {
    maxProcessUs = m_processProfiler.getMax();
    maxIntervalUs = m_intervalProfiler.getMax();
    avgGenUs = m_genProfiler.getAvg();
    
    // Reset max values after reading to capture transient spikes
    m_processProfiler.reset();
    m_intervalProfiler.reset();
    m_genProfiler.reset();
}

void MotionPlanner::getTelemetry(PlannerTelemetry& out) {
    int queueDepth = STEP_QUEUE_SIZE - 1 - getStepQueueSpace();
    if (queueDepth < 0) queueDepth = 0;
    out.queueDepth = static_cast<uint32_t>(queueDepth);
    out.minQueueDepth = (m_minQueueDepth == 0xFFFFFFFFu) ? out.queueDepth : m_minQueueDepth;
    out.maxQueueDepth = m_maxQueueDepth;
    out.underruns = m_underrunCount.load();
    out.consecutiveUnderruns = m_consecutiveUnderruns.load();
    out.maxConsecutiveUnderruns = m_maxConsecutiveUnderruns.exchange(out.consecutiveUnderruns);
    out.lastUnderrunUs = m_lastUnderrunUs.load();
    out.segmentHead = static_cast<uint32_t>(m_segmentHead);
    out.segmentTail = static_cast<uint32_t>(m_segmentTail);
    out.genSegmentIdx = static_cast<uint32_t>(m_genSegmentIdx);
    out.completedCount = m_completedCount;
    out.fillStopHorizon = m_fillStopHorizonCount.exchange(0);
    out.fillStopQueueFull = m_fillStopQueueFullCount.exchange(0);
    out.fillStopTimeBudget = m_fillStopTimeBudgetCount.exchange(0);
    out.lastFillStopReason = m_lastFillStopReason.load();
    out.timerActive = m_timerActive;
    out.running = m_running;

    m_minQueueDepth = out.queueDepth;
    m_maxQueueDepth = out.queueDepth;
}

int32_t MotionPlanner::thetaToSteps(float theta) const {
    return (int32_t)(theta * m_stepsPerRadT);
}

int32_t MotionPlanner::rhoToSteps(float rho) const {
    return (int32_t)(rho * m_stepsPerMmR);
}

float MotionPlanner::stepsToTheta(int32_t steps) const {
    return (float)steps / m_stepsPerRadT;
}

float MotionPlanner::stepsToRho(int32_t steps) const {
    return (float)steps / m_stepsPerMmR;
}

// ISR callback - called at 10kHz (100us period)
void IRAM_ATTR MotionPlanner::stepTimerISR(void* arg) {
    MotionPlanner* planner = static_cast<MotionPlanner*>(arg);
    planner->handleStepTimer();
}

void IRAM_ATTR MotionPlanner::handleStepTimer() {
    // Check if there's a step event ready to execute
    if (m_stepQueueHead == m_stepQueueTail) {
        if (m_running) {
            uint32_t now = micros();
            m_underrunCount++;
            m_lastUnderrunUs.store(now);
            uint32_t consecutive = m_consecutiveUnderruns.fetch_add(1) + 1;
            uint32_t prevMax = m_maxConsecutiveUnderruns.load();
            while (consecutive > prevMax &&
                   !m_maxConsecutiveUnderruns.compare_exchange_weak(prevMax, consecutive)) {
            }
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
        FastGPIO::write(T_DIR_PIN, (event.dirMask & 0x01));
    }
    if (event.stepMask & 0x02) {
        FastGPIO::write(R_DIR_PIN, (event.dirMask & 0x02));
    }

    // Small delay for direction setup (inline)
    for (volatile int i = 0; i < 10; i++) {}

    // Generate step pulses
    if (event.stepMask & 0x01) {
        FastGPIO::setHigh(T_STEP_PIN);
    }
    if (event.stepMask & 0x02) {
        FastGPIO::setHigh(R_STEP_PIN);
    }

    // Brief pulse width delay
    for (volatile int i = 0; i < 20; i++) {}

    // End step pulses
    if (event.stepMask & 0x01) {
        FastGPIO::setLow(T_STEP_PIN);
        // Update executed position
        if (event.dirMask & 0x01) {
            m_executedTSteps++;
        } else {
            m_executedTSteps--;
        }
    }
    if (event.stepMask & 0x02) {
        FastGPIO::setLow(R_STEP_PIN);
        if (event.dirMask & 0x02) {
            m_executedRSteps++;
        } else {
            m_executedRSteps--;
        }
    }

    // Advance queue tail
    m_stepQueueTail = (m_stepQueueTail + 1) % STEP_QUEUE_SIZE;
    m_consecutiveUnderruns.store(0);
}
