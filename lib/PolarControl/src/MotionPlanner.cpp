#include "MotionPlanner.hpp"
#include "PolarUtils.hpp"
#include <Arduino.h>
#include <cmath>
#include <algorithm>
#include "driver/gpio.h"

// Fast GPIO macros for ESP32
#define GPIO_SET_HIGH(pin) \
    if (pin < 32) GPIO.out_w1ts = (1 << pin); \
    else GPIO.out1_w1ts.val = (1 << (pin - 32))

#define GPIO_SET_LOW(pin) \
    if (pin < 32) GPIO.out_w1tc = (1 << pin); \
    else GPIO.out1_w1tc.val = (1 << (pin - 32))

MotionPlanner::MotionPlanner() {
}

MotionPlanner::~MotionPlanner() {
    stop();
    if (m_stepTimer) {
        esp_timer_delete(m_stepTimer);
        m_stepTimer = nullptr;
    }
    if (m_mutex) {
        vSemaphoreDelete(m_mutex);
        m_mutex = nullptr;
    }
}

void MotionPlanner::init(
    int stepsPerMmR,
    int stepsPerRadT,
    double maxRho,
    double rMaxVelocity,
    double rMaxAccel,
    double rMaxJerk,
    double tMaxVelocity,
    double tMaxAccel,
    double tMaxJerk
) {
    m_stepsPerMmR = stepsPerMmR;
    m_stepsPerRadT = stepsPerRadT;
    m_maxRho = maxRho;
    m_rMaxVelocity = rMaxVelocity;
    m_rMaxAccel = rMaxAccel;
    m_rMaxJerk = rMaxJerk;
    m_tMaxVelocity = tMaxVelocity;
    m_tMaxAccel = tMaxAccel;
    m_tMaxJerk = tMaxJerk;

    // Create mutex
    if (!m_mutex) {
        m_mutex = xSemaphoreCreateMutex();
    }

    // Create timer for step generation
    if (!m_stepTimer) {
        esp_timer_create_args_t timerArgs = {
            .callback = stepTimerISR,
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "step_timer",
            .skip_unhandled_events = true
        };
        esp_timer_create(&timerArgs, &m_stepTimer);
    }

    // Initialize GPIO
    pinMode(m_rStepPin, OUTPUT);
    pinMode(m_rDirPin, OUTPUT);
    pinMode(m_tStepPin, OUTPUT);
    pinMode(m_tDirPin, OUTPUT);

    // Clear block buffer
    m_head = m_tail = m_count = 0;
    m_completedCount = 0;
    m_endOfPattern = false;
    for (int i = 0; i < LOOKAHEAD_SIZE; i++) {
        m_blocks[i].planned = false;
        m_blocks[i].executing = false;
    }

    Serial.println("MotionPlanner initialized");
}

void MotionPlanner::setSpeedMultiplier(double mult) {
    m_speedMult = std::max(0.1, std::min(1.0, mult));
}

void MotionPlanner::setEndOfPattern(bool ending) {
    if (m_endOfPattern == ending) return;  // No change

    m_endOfPattern = ending;

    // If pattern is ending and we have blocks queued, recalculate to plan deceleration
    if (ending && m_count > 0) {
        xSemaphoreTake(m_mutex, portMAX_DELAY);
        recalculate();
        xSemaphoreGive(m_mutex);
    }
}

bool MotionPlanner::hasSpace() const {
    return m_count < LOOKAHEAD_SIZE - 2;  // Keep some buffer room
}

bool MotionPlanner::isRunning() const {
    return m_running;
}

bool MotionPlanner::isIdle() const {
    return !m_running && m_count == 0;
}

bool MotionPlanner::addSegment(double theta, double rho) {
    if (m_count >= LOOKAHEAD_SIZE - 1) {
        return false;  // Queue full
    }

    xSemaphoreTake(m_mutex, portMAX_DELAY);

    PlannerBlock& block = m_blocks[m_tail];

    // Calculate target steps
    block.tTargetSteps = (int32_t)round(theta * m_stepsPerRadT);
    block.rTargetSteps = (int32_t)round(rho * m_stepsPerMmR);

    // Calculate delta from previous position
    int32_t prevR, prevT;
    double prevTheta, prevRho;
    if (m_count > 0) {
        int prevIdx = prevIndex(m_tail);
        prevR = m_blocks[prevIdx].rTargetSteps;
        prevT = m_blocks[prevIdx].tTargetSteps;
    } else {
        prevR = m_currentRSteps;
        prevT = m_currentTSteps;
    }
    prevTheta = (double)prevT / m_stepsPerRadT;
    prevRho = (double)prevR / m_stepsPerMmR;

    block.rDeltaSteps = block.rTargetSteps - prevR;
    block.tDeltaSteps = block.tTargetSteps - prevT;

    // Calculate physical deltas
    block.dRho = rho - prevRho;          // mm
    block.dTheta = theta - prevTheta;    // radians

    // Calculate Cartesian deltas for junction direction analysis
    double x1 = prevRho * cos(prevTheta);
    double y1 = prevRho * sin(prevTheta);
    double x2 = rho * cos(theta);
    double y2 = rho * sin(theta);
    double dx = x2 - x1;
    double dy = y2 - y1;

    // Calculate distance for this segment
    // For a polar machine, the path is an Archimedean spiral.
    // We approximate the distance with the arc length.
    double rhoAvg = (prevRho + rho) / 2.0;
    double dTheta = theta - prevTheta;
    block.distance = sqrt(block.dRho * block.dRho + (rhoAvg * dTheta) * (rhoAvg * dTheta));

    if (block.distance < 0.01) {
        block.distance = 0.01;  // Minimum distance
    }

    // Calculate effective limits based on which axis is more constraining
    // Time needed if only rho moved at max speed
    double timeR = fabs(block.dRho) / m_rMaxVelocity;
    // Time needed if only theta moved at max speed
    double timeT = fabs(block.dTheta) / m_tMaxVelocity;
    // Minimum time is the max of these (limited by slower axis)
    double minTime = std::max(timeR, timeT);

    if (minTime > 0.0001) {
        // Effective max velocity = Cartesian distance / min time
        block.maxVelocity = block.distance / minTime;
    } else {
        block.maxVelocity = m_rMaxVelocity;  // Fallback for tiny moves
    }

    // Similarly for acceleration: calculate effective limits
    // Time to accelerate rho to max speed: t = v / a
    // We want to ensure neither axis exceeds its accel limit
    // For a given Cartesian accel 'a', the axis accels are proportional to their deltas
    if (block.distance > 0.001) {
        double rRatio = fabs(block.dRho) / block.distance;
        double tRatio = fabs(block.dTheta) * m_maxRho / block.distance;  // Convert rad to mm-equivalent at max radius

        // Effective accel is limited by the axis that would exceed its limit first
        double effectiveAccelFromR = (rRatio > 0.001) ? m_rMaxAccel / rRatio : m_rMaxAccel * 1000;
        double effectiveAccelFromT = (tRatio > 0.001) ? m_tMaxAccel * m_maxRho / tRatio : m_tMaxAccel * m_maxRho * 1000;
        block.maxAccel = std::min(effectiveAccelFromR, effectiveAccelFromT);

        // Same for jerk
        double effectiveJerkFromR = (rRatio > 0.001) ? m_rMaxJerk / rRatio : m_rMaxJerk * 1000;
        double effectiveJerkFromT = (tRatio > 0.001) ? m_tMaxJerk * m_maxRho / tRatio : m_tMaxJerk * m_maxRho * 1000;
        block.maxJerk = std::min(effectiveJerkFromR, effectiveJerkFromT);
    } else {
        block.maxAccel = m_rMaxAccel;
        block.maxJerk = m_rMaxJerk;
    }

    // Store direction for junction calculation
    if (block.distance > 0.001) {
        block.unitDirX = dx / block.distance;
        block.unitDirY = dy / block.distance;
        m_prevDirX = block.unitDirX;
        m_prevDirY = block.unitDirY;
        m_hasPrevDir = true;
    } else {
        block.unitDirX = m_prevDirX;
        block.unitDirY = m_prevDirY;
    }

    // Initial velocity settings (will be refined by recalculate)
    block.cruiseSpeed = block.maxVelocity * m_speedMult;
    block.entrySpeed = 0;
    block.exitSpeed = 0;
    block.planned = false;
    block.executing = false;

    m_lastTheta = theta;
    m_lastRho = rho;

    m_tail = nextIndex(m_tail);
    m_count++;

    // Recalculate velocity profile with lookahead
    recalculate();

    xSemaphoreGive(m_mutex);
    return true;
}

double MotionPlanner::calculateJunctionSpeed(int blockIdx) {
    // Get previous and current block
    int prevIdx = prevIndex(blockIdx);
    if (prevIdx == blockIdx || m_count < 2) {
        return 0;  // First block, start from zero
    }

    PlannerBlock& prev = m_blocks[prevIdx];
    PlannerBlock& curr = m_blocks[blockIdx];

    // Use the more restrictive of the two blocks' limits
    double maxVel = std::min(prev.maxVelocity, curr.maxVelocity) * m_speedMult;
    double maxAcc = std::min(prev.maxAccel, curr.maxAccel);

    // Dot product of unit direction vectors gives cos of angle
    double cosAngle = prev.unitDirX * curr.unitDirX + prev.unitDirY * curr.unitDirY;

    // If nearly straight, allow full speed
    if (cosAngle > 0.999) {
        return maxVel;
    }

    // If reversing or sharp corner, slow down significantly
    if (cosAngle < -0.99) {
        return 0;
    }

    // Junction velocity from centripetal acceleration
    // v² = a * r, where r ≈ deviation / (1 - cosAngle)
    // Actually, a more standard formula for junction speed:
    // v = sqrt( (a * deviation * sin(theta/2)) / (1 - sin(theta/2)) )
    // A common approximation for GRBL:
    // v = sqrt( (a * deviation * cosTheta) / (1 - cosTheta) )
    // Where cosTheta is the angle between segments.
    
    // Using the simplified GRBL-style junction velocity formula
    double junctionSpeed = sqrt(maxAcc * JUNCTION_DEVIATION * cosAngle / (1.0 - cosAngle));
    
    // If cosAngle is negative, it's a sharp turn, use a small fraction of speed
    if (cosAngle < 0) {
        junctionSpeed = 0.1 * maxVel;
    }

    return std::min(junctionSpeed, maxVel);
}

void MotionPlanner::planBlock(int blockIdx) {
    PlannerBlock& block = m_blocks[blockIdx];

    if (block.distance < 0.001) {
        block.profile.totalTime = 0;
        block.planned = true;
        return;
    }

    // Calculate S-curve profile for this block using block-specific limits
    SCurve::calculate(
        block.distance,
        block.entrySpeed,
        block.exitSpeed,
        block.cruiseSpeed,
        block.maxAccel,
        block.maxJerk,
        block.profile
    );

    block.planned = true;
}

void MotionPlanner::recalculate() {
    if (m_count == 0) return;

    // Forward pass: limit entry speeds based on what we can accelerate to
    double entrySpeed = m_currentVelocity;
    int idx = m_head;
    for (int i = 0; i < m_count; i++) {
        PlannerBlock& block = m_blocks[idx];
        double blockMaxSpeed = block.maxVelocity * m_speedMult;

        // Junction speed limit
        double junctionLimit = calculateJunctionSpeed(idx);

        // Can't enter faster than junction allows
        block.entrySpeed = std::min(entrySpeed, junctionLimit);

        // Calculate max exit speed given entry speed and distance
        // v² = v0² + 2*a*d
        double maxExitFromAccel = sqrt(
            block.entrySpeed * block.entrySpeed + 2.0 * block.maxAccel * block.distance
        );
        block.exitSpeed = std::min(maxExitFromAccel, blockMaxSpeed);

        entrySpeed = block.exitSpeed;
        idx = nextIndex(idx);
    }

    // Reverse pass: ensure we can decelerate to required exit speeds
    idx = prevIndex(m_tail);

    // If more segments are coming, maintain speed at end of buffer
    // Only decelerate to zero when pattern is actually ending
    double exitSpeed;
    if (m_endOfPattern) {
        exitSpeed = 0;  // Pattern ending, decelerate to stop
    } else {
        // More segments expected - use last block's cruise speed as exit target
        // This allows smooth continuation when new segments are added
        exitSpeed = m_blocks[idx].maxVelocity * m_speedMult;
    }

    for (int i = m_count - 1; i >= 0; i--) {
        PlannerBlock& block = m_blocks[idx];

        // Limit exit speed
        block.exitSpeed = std::min(block.exitSpeed, exitSpeed);

        // Calculate max entry speed given exit speed and distance
        // v0² = v² - 2*a*d  =>  v0 = sqrt(v² + 2*a*d) for decel
        double maxEntryFromDecel = sqrt(
            block.exitSpeed * block.exitSpeed + 2.0 * block.maxAccel * block.distance
        );
        block.entrySpeed = std::min(block.entrySpeed, maxEntryFromDecel);

        exitSpeed = block.entrySpeed;
        idx = prevIndex(idx);
    }

    // Plan each block's S-curve profile
    idx = m_head;
    for (int i = 0; i < m_count; i++) {
        if (!m_blocks[idx].executing) {
            planBlock(idx);
        }
        idx = nextIndex(idx);
    }
}

void MotionPlanner::start() {
    if (m_count == 0) return;

    xSemaphoreTake(m_mutex, portMAX_DELAY);

    m_running = true;
    m_executingBlock = -1;
    m_needNextBlock = false;

    // Start first block
    startNextBlock();

    xSemaphoreGive(m_mutex);
}

void MotionPlanner::startNextBlock() {
    // Skip any zero-step blocks
    while (m_count > 0) {
        PlannerBlock& block = m_blocks[m_head];
        int32_t absR = abs(block.rDeltaSteps);
        int32_t absT = abs(block.tDeltaSteps);

        if (absR == 0 && absT == 0) {
            // Zero-step block, skip it
            Serial.println("Skipping zero-step block");
            block.executing = false;
            m_head = nextIndex(m_head);
            m_count--;
            continue;
        }
        break;
    }

    if (m_count == 0) {
        m_running = false;
        if (m_stepTimer) {
            esp_timer_stop(m_stepTimer);
        }
        Serial.println("Motion complete");
        return;
    }

    PlannerBlock& block = m_blocks[m_head];
    block.executing = true;
    m_executingBlock = m_head;

    // Set up for this block
    m_blockStartTime = micros() / 1000000.0;
    m_blockStartR = m_currentRSteps;
    m_blockStartT = m_currentTSteps;

    // Set up Bresenham for coordinated stepping
    int32_t absR = abs(block.rDeltaSteps);
    int32_t absT = abs(block.tDeltaSteps);

    if (absR >= absT) {
        m_rIsMajor = true;
        m_bresenhamDeltaMajor = absR;
        m_bresenhamDeltaMinor = absT;
    } else {
        m_rIsMajor = false;
        m_bresenhamDeltaMajor = absT;
        m_bresenhamDeltaMinor = absR;
    }

    m_bresenhamError = m_bresenhamDeltaMajor / 2;
    m_stepsRemaining = m_bresenhamDeltaMajor;

    m_rDir = (block.rDeltaSteps >= 0) ? 1 : -1;
    m_tDir = (block.tDeltaSteps >= 0) ? 1 : -1;

    // Set direction pins
    digitalWrite(m_rDirPin, m_rDir > 0 ? HIGH : LOW);
    digitalWrite(m_tDirPin, m_tDir > 0 ? HIGH : LOW);

    // Calculate initial step interval from entry speed
    double speed = std::max(0.1, block.entrySpeed);
    double stepsPerMm;
    if (m_rIsMajor) {
        stepsPerMm = m_stepsPerMmR;
    } else {
        stepsPerMm = (double)m_bresenhamDeltaMajor / block.distance;
    }
    if (stepsPerMm < 1.0) stepsPerMm = 100.0;  // Fallback

    m_stepInterval = (uint32_t)(1000000.0 / (speed * stepsPerMm));
    uint32_t interval = m_stepInterval;
    m_stepInterval = std::max((uint32_t)50, std::min((uint32_t)100000, interval));

    // Start timer
    if (m_stepTimer) {
        esp_timer_stop(m_stepTimer);
        esp_timer_start_periodic(m_stepTimer, m_stepInterval);
    }

    Serial.printf("Block: dR=%ld dT=%ld dist=%.2f spd=%.1f int=%lu major=%c\r\n",
        (long)block.rDeltaSteps, (long)block.tDeltaSteps, block.distance,
        speed, (unsigned long)m_stepInterval, m_rIsMajor ? 'R' : 'T');
}

void MotionPlanner::stop() {
    m_running = false;
    if (m_stepTimer) {
        esp_timer_stop(m_stepTimer);
    }

    // Clear queue
    m_head = m_tail = m_count = 0;
    m_completedCount = 0;
    m_executingBlock = -1;
    m_endOfPattern = false;
}

void MotionPlanner::getCurrentPosition(double& theta, double& rho) const {
    theta = (double)m_currentTSteps / m_stepsPerRadT;
    rho = (double)m_currentRSteps / m_stepsPerMmR;
}

void MotionPlanner::resetTheta() {
    xSemaphoreTake(m_mutex, portMAX_DELAY);

    // Normalize current theta to [-PI, PI]
    double currentTheta = (double)m_currentTSteps / m_stepsPerRadT;
    double normalizedTheta = PolarUtils::normalizeTheta(currentTheta);

    // Update current steps and last theta to the normalized value
    m_currentTSteps = (int32_t)round(normalizedTheta * m_stepsPerRadT);
    m_lastTheta = normalizedTheta;

    // Clear previous direction info so next segment starts fresh
    m_hasPrevDir = false;
    m_prevDirX = 0;
    m_prevDirY = 0;

    Serial.printf("Theta reset to %.4f (was %.4f)\r\n", normalizedTheta, currentTheta);

    xSemaphoreGive(m_mutex);
}

void MotionPlanner::process() {
    // This is called from the main loop
    if (!m_running) return;

    // Check if ISR signaled we need the next block
    if (m_needNextBlock) {
        m_needNextBlock = false;
        startNextBlock();
    }
}

void IRAM_ATTR MotionPlanner::stepTimerISR(void* arg) {
    MotionPlanner* planner = (MotionPlanner*)arg;
    planner->generateSteps();
}

void IRAM_ATTR MotionPlanner::generateSteps() {
    if (!m_running || m_executingBlock < 0 || m_stepsRemaining <= 0) {
        esp_timer_stop(m_stepTimer);
        
        if (m_running && m_executingBlock >= 0) {
            m_completedCount++;
        }

        // Signal main loop to start next block (don't call from ISR!)
        if (m_count > 0) {
            m_blocks[m_head].executing = false;
            m_head = nextIndex(m_head);
            m_count--;
            m_needNextBlock = true;  // Flag for process() to handle
        } else {
            m_running = false;
        }
        return;
    }

    PlannerBlock& block = m_blocks[m_executingBlock];

    // Calculate current progress through the block
    double elapsed = (micros() / 1000000.0) - m_blockStartTime;
    double targetPos = SCurve::getPosition(block.profile, elapsed);
    double currentSpeed = SCurve::getVelocity(block.profile, elapsed);

    // Generate step pulse using Bresenham
    if (m_rIsMajor) {
        // Step R axis
        GPIO_SET_HIGH(m_rStepPin);
        delayMicroseconds(2);
        GPIO_SET_LOW(m_rStepPin);
        m_currentRSteps += m_rDir;

        // Conditionally step T axis
        m_bresenhamError -= m_bresenhamDeltaMinor;
        if (m_bresenhamError < 0) {
            m_bresenhamError += m_bresenhamDeltaMajor;
            GPIO_SET_HIGH(m_tStepPin);
            delayMicroseconds(2);
            GPIO_SET_LOW(m_tStepPin);
            m_currentTSteps += m_tDir;
        }
    } else {
        // Step T axis
        GPIO_SET_HIGH(m_tStepPin);
        delayMicroseconds(2);
        GPIO_SET_LOW(m_tStepPin);
        m_currentTSteps += m_tDir;

        // Conditionally step R axis
        m_bresenhamError -= m_bresenhamDeltaMinor;
        if (m_bresenhamError < 0) {
            m_bresenhamError += m_bresenhamDeltaMajor;
            GPIO_SET_HIGH(m_rStepPin);
            delayMicroseconds(2);
            GPIO_SET_LOW(m_rStepPin);
            m_currentRSteps += m_rDir;
        }
    }

    m_stepsRemaining--;

    // Update step interval based on S-curve velocity
    if (currentSpeed > 0.1) {
        double stepsPerSec = currentSpeed * m_stepsPerMmR;
        uint32_t newInterval = (uint32_t)(1000000.0 / stepsPerSec);
        newInterval = std::max((uint32_t)50, std::min((uint32_t)100000, newInterval));

        // Only update timer if interval changed significantly (> 5%)
        // or if we are nearing the end of the block
        uint32_t diff = abs((int32_t)newInterval - (int32_t)m_stepInterval);
        if (diff > m_stepInterval / 20 || m_stepsRemaining < 5) {
            m_stepInterval = newInterval;
            esp_timer_stop(m_stepTimer);
            esp_timer_start_periodic(m_stepTimer, m_stepInterval);
        }
    }
}
