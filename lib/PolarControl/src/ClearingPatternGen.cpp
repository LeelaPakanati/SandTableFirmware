#include "ClearingPatternGen.hpp"
#include "Logger.hpp"

ClearingPatternGen::ClearingPatternGen(ClearingPattern pattern, float maxRho)
    : m_pattern(pattern),
      m_maxRho(maxRho),
      m_currentTheta(0.0),
      m_currentRho(0.0),
      m_circleIndex(0),
      m_spokeIndex(0),
      m_inward(false),
      m_complete(false) {

    if (m_pattern == ZIGZAG_RADIAL) {
        m_currentRho = m_maxRho;
        m_inward = true;
    }

    LOG("ClearingPatternGen created: pattern=%d, maxRho=%.2f\r\n", pattern, maxRho);
}

PolarCord_t ClearingPatternGen::getNextPos() {
    if (m_complete) {
        return {std::nan(""), std::nan("")};
    }

    switch (m_pattern) {
        case CLEARING_NONE:
            m_complete = true;
            return {std::nan(""), std::nan("")};
        case SPIRAL_OUTWARD:
            return generateSpiralOutward();
        case SPIRAL_INWARD:
            return generateSpiralInward();
        case CONCENTRIC_CIRCLES:
            return generateConcentricCircles();
        case ZIGZAG_RADIAL:
            return generateZigzagRadial();
        case PETAL_FLOWER:
            return generatePetalFlower();
        case CLEARING_RANDOM:
            // Should not happen - caller should resolve RANDOM before creating
            m_pattern = getRandomClearingPattern();
            return getNextPos();
        default:
            m_complete = true;
            return {std::nan(""), std::nan("")};
    }
}

PolarCord_t ClearingPatternGen::generateSpiralOutward() {
    // Spiral from edge (rho=maxRho) to center (rho=0) over 10 rotations (clockwise)
    static constexpr float NUM_ROTATIONS = 10.0f;
    static constexpr float TOTAL_THETA = NUM_ROTATIONS * 2.0f * PI;
    static constexpr float THETA_STEP = 0.1f;

    if (m_currentTheta >= TOTAL_THETA) {
        m_complete = true;
        LOG("Spiral outward complete\r\n");
        return {std::nan(""), std::nan("")};
    }

    if (m_currentTheta + THETA_STEP >= TOTAL_THETA) {
        m_complete = true;
        LOG("Spiral outward complete\r\n");
        return {-TOTAL_THETA, 0.0f};
    }

    // Linear rho decrease from maxRho to 0
    float rho = m_maxRho * (1.0f - m_currentTheta / TOTAL_THETA);
    float theta = -m_currentTheta;

    m_currentTheta += THETA_STEP;

    return {theta, rho};
}

PolarCord_t ClearingPatternGen::generateSpiralInward() {
    // Spiral from edge (rho=maxRho) to center (rho=0) over 10 rotations (counterclockwise)
    static constexpr float NUM_ROTATIONS = 10.0f;
    static constexpr float TOTAL_THETA = NUM_ROTATIONS * 2.0f * PI;
    static constexpr float THETA_STEP = 0.1f;

    if (m_currentTheta >= TOTAL_THETA) {
        m_complete = true;
        LOG("Spiral inward complete\r\n");
        return {std::nan(""), std::nan("")};
    }

    if (m_currentTheta + THETA_STEP >= TOTAL_THETA) {
        m_complete = true;
        LOG("Spiral inward complete\r\n");
        return {TOTAL_THETA, 0.0f};
    }

    // Linear rho decrease from maxRho to 0
    float rho = m_maxRho * (1.0f - m_currentTheta / TOTAL_THETA);
    float theta = m_currentTheta;

    m_currentTheta += THETA_STEP;

    return {theta, rho};
}

PolarCord_t ClearingPatternGen::generateConcentricCircles() {
    // 9 circles at evenly spaced radii
    static constexpr int NUM_CIRCLES = 9;
    static constexpr float THETA_STEP = 0.1f;
    static constexpr float CIRCLE_THETA_MAX = 2.0f * PI;
    static constexpr int LAST_CIRCLE_INDEX = NUM_CIRCLES - 1;

    if (m_circleIndex >= NUM_CIRCLES) {
        m_complete = true;
        LOG("Concentric circles complete\r\n");
        return {std::nan(""), std::nan("")};
    }

    // Current circle radius
    float rho = 0.0f;
    if (LAST_CIRCLE_INDEX > 0) {
        rho = m_maxRho * (LAST_CIRCLE_INDEX - m_circleIndex) / static_cast<float>(LAST_CIRCLE_INDEX);
    }

    // Generate points around the circle
    if (m_currentTheta >= CIRCLE_THETA_MAX) {
        // Move to next circle
        m_circleIndex++;
        m_currentTheta = 0.0;

        // Return to the start of the next circle
        if (m_circleIndex < NUM_CIRCLES) {
            rho = 0.0f;
            if (LAST_CIRCLE_INDEX > 0) {
                rho = m_maxRho * (LAST_CIRCLE_INDEX - m_circleIndex) / static_cast<float>(LAST_CIRCLE_INDEX);
            }
            return {0.0, rho};
        } else {
            m_complete = true;
            return {std::nan(""), std::nan("")};
        }
    }

    float theta = m_currentTheta;
    m_currentTheta += THETA_STEP;

    return {theta, rho};
}

PolarCord_t ClearingPatternGen::generateZigzagRadial() {
    // 16 spokes at 22.5° intervals (360° / 16 = 22.5°)
    static constexpr int NUM_SPOKES = 16;
    static constexpr float SPOKE_ANGLE = (2.0f * PI) / NUM_SPOKES;
    static constexpr float RHO_STEP = 20.0f; // mm per step

    if (m_spokeIndex >= NUM_SPOKES) {
        m_complete = true;
        LOG("Zigzag radial complete\r\n");
        return {std::nan(""), std::nan("")};
    }

    float theta = m_spokeIndex * SPOKE_ANGLE;

    if (!m_inward) {
        // Moving outward (center → edge)
        if (m_currentRho >= m_maxRho) {
            m_inward = true;
            return {theta, m_maxRho};
        }
        float rho = m_currentRho;
        m_currentRho += RHO_STEP;
        return {theta, rho};
    } else {
        // Moving inward (edge → center)
        if (m_currentRho <= 0.0) {
            if (m_spokeIndex >= NUM_SPOKES - 1) {
                m_complete = true;
                return {theta, 0.0};
            }

            // Move to next spoke
            m_spokeIndex++;
            m_inward = false;
            m_currentRho = 0.0;
            return {m_spokeIndex * SPOKE_ANGLE, 0.0};
        }
        float rho = m_currentRho;
        m_currentRho -= RHO_STEP;
        return {theta, rho};
    }
}

PolarCord_t ClearingPatternGen::generatePetalFlower() {
    // Petal flower with 8 petals, multiple passes at decreasing amplitudes
    static constexpr int NUM_PETALS = 8;
    static constexpr int NUM_PASSES = 5; // 5 passes at different amplitudes
    static constexpr float THETA_STEP = 0.05f; // Finer step for smoother petals
    static constexpr float PASS_THETA_MAX = 2.0f * PI;
    static constexpr int LAST_PASS_INDEX = NUM_PASSES - 1;

    if (m_circleIndex >= NUM_PASSES) {
        m_complete = true;
        LOG("Petal flower complete\r\n");
        return {std::nan(""), std::nan("")};
    }

    // Amplitude decreases with each pass
    float amplitude = 0.0f;
    if (LAST_PASS_INDEX > 0) {
        amplitude = m_maxRho * (LAST_PASS_INDEX - m_circleIndex) / static_cast<float>(LAST_PASS_INDEX);
    }

    if (m_currentTheta >= PASS_THETA_MAX) {
        // Move to next pass
        m_circleIndex++;
        m_currentTheta = 0.0;

        if (m_circleIndex >= NUM_PASSES) {
            m_complete = true;
            return {std::nan(""), std::nan("")};
        }

        // Start next pass
        amplitude = 0.0f;
        if (LAST_PASS_INDEX > 0) {
            amplitude = m_maxRho * (LAST_PASS_INDEX - m_circleIndex) / static_cast<float>(LAST_PASS_INDEX);
        }
    }

    float theta = m_currentTheta;
    // Petal function: rho = amplitude * |cos(NUM_PETALS * theta)|
    float rho = amplitude * std::abs(cos(NUM_PETALS * theta));

    m_currentTheta += THETA_STEP;

    return {theta, rho};
}
