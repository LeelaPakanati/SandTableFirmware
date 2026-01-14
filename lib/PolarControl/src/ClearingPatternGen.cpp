#include "ClearingPatternGen.hpp"

ClearingPatternGen::ClearingPatternGen(ClearingPattern pattern, double maxRho)
    : m_pattern(pattern),
      m_maxRho(maxRho),
      m_currentTheta(0.0),
      m_currentRho(0.0),
      m_circleIndex(0),
      m_spokeIndex(0),
      m_inward(false),
      m_complete(false) {

    Serial.print("ClearingPatternGen created: pattern=");
    Serial.print(pattern);
    Serial.print(", maxRho=");
    Serial.println(maxRho);
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
    // Spiral from center (rho=0) to edge (rho=maxRho) over 10 rotations
    static constexpr double NUM_ROTATIONS = 10.0;
    static constexpr double TOTAL_THETA = NUM_ROTATIONS * 2.0 * PI;
    static constexpr double THETA_STEP = 0.1;

    if (m_currentTheta > TOTAL_THETA) {
        m_complete = true;
        Serial.println("Spiral outward complete");
        return {std::nan(""), std::nan("")};
    }

    // Linear rho increase from 0 to maxRho
    double rho = m_maxRho * (m_currentTheta / TOTAL_THETA);
    double theta = m_currentTheta;

    m_currentTheta += THETA_STEP;

    return {theta, rho};
}

PolarCord_t ClearingPatternGen::generateSpiralInward() {
    // Spiral from edge (rho=maxRho) to center (rho=0) over 10 rotations
    static constexpr double NUM_ROTATIONS = 10.0;
    static constexpr double TOTAL_THETA = NUM_ROTATIONS * 2.0 * PI;
    static constexpr double THETA_STEP = 0.1;

    if (m_currentTheta > TOTAL_THETA) {
        m_complete = true;
        Serial.println("Spiral inward complete");
        return {std::nan(""), std::nan("")};
    }

    // Linear rho decrease from maxRho to 0
    double rho = m_maxRho * (1.0 - m_currentTheta / TOTAL_THETA);
    double theta = m_currentTheta;

    m_currentTheta += THETA_STEP;

    return {theta, rho};
}

PolarCord_t ClearingPatternGen::generateConcentricCircles() {
    // 9 circles at evenly spaced radii
    static constexpr int NUM_CIRCLES = 9;
    static constexpr double THETA_STEP = 0.1;
    static constexpr double CIRCLE_THETA_MAX = 2.0 * PI;

    if (m_circleIndex >= NUM_CIRCLES) {
        m_complete = true;
        Serial.println("Concentric circles complete");
        return {std::nan(""), std::nan("")};
    }

    // Current circle radius
    double rho = m_maxRho * (m_circleIndex + 1) / NUM_CIRCLES;

    // Generate points around the circle
    if (m_currentTheta >= CIRCLE_THETA_MAX) {
        // Move to next circle
        m_circleIndex++;
        m_currentTheta = 0.0;

        // Return to the start of the next circle
        if (m_circleIndex < NUM_CIRCLES) {
            rho = m_maxRho * (m_circleIndex + 1) / NUM_CIRCLES;
            return {0.0, rho};
        } else {
            m_complete = true;
            return {std::nan(""), std::nan("")};
        }
    }

    double theta = m_currentTheta;
    m_currentTheta += THETA_STEP;

    return {theta, rho};
}

PolarCord_t ClearingPatternGen::generateZigzagRadial() {
    // 16 spokes at 22.5° intervals (360° / 16 = 22.5°)
    static constexpr int NUM_SPOKES = 16;
    static constexpr double SPOKE_ANGLE = (2.0 * PI) / NUM_SPOKES;
    static constexpr double RHO_STEP = 20.0; // mm per step

    if (m_spokeIndex >= NUM_SPOKES) {
        m_complete = true;
        Serial.println("Zigzag radial complete");
        return {std::nan(""), std::nan("")};
    }

    double theta = m_spokeIndex * SPOKE_ANGLE;

    if (!m_inward) {
        // Moving outward (center → edge)
        if (m_currentRho >= m_maxRho) {
            m_inward = true;
            return {theta, m_maxRho};
        }
        double rho = m_currentRho;
        m_currentRho += RHO_STEP;
        return {theta, rho};
    } else {
        // Moving inward (edge → center)
        if (m_currentRho <= 0.0) {
            // Move to next spoke
            m_spokeIndex++;
            m_inward = false;
            m_currentRho = 0.0;

            if (m_spokeIndex >= NUM_SPOKES) {
                m_complete = true;
                return {std::nan(""), std::nan("")};
            }

            return {m_spokeIndex * SPOKE_ANGLE, 0.0};
        }
        double rho = m_currentRho;
        m_currentRho -= RHO_STEP;
        return {theta, rho};
    }
}

PolarCord_t ClearingPatternGen::generatePetalFlower() {
    // Petal flower with 8 petals, multiple passes at increasing amplitudes
    static constexpr int NUM_PETALS = 8;
    static constexpr int NUM_PASSES = 5; // 5 passes at different amplitudes
    static constexpr double THETA_STEP = 0.05; // Finer step for smoother petals
    static constexpr double PASS_THETA_MAX = 2.0 * PI;

    if (m_circleIndex >= NUM_PASSES) {
        m_complete = true;
        Serial.println("Petal flower complete");
        return {std::nan(""), std::nan("")};
    }

    // Amplitude increases with each pass
    double amplitude = m_maxRho * (m_circleIndex + 1) / NUM_PASSES;

    if (m_currentTheta >= PASS_THETA_MAX) {
        // Move to next pass
        m_circleIndex++;
        m_currentTheta = 0.0;

        if (m_circleIndex >= NUM_PASSES) {
            m_complete = true;
            return {std::nan(""), std::nan("")};
        }

        // Start next pass
        amplitude = m_maxRho * (m_circleIndex + 1) / NUM_PASSES;
    }

    double theta = m_currentTheta;
    // Petal function: rho = amplitude * |cos(NUM_PETALS * theta)|
    double rho = amplitude * std::abs(cos(NUM_PETALS * theta));

    m_currentTheta += THETA_STEP;

    return {theta, rho};
}
