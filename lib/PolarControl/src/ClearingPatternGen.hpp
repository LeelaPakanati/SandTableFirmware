#pragma once
#include <PosGen.hpp>
#include <cmath>

enum ClearingPattern {
    SPIRAL_OUTWARD,      // Center → edge
    SPIRAL_INWARD,       // Edge → center
    CONCENTRIC_CIRCLES,  // Series of circles from center outward
    ZIGZAG_RADIAL,       // Radial zigzag pattern
    PETAL_FLOWER         // Flower clearing pattern
};

class ClearingPatternGen : public PosGen {
public:
    ClearingPatternGen(ClearingPattern pattern, double maxRho = 450.0);
    PolarCord_t getNextPos() override;

private:
    ClearingPattern m_pattern;
    double m_maxRho;
    double m_currentTheta;
    double m_currentRho;
    int m_circleIndex;  // For concentric circles
    int m_spokeIndex;   // For zigzag radial
    bool m_inward;      // For zigzag radial direction
    bool m_complete;

    // Pattern-specific generation methods
    PolarCord_t generateSpiralOutward();
    PolarCord_t generateSpiralInward();
    PolarCord_t generateConcentricCircles();
    PolarCord_t generateZigzagRadial();
    PolarCord_t generatePetalFlower();
};
