#pragma once

#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#include <iostream>

// THR file format: theta rho (one pair per line)
// theta: radians (can exceed 2*PI for multiple rotations)
// rho: 0-1 scaled (multiply by maxRho for actual mm)

struct ThrPosition {
    double theta;  // radians
    double rho;    // 0-1 normalized
};

class ThrReader {
public:
    ThrReader() : m_maxRho(450.0), m_currentIndex(0) {}

    void setMaxRho(double maxRho) {
        m_maxRho = maxRho;
    }

    bool load(const std::string& filepath) {
        m_positions.clear();
        m_currentIndex = 0;

        std::ifstream file(filepath);
        if (!file.is_open()) {
            std::cerr << "Failed to open file: " << filepath << std::endl;
            return false;
        }

        std::string line;
        int lineNum = 0;
        while (std::getline(file, line)) {
            lineNum++;

            // Skip empty lines
            if (line.empty() || line[0] == '#') {
                continue;
            }

            std::istringstream iss(line);
            double theta, rho;
            if (!(iss >> theta >> rho)) {
                std::cerr << "Parse error at line " << lineNum << ": " << line << std::endl;
                continue;
            }

            m_positions.push_back({theta, rho});
        }

        std::cout << "Loaded " << m_positions.size() << " positions from " << filepath << std::endl;
        return !m_positions.empty();
    }

    // Get next position in physical units (theta in rad, rho in mm)
    // Returns NaN when end is reached
    bool getNextPosition(double& theta, double& rho) {
        if (m_currentIndex >= m_positions.size()) {
            theta = std::nan("");
            rho = std::nan("");
            return false;
        }

        const ThrPosition& pos = m_positions[m_currentIndex++];
        theta = pos.theta;
        rho = pos.rho * m_maxRho;
        return true;
    }

    void reset() {
        m_currentIndex = 0;
    }

    size_t size() const {
        return m_positions.size();
    }

    size_t currentIndex() const {
        return m_currentIndex;
    }

    // Get position at specific index
    bool getPositionAt(size_t index, double& theta, double& rho) const {
        if (index >= m_positions.size()) {
            return false;
        }
        theta = m_positions[index].theta;
        rho = m_positions[index].rho * m_maxRho;
        return true;
    }

private:
    std::vector<ThrPosition> m_positions;
    double m_maxRho;
    size_t m_currentIndex;
};
