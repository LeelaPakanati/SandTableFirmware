#pragma once
#include <Arduino.h>
#include <cmath>

struct PolarCord_t {
    double theta;
    double rho;

    String getStr() const {
        return "T:" + String(theta) + "|R:" + String(rho);
    }

    bool isNan() const {
        return std::isnan(theta) || std::isnan(rho);
    }

    PolarCord_t operator - (const PolarCord_t& other) const {
        return {theta - other.theta, rho - other.rho};
    }

    bool operator == (const PolarCord_t& other) const {
        return (theta == other.theta) && (rho == other.rho);
    }

    PolarCord_t operator * (double mult) const {
        return {theta * mult, rho * mult};
    }

    PolarCord_t operator + (const PolarCord_t& other) const {
        return {theta + other.theta, rho + other.rho};
    }

    PolarCord_t operator / (double div) const {
        return {theta / div, rho / div};
    }

    PolarCord_t operator / (int div) const {
        return {theta / (double)div, rho / (double)div};
    }
};

struct CartesianCord_t {
    double x;
    double y;
};

class PolarUtils {
public:
    static CartesianCord_t toCartesian(const PolarCord_t& polar) {
        return {
            polar.rho * std::cos(polar.theta),
            polar.rho * std::sin(polar.theta)
        };
    }

    static CartesianCord_t toNormalizedCartesian(const PolarCord_t& polar, double maxRho) {
        if (maxRho <= 0) return {0.5, 0.5};
        CartesianCord_t cart = toCartesian(polar);
        return {
            (cart.x / maxRho + 1.0) / 2.0,
            (cart.y / maxRho + 1.0) / 2.0
        };
    }

    static double normalizeTheta(double theta) {
        return std::atan2(std::sin(theta), std::cos(theta));
    }
};
