#include "Math.hpp"
#include <cmath>

// QAngularSpeed linearToWheelVelocity(QSpeed velocity, QLength wheelDiameter) {
//     return velocity / (wheelDiameter / 2) * radian;
// }

double constrainAngle360(double iAngle) {
    return iAngle - 360.0 * std::floor(iAngle * (1.0 / 360.0));
}

double constrainAngle180(double iAngle) {
    return iAngle - 360.0 * std::floor((iAngle + 180.0) * (1.0 / 360.0));
}

std::optional<std::pair<double, double>> quadraticFormula(double a, double b, double c) {
    double discriminant = b * b - 4 * a * c;
    if (discriminant == 0) {
        return std::make_pair(-b / (2 * a), -b / (2 * a));
    } else if (discriminant > 0) {
        return std::make_pair((-b - std::sqrt(discriminant)) / (2 * a), (-b + std::sqrt(discriminant)) / (2 * a));
    }

    return std::nullopt;
}

// std::pair<double, double> wheelForwardKinematics(double linearVelocity, double curvature, double wheelTrack) {
//     const auto left = linearVelocity * (2 + curvature * wheelTrack) / 2;
//     const auto right = linearVelocity * (2 - curvature * wheelTrack) / 2;
//     return {left, right};
// }

