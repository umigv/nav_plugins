#include "Math.hpp"
#include <cmath>

double constrainAngle180(double angle) {
    while(angle > 180.0){
        angle -= 360.0;
    }

    while(angle < -180.0){
        angle += 360.0;
    }

    return angle;
}

std::optional<std::pair<double, double>> quadraticFormula(double a, double b, double c) {
    const double discriminant = b * b - 4 * a * c;

    if(discriminant < 0){
        return std::nullopt;
    }

    return std::make_pair((-b - std::sqrt(discriminant)) / (2 * a), (-b + std::sqrt(discriminant)) / (2 * a));
}
