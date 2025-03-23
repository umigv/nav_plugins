#include "Math.hpp"
#include <cmath>

auto constrainAngle180(double angle) -> double {
    while (angle > 180.0) {
        angle -= 360.0;
    }

    while (angle < -180.0) {
        angle += 360.0;
    }

    return angle;
}

auto quadraticFormula(double a, double b, double c) -> std::optional<QuadraticRoots> {
    const double discriminant = b * b - 4 * a * c;

    if (discriminant < 0) {
        return std::nullopt;
    }

    const double root1 = (-b - std::sqrt(discriminant)) / (2 * a);
    const double root2 = (-b + std::sqrt(discriminant)) / (2 * a);
    return std::make_optional<QuadraticRoots>(root1, root2);
}
