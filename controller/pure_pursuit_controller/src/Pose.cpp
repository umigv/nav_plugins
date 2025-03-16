#include "Pose.hpp"
#include <cmath>

Pose::Pose(const Point& point, const Rotation& rotation) 
    : point(point), rotation(rotation) {}

Pose::Pose(double x, double y, const Rotation& rotation) 
    : point(x, y), rotation(rotation) {}

auto Pose::getPoint() const -> const Point& {
    return point;
}

auto Pose::getRotation() const -> const Rotation& {
    return rotation;
}

auto Pose::X() const -> double {
    return point.X();
}

auto Pose::Y() const -> double {
    return point.Y();
}

auto Pose::Theta() const -> double {
    return rotation.Theta();
}

auto curvatureToPoint(const Pose& position, const Point& point) -> double {
    const double sideL = std::sin(position.Theta()) * (point.X() - position.X()) - 
                         std::cos(position.Theta()) * (point.Y() - position.Y());

    if (sideL == 0) {
        return 0;
    }

    const double a = -std::tan(position.Theta());
    const double b = 1;
    const double c = std::tan(position.Theta()) * position.X() - position.Y();
    const double x = std::abs(point.X() * a + point.Y() * b + c) / std::sqrt(a * a + b * b);
    const double chord = position.getPoint().distTo(point);

    if (std::signbit(sideL)) {
        return 2 * x / (chord * chord);
    } 
    else {
        return -2 * x / (chord * chord);
    }
}
