#include "Pose.hpp"
#include <cmath>

Pose::Pose(const Point& point, const Rotation& rotation) 
    : point(point), rotation(rotation) {}

Pose::Pose(double x, double y, const Rotation& rotation) 
    : point(x, y), rotation(rotation) {}

Pose::Pose(double x, double y, double theta) 
    : point(x, y), rotation(theta) {}

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
    const double sideL = position.getRotation().Sin() * (point.X() - position.X()) - 
                         position.getRotation().Cos() * (point.Y() - position.Y());

    if (sideL == 0) {
        return 0;
    }

    const double a = -position.getRotation().Tan();
    const double b = 1;
    const double c = position.getRotation().Tan() * position.X() - position.Y();
    const double x = std::abs(point.X() * a + point.Y() * b + c) / std::sqrt(a * a + b * b);
    const double chord = position.getPoint().distTo(point);

    if (std::signbit(sideL)) {
        return 2 * x / (chord * chord);
    } 
    else {
        return -2 * x / (chord * chord);
    }
}
