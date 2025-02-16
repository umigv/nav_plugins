#include "Pose.hpp"
#include <cmath>

Pose::Pose(const Point& point, const Rotation& rotation) : point(point), rotation(rotation) {}

Pose::Pose(double x, double y, const Rotation& rotation) : point(x, y), rotation(rotation) {}

const Point& Pose::getPoint() const {
    return point;
}

const Rotation& Pose::getRotation() const {
    return rotation;
}

double Pose::X() const {
    return point.X();
}

double Pose::Y() const {
    return point.Y();
}

double Pose::Theta() const {
    return rotation.Theta();
}

bool Pose::operator==(const Pose& rhs) const {
    return point == rhs.point && rotation == rhs.rotation;
}

bool Pose::operator!=(const Pose& rhs) const {
    return !(*this==rhs);
}

double curvatureToPoint(const Pose& position, const Point& point) {
    const double a = -std::tan(position.Theta());
    const double b = 1;
    const double c = std::tan(position.Theta()) * position.X() - position.Y();

    const double x = std::abs(point.X() * a + point.Y() * b + c) / std::sqrt(a * a + b * b);
    const double sideL = std::sin(position.Theta()) * (point.X() - position.X()) - 
                         std::cos(position.Theta()) * (point.Y() - position.Y());

    if (sideL == 0) {
        return 0;
    }

    const double chord = position.getPoint().distTo(point);

    if(std::signbit(sideL)){
        return 2 * x / (chord * chord);
    }
    else{
        return -2 * x / (chord * chord);
    }
}

