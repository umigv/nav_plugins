#pragma once
#include "Point.hpp"

class Pose {
    public:
    constexpr Pose() = default;

    Pose(const Point& point, const Rotation& rotation);

    Pose(double x, double y, const Rotation& rotation);

    const Point& getPoint() const;

    const Rotation& getRotation() const;

    double X() const;

    double Y() const;

    double Theta() const;

    bool operator==(const Pose& rhs) const;

    bool operator!=(const Pose& rhs) const;

    private:
    Point point;
    Rotation rotation;
};

double curvatureToPoint(const Pose& position, const Point& point);
