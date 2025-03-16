#pragma once
#include "Point.hpp"

class Pose {
    public:
    constexpr Pose() = default;

    Pose(const Point& point, const Rotation& rotation);

    Pose(double x, double y, const Rotation& rotation);

    Pose(double x, double y, double theta);

    auto getPoint() const -> const Point&;

    auto getRotation() const -> const Rotation&;

    auto X() const -> double;

    auto Y() const -> double;

    auto Theta() const -> double;

    private:
    Point point;
    Rotation rotation;
};

auto curvatureToPoint(const Pose& position, const Point& point) -> double;
