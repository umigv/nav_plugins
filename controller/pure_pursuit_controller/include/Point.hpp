#pragma once
#include "Rotation.hpp"
#include <optional>

class Rotation;

class Point {
    public:
    constexpr Point() = default;

    Point(double x, double y);

    Point(double mag, const Rotation& angle);

    auto X() const -> double;

    auto Y() const -> double;

    auto operator+(const Point& rhs) const -> Point;

    auto operator-(const Point& rhs) const -> Point;

    auto operator-() const -> Point;

    auto operator*(double scalar) const -> Point;

    auto operator/(double scalar) const -> Point;

    auto theta() const -> double;

    auto mag() const -> double;

    auto distTo(const Point& rhs) const -> double;

    auto angleTo(const Point& rhs) const -> double;

    auto dot(const Point& rhs) const -> double;

    auto rotateBy(const Rotation& rhs) const -> Point;

    private:
    double x{0.0};
    double y{0.0};
};

auto lerp(const Point& start, const Point& end, double t) -> Point;

auto circumradius(const Point& left, const Point& mid, const Point& right) -> double;

auto circleLineIntersection(const Point& start, const Point& end, const Point& point, double radius) 
    -> std::optional<double>;
