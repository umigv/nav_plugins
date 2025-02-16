#pragma once
#include "Rotation.hpp"
#include <optional>

class Rotation;

class Point {
    public:
    constexpr Point() = default;

    Point(double x, double y);

    Point(double mag, const Rotation& angle);

    double X() const;

    double Y() const;

    Point operator+(const Point& rhs) const;

    Point operator-(const Point& rhs) const;

    Point operator-() const;

    Point operator*(double scalar) const;

    Point operator/(double scalar) const;

    bool operator==(const Point& rhs) const;

    bool operator!=(const Point& rhs) const;

    double theta() const;

    double mag() const;

    double distTo(const Point& rhs) const;

    double angleTo(const Point& rhs) const;

    double dot(const Point& rhs) const;

    double wedge(const Point& rhs) const;

    Point project(const Point& rhs) const;

    Point rotateBy(const Rotation& rhs) const;

    private:
    double x{0.0};
    double y{0.0};
};

double circumradius(const Point& left, const Point& mid, const Point& right);

std::optional<double> circleLineIntersection(const Point& start, const Point& end, const Point& point,
                                             double radius);
