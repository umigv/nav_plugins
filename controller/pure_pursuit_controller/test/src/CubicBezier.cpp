#include "CubicBezier.hpp"
#include <cmath>

CubicBezier::Knot::Knot(double x, double y, double theta, double magnitude)
    : point(x, y), 
      forwardControl(x + magnitude * std::cos(theta), y + magnitude * std::sin(theta)),
      backwardControl(x + magnitude * std::cos(theta + M_PI), y + magnitude * std::sin(theta + M_PI)) {}

auto CubicBezier::Knot::getPoint() const -> const Point& {
    return point;
}

auto CubicBezier::Knot::getForwardControl() const -> const Point& {
    return forwardControl;
}

auto CubicBezier::Knot::getBackwardControl() const -> const Point& {
    return backwardControl;
}

CubicBezier::CubicBezier(const Knot& start, const Knot& end)
    : c0(start.getPoint()), c1(start.getForwardControl()), c2(end.getBackwardControl()), c3(end.getPoint()) {}

auto CubicBezier::getPoint(double t) const -> Point {
    // clang-format off
    return c0 * (1 - t) * (1 - t) * (1 - t) + 
           c1 * 3 * (1 - t) * (1 - t) * t +
           c2 * 3 * (1 - t) * t * t + 
           c3 * t * t * t;
    // clang-format on
}

auto CubicBezier::getVelocity(double t) const -> Point {
    // clang-format off
    return (c1 - c0) * 3 * (1 - t) * (1 - t) + 
           (c2 - c1) * 6 * (1 - t) * t  + 
           (c3 - c2) * 3 * t * t;
    // clang-format on
}

auto CubicBezier::getAcceleration(double t) const -> Point {
    // clang-format off
    return (c2 - c1 * 2 + c0) * 6 * (1 - t) + 
           (c3 - c2 * 2 + c1) * 6 * t;
    // clang-format on
}

auto CubicBezier::toDiscretePath(std::size_t numSamples) const -> DiscretePath {
    const double resolution = 1.0 / (numSamples - 1);
    std::vector<Point> path;
    path.reserve(numSamples);

    for (std::size_t i = 0; i < numSamples; ++i) {
        path.push_back(getPoint(i * resolution));
    }

    return DiscretePath(path);
}