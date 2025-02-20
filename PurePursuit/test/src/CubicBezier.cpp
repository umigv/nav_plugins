#include "CubicBezier.hpp"
#include <math.h>

CubicBezier::Knot::Knot(double x, double y, double theta, double magnitude)
    : point(x, y), 
      forwardControl(x + magnitude * std::cos(theta), y + magnitude * std::sin(theta)),
      backwardControl(x + magnitude * std::cos(theta + M_PI), y + magnitude * std::sin(theta + M_PI)){}

const Point& CubicBezier::Knot::getPoint() const {
    return point;
}

const Point& CubicBezier::Knot::getForwardControl() const {
    return forwardControl;
}

const Point& CubicBezier::Knot::getBackwardControl() const {
    return backwardControl;
}

CubicBezier::CubicBezier(const Knot& start, const Knot& end)
    : c0(start.getPoint()), c1(start.getForwardControl()), c2(end.getBackwardControl()), c3(end.getPoint()){};

Point CubicBezier::getPoint(double t) const {
    // clang-format off
    return c0 * (1 - t) * (1 - t) * (1 - t) + 
           c1 * 3 * (1 - t) * (1 - t) * t +
           c2 * 3 * (1 - t) * t * t + 
           c3 * t * t * t;
    // clang-format on
}

Point CubicBezier::getVelocity(double t) const {
    // clang-format off
    return (c1 - c0) * 3 * (1 - t) * (1 - t) + 
           (c2 - c1) * 6 * (1 - t) * t  + 
           (c3 - c2) * 3 * t * t;
    // clang-format on
}

Point CubicBezier::getAcceleration(double t) const {
    // clang-format off
    return (c2 - c1 * 2 + c0) * 6 * (1 - t) + 
           (c3 - c2 * 2 + c1) * 6 * t;
    // clang-format on
}

DiscretePath CubicBezier::toDiscretePath(std::size_t numSamples) const {
    const double resolution = 1.0 / (numSamples - 1);
    std::vector<Point> path;
    path.reserve(numSamples);

    for (double t = 0; t <= 1; t += resolution) {
        path.push_back(getPoint(t));
    }

    return path;
}