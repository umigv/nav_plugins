#pragma once
#include "Point.hpp"
#include "DiscretePath.hpp"

class CubicBezier {
    public:
    class Knot {
        public:
        Knot(double x, double y, double theta, double magnitude);

        auto getPoint() const -> const Point&;

        auto getForwardControl() const -> const Point&;

        auto getBackwardControl() const -> const Point&;

        private:
        Point point;
        Point forwardControl;
        Point backwardControl;
    };

    CubicBezier(const Knot& start, const Knot& end);

    auto getPoint(double t) const -> Point;

    auto getVelocity(double t) const -> Point;

    auto getAcceleration(double t) const -> Point;

    auto toDiscretePath(std::size_t numSamples) const -> DiscretePath;

    private:
    Point c0;
    Point c1;
    Point c2;
    Point c3;
};