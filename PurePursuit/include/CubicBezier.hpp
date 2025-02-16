#pragma once
#include "Point.hpp"
#include "DiscretePath.hpp"

class CubicBezier{
    public:
    class Knot {
        public:
        Knot(double x, double y, double theta, double magnitude);

        const Point& getPoint() const;

        const Point& getForwardControl() const;

        const Point& getBackwardControl() const;

        private:
        Point point;
        Point forwardControl;
        Point backwardControl;
    };

    CubicBezier(const Knot& start, const Knot& end);

    Point getPoint(double t) const;

    Point getVelocity(double t) const;

    Point getAcceleration(double t) const;

    DiscretePath toDiscretePath(std::size_t numSamples) const;

    private:
    Point c0;
    Point c1;
    Point c2;
    Point c3;
};
