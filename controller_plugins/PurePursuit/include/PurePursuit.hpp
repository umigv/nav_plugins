#pragma once
#include "Pose.hpp"
#include "DiscretePath.hpp"
#include "Twist.hpp"

class PurePursuit {
    public:
    class Gains {
        public:
        Gains(double maxVelocity, double maxAcceleration, double trackWidth, double lookAheadDistance);
        
        double MaxVelocity() const;
        double MaxAcceleration() const;
        double MaxAngularVelocity() const;
        double LookAheadDistance() const;

        private:
        double maxVelocity;
        double maxAcceleration;
        double maxAngularVelocity;
        double lookAheadDistance;
    };

    PurePursuit(const Gains& gains);

    void setPath(const DiscretePath& path);

    Twist step(const Pose& pose) const;

    bool isFinished() const;

    private:
    class Trajectory{
        public:
        Trajectory() = default;

        Trajectory(const DiscretePath& path, const Gains& gains);

        double operator[](size_t index) const;

        private:
        std::vector<double> velocity;
    };

    std::optional<Point> getLookaheadPoint(const Point& point) const;

    Gains gains;
    DiscretePath path;
    Trajectory trajectory;
    mutable double minSearchIndex;
    mutable DiscretePath::const_iterator closestPointIter;
    mutable Point lookAheadPoint;
    mutable bool finished;
};
