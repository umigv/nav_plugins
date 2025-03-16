#pragma once
#include "Pose.hpp"
#include "DiscretePath.hpp"
#include "Twist.hpp"

class PurePursuit {
    public:
    class Gains {
        public:
        Gains(double maxVelocity, double maxAcceleration, double trackWidth, double lookAheadDistance);
        
        auto MaxVelocity() const -> double;
        auto MaxAcceleration() const -> double;
        auto MaxAngularVelocity() const -> double;
        auto LookAheadDistance() const -> double;

        private:
        double maxVelocity;
        double maxAcceleration;
        double maxAngularVelocity;
        double lookAheadDistance;
    };

    PurePursuit(const Gains& gains);

    auto setPath(const DiscretePath& path) -> void;

    auto step(const Pose& pose) const -> Twist;

    auto isFinished() const -> bool;

    private:
    class Trajectory {
        public:
        Trajectory() = default;

        Trajectory(const DiscretePath& path, const Gains& gains);

        auto operator[](size_t index) const -> double;

        private:
        std::vector<double> velocity;
    };

    auto getLookaheadPoint(const Point& point) const -> std::optional<Point>;

    Gains gains;
    DiscretePath path;
    mutable Trajectory trajectory;
    mutable double minSearchIndex;
    mutable DiscretePath::const_iterator closestPointIter;
    mutable Point lookAheadPoint;
    mutable bool finished;
};
