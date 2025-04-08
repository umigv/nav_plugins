#include "PurePursuit.hpp"
#include <cmath>
#include <iostream>
#include "rclcpp/rclcpp.hpp"

PurePursuit::PurePursuit(const Gains& gains) : gains(gains) {}

void PurePursuit::setPath(const DiscretePath& path) {
    const auto pointsEqual = [](const Point& a, const Point& b) {
        const double dx = std::abs(a.X() - b.X());
        const double dy = std::abs(a.Y() - b.Y());
        return dx < 1e-3 && dy < 1e-3;
    };

    if(std::equal(path.begin(), path.end(), this->path.begin(), this->path.end(), pointsEqual)) {
        //RCLCPP_INFO(rclcpp::get_logger("PurePursuit"), "Path already set");
        return;
    }

    this->path = path;
    minSearchIndex = 0;
    trajectory = Trajectory(this->path, gains);
    closestPointIter = this->path.begin();
    lookAheadPoint = this->path.front();
    finished = false;

    RCLCPP_INFO(rclcpp::get_logger("PurePursuit"), "Path set with %zu points", path.size());
    for(const auto& point : path) {
        RCLCPP_INFO(rclcpp::get_logger("PurePursuit"), "Path point: (%f, %f)", point.X(), point.Y());
    }
}

auto PurePursuit::step(const Pose& pose) const -> Twist {
    if(path.size() == 0 || finished) {
        return Twist{0, 0};
    }

    if (pose.getPoint().distTo(path.back()) < gains.LookAheadDistance()) {
        RCLCPP_INFO(rclcpp::get_logger("PurePursuit"), "Finished path");
        finished = true;
        return Twist{0, 0};
    }

    RCLCPP_INFO(rclcpp::get_logger("PurePursuit"), "Current pose: (%f, %f, %f)", pose.X(), pose.Y(), pose.Theta());
    closestPointIter = closestPoint(closestPointIter, path.end(), pose.getPoint());
    lookAheadPoint = getLookaheadPoint(pose.getPoint()).value_or(lookAheadPoint);
    const std::size_t closestPointIndex = closestPointIter - path.begin();
    const double velocity = trajectory[closestPointIndex];
    const double curvature = curvatureToPoint(pose, lookAheadPoint);

    return Twist{velocity, velocity * curvature};
}

auto PurePursuit::isFinished() const -> bool {
    return finished;
}

auto PurePursuit::getLookaheadPoint(const Point& point) const -> std::optional<Point> {
    for (std::size_t i = static_cast<std::size_t>(minSearchIndex); i < path.size() - 1; i++) {
        const Point& start = path[i];
        const Point& end = path[i + 1];
        const auto t = circleLineIntersection(start, end, point, gains.LookAheadDistance());

        if (!t) {
            continue;
        }

        const double searchIndex = i + *t;
        if (searchIndex >= minSearchIndex) {
            minSearchIndex = searchIndex;
            return start + (end - start) * *t;
        }
    }

    return std::nullopt;
}

PurePursuit::Trajectory::Trajectory(const DiscretePath& path, const Gains& gains)
    : velocity(path.size(), gains.MaxVelocity()) {
    using std::sqrt;
    using std::min;
    velocity.front() = 0;
    velocity.back() = 0;

    // Angular Acceleration
    for (std::size_t i = 1; i < velocity.size() - 1; i++) {
        const double limit = gains.MaxAngularVelocity() / path.getCurvature(i);
        velocity[i] = min(velocity[i], limit);
    }

    // Acceleration
    for (std::size_t i = 1; i < velocity.size(); i++) {
        const double dist = path[i - 1].distTo(path[i]);
        const double limit = sqrt(velocity[i - 1] * velocity[i - 1] + 2 * gains.MaxAcceleration() * dist);
        velocity[i] = min(velocity[i], limit);
    }

    // Deceleration
    for (int i = velocity.size() - 2; i >= 0; i--) {
        const double dist = path[i].distTo(path[i + 1]);
        const double limit = sqrt(velocity[i + 1] * velocity[i + 1] + 2 * gains.MaxAcceleration() * dist);
        velocity[i] = min(velocity[i], limit);
    }

    velocity.front() = velocity[1];
}

PurePursuit::Gains::Gains(double maxVelocity, double maxAcceleration, double trackWidth, double lookAheadDistance)
    : maxVelocity(maxVelocity), 
      maxAcceleration(maxAcceleration), 
      maxAngularVelocity(2 * maxVelocity / trackWidth), 
      lookAheadDistance(lookAheadDistance) {}

auto PurePursuit::Gains::MaxVelocity() const -> double {
    return maxVelocity;
}

auto PurePursuit::Gains::MaxAcceleration() const -> double {
    return maxAcceleration;
}

auto PurePursuit::Gains::MaxAngularVelocity() const -> double {
    return maxAngularVelocity;
}

auto PurePursuit::Gains::LookAheadDistance() const -> double {
    return lookAheadDistance;
}

auto PurePursuit::Trajectory::operator[](size_t index) const -> double {
    return velocity[index];
}
