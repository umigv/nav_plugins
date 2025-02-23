#include "PurePursuit.hpp"
#include <cmath>
#include <iostream>

PurePursuit::PurePursuit(const Gains& gains) : gains(gains){}

void PurePursuit::setPath(const DiscretePath& path){
    this->path = path;
    minSearchIndex = 0;
    trajectory = Trajectory(this->path, gains);
    closestPointIter = this->path.begin();
    lookAheadPoint = this->path.front();
    finished = false;
}

Twist PurePursuit::step(const Pose& pose) const{
    if(pose.getPoint().distTo(path.back()) < gains.LookAheadDistance()){
        finished = true;
        return Twist{0, 0};
    }

    closestPointIter = closestPoint(closestPointIter, path.end(), pose.getPoint());
    lookAheadPoint = getLookaheadPoint(pose.getPoint()).value_or(lookAheadPoint);
    const std::size_t closestPointIndex = closestPointIter - path.begin();
    const double velocity = trajectory[closestPointIndex];
    const double curvature = curvatureToPoint(pose, lookAheadPoint);

    return Twist{velocity, velocity * curvature};   
}

bool PurePursuit::isFinished() const{
    return finished;
}

std::optional<Point> PurePursuit::getLookaheadPoint(const Point& point) const{
    for (std::size_t i = static_cast<std::size_t>(minSearchIndex); i < path.size(); i++) {
        const Point& start = path[i];
        const Point& end = path[i + 1];
        const auto t = circleLineIntersection(start, end, point, gains.LookAheadDistance());

        if(!t){
            continue;
        }

        const double searchIndex = i + *t;
        if(searchIndex >= minSearchIndex){
            minSearchIndex = searchIndex;
            return start + (end - start) * *t;
        }
    }

    return std::nullopt;
}

PurePursuit::Trajectory::Trajectory(const DiscretePath& path, const Gains& gains)
    : velocity(path.size(), gains.MaxVelocity()){
    using std::sqrt;
    using std::min;
    velocity.front() = 0;
    velocity.back() = 0;

    // Angular Acceleration
    for(std::size_t i = 1; i < velocity.size()-1; i++) {
        const double limit = gains.MaxAngularVelocity() / path.getCurvature(i);
        velocity[i] = min(velocity[i], limit);
    }

    // Acceleration
    for(std::size_t i = 1; i < velocity.size(); i++){
        const double dist = path[i-1].distTo(path[i]);
        const double limit = sqrt(velocity[i-1] * velocity[i-1] + 2 * gains.MaxAcceleration() * dist);
        velocity[i] = min(velocity[i], limit);
    }

    // Deceleration
    for(int i = velocity.size()-2; i >= 0; i--){
        const double dist = path[i].distTo(path[i+1]);
        const double limit = sqrt(velocity[i+1] * velocity[i+1] + 2 * gains.MaxAcceleration() * dist);
        velocity[i] = min(velocity[i], limit);
    }

    velocity.front() = velocity[1];
}

PurePursuit::Gains::Gains(double maxVelocity, double maxAcceleration, double trackWidth, double lookAheadDistance)
    : maxVelocity(maxVelocity), 
      maxAcceleration(maxAcceleration), 
      maxAngularVelocity(2 * maxVelocity / trackWidth), 
      lookAheadDistance(lookAheadDistance){}

double PurePursuit::Gains::MaxVelocity() const{
    return maxVelocity;
}

double PurePursuit::Gains::MaxAcceleration() const{
    return maxAcceleration;
}

double PurePursuit::Gains::MaxAngularVelocity() const{
    return maxAngularVelocity;
}

double PurePursuit::Gains::LookAheadDistance() const{
    return lookAheadDistance;
}

double PurePursuit::Trajectory::operator[](size_t index) const{
    return velocity[index];
}
