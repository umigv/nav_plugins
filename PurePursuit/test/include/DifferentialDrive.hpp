#pragma once
#include "Twist.hpp"
#include "Pose.hpp"
#include <cmath>

class DifferentialDrive {
public:
    DifferentialDrive(const Pose& startPose, double trackWidth)
        : pose(startPose), trackWidth(trackWidth){}

    const Pose& getState() const {
        return pose;
    }

    void move(const Twist& twist, double dt) {
        const double dx = (twist.linearVelocity) * std::cos(pose.Theta()) * dt;
        const double dy = (twist.linearVelocity) * std::sin(pose.Theta()) * dt;
        const double dtheta = twist.angularVelocity * dt;

        pose = Pose(pose.X() + dx, 
                    pose.Y() + dy, 
                    pose.Theta() + dtheta);
    }

private:
    Pose pose;
    double trackWidth;
};