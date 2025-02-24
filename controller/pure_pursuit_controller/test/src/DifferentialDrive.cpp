#include "DifferentialDrive.hpp"

const Pose& DifferentialDrive::getState() const {
    return pose;
}

void DifferentialDrive::move(const Twist& twist, double dt) {
    const double dx = (twist.linearVelocity) * std::cos(pose.Theta()) * dt;
    const double dy = (twist.linearVelocity) * std::sin(pose.Theta()) * dt;
    const double dtheta = twist.angularVelocity * dt;

    pose = Pose(pose.X() + dx, 
                pose.Y() + dy, 
                pose.Theta() + dtheta);
}
