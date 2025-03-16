#include "DifferentialDrive.hpp"

auto DifferentialDrive::getState() const -> const Pose& {
    return pose;
}

void DifferentialDrive::move(const Twist& twist, double dt) {
    const double dx = (twist.linearVelocity) * pose.getRotation().Cos() * dt;
    const double dy = (twist.linearVelocity) * pose.getRotation().Sin() * dt;
    const double dtheta = twist.angularVelocity * dt;

    pose = Pose(pose.X() + dx, pose.Y() + dy, pose.Theta() + dtheta);
}
