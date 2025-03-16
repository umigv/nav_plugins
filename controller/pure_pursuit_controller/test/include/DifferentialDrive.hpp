#pragma once
#include "Twist.hpp"
#include "Pose.hpp"
#include <cmath>

class DifferentialDrive {
public:
    DifferentialDrive() = default;

    DifferentialDrive(const Pose& startPose) : pose(startPose){}

    auto getState() const -> const Pose&;

    void move(const Twist& twist, double dt);

private:
    Pose pose;
};
