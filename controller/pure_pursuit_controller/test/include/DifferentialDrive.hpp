#pragma once
#include "Twist.hpp"
#include "Pose.hpp"
#include <cmath>

class DifferentialDrive {
public:
    DifferentialDrive() = default;

    DifferentialDrive(const Pose& startPose) : pose(startPose){}

    const Pose& getState() const;

    void move(const Twist& twist, double dt);

private:
    Pose pose;
};
