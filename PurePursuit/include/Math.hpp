#pragma once
#include <optional>

// QAngularSpeed linearToWheelVelocity(QSpeed velocity, QLength wheelDiameter);

// QSpeed wheelToLinearVelocity(QAngularSpeed velocity, QLength wheelDiameter);

double constrainAngle360(double iAngle);

double constrainAngle180(double iAngle);

std::optional<std::pair<double, double>> quadraticFormula(double a, double b, double c);

//std::pair<double, double> wheelForwardKinematics(double linearVelocity, double curvature, double wheelTrack);

// std::pair<QAcceleration, QAcceleration> wheelForwardKinematics(QAcceleration linearAcceleration, QCurvature curvature,
//                                                                QLength wheelTrack);
