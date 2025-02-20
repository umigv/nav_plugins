#pragma once
#include <optional>

double constrainAngle180(double iAngle);

std::optional<std::pair<double, double>> quadraticFormula(double a, double b, double c);
