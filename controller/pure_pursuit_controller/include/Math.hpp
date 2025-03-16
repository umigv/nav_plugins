#pragma once
#include <optional>

struct QuadraticRoots{
    double root1;
    double root2;
};

auto constrainAngle180(double iAngle) -> double;

auto quadraticFormula(double a, double b, double c) -> std::optional<QuadraticRoots>;
