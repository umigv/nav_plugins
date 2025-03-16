#include "Rotation.hpp"
#include "Math.hpp"
#include <cmath>

Rotation::Rotation(double theta) 
    : theta(constrainAngle180(theta)), 
      cosine(std::cos(theta)), 
      sine(std::sin(theta)) {}

Rotation::Rotation(double iX, double iY) {
    const auto magnitude = std::hypot(iX, iY);
    if (magnitude > 1e-6) {
        sine = (iY / magnitude);
        cosine = (iX / magnitude);
    } 
    else {
        sine = 0.0;
        cosine = 1.0;
    }
    theta = std::atan2(sine, cosine);
}

auto Rotation::Theta() const -> double {
    return theta;
}

auto Rotation::Sin() const -> double {
    return sine;
}

auto Rotation::Cos() const -> double {
    return cosine;
}

auto Rotation::Tan() const -> double {
    return sine / cosine;
}

auto Rotation::operator+(const Rotation& rhs) const -> Rotation {
    return rotateBy(rhs);
}

auto Rotation::operator-(const Rotation& rhs) const -> Rotation {
    return *this + -rhs;
}

auto Rotation::operator-() const -> Rotation {
    return *this * -1;
}

auto Rotation::operator*(double scalar) const -> Rotation {
    return Rotation(theta * scalar);
}

auto Rotation::operator/(double scalar) const -> Rotation {
    return *this * (1.0 / scalar);
}

auto Rotation::rotateBy(const Rotation& rhs) const -> Rotation {
    return Rotation((cosine * rhs.cosine - sine * rhs.sine), 
                    (cosine * rhs.sine + sine * rhs.cosine));
}
