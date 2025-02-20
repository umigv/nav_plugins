#include "Rotation.hpp"
#include "Math.hpp"
#include <cmath>

Rotation::Rotation(double theta) : theta(constrainAngle180(theta)), cosine(std::cos(theta)), sine(std::sin(theta)){}

Rotation::Rotation(double iX, double iY) {
    const auto magnitude = hypot(iX, iY);
    if(magnitude > 1e-6){
        sine = (iY / magnitude);
        cosine = (iX / magnitude);
    } 
    else{
        sine = 0.0;
        cosine = 1.0;
    }
    theta = std::atan2(sine, cosine);
}

double Rotation::Theta() const {
    return theta;
}

double Rotation::Sin() const {
    return sine;
}

double Rotation::Cos() const {
    return cosine;
}

double Rotation::Tan() const {
    return sine / cosine;
}

Rotation Rotation::operator+(const Rotation& rhs) const {
    return rotateBy(rhs);
}

Rotation Rotation::operator-(const Rotation& rhs) const {
    return *this + -rhs;
}

Rotation Rotation::operator-() const {
    return *this * -1;
}

Rotation Rotation::operator*(double scalar) const {
    return Rotation(theta * scalar);
}

Rotation Rotation::operator/(double scalar) const {
    return *this * (1.0 / scalar);
}

Rotation Rotation::rotateBy(const Rotation& rhs) const {
    return Rotation((cosine * rhs.cosine - sine * rhs.sine), (cosine * rhs.sine + sine * rhs.cosine));
}
