#include "Point.hpp"
#include "Math.hpp"
#include <cmath>

Point::Point(double x, double y) : x(x), y(y) {}

Point::Point(double mag, const Rotation& angle) : x(mag * angle.Cos()), y(mag * angle.Sin()) {}

double Point::X() const {
    return x;
}

double Point::Y() const {
    return y;
}

Point Point::operator+(const Point& rhs) const {
    return Point(x + rhs.x, y + rhs.y);
}

Point Point::operator-(const Point& rhs) const {
    return -rhs + *this;
}

Point Point::operator-() const {
    return *this * -1;
}

Point Point::operator*(double scalar) const {
    return Point(x * scalar, y * scalar);
}

Point Point::operator/(double scalar) const {
    return *this * (1.0 / scalar);
}

double Point::theta() const {
    return std::atan2(y, x);
}

double Point::mag() const {
    return std::hypot(x, y);
}

double Point::distTo(const Point& rhs) const {
    return Point(rhs.x - x, rhs.y - y).mag();
}

double Point::angleTo(const Point& rhs) const {
    return constrainAngle180(rhs.theta() - theta());
}

double Point::dot(const Point& rhs) const {
    return x * rhs.x + y * rhs.y;
}

Point Point::rotateBy(const Rotation& rhs) const {
    return {x * rhs.Cos() - y * rhs.Sin(), x * rhs.Sin() + y * rhs.Cos()};
}

double circumradius(const Point& left, const Point& mid, const Point& right) {
    const Point& A = left;
    const Point& B = mid;
    const Point& C = right;

    const double a = B.distTo(C);
    const double b = A.distTo(C);
    const double c = A.distTo(B);
    const auto a2 = a * a, b2 = b * b, c2 = c * c;

    const Point pa = A * (a2 * (b2 + c2 - a2) / ((b + c) * (b + c) - a2) / (a2 - (b - c) * (b - c)));
    const Point pb = B * (b2 * (a2 + c2 - b2) / ((a + c) * (a + c) - b2) / (b2 - (a - c) * (a - c)));
    const Point pc = C * (c2 * (a2 + b2 - c2) / ((a + b) * (a + b) - c2) / (c2 - (a - b) * (a - b)));

    const Point center = pa + pb + pc;

    const double radius = center.distTo(A);

    return radius;
}

std::optional<double> circleLineIntersection(const Point& start, const Point& end, const Point& point, double radius){
    const Point d = end - start;
    const Point f = start - point;

    const auto a = d.dot(d);
    const auto b = 2 * (f.dot(d));
    const auto c = f.dot(f) - radius * radius;
    const auto discriminant = b * b - 4 * a * c;

    if (discriminant >= 0) {
        const auto dis = std::sqrt(discriminant);
        const double t1 = ((-1 * b - dis) / (2 * a));
        const double t2 = ((-1 * b + dis) / (2 * a));

        if (t2 >= 0 && t2 <= 1) {
            return t2;
        }
        
        if (t1 >= 0 && t1 <= 1) {
            return t1;
        }
    }

    return std::nullopt;
}
