#include "Point.hpp"
#include "Math.hpp"
#include <cmath>

Point::Point(double x, double y) : x(x), y(y) {}

Point::Point(double mag, const Rotation& angle) 
    : x(mag * angle.Cos()), y(mag * angle.Sin()) {}

auto Point::X() const -> double {
    return x;
}

auto Point::Y() const -> double {
    return y;
}

auto Point::operator+(const Point& rhs) const -> Point {
    return Point(x + rhs.x, y + rhs.y);
}

auto Point::operator-(const Point& rhs) const -> Point {
    return -rhs + *this;
}

auto Point::operator-() const -> Point {
    return *this * -1;
}

auto Point::operator*(double scalar) const -> Point {
    return Point(x * scalar, y * scalar);
}

auto Point::operator/(double scalar) const -> Point {
    return *this * (1.0 / scalar);
}

auto Point::theta() const -> double {
    return std::atan2(y, x);
}

auto Point::mag() const -> double {
    return std::hypot(x, y);
}

auto Point::distTo(const Point& rhs) const -> double {
    return Point(rhs.x - x, rhs.y - y).mag();
}

auto Point::angleTo(const Point& rhs) const -> double {
    return constrainAngle180(rhs.theta() - theta());
}

auto Point::dot(const Point& rhs) const -> double {
    return x * rhs.x + y * rhs.y;
}

auto Point::rotateBy(const Rotation& rhs) const -> Point {
    return {x * rhs.Cos() - y * rhs.Sin(), x * rhs.Sin() + y * rhs.Cos()};
}

auto lerp(const Point& start, const Point& end, double t) -> Point {
    return start + (end - start) * t;
}

auto circumradius(const Point& left, const Point& mid, const Point& right) -> double {
    const Point& A = left;
    const Point& B = mid;
    const Point& C = right;

    const double a = B.distTo(C), b = A.distTo(C), c = A.distTo(B);
    const auto a2 = a * a, b2 = b * b, c2 = c * c;

    const Point pa = A * (a2 * (b2 + c2 - a2) / ((b + c) * (b + c) - a2) / (a2 - (b - c) * (b - c)));
    const Point pb = B * (b2 * (a2 + c2 - b2) / ((a + c) * (a + c) - b2) / (b2 - (a - c) * (a - c)));
    const Point pc = C * (c2 * (a2 + b2 - c2) / ((a + b) * (a + b) - c2) / (c2 - (a - b) * (a - b)));

    const Point center = pa + pb + pc;

    const double radius = center.distTo(A);

    return radius;
}

auto circleLineIntersection(const Point& start, 
                            const Point& end, 
                            const Point& point,
                            double radius) -> std::optional<double> {
    const Point d = end - start;
    const Point f = start - point;

    const auto a = d.dot(d);
    const auto b = 2 * (f.dot(d));
    const auto c = f.dot(f) - radius * radius;
    const auto roots = quadraticFormula(a, b, c);

    if(roots){
        const auto [t1, t2] = *roots;

        if (t2 >= 0 && t2 <= 1) {
            return t2;
        }
        
        if (t1 >= 0 && t1 <= 1) {
            return t1;
        }
    }

    return std::nullopt;
}
