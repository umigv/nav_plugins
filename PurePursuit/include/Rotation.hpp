#pragma once

class Rotation {
    public:
    constexpr Rotation() = default;

    Rotation(double theta);

    Rotation(double x, double y);

    double Theta() const;

    double Sin() const;

    double Cos() const;

    double Tan() const;

    Rotation operator+(const Rotation& rhs) const;

    Rotation operator-(const Rotation& rhs) const;

    Rotation operator-() const;

    Rotation operator*(double scalar) const;

    Rotation operator/(double scalar) const;

    bool operator==(const Rotation& rhs) const;

    bool operator!=(const Rotation& rhs) const;

    Rotation rotateBy(const Rotation& rhs) const;

    private:
    double theta{0.0};
    double cosine{1.0};
    double sine{0.0};
};
