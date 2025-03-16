#pragma once

class Rotation {
    public:
    constexpr Rotation() = default;

    Rotation(double theta);

    Rotation(double x, double y);

    auto Theta() const -> double;

    auto Sin() const -> double;

    auto Cos() const -> double;

    auto Tan() const -> double;

    auto operator+(const Rotation& rhs) const -> Rotation;

    auto operator-(const Rotation& rhs) const -> Rotation;

    auto operator-() const -> Rotation;

    auto operator*(double scalar) const -> Rotation;

    auto operator/(double scalar) const -> Rotation;

    auto rotateBy(const Rotation& rhs) const -> Rotation;

    private:
    double theta{0.0};
    double cosine{1.0};
    double sine{0.0};
};
