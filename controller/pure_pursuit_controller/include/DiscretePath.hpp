#pragma once
#include "Point.hpp"
#include <vector>

class DiscretePath {
    public:
    using iterator = typename std::vector<Point>::iterator;
    using const_iterator = typename std::vector<Point>::const_iterator;
    using reverse_iterator = typename std::vector<Point>::reverse_iterator;
    using const_reverse_iterator = typename std::vector<Point>::const_reverse_iterator;

    DiscretePath() = default;
    explicit DiscretePath(const std::initializer_list<Point>& waypoint);
    explicit DiscretePath(const std::vector<Point>& waypoint);

    auto begin() -> iterator;
    auto begin() const -> const_iterator;
    auto end() -> iterator;
    auto end() const -> const_iterator;

    auto rbegin() -> reverse_iterator;
    auto rbegin() const -> const_reverse_iterator;
    auto rend() -> reverse_iterator;
    auto rend() const -> const_reverse_iterator;

    auto operator[](std::size_t index) -> Point&;
    auto operator[](std::size_t index) const -> const Point&;
    auto front() -> Point&;
    auto front() const -> const Point&;
    auto back() -> Point&;
    auto back() const -> const Point&;
    auto getCurvature(std::size_t index) const -> double;
    auto size() const -> std::size_t;

    private:
    auto insert(const Point& point) -> void;

    static constexpr auto RESOLUTION = 0.05;
    std::vector<Point> path;
};

auto closestPoint(DiscretePath::const_iterator begin,
                  DiscretePath::const_iterator end, 
                  const Point& point) 
    -> DiscretePath::const_iterator;
