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
    DiscretePath(const std::initializer_list<Point>& waypoint);
    DiscretePath(const std::vector<Point>& waypoint);

    iterator begin();
    const_iterator begin() const;
    iterator end();
    const_iterator end() const;

    reverse_iterator rbegin();
    const_reverse_iterator rbegin() const;
    reverse_iterator rend();
    const_reverse_iterator rend() const;

    Point& operator[](std::size_t index);
    const Point& operator[](std::size_t index) const;
    Point& front();
    const Point& front() const;
    Point& back();
    const Point& back() const;
    double getCurvature(std::size_t index) const;
    std::size_t size() const;

    private:
    std::vector<Point> path;
};

DiscretePath::const_iterator closestPoint(DiscretePath::const_iterator begin,
                                          DiscretePath::const_iterator end, 
                                          const Point& point);
