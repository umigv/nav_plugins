#include "DiscretePath.hpp"
#include <cmath>
#include <algorithm>

DiscretePath::DiscretePath(const std::initializer_list<Point>& waypoint) : path(waypoint) {}

DiscretePath::DiscretePath(const std::vector<Point>& waypoint) : path(waypoint) {}

DiscretePath::iterator DiscretePath::begin() {
    return path.begin();
}

DiscretePath::const_iterator DiscretePath::begin() const {
    return path.begin();
}

DiscretePath::iterator DiscretePath::end() {
    return path.end();
}

DiscretePath::const_iterator DiscretePath::end() const {
    return path.end();
}

DiscretePath::reverse_iterator DiscretePath::rbegin() {
    return path.rbegin();
}

DiscretePath::const_reverse_iterator DiscretePath::rbegin() const {
    return path.rbegin();
}

DiscretePath::reverse_iterator DiscretePath::rend() {
    return path.rend();
}

DiscretePath::const_reverse_iterator DiscretePath::rend() const {
    return path.rend();
}

Point& DiscretePath::operator[](std::size_t index) {
    return path[index];
}

const Point& DiscretePath::operator[](std::size_t index) const {
    return path[index];
}

Point& DiscretePath::front() {
    return path.front();
}

const Point& DiscretePath::front() const {
    return path.front();
}

Point& DiscretePath::back() {
    return path.back();
}

const Point& DiscretePath::back() const {
    return path.back();
}

std::size_t DiscretePath::size() const {
    return path.size();
}

double DiscretePath::getCurvature(std::size_t index) const {
    if (index == 0 || index == path.size() - 1) {
        return 0;
    }

    const double radius = circumradius(path[index - 1], path[index], path[index + 1]);

    if (std::isnan(radius) || radius == 0) {
        return 0;
    }

    return 1 / radius;
}

DiscretePath::const_iterator closestPoint(DiscretePath::const_iterator begin,
                                          DiscretePath::const_iterator end,
                                          const Point& point) {
    auto comparison = [point](const Point& a, const Point& b) { 
        return a.distTo(point) < b.distTo(point); 
    };

    return std::min_element(begin, end, comparison);
}
