#include "DiscretePath.hpp"
#include <cmath>
#include <algorithm>

DiscretePath::DiscretePath(const std::initializer_list<Point>& waypoint){
    path.push_back(*waypoint.begin());

    for(auto it = waypoint.begin() + 1; it != waypoint.end(); ++it) {
        insert(*it);
    }
}

DiscretePath::DiscretePath(const std::vector<Point>& waypoint){
    path.push_back(waypoint.front());

    for(auto it = waypoint.begin() + 1; it != waypoint.end(); ++it) {
        insert(*it);
    }
}

auto DiscretePath::begin() -> iterator {
    return path.begin();
}

auto DiscretePath::begin() const -> const_iterator {
    return path.begin();
}

auto DiscretePath::end() -> iterator {
    return path.end();
}

auto DiscretePath::end() const -> const_iterator {
    return path.end();
}

auto DiscretePath::rbegin() -> reverse_iterator {
    return path.rbegin();
}

auto DiscretePath::rbegin() const -> const_reverse_iterator {
    return path.rbegin();
}

auto DiscretePath::rend() -> reverse_iterator {
    return path.rend();
}

auto DiscretePath::rend() const -> const_reverse_iterator {
    return path.rend();
}

auto DiscretePath::operator[](std::size_t index) -> Point& {
    return path[index];
}

auto DiscretePath::operator[](std::size_t index) const -> const Point& {
    return path[index];
}

auto DiscretePath::front() -> Point& {
    return path.front();
}

auto DiscretePath::front() const -> const Point& {
    return path.front();
}

auto DiscretePath::back() -> Point& {
    return path.back();
}

auto DiscretePath::back() const -> const Point& {
    return path.back();
}

auto DiscretePath::size() const -> std::size_t {
    return path.size();
}

auto DiscretePath::getCurvature(std::size_t index) const -> double {
    if (index == 0 || index == path.size() - 1) {
        return 0;
    }

    const double radius = circumradius(path[index - 1], path[index], path[index + 1]);

    if (std::isnan(radius) || radius == 0) {
        return 0;
    }

    return 1 / radius;
}

auto DiscretePath::insert(const Point& point) -> void {
    const Point start = path.back();
    const std::size_t count = ceil(start.distTo(point) / RESOLUTION);

    for(std::size_t i = 1; i <= count; ++i) {
        path.push_back(lerp(start, point, static_cast<double>(i) / count));
    }
}

auto closestPoint(DiscretePath::const_iterator begin,
                  DiscretePath::const_iterator end,
                  const Point& point) -> DiscretePath::const_iterator {
    auto comparison = [point](const Point& a, const Point& b) { 
        return a.distTo(point) < b.distTo(point); 
    };

    return std::min_element(begin, end, comparison);
}
