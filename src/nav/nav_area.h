#pragma once

#include "nav/nav_node.h"

#include <vector>

namespace navfast {

struct Portal {
    Vec3 center;
    float halfWidth;
};

/// Source area geometry, with NW/SE extents and a height at every corner.
struct NavArea {
    std::array<Vec3, 4> points{};
    std::array<NodeId, 4> nodes{};
    std::uint32_t attributes = 0;
    std::array<std::vector<AreaId>, 4> outgoing{};
    std::array<std::vector<AreaId>, 4> incoming{};
    bool alive = true;

    float width() const { return points[SE].x - points[NW].x; }
    float height() const { return points[SE].y - points[NW].y; }
    bool hasNodes() const { return nodes[NW] != NodeId{}; }
    Vec3 center() const { return (points[NW] + points[SE]) * 0.5f; }
    Aabb bounds() const;
    float getZ(float x, float y) const;
    Vec3 closestPoint(Vec3 point) const;
    Vec3 normal(bool alternate = false) const;
    bool overlaps(Vec3 point, float tolerance = 0) const;
    bool overlaps(const NavArea& other) const;
    float overlapAlong(const NavArea& other, Direction dir) const;
    Portal portal(const NavArea& other, Direction dir) const;
};

} // namespace navfast
