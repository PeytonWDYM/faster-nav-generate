#include "nav/nav_area.h"

#include <algorithm>

namespace navfast {

Aabb NavArea::bounds() const {
    Aabb box{points[NW], points[SE]};
    for (const auto& point : points) {
        box.mins.z = std::min(box.mins.z, point.z);
        box.maxs.z = std::max(box.maxs.z, point.z);
    }
    return box;
}

float NavArea::getZ(float x, float y) const {
    const auto u = std::clamp((x - points[NW].x) / width(), 0.0f, 1.0f);
    const auto v = std::clamp((y - points[NW].y) / height(), 0.0f, 1.0f);
    const auto north = points[NW].z + u * (points[NE].z - points[NW].z);
    const auto south = points[SW].z + u * (points[SE].z - points[SW].z);
    return north + v * (south - north);
}

Vec3 NavArea::closestPoint(Vec3 pos) const {
    pos.x = std::clamp(pos.x, points[NW].x, points[SE].x);
    pos.y = std::clamp(pos.y, points[NW].y, points[SE].y);
    pos.z = getZ(pos.x, pos.y);
    return pos;
}

Vec3 NavArea::normal(bool alternate) const {
    const auto cross = alternate ? (points[SW] - points[SE]).cross(points[NE] - points[SE])
                                 : (points[NE] - points[NW]).cross(points[SW] - points[NW]);
    return cross * (1.0f / cross.length());
}

bool NavArea::overlaps(Vec3 pos, float tolerance) const {
    return pos.x + tolerance >= points[NW].x && pos.x - tolerance <= points[SE].x &&
           pos.y + tolerance >= points[NW].y && pos.y - tolerance <= points[SE].y;
}

bool NavArea::overlaps(const NavArea& other) const {
    return overlapAlong(other, North) > 0 && overlapAlong(other, East) > 0;
}

float NavArea::overlapAlong(const NavArea& other, Direction dir) const {
    if (alongX(dir))
        return std::min(points[SE].y, other.points[SE].y) - std::max(points[NW].y, other.points[NW].y);
    return std::min(points[SE].x, other.points[SE].x) - std::max(points[NW].x, other.points[NW].x);
}

Portal NavArea::portal(const NavArea& other, Direction dir) const {
    const auto edge = points[edgeCorners[dir][0]];
    Vec3 middle = edge;
    const auto& min = points[NW];
    const auto& max = points[SE];
    if (alongX(dir)) {
        middle.y = (std::clamp(other.points[NW].y, min.y, max.y) +
                    std::clamp(other.points[SE].y, min.y, max.y)) * 0.5f;
    } else {
        middle.x = (std::clamp(other.points[NW].x, min.x, max.x) +
                    std::clamp(other.points[SE].x, min.x, max.x)) * 0.5f;
    }
    middle.z = getZ(middle.x, middle.y);
    return {middle, std::max(0.0f, overlapAlong(other, dir) * 0.5f)};
}

} // namespace navfast
