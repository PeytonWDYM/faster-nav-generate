#include "world/geometry.h"

#include <numbers>
#include <stdexcept>

#include "world/source_flags.h"

namespace navfast::detail {

Transform angleTransform(Vec3 origin, Vec3 angles) {
    if (!finite(origin) || !finite(angles)) throw std::runtime_error("invalid collision model transform");
    angles = angles * (std::numbers::pi_v<float> / 180.0f);
    const auto sp = std::sin(angles.x), cp = std::cos(angles.x);
    const auto sy = std::sin(angles.y), cy = std::cos(angles.y);
    const auto sr = std::sin(angles.z), cr = std::cos(angles.z);
    return {origin, {Vec3{cp * cy, cp * sy, -sp},
                     Vec3{sp * sr * cy - cr * sy, sp * sr * sy + cr * cy, cp * sr},
                     Vec3{sp * cr * cy + sr * sy, sp * cr * sy - sr * cy, cp * cr}}};
}

void Geometry::addConvex(Aabb bounds, std::uint32_t contents, std::span<const Plane> hull,
                         std::size_t faces) {
    const auto first = static_cast<std::uint32_t>(planes.size());
    planes.insert(planes.end(), hull.begin(), hull.end());
    solids.push_back({bounds, contents, first, static_cast<std::uint32_t>(hull.size()),
                      static_cast<std::uint32_t>(faces)});
}

void Geometry::addDisplacement(std::array<Vec3, 3> points, std::uint32_t surfaceFlags,
                               bool rayCollision, bool hullCollision) {
    const auto normal = (points[1] - points[0]).cross(points[2] - points[0]);
    const auto length = normal.length();
    if (length == 0) return;
    const Aabb bounds{minVec(points[0], minVec(points[1], points[2])),
                      maxVec(points[0], maxVec(points[1], points[2]))};
    solids.push_back({bounds, source::CONTENTS_SOLID, static_cast<std::uint32_t>(triangles.size()), 0, 0});
    triangles.push_back({points, normal * (1.0f / length), surfaceFlags, rayCollision, hullCollision});
    ++stats.displacementTriangles;
}

} // namespace navfast::detail
