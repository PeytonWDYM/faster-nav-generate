#include "world/collision_world.h"
#include "world/geometry.h"
#include "world/source_flags.h"

namespace navfast::detail {
namespace {

bool walkable(Vec3 normal, std::uint32_t flags, float minNormalZ) {
    return normal.z > 0 && normal.z >= minNormalZ &&
           !(flags & (source::SURF_SKY | source::SURF_SKY2D));
}

// Intersect the face plane with the true brush half-spaces. Bevels only serve hull sweeps.
std::vector<Vec3> polygon(const Plane& face, Aabb bounds, std::span<const Plane> planes) {
    const auto at = [&](float x, float y) {
        return Vec3{x, y, (face.distance - face.normal.x * x - face.normal.y * y) / face.normal.z};
    };
    std::vector<Vec3> points{at(bounds.mins.x, bounds.mins.y), at(bounds.maxs.x, bounds.mins.y),
                             at(bounds.maxs.x, bounds.maxs.y), at(bounds.mins.x, bounds.maxs.y)};
    std::vector<Vec3> clipped;
    clipped.reserve(planes.size());
    for (const auto& plane : planes) {
        if (&plane == &face) continue;
        const auto distance = [&](Vec3 p) { return p.dot(plane.normal) - plane.distance; };
        auto a = points.back();
        auto da = distance(a);
        for (const auto b : points) {
            const auto db = distance(b);
            constexpr float epsilon = 0.002f;
            if ((da > epsilon) != (db > epsilon)) {
                const auto t = std::clamp(da / (da - db), 0.0f, 1.0f);
                clipped.push_back(a + (b - a) * t);
            }
            if (db <= epsilon) clipped.push_back(b);
            a = b;
            da = db;
        }
        points.swap(clipped);
        clipped.clear();
        if (points.size() < 3) return {};
    }
    float area = 0;
    for (std::size_t i = 2; i < points.size(); ++i)
        area += (points[i - 1] - points[0]).cross(points[i] - points[0]).z;
    return std::abs(area) > 0.001f ? points : std::vector<Vec3>{};
}

} // namespace

std::vector<FloorFace> floorFaces(const Geometry& geometry, std::uint32_t mask, float minNormalZ) {
    std::vector<FloorFace> faces;
    for (const auto& solid : geometry.solids) {
        if (!(solid.contents & mask)) continue;
        if (solid.planeCount == 0) {
            const auto& tri = geometry.triangles[solid.first];
            if (tri.hullCollision && walkable(tri.normal, tri.surfaceFlags, minNormalZ))
                faces.push_back({{tri.points.begin(), tri.points.end()},
                                 {tri.normal, tri.normal.dot(tri.points[0])}, true});
            continue;
        }
        const auto planes = std::span(geometry.planes).subspan(solid.first, solid.faceCount);
        for (const auto& plane : planes) {
            if (!walkable(plane.normal, plane.surfaceFlags, minNormalZ)) continue;
            auto points = polygon(plane, solid.bounds, planes);
            if (!points.empty())
                faces.push_back({std::move(points), {plane.normal, plane.distance}, false});
        }
    }
    return faces;
}

} // namespace navfast::detail
