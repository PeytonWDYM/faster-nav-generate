#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <span>

#include "world/world_stats.h"

namespace navfast::detail {

inline constexpr float traceEpsilon = 0.03125f;
inline constexpr Vec3 axes[]{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

inline Vec3 minVec(Vec3 a, Vec3 b) {
    return {std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z)};
}

inline Vec3 maxVec(Vec3 a, Vec3 b) {
    return {std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z)};
}

inline Vec3 absVec(Vec3 v) {
    return {std::abs(v.x), std::abs(v.y), std::abs(v.z)};
}

inline bool finite(Vec3 v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

inline bool contains(Aabb box, Vec3 p) {
    return p.x >= box.mins.x && p.x <= box.maxs.x &&
           p.y >= box.mins.y && p.y <= box.maxs.y &&
           p.z >= box.mins.z && p.z <= box.maxs.z;
}

inline Aabb unite(Aabb a, Aabb b) {
    return {minVec(a.mins, b.mins), maxVec(a.maxs, b.maxs)};
}

inline Aabb emptyBox() {
    constexpr auto inf = std::numeric_limits<float>::infinity();
    return {{inf, inf, inf}, {-inf, -inf, -inf}};
}

struct Transform {
    Vec3 origin;
    std::array<Vec3, 3> basis;

    Vec3 rotate(Vec3 v) const { return basis[0] * v.x + basis[1] * v.y + basis[2] * v.z; }
    Vec3 point(Vec3 v) const { return rotate(v) + origin; }
};

// Source's AngleMatrix uses pitch, yaw, roll and a forward/left/up basis.
Transform angleTransform(Vec3 origin, Vec3 angles);

struct Plane {
    Vec3 normal;
    float distance;
    std::uint32_t surfaceFlags = 0;
};

struct Triangle {
    std::array<Vec3, 3> points;
    Vec3 normal;
    std::uint32_t surfaceFlags;
    bool rayCollision, hullCollision;
};

struct Solid {
    Aabb bounds;
    std::uint32_t contents;
    std::uint32_t first;     // Plane pool offset, or triangle index when planeCount is zero.
    std::uint32_t planeCount;
    std::uint32_t faceCount; // Ray traces use only the leading face planes.
};

struct Geometry {
    std::vector<Solid> solids;
    std::vector<Plane> planes;
    std::vector<Triangle> triangles;
    WorldStats stats;

    void addConvex(Aabb bounds, std::uint32_t contents, std::span<const Plane> hull, std::size_t faces);
    void addDisplacement(std::array<Vec3, 3> points, std::uint32_t surfaceFlags,
                         bool rayCollision, bool hullCollision);
};

std::vector<FloorFace> floorFaces(const Geometry& geometry, std::uint32_t mask, float minNormalZ);

struct Sweep {
    Vec3 start, end, delta, extents;
    bool ray() const { return extents == Vec3{}; }
    bool stationary() const { return delta == Vec3{}; }
};

struct SweepHit {
    TraceResult result;
    // Prune against geometric contact. A glancing plane can back off much
    // farther along the ray than the BVH's spatial padding.
    float contact = 1;

    void consider(float time, float backedOff, Vec3 normal, std::uint32_t contents,
                  std::uint32_t surfaceFlags, bool displacement) {
        if (time > contact || (time == contact && backedOff >= result.fraction)) return;
        contact = time;
        result.fraction = backedOff;
        result.normal = normal;
        result.contents = contents;
        result.surfaceFlags = surfaceFlags;
        result.hitDisplacement = displacement;
    }
};

} // namespace navfast::detail
