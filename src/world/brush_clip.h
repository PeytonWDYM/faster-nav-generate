#pragma once

#include "world/geometry.h"

namespace navfast::detail {

// Source brush plane clipping, with the plane support point of a swept box.
void traceConvex(const Sweep& sweep, const Solid& solid, const Plane* planes, SweepHit& trace);

inline bool pointInConvex(Vec3 point, std::span<const Plane> faces) {
    for (const auto& plane : faces) {
        if (plane.normal.dot(point) > plane.distance) return false;
    }
    return true;
}

} // namespace navfast::detail
