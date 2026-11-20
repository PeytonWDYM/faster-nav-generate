#pragma once

#include "world/geometry.h"

namespace navfast::detail {

// Add the box axes and edge-cross-box axes to the hull's face planes.
void appendBoxBevels(std::vector<Plane>& planes, std::span<const Vec3> points,
                     std::span<const Vec3> edges);

// Exact swept AABB/triangle intersection, with Source's displacement hit offset.
void traceDisplacement(const Sweep& sweep, const Triangle& tri, SweepHit& trace);

} // namespace navfast::detail
