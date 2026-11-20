#pragma once

#include "nav/nav_node.h"

#include <span>

namespace navfast {

class ThreadPool;

/// Trace with Source's narrow, crouch-height generation hull.
inline TraceResult traceNav(const CollisionWorld& world, Vec3 start, Vec3 end) {
    return world.traceHull(start, end, NavTraceMins, NavTraceMaxs, GenerationTraceMask);
}

bool traceAdjacentNode(const CollisionWorld& world, const GenerationParams& params,
                       Vec3 start, Vec3 end, TraceResult& trace, float drop = DeathDrop);

/// Sample all viable seeds in deterministic waves, committing links on the calling thread.
NavNodes sampleWalkableSpace(const CollisionWorld& world, std::span<const Vec3> seeds,
                             const GenerationParams& params, ThreadPool& pool, const Progress& progress);

} // namespace navfast
