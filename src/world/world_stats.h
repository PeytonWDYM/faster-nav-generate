#pragma once

#include "world/collision_world.h"

namespace navfast {

struct WorldStats {
    std::size_t brushes = 0;
    std::size_t displacementTriangles = 0;
    std::size_t propsLoaded = 0;
    std::size_t propsSkipped = 0;
    std::size_t propHulls = 0;
    std::size_t bvhNodes = 0;
};

// Build the snapshot and report its geometry counts to the offline probe.
std::unique_ptr<CollisionWorld> buildCollisionWorldWithStats(const WorldInputs& inputs, WorldStats& stats);

} // namespace navfast
