#pragma once

#include "nav/generation.h"
#include "world/collision_world.h"

#include <algorithm>

namespace navfast::exact {

inline constexpr float FloorSkin = 0.125f;

// The classic builder allows five units from the plane at the default slope tolerance.
inline float planeHeightTolerance(const GenerationParams& params) {
    return std::clamp(params.slopeTolerance * 2 * GenerationStepSize, FloorSkin, StepHeight * 0.5f);
}

enum class Fit { Blocked, Crouching, Standing };

struct Clearance {
    Fit fit;
    float lift = 0;
};

bool hullFits(const CollisionWorld& world, Vec3 floor, float height);
Clearance floorClearance(const CollisionWorld& world, Vec3 floor, Vec3 normal, float slopeLimit);

} // namespace navfast::exact
