#include "nav/exact/clearance.h"

#include <algorithm>

namespace navfast::exact {
namespace {

constexpr Vec3 hullMins{-HalfHumanWidth, -HalfHumanWidth, 0};
constexpr Vec3 hullMaxs{HalfHumanWidth, HalfHumanWidth, 0};

} // namespace

bool hullFits(const CollisionWorld& world, Vec3 floor, float height) {
    floor.z += FloorSkin;
    auto maxs = hullMaxs;
    maxs.z = height;
    return !world.traceHull(floor, floor, hullMins, maxs, GenerationTraceMask).startSolid;
}

Clearance floorClearance(const CollisionWorld& world, Vec3 floor, Vec3 normal, float slopeLimit) {
    if (hullFits(world, floor, HumanHeight)) return {Fit::Standing};
    if (hullFits(world, floor, HumanCrouchHeight)) return {Fit::Crouching};

    // A wide hull touches the high side of a slope or tread before its center does.
    // Allow support up to one step (or the rise of this plane under the hull).
    const auto rise = HalfHumanWidth * (std::abs(normal.x) + std::abs(normal.y)) / normal.z;
    const Vec3 up{0, 0, std::max(StepHeight, rise) + FloorSkin};
    const auto footing = world.traceHull(floor + up, floor, hullMins,
                                        hullMaxs + Vec3{0, 0, FloorSkin}, GenerationTraceMask);
    if (footing.startSolid || footing.fraction == 1 || footing.normal.z < slopeLimit ||
        (footing.surfaceFlags & (source::SURF_SKY | source::SURF_SKY2D))) return {Fit::Blocked};
    const auto lift = footing.endpos.z - floor.z;
    if (hullFits(world, footing.endpos, HumanHeight)) return {Fit::Standing, lift};
    if (hullFits(world, footing.endpos, HumanCrouchHeight)) return {Fit::Crouching, lift};
    return {Fit::Blocked};
}

} // namespace navfast::exact
