#pragma once

#include "nav/exact/rectangles.h"

namespace navfast::exact {

std::size_t mergeStairRuns(NavMesh& mesh, Heightfield& field, std::span<const Rectangle> rectangles,
                           std::span<const NavArea> areas, const CollisionWorld& world,
                           const GenerationParams& params, ThreadPool& pool);

} // namespace navfast::exact
