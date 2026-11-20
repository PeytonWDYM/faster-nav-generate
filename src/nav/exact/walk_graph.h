#pragma once

#include "nav/exact/heightfield.h"

namespace navfast::exact {

std::size_t reachFloors(Heightfield& field, const CollisionWorld& world, std::span<const Vec3> seeds,
                        const GenerationParams& params, ThreadPool& pool, const Progress& progress);
void markStairTreads(Heightfield& field, ThreadPool& pool);

} // namespace navfast::exact
