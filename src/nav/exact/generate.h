#pragma once

#include "nav/nav_mesh.h"

namespace navfast {

class ThreadPool;

std::size_t generateExactNav(NavMesh& mesh, const CollisionWorld& world, std::span<const Vec3> seeds,
                             const GenerationParams& params, ThreadPool& pool,
                             const Phase& phase, const Progress& progress);

} // namespace navfast
