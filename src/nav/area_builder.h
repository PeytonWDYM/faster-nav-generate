#pragma once

#include "nav/nav_mesh.h"

namespace navfast {

class ThreadPool;

/// Greedily cover sampled cells with the SDK's descending rectangular area sizes.
void createNavAreasFromNodes(NavMesh& mesh, const CollisionWorld& world, const GenerationParams& params,
                             ThreadPool& pool, const Progress& progress);

/// Run generation-only area connection and refinement phases.
void refineGeneratedAreas(NavMesh& mesh, const CollisionWorld& world, const GenerationParams& params,
                          ThreadPool& pool, const Phase& phase);

} // namespace navfast
