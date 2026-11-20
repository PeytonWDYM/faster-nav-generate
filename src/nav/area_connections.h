#pragma once

#include "nav/nav_mesh.h"

namespace navfast {

class ThreadPool;

void connectGeneratedAreas(NavMesh& mesh, const CollisionWorld& world, const GenerationParams& params,
                           ThreadPool& pool);
void stitchAndRemoveJumpAreas(NavMesh& mesh, const GenerationParams& params, ThreadPool& pool);
void fixUpGeneratedAreas(NavMesh& mesh, const CollisionWorld& world, const GenerationParams& params,
                         ThreadPool& pool);

} // namespace navfast
