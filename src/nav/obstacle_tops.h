#pragma once

#include "nav/nav_mesh.h"

namespace navfast {

class ThreadPool;

/// Raise and connect small traversal areas for fences and railings.
void handleObstacleTopAreas(NavMesh& mesh, const GenerationParams& params, ThreadPool& pool);

} // namespace navfast
