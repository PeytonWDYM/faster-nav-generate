#pragma once

#include "nav/nav_mesh.h"

#include <filesystem>

namespace navfast {

struct NavInspection {
    std::uint32_t version = 0;
    std::uint32_t subVersion = 0;
    std::uint32_t bspSize = 0;
    bool analyzed = false;
    std::uint32_t areas = 0;
    std::uint64_t connections = 0;
    std::uint32_t ladders = 0;
    Aabb bounds{};
};

/// Encode a GMod version-16 mesh and keep any replaced file as .bak.
void writeNav(const std::filesystem::path& path, const NavMesh& mesh, std::uint32_t bspSize);

/// Read area extents and directed connection counts, including analyzed version-16 files.
NavInspection inspectNav(const std::filesystem::path& path);

} // namespace navfast
