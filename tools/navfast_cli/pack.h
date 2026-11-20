#pragma once

#include <filesystem>

namespace navfast {

/// Pack a BSP, its static prop PHY files, and info_player_* seed origins for offline replay.
void pack(const std::filesystem::path& bspPath, const std::filesystem::path& phyDir,
          const std::filesystem::path& jobPath);

} // namespace navfast
