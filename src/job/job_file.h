#pragma once

#include "job/job.h"

#include <filesystem>

namespace navfast {

/// Store or restore versioned, portable job inputs, including original BSP/PHY bytes.
void writeJobFile(const std::filesystem::path& path, const JobInputs& inputs);
OwnedJobInputs readJobFile(const std::filesystem::path& path);

} // namespace navfast
