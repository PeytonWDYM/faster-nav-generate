#pragma once

#include <filesystem>
#include <iosfwd>

namespace navfast {

// Read x y z nx ny nz crouch records and report hull clearance at their floor heights.
void checkClearance(const std::filesystem::path& job, std::istream& points, std::ostream& report);

} // namespace navfast
