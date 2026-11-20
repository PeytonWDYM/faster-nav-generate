#pragma once

#include "job/job.h"
#include "nav/nav_file.h"

#include <iosfwd>

namespace navfast {

void reportJob(std::ostream& stream, const JobStats& stats, bool json);
void reportNav(std::ostream& stream, const NavInspection& nav, bool json);

} // namespace navfast
