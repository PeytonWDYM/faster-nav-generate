#include "clearance_check.h"

#include "job/job_file.h"
#include "nav/exact/clearance.h"

#include <istream>
#include <ostream>

namespace navfast {

void checkClearance(const std::filesystem::path& job, std::istream& points, std::ostream& report) {
    auto owned = readJobFile(job);
    auto& inputs = owned.inputs;
    inputs.world.bsp = owned.bsp;
    const auto world = buildCollisionWorld(inputs.world);
    std::size_t samples = 0, blocked = 0, blockedAtFloor = 0;
    for (points >> std::ws; points.peek() != std::char_traits<char>::eof(); points >> std::ws) {
        Vec3 p, normal;
        bool crouch;
        if (!(points >> p.x >> p.y >> p.z >> normal.x >> normal.y >> normal.z >> crouch) || normal.z <= 0)
            throw std::runtime_error("Invalid clearance point");
        ++samples;
        if (exact::hullFits(*world, p, crouch ? HumanCrouchHeight : HumanHeight)) continue;
        ++blockedAtFloor;
        const auto fit = exact::floorClearance(*world, p, normal, inputs.params.slopeLimit).fit;
        if (fit == exact::Fit::Blocked || (!crouch && fit == exact::Fit::Crouching)) ++blocked;
    }
    report << "{\"samples\":" << samples << ",\"blockedAtFloor\":" << blockedAtFloor
           << ",\"blockedAfterStep\":" << blocked << "}\n";
}

} // namespace navfast
