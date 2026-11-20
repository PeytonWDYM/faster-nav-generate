#include "report.h"

#include <iomanip>
#include <ostream>

namespace navfast {
namespace {

void vector(std::ostream& stream, Vec3 v) {
    stream << '[' << v.x << ", " << v.y << ", " << v.z << ']';
}

} // namespace

void reportJob(std::ostream& stream, const JobStats& stats, bool json) {
    stream << std::fixed << std::setprecision(6);
    if (json) {
        stream << "{\n  \"threads\": " << stats.threads << ",\n  \"nodes\": " << stats.nodes
               << ",\n  \"areas\": " << stats.areas << ",\n  \"connections\": " << stats.connections
               << ",\n  \"seconds\": " << stats.seconds << ",\n  \"phaseSeconds\": {\n";
        for (std::size_t i = 0; i < stats.phases.size(); ++i) {
            const auto& phase = stats.phases[i];
            if (i) stream << ",\n";
            stream << "    \"" << phase.name << "\": " << phase.seconds;
        }
        stream << "\n  }\n}\n";
        return;
    }
    stream << stats.nodes << " nodes, " << stats.areas << " areas, " << stats.connections << " connections in "
           << stats.seconds << " s with " << stats.threads << " threads\n";
    for (const auto& phase : stats.phases) stream << phase.name << "  " << phase.seconds << " s\n";
}

void reportNav(std::ostream& stream, const NavInspection& nav, bool json) {
    stream << std::setprecision(8);
    if (json) {
        stream << "{\n  \"version\": " << nav.version << ",\n  \"subVersion\": " << nav.subVersion
               << ",\n  \"bspSize\": " << nav.bspSize << ",\n  \"analyzed\": " << std::boolalpha << nav.analyzed
               << ",\n  \"areas\": " << nav.areas << ",\n  \"connections\": " << nav.connections
               << ",\n  \"ladders\": " << nav.ladders << ",\n  \"bounds\": {\n    \"mins\": ";
        vector(stream, nav.bounds.mins);
        stream << ",\n    \"maxs\": ";
        vector(stream, nav.bounds.maxs);
        stream << "\n  }\n}\n";
        return;
    }
    stream << "Version " << nav.version << '.' << nav.subVersion << ", BSP " << nav.bspSize << " bytes, "
           << nav.areas << " areas, " << nav.connections << " connections, " << nav.ladders << " ladders\n";
    stream << "Bounds ";
    vector(stream, nav.bounds.mins);
    stream << " to ";
    vector(stream, nav.bounds.maxs);
    stream << '\n';
}

} // namespace navfast
