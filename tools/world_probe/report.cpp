#include "probe.h"

#include <iomanip>
#include <ostream>

namespace navfast::probe {

void print(const Report& r, std::ostream& out) {
    out << r.world.brushes << " brushes, " << r.world.displacementTriangles << " displacement triangles, "
        << r.world.propsLoaded << " props loaded, " << r.world.propsSkipped << " skipped, "
        << r.world.propHulls << " prop hulls, " << r.world.bvhNodes << " BVH nodes\n";
    out << std::fixed << std::setprecision(2) << "Build " << r.buildMs << " ms\n"
        << r.drops.floorHits << '/' << r.drops.traces << " hull drops found floors at z="
        << r.drops.lowestFloor << " to " << r.drops.highestFloor << " (" << r.drops.displacementHits
        << " on displacements, " << r.drops.startSolid << " initial overlaps)\n";
    if (r.flatgrassSpawn.expectedZ != 0) {
        out << "Flatgrass spawn deck z=" << r.flatgrassSpawn.trace.endpos.z
            << " (BSP plane " << r.flatgrassSpawn.expectedZ << "), "
            << r.drops.flatgrassTerrainHits << " drops near terrain z=-12800\n";
    }
    out << r.rays.pairs << " open-point ray pairs, " << r.rays.blockedBothWays
        << " blocked both ways, " << r.rays.startSolid << " initial overlaps, "
        << r.rays.asymmetric << " asymmetric\n";
    out << std::setprecision(0) << "Hull traces/s " << r.hullRate.single << " single, "
        << r.hullRate.allCores << " on " << r.workers << " threads\n"
        << "Line traces/s " << r.rayRate.single << " single, " << r.rayRate.allCores
        << " on " << r.workers << " threads\n" << (r.passed ? "PASS\n" : "FAIL\n");
}

void writeJson(const Report& r, std::ostream& out) {
    const auto vec = [&](Vec3 p) { out << '[' << p.x << ", " << p.y << ", " << p.z << ']'; };
    const auto rate = [&](const Throughput& t) {
        out << "{\"traces\": " << t.traces << ", \"single\": " << t.single
            << ", \"allCores\": " << t.allCores << ", \"checksum\": " << t.checksum
            << ", \"deterministic\": " << t.deterministic << '}';
    };
    out << std::boolalpha << std::setprecision(9)
        << "{\n  \"passed\": " << r.passed
        << ",\n  \"bounds\": {\"mins\": ";
    vec(r.bounds.mins);
    out << ", \"maxs\": ";
    vec(r.bounds.maxs);
    out << "},\n  \"world\": {\"brushes\": " << r.world.brushes
        << ", \"displacementTriangles\": " << r.world.displacementTriangles
        << ", \"propsLoaded\": " << r.world.propsLoaded << ", \"propsSkipped\": " << r.world.propsSkipped
        << ", \"propHulls\": " << r.world.propHulls << ", \"bvhNodes\": " << r.world.bvhNodes
        << "},\n  \"buildMs\": " << r.buildMs
        << ",\n  \"drops\": {\"traces\": " << r.drops.traces << ", \"floorHits\": " << r.drops.floorHits
        << ", \"startSolid\": " << r.drops.startSolid << ", \"displacementHits\": " << r.drops.displacementHits
        << ", \"lowestFloor\": " << r.drops.lowestFloor << ", \"highestFloor\": " << r.drops.highestFloor << '}';
    if (r.flatgrassSpawn.expectedZ != 0) {
        out << ",\n  \"flatgrass\": {\"expectedSpawnDeckZ\": " << r.flatgrassSpawn.expectedZ
            << ", \"spawnDeckZ\": " << r.flatgrassSpawn.trace.endpos.z
            << ", \"spawnDeckStartSolid\": " << r.flatgrassSpawn.trace.startSolid
            << ", \"dropsNearZMinus12800\": " << r.drops.flatgrassTerrainHits << '}';
    }
    out << ",\n  \"rays\": {\"pairs\": " << r.rays.pairs << ", \"blockedBothWays\": " << r.rays.blockedBothWays
        << ", \"startSolid\": " << r.rays.startSolid << ", \"asymmetric\": " << r.rays.asymmetric
        << "},\n  \"workers\": " << r.workers << ",\n  \"hullTracesPerSecond\": ";
    rate(r.hullRate);
    out << ",\n  \"lineTracesPerSecond\": ";
    rate(r.rayRate);
    out << "\n}\n";
}

} // namespace navfast::probe
