#pragma once

#include <cstdint>
#include <iosfwd>
#include <string_view>

#include "world/world_stats.h"

namespace navfast::probe {

struct DropStats {
    unsigned traces = 0;
    unsigned floorHits = 0;
    unsigned startSolid = 0;
    unsigned displacementHits = 0;
    unsigned flatgrassTerrainHits = 0;
    float lowestFloor = 0;
    float highestFloor = 0;
};

struct FloorCheck {
    TraceResult trace;
    float expectedZ = 0;
};

struct RayStats {
    unsigned pairs = 0;
    unsigned blockedBothWays = 0;
    unsigned asymmetric = 0;
    unsigned startSolid = 0;
};

struct Throughput {
    unsigned traces = 0;
    double single = 0;
    double allCores = 0;
    std::uint64_t checksum = 0;
    bool deterministic = false;
};

struct Report {
    Aabb bounds;
    WorldStats world;
    double buildMs = 0;
    DropStats drops;
    FloorCheck flatgrassSpawn;
    RayStats rays;
    unsigned workers = 0;
    Throughput hullRate, rayRate;
    bool passed = false;
};

// Run deterministic geometry checks and benchmark the same inputs on one and all logical cores.
void run(const CollisionWorld& world, std::string_view map, Report& report);
void print(const Report& report, std::ostream& out);
void writeJson(const Report& report, std::ostream& out);

} // namespace navfast::probe
