#include "probe.h"

#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <random>
#include <stdexcept>
#include <thread>

#include "world/source_flags.h"

namespace navfast::probe {
namespace {

constexpr auto mask = source::MASK_NPCSOLID_BRUSHONLY;
constexpr Vec3 hullMins{-16, -16, 0}, hullMaxs{16, 16, 72};
constexpr unsigned gridWidth = 64, rayPairs = 50000, benchTraces = 1u << 20;

struct Segment { Vec3 start, end; };

float randomUnit(std::mt19937& random) {
    return static_cast<float>(random() >> 8) * (1.0f / 16777216.0f);
}

std::vector<Segment> gridDrops(Aabb bounds) {
    const auto span = bounds.maxs - bounds.mins;
    std::vector<Segment> drops;
    drops.reserve(gridWidth * gridWidth);
    for (unsigned y = 0; y < gridWidth; ++y) {
        for (unsigned x = 0; x < gridWidth; ++x) {
            const auto px = bounds.mins.x + span.x * (static_cast<float>(x) + 0.5f) / gridWidth;
            const auto py = bounds.mins.y + span.y * (static_cast<float>(y) + 0.5f) / gridWidth;
            drops.push_back({{px, py, bounds.mins.z + span.z * 0.75f}, {px, py, bounds.mins.z}});
        }
    }
    return drops;
}

TraceResult drop(const CollisionWorld& world, const Segment& line) {
    return world.traceHull(line.start, line.end, hullMins, hullMaxs, mask);
}

void checkDrops(const CollisionWorld& world, std::span<const Segment> drops, DropStats& stats) {
    for (const auto& segment : drops) {
        ++stats.traces;
        const auto hit = drop(world, segment);
        if (hit.startSolid) {
            ++stats.startSolid;
            continue;
        }
        if (!hit.didHit() || hit.normal.z < 0.7f ||
            (hit.surfaceFlags & (source::SURF_SKY | source::SURF_SKY2D))) continue;

        const auto z = hit.endpos.z;
        if (stats.floorHits == 0) stats.lowestFloor = stats.highestFloor = z;
        stats.lowestFloor = std::min(stats.lowestFloor, z);
        stats.highestFloor = std::max(stats.highestFloor, z);
        ++stats.floorHits;
        if (hit.hitDisplacement) ++stats.displacementHits;
        if (std::abs(z - (-12800.0f)) < 64.0f) ++stats.flatgrassTerrainHits;
    }
}

std::vector<Vec3> openPoints(const CollisionWorld& world, Aabb bounds, std::mt19937& random) {
    const auto extent = bounds.maxs - bounds.mins;
    std::vector<Vec3> points;
    points.reserve(16384);
    for (unsigned n = 0; n < 131072 && points.size() < points.capacity(); ++n) {
        const Vec3 p{bounds.mins.x + extent.x * randomUnit(random),
                     bounds.mins.y + extent.y * randomUnit(random),
                     bounds.mins.z + extent.z * randomUnit(random)};
        if ((world.pointContents(p) & mask) == 0) points.push_back(p);
    }
    if (points.size() < 2) throw std::runtime_error("could not sample open points");
    return points;
}

std::vector<Segment> randomLines(std::span<const Vec3> points, std::mt19937& random) {
    std::vector<Segment> lines;
    lines.reserve(rayPairs);
    for (unsigned i = 0; i < rayPairs; ++i) {
        lines.push_back({points[random() % points.size()], points[random() % points.size()]});
    }
    return lines;
}

void checkLines(const CollisionWorld& world, std::span<const Segment> rays, RayStats& stats) {
    for (const auto& ray : rays) {
        const auto a = world.traceLine(ray.start, ray.end, mask);
        const auto b = world.traceLine(ray.end, ray.start, mask);
        ++stats.pairs;
        if (a.startSolid || b.startSolid) ++stats.startSolid;
        if (a.didHit() && b.didHit()) ++stats.blockedBothWays;
        if (a.didHit() != b.didHit()) ++stats.asymmetric;
    }
}

std::uint64_t hash(const TraceResult& result) {
    const auto bits = [](float f) { return std::uint64_t{std::bit_cast<std::uint32_t>(f)}; };
    return bits(result.fraction) + 3 * result.contents + 5 * result.surfaceFlags +
           7 * result.startSolid + 11 * result.allSolid + 13 * result.hitDisplacement +
           17 * bits(result.normal.x) + 19 * bits(result.normal.y) + 23 * bits(result.normal.z);
}

template<class Trace>
Throughput benchmark(std::span<const Segment> segments, unsigned workers, Trace trace) {
    const auto runRange = [&](unsigned first, unsigned last) {
        std::uint64_t sum = 0;
        for (auto i = first; i < last; ++i) sum += hash(trace(segments[i % segments.size()]));
        return sum;
    };

    const auto startSingle = std::chrono::steady_clock::now();
    const auto singleSum = runRange(0, benchTraces);
    const auto singleTime = std::chrono::duration<double>(std::chrono::steady_clock::now() - startSingle).count();

    std::vector<std::uint64_t> sums(workers);
    const auto startParallel = std::chrono::steady_clock::now();
    {
        std::vector<std::jthread> threads;
        threads.reserve(workers);
        for (unsigned i = 0; i < workers; ++i) {
            threads.emplace_back([&, i] { sums[i] = runRange(i * benchTraces / workers, (i + 1) * benchTraces / workers); });
        }
    }
    const auto parallelTime = std::chrono::duration<double>(std::chrono::steady_clock::now() - startParallel).count();
    std::uint64_t parallelSum = 0;
    for (const auto sum : sums) parallelSum += sum;
    return {benchTraces, benchTraces / singleTime, benchTraces / parallelTime, singleSum, parallelSum == singleSum};
}

} // namespace

void run(const CollisionWorld& world, std::string_view map, Report& report) {
    report.bounds = world.bounds();
    const auto drops = gridDrops(report.bounds);
    checkDrops(world, drops, report.drops);

    bool floorPassed = report.drops.floorHits != 0;
    if (map == "gm_flatgrass") {
        // The BSP player start is (-512, 576, -12287), one unit above the spawn deck.
        constexpr Vec3 spawn{-512, 576, -12287};
        report.flatgrassSpawn = {drop(world, {spawn + Vec3{0, 0, 128}, spawn - Vec3{0, 0, 128}}), -12288};
        const auto& floor = report.flatgrassSpawn;
        floorPassed = floorPassed && report.drops.flatgrassTerrainHits != 0 && !floor.trace.startSolid &&
                      floor.trace.didHit() && std::abs(floor.trace.endpos.z - floor.expectedZ) < 0.125f;
    }
    std::mt19937 random(0x4e415646);
    const auto points = openPoints(world, report.bounds, random);
    const auto lines = randomLines(points, random);
    checkLines(world, lines, report.rays);

    report.workers = std::max(1u, std::thread::hardware_concurrency());
    report.hullRate = benchmark(drops, report.workers, [&](const Segment& line) { return drop(world, line); });
    report.rayRate = benchmark(lines, report.workers, [&](const Segment& line) {
        return world.traceLine(line.start, line.end, mask);
    });

    report.passed = floorPassed && report.rays.blockedBothWays != 0 && report.rays.startSolid == 0 &&
                    report.rays.asymmetric == 0 && report.hullRate.deterministic && report.rayRate.deterministic;
}

} // namespace navfast::probe
