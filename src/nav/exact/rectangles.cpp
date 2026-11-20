#include "nav/exact/rectangles.h"

#include "job/thread_pool.h"
#include "nav/exact/clearance.h"

#include <algorithm>
#include <bitset>
#include <string>

namespace navfast::exact {
namespace {

CellRef adjacent(const Heightfield& field, CellRef from, Direction dir) {
    const auto to = field[from].links[dir];
    if (!to || field[to].links[opposite(dir)] != from ||
        field.point(to) != moved(field.point(from), dir)) return {};
    return to;
}

bool fitsPlane(FloorPlane plane, FloorPlane other, Vec3 p, const GenerationParams& params, bool tread) {
    if (std::abs(other.normal.z - plane.normal.z) > std::max(params.slopeTolerance, 0.0f)) return false;
    const auto x = other.normal.x / other.normal.z - plane.normal.x / plane.normal.z;
    const auto y = other.normal.y / other.normal.z - plane.normal.y / plane.normal.z;
    // Bound deviation at the cell corners as well as its center.
    const auto deviation = std::abs(p.z - plane.heightAt(p)) + CellSize * 0.5f * (std::abs(x) + std::abs(y));
    return deviation <= (tread ? FloorSkin : planeHeightTolerance(params));
}

constexpr int TerrainPatchWidth = 64;

GridPoint patchOrigin(GridPoint point) {
    return {point.x & ~(TerrainPatchWidth - 1), point.y & ~(TerrainPatchWidth - 1)};
}

// Convex floors can use a source face directly. Keep continuous displacement
// sheets together within a patch so the rectangle fitter can follow their trend.
std::vector<CellRef> collectRegion(Heightfield& field, CellRef seed, std::uint32_t id,
                                   const GenerationParams& params, ThreadPool& pool) {
    const auto& base = field[seed];
    const auto& face = field.faces[base.face];
    const auto start = field.point(seed);
    const auto origin = patchOrigin(start);
    const auto index = [&](GridPoint p) { return (p.y - origin.y) * TerrainPatchWidth + p.x - origin.x; };
    std::bitset<TerrainPatchWidth * TerrainPatchWidth> claimed;
    claimed.set(index(start));
    std::vector<CellRef> cells{seed};
    field[seed].region = id;
    for (std::size_t i = 0; i < cells.size(); ++i) {
        if (i % 4096 == 0) pool.checkCancelled();
        for (const auto dir : directions) {
            const auto to = adjacent(field, cells[i], dir);
            if (!to) continue;
            auto& cell = field[to];
            if (!cell.reachable || cell.region || cell.attributes != base.attributes) continue;
            const auto& other = field.faces[cell.face];
            if (face.displacement) {
                const auto p = field.point(to);
                if (!other.displacement || patchOrigin(p) != origin || claimed.test(index(p))) continue;
                const auto mid = (field.position(cells[i]) + field.position(to)) * 0.5f;
                const auto& prior = field.faces[field[cells[i]].face].plane;
                if (std::abs(prior.heightAt(mid) - other.plane.heightAt(mid)) > planeHeightTolerance(params))
                    continue;
                claimed.set(index(p));
            } else if (other.displacement ||
                       (cell.face != base.face && !fitsPlane(face.plane, other.plane, field.position(to),
                                                            params, (base.attributes & Stairs) != 0))) {
                continue;
            }
            cell.region = id;
            cells.push_back(to);
        }
    }
    return cells;
}

} // namespace

std::vector<Rectangle> partitionFloors(Heightfield& field, std::size_t reachable,
                                       const GenerationParams& params, ThreadPool& pool, const Progress& progress) {
    const auto limit = static_cast<int>(std::min(32768.0f, GenerationStepSize * params.areaMaxSize) / CellSize);
    std::vector<Rectangle> result;
    std::uint32_t region = 0;
    std::size_t processed = 0, reported = 0;
    for (std::uint32_t tile = 1; tile < field.tiles.size(); ++tile) {
        pool.checkCancelled();
        for (std::uint32_t i = 0; i < field.tiles[tile].cells.size(); ++i) {
            const CellRef first{tile, i};
            const auto& cell = field[first];
            if (!cell.reachable || cell.region) continue;
            const auto cells = collectRegion(field, first, ++region, params, pool);
            packRegion(result, field, cells, params, limit, pool);
            processed += cells.size();
            if (processed - reported >= 4096) {
                progress(static_cast<float>(processed) / static_cast<float>(reachable),
                         std::to_string(result.size()) + " rectangles");
                reported = processed;
            }
        }
    }
    progress(1, std::to_string(result.size()) + " rectangles");
    return result;
}

} // namespace navfast::exact
