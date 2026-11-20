#include "nav/exact/generate.h"

#include "job/thread_pool.h"
#include "nav/area_builder.h"
#include "nav/exact/rectangles.h"
#include "nav/exact/walk_graph.h"

#include <algorithm>
#include <tuple>

namespace navfast {
namespace {

void connectAreas(NavMesh& mesh, const exact::Heightfield& field, ThreadPool& pool) {
    std::vector<std::tuple<AreaId, Direction, AreaId>> links;
    for (const auto& tile : field.tiles) {
        pool.checkCancelled();
        for (const auto& cell : tile.cells) {
            if (cell.area == AreaId{}) continue;
            for (const auto dir : directions) {
                const auto to = cell.links[dir];
                if (!to) continue;
                const auto area = field[to].area;
                if (area == AreaId{} || area == cell.area) continue;
                if (mesh[cell.area].overlapAlong(mesh[area], dir) > 0)
                    links.emplace_back(cell.area, dir, area);
            }
        }
    }
    std::ranges::sort(links);
    const auto repeats = std::ranges::unique(links);
    for (auto [from, dir, to] : std::ranges::subrange(links.begin(), repeats.begin())) mesh.connect(from, dir, to);
}

} // namespace

std::size_t generateExactNav(NavMesh& mesh, const CollisionWorld& world, std::span<const Vec3> seeds,
                             const GenerationParams& params, ThreadPool& pool,
                             const Phase& phase, const Progress& progress) {
    exact::Heightfield field(world.floorFaces(GenerationTraceMask, params.slopeLimit), pool);
    progress(1, {});
    phase("ExactReachableFloors");
    const auto reachable = exact::reachFloors(field, world, seeds, params, pool, progress);
    if (!reachable) return 0;
    phase("ExactPartitionFloors");
    exact::markStairTreads(field, pool);
    const auto rectangles = exact::partitionFloors(field, reachable, params, pool, progress);
    phase("ExactRefineEdges");
    std::vector<NavArea> areas(rectangles.size());
    constexpr std::size_t batch = 256;
    for (std::size_t start = 0; start < rectangles.size(); start += batch) {
        const auto count = std::min(batch, rectangles.size() - start);
        pool.run(count, [&](std::size_t i) {
            areas[start + i] = exact::refineRectangle(rectangles[start + i], field, world, params);
        });
        progress(static_cast<float>(start + count) / static_cast<float>(rectangles.size()), {});
    }
    for (std::size_t i = 0; i < areas.size(); ++i) {
        if (areas[i].width() < 1 || areas[i].height() < 1) continue;
        const auto id = mesh.add(std::move(areas[i]));
        for (const auto cell : rectangles[i].cells) field[cell].area = id;
    }
    phase("ExactConnectAreas");
    connectAreas(mesh, field, pool);
    markStairAreas(mesh, world, params, pool);
    progress(1, {});
    return reachable;
}

} // namespace navfast
