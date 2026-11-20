#include "nav/exact/walk_graph.h"

#include "job/thread_pool.h"
#include "nav/exact/clearance.h"
#include "nav/sampling.h"

#include <algorithm>
#include <limits>
#include <string>

namespace navfast::exact {
namespace {

constexpr float SweepSkin = 0.75f; // Keep the narrow hull clear of the plane under its corners.
constexpr int GapCells = 5; // Reach past the clearance setback at a ledge.

bool clear(const CollisionWorld& world, Vec3 from, Vec3 to, bool wide) {
    if (!wide) return !traceNav(world, from, to).didHit();
    constexpr Vec3 mins{-HalfHumanWidth, -HalfHumanWidth, 0};
    constexpr Vec3 maxs{HalfHumanWidth, HalfHumanWidth, HumanCrouchHeight};
    return !world.traceHull(from, to, mins, maxs, GenerationTraceMask).didHit();
}

bool canMove(const CollisionWorld& world, Vec3 from, Vec3 to, bool wide) {
    from.z += wide ? FloorSkin : SweepSkin;
    to.z += wide ? FloorSkin : SweepSkin;
    if (std::abs(to.z - from.z) <= StepHeight && clear(world, from, to, wide)) return true;
    const auto height = std::max(from.z, to.z);
    const Vec3 takeoff{from.x, from.y, height};
    const Vec3 landing{to.x, to.y, height};
    return (from == takeoff || clear(world, from, takeoff, wide)) &&
           clear(world, takeoff, landing, wide) &&
           (landing == to || clear(world, landing, to, wide));
}

std::uint32_t neighborTile(const Heightfield& field, std::uint32_t id, GridPoint p, Direction dir) {
    const auto& tile = field.tiles[id];
    const auto x = p.x - tile.origin.x, y = p.y - tile.origin.y;
    return x >= 0 && x < TileWidth && y >= 0 && y < TileWidth ? id : tile.neighbors[dir];
}

CellRef findLink(const Heightfield& field, const CollisionWorld& world, CellRef from,
                 GridPoint point, Direction dir, float maxRise) {
    const auto& cell = field[from];
    const auto start = center(point, cell.z);
    for (int distance = 1; distance <= GapCells; ++distance) {
        const auto next = moved(point, dir, distance);
        const auto tile = neighborTile(field, from.tile, next, dir);
        if (!tile) break;
        CellRef result;
        float best = DeathDrop;
        for (const auto& candidate : field.column(tile, next)) {
            const auto rise = candidate.z - cell.z;
            const auto delta = std::abs(rise);
            if (rise > maxRise || delta >= best || (distance > 1 && delta <= StepHeight)) continue;
            const auto adjustedRise = rise + candidate.lift - cell.lift;
            if (adjustedRise > maxRise || adjustedRise <= -DeathDrop) continue;
            // At one cell apart, a narrow sweep fits inside both endpoint footprints.
            // Overlapping clearance volumes cover its height range on normal steps.
            const bool covered = distance == 1 && delta <= StepHeight && std::abs(adjustedRise) <= StepHeight;
            if (!covered && !canMove(world, start + Vec3{0, 0, cell.lift},
                                     center(next, candidate.z + candidate.lift), true)) continue;
            result = field.ref(tile, candidate);
            best = delta;
        }
        if (result) return result;
    }
    return {};
}

// Match classic's seed ground search, then pick a nearby analytic floor on that level.
CellRef findSeed(const Heightfield& field, const CollisionWorld& world, Vec3 seed) {
    const Vec3 snapped{snapToGrid(seed.x), snapToGrid(seed.y), seed.z};
    for (const auto position : {snapped, seed}) {
        const auto ground = traceNav(world, position + Vec3{0, 0, DuckHullHeight - 0.1f},
                                     position - Vec3{0, 0, DeathDrop});
        if (!ground.didHit() || ground.allSolid ||
            (ground.surfaceFlags & (source::SURF_SKY | source::SURF_SKY2D))) continue;
        const auto p = gridPoint(position);
        CellRef result;
        float best = std::numeric_limits<float>::infinity();
        for (int y = p.y - 2; y <= p.y + 2; ++y) {
            for (int x = p.x - 2; x <= p.x + 2; ++x) {
                const GridPoint point{x, y};
                const auto tile = field.tileAt(point);
                if (!tile) continue;
                for (const auto& candidate : field.column(tile, point)) {
                    const auto target = center(point, candidate.z);
                    const auto delta = target - ground.endpos;
                    const auto score = delta.lengthSqr() + 3 * delta.z * delta.z;
                    if (score >= best || std::abs(delta.z) > StepHeight ||
                        !canMove(world, ground.endpos, target, false)) continue;
                    result = field.ref(tile, candidate);
                    best = score;
                }
            }
        }
        if (result) return result;
    }
    return {};
}

void rasterizeTiles(Heightfield& field, const CollisionWorld& world, const GenerationParams& params,
                    ThreadPool& pool, std::vector<std::uint32_t>& tiles) {
    std::ranges::sort(tiles);
    const auto duplicates = std::ranges::unique(tiles);
    tiles.erase(duplicates.begin(), duplicates.end());
    pool.run(tiles.size(), [&](std::size_t i) { field.rasterize(tiles[i], world, params); });
    tiles.clear();
}

struct TileWork {
    std::vector<CellRef> pending;
    std::vector<CellRef> exits;
    std::size_t reached = 0;
};

// Each worker owns one tile's mutable state. Other tiles receive exits after the batch joins.
void visitTile(Heightfield& field, const CollisionWorld& world, float maxRise, TileWork& work) {
    for (std::size_t i = 0; i < work.pending.size(); ++i) {
        const auto from = work.pending[i];
        auto& cell = field[from];
        if (cell.reachable) continue;
        cell.reachable = true;
        ++work.reached;
        const auto point = field.point(from);
        for (const auto dir : directions) {
            const auto to = findLink(field, world, from, point, dir, maxRise);
            cell.links[dir] = to;
            if (!to) continue;
            if (to.tile != from.tile) work.exits.push_back(to);
            else if (!field[to].reachable) work.pending.push_back(to);
        }
    }
    work.pending.clear();
}

// Follow level cells to the next distinct tread, within a normal stair tread depth.
CellRef nextTread(const Heightfield& field, CellRef from, Direction dir, float sign) {
    const auto z = field[from].z;
    auto at = from;
    for (int i = 0; i < 4; ++i) {
        const auto to = field[at].links[dir];
        if (!to || field[to].links[opposite(dir)] != at) return {};
        const auto& cell = field[to];
        const auto& face = field.faces[cell.face];
        if (face.displacement || face.plane.normal.z < 0.99f) return {};
        const auto rise = sign * (cell.z - z);
        if (std::abs(rise) <= FloorSkin) at = to;
        else return rise >= 5 && rise <= StepHeight ? to : CellRef{};
    }
    return {};
}

} // namespace

std::size_t reachFloors(Heightfield& field, const CollisionWorld& world, std::span<const Vec3> seeds,
                        const GenerationParams& params, ThreadPool& pool, const Progress& progress) {
    std::vector<std::uint32_t> raster;
    std::vector<std::uint32_t> active, next;
    std::vector<TileWork> work(field.tiles.size());
    const auto queue = [&](CellRef ref, std::vector<std::uint32_t>& frontier) {
        if (!ref || field[ref].reachable) return;
        auto& pending = work[ref.tile].pending;
        if (pending.empty()) frontier.push_back(ref.tile);
        pending.push_back(ref);
    };
    for (const auto seed : seeds) {
        if (!std::isfinite(seed.x) || !std::isfinite(seed.y) || !std::isfinite(seed.z)) continue;
        const Vec3 snapped{snapToGrid(seed.x), snapToGrid(seed.y), seed.z};
        for (const auto p : {gridPoint(seed), gridPoint(snapped)}) {
            for (int y = p.y - 2; y <= p.y + 2; ++y)
                for (int x = p.x - 2; x <= p.x + 2; ++x)
                    if (const auto tile = field.tileAt({x, y})) raster.push_back(tile);
        }
    }
    rasterizeTiles(field, world, params, pool, raster);
    for (const auto seed : seeds)
        if (std::isfinite(seed.x) && std::isfinite(seed.y) && std::isfinite(seed.z))
            queue(findSeed(field, world, seed), active);

    const float maxRise = params.generateJumpConnections ? JumpCrouchHeight : StepHeight;
    std::size_t reached = 0;
    while (!active.empty()) {
        for (const auto id : active) {
            for (const auto neighbor : field.tiles[id].neighbors)
                if (neighbor && !field.tiles[neighbor].rasterized) raster.push_back(neighbor);
        }
        rasterizeTiles(field, world, params, pool, raster);
        pool.run(active.size(), [&](std::size_t i) { visitTile(field, world, maxRise, work[active[i]]); });
        for (const auto id : active) {
            reached += work[id].reached;
            work[id].reached = 0;
            for (const auto exit : work[id].exits) queue(exit, next);
            work[id].exits.clear();
        }
        active.swap(next);
        next.clear();
        const auto count = static_cast<float>(reached);
        progress(count / (count + 16384), std::to_string(reached) + " reachable cells");
    }
    progress(1, std::to_string(reached) + " reachable cells");
    return reached;
}

void markStairTreads(Heightfield& field, ThreadPool& pool) {
    pool.run(field.tiles.size() - 1, [&](std::size_t index) {
        const auto id = static_cast<std::uint32_t>(index + 1);
        for (auto& cell : field.tiles[id].cells) {
            const auto& face = field.faces[cell.face];
            if (!cell.reachable || face.displacement || face.plane.normal.z < 0.99f) continue;
            const auto here = field.ref(id, cell);
            for (const auto dir : directions) {
                for (const auto sign : {-1.0f, 1.0f}) {
                    const auto first = nextTread(field, here, dir, sign);
                    if (first && (nextTread(field, first, dir, sign) ||
                                  nextTread(field, here, opposite(dir), -sign))) {
                        cell.attributes |= Stairs;
                    }
                }
            }
        }
    });
}

} // namespace navfast::exact
