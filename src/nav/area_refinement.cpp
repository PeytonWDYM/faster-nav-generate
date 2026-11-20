#include "nav/area_builder.h"
#include "nav/area_connections.h"
#include "nav/obstacle_tops.h"
#include "job/thread_pool.h"

#include <algorithm>

namespace navfast {
namespace {

void markJumpAreas(NavMesh& mesh, const CollisionWorld& world, const GenerationParams& params, ThreadPool& pool) {
    const auto ids = mesh.areaIds();
    pool.run(ids.size(), [&](std::size_t i) {
        auto& area = mesh[ids[i]];
        const auto z = std::min(area.normal().z, area.normal(true).z);
        if (z < params.slopeLimit) {
            area.attributes |= Jump | NoMerge;
        } else if (z < params.slopeLimit + params.slopeTolerance) {
            const auto start = area.center() + Vec3{0,0,HalfHumanHeight};
            const auto ground = world.traceLine(start, start - Vec3{0,0,9999.9f}, GenerationTraceMask);
            if (!ground.startSolid && std::abs(ground.normal.z - z) > params.slopeTolerance)
                area.attributes |= Jump | NoMerge;
        }
    });
}

bool onDisplacement(const NavMesh& mesh, const NavArea& area) {
    return std::ranges::any_of(area.nodes, [&](NodeId id) { return mesh.nodes[id].displacement; });
}

bool coplanar(const NavMesh& mesh, const NavArea& a, const NavArea& b) {
    const auto normalA = a.normal();
    const auto normalB = b.normal();
    const bool displaced = onDisplacement(mesh, a);
    if (!displaced && normalA.dot(a.normal(true)) <= 0.99f) return false;
    if (!onDisplacement(mesh, b) && normalB.dot(b.normal(true)) <= 0.99f) return false;
    return normalA.dot(normalB) > (displaced ? 0.7f : 0.99f);
}

bool mergeNext(NavMesh& mesh, AreaId id, float maxSize) {
    const auto& area = mesh[id];
    if (!area.alive || !area.hasNodes() || (area.attributes & NoMerge)) return false;
    for (const auto dir : {North, South, West, East}) {
        const auto [a,b] = edgeCorners[dir];
        const auto [c,d] = edgeCorners[opposite(dir)];
        for (const auto other : area.outgoing[dir]) {
            const auto& adj = mesh[other];
            if (!adj.hasNodes() || (adj.attributes & NoMerge) || adj.attributes != area.attributes ||
                area.nodes[a] != adj.nodes[c] || area.nodes[b] != adj.nodes[d]) continue;
            const auto size = alongX(dir) ? area.width() + adj.width() : area.height() + adj.height();
            if (size > maxSize || !coplanar(mesh, area, adj)) continue;
            auto combined = area.nodes;
            combined[a] = adj.nodes[a];
            combined[b] = adj.nodes[b];
            mesh.merge(id, other, combined);
            return true;
        }
    }
    return false;
}

void mergeGeneratedAreas(NavMesh& mesh, const GenerationParams& params, ThreadPool& pool) {
    const auto ids = mesh.areaIds();
    const auto maxSize = GenerationStepSize * static_cast<float>(params.areaMaxSize);
    for (;;) {
        bool merged = false;
        for (const auto id : ids) {
            pool.checkCancelled();
            if (mergeNext(mesh, id, maxSize)) {
                merged = true;
                break;
            }
        }
        if (!merged) return;
    }
}

bool splitOverhang(NavMesh& mesh, AreaId from, Direction dir, AreaId to) {
    if (!mesh[from].overlaps(mesh[to])) return false;
    const auto a = mesh[from].bounds();
    const auto b = mesh[to].bounds();
    if (a.mins.z <= b.maxs.z + HumanCrouchHeight && b.mins.z <= a.maxs.z + HumanCrouchHeight) return false;
    const bool fromAbove = a.mins.z > b.mins.z;
    const auto above = fromAbove ? from : to;
    const auto below = fromAbove ? to : from;
    const auto down = fromAbove ? dir : opposite(dir);
    const auto up = opposite(down);
    const auto top = fromAbove ? a : b;
    const auto bottom = fromAbove ? b : a;
    const bool splitY = !alongX(down);
    const auto coord = [splitY](Vec3 p) { return splitY ? p.y : p.x; };
    const auto bottomMin = coord(bottom.mins);
    const auto bottomMax = coord(bottom.maxs);
    const auto topMax = coord(top.maxs);
    const bool cutAfterShadow = topMax < bottomMax;
    auto edge = cutAfterShadow ? topMax : coord(top.mins);
    const auto shadow = cutAfterShadow ? edge - bottomMin : bottomMax - edge;
    const bool useFirst = down == North || down == West;
    if (shadow < GenerationStepSize) {
        if (shadow < GenerationStepSize * 0.3f || bottomMax - bottomMin <= GenerationStepSize * 2) return false;
        edge += (GenerationStepSize - shadow) * (useFirst ? -1.0f : 1.0f);
    }
    if (edge <= bottomMin + 1 || edge >= bottomMax - 1) return false;
    const bool hadDown = std::ranges::find(mesh[above].outgoing[down], below) != mesh[above].outgoing[down].end();
    const bool hadUp = std::ranges::find(mesh[below].outgoing[up], above) != mesh[below].outgoing[up].end();
    if (hadDown) mesh.disconnect(above, below);
    if (hadUp) mesh.disconnect(below, above);
    const auto [first, second] = mesh.split(below, splitY, edge);
    const auto exposed = useFirst ? first : second;
    if (hadDown) mesh.connect(above, down, exposed);
    if (hadUp) mesh.connect(exposed, up, above);
    return true;
}

bool splitNextOverhang(NavMesh& mesh, ThreadPool& pool) {
    for (const auto id : mesh.areaIds()) {
        pool.checkCancelled();
        for (const auto dir : directions)
            for (const auto to : mesh[id].outgoing[dir])
                if (splitOverhang(mesh, id, dir, to)) return true;
    }
    return false;
}

void squareUpAreas(NavMesh& mesh, ThreadPool& pool) {
    const auto square = [&](auto&& self, AreaId id, bool splitY) -> void {
        pool.checkCancelled();
        const auto& area = mesh[id];
        const auto aspect = area.width() / area.height();
        if (aspect >= 1.0f / 3.01f && aspect <= 3.01f) return;
        const auto center = area.center();
        const auto [a,b] = mesh.split(id, splitY, snapToGrid(splitY ? center.y : center.x));
        if (a != AreaId{}) {
            self(self, a, splitY);
            self(self, b, splitY);
        }
    };
    for (const auto id : mesh.areaIds()) {
        if (mesh[id].hasNodes()) square(square, id, mesh[id].height() > mesh[id].width());
    }
}

enum class StairTest { No, Maybe, Yes };

StairTest testStairLine(const CollisionWorld& world, const GenerationParams& params, Vec3 start, Vec3 end,
                       StairTest result) {
    if (result == StairTest::No || std::abs(start.z - end.z) <= StepHeight) return result;

    constexpr float spacing = 5.0f;
    const float minStep = spacing * std::tan(std::acos(params.slopeLimit));
    const bool x = std::abs(start.x - end.x) > std::abs(start.y - end.y);
    const Vec3 mins{x ? -8.0f : -spacing / 2, x ? -spacing / 2 : -8.0f, 0};
    const Vec3 maxs{-mins.x,-mins.y,1};
    const Vec3 lift{0,0,DuckHullHeight};
    const auto traceAt = [&](Vec3 p) {
        return world.traceHull(p + lift, p - lift, mins, maxs, GenerationTraceMask);
    };
    const auto initial = traceAt(start);
    if (initial.startSolid || initial.hitDisplacement) return StairTest::No;
    float priorZ = initial.endpos.z;
    const auto increment = spacing / std::hypot(end.x - start.x, end.y - start.y);
    for (float t = 0; t <= 1; t += increment) {
        const auto trace = traceAt(start + (end - start) * t);
        const auto height = trace.endpos.z;
        if (trace.startSolid || trace.hitDisplacement || trace.normal.z < 0.97f ||
            (t == 0 && std::abs(height - start.z) > StepHeight) ||
            (t == 1 && std::abs(height - end.z) > StepHeight)) return StairTest::No;
        const auto diff = std::abs(height - priorZ);
        if (diff > StepHeight) return StairTest::No;
        if (diff >= minStep) result = StairTest::Yes;
        priorZ = height;
    }
    return result;
}

bool testStairs(const CollisionWorld& world, const GenerationParams& params, const NavArea& area) {
    if ((area.width() <= GenerationStepSize && area.height() <= GenerationStepSize) ||
        area.normal().dot(area.normal(true)) < 0.95f) return false;
    constexpr float inset = 5;
    const auto [nw, ne, se, sw] = area.points;
    const auto a = nw + Vec3{inset,inset,0};
    const auto b = ne + Vec3{-inset,inset,0};
    const auto c = se + Vec3{-inset,-inset,0};
    const auto d = sw + Vec3{inset,-inset,0};
    auto result = StairTest::Maybe;
    for (const auto& [from,to] : {std::pair{a,b}, std::pair{d,c}, std::pair{a,d}, std::pair{b,c},
                                std::pair{(a+b)*0.5f,(d+c)*0.5f}, std::pair{(b+c)*0.5f,(a+d)*0.5f}})
        result = testStairLine(world, params, from, to, result);
    return result == StairTest::Yes;
}

} // namespace

void markStairAreas(NavMesh& mesh, const CollisionWorld& world, const GenerationParams& params, ThreadPool& pool) {
    const auto ids = mesh.areaIds();
    pool.run(ids.size(), [&](std::size_t i) {
        auto& area = mesh[ids[i]];
        if (testStairs(world, params, area)) area.attributes |= Stairs;
    });
}

void refineGeneratedAreas(NavMesh& mesh, const CollisionWorld& world, const GenerationParams& params,
                          ThreadPool& pool, const Phase& phase) {
    phase("ConnectGeneratedAreas");
    connectGeneratedAreas(mesh, world, params, pool);
    // The SDK's MarkPlayerClipAreas only marks areas in TERROR builds.
    phase("MarkJumpAreas");
    markJumpAreas(mesh, world, params, pool);
    phase("MergeGeneratedAreas");
    mergeGeneratedAreas(mesh, params, pool);
    phase("SplitAreasUnderOverhangs");
    while (splitNextOverhang(mesh, pool)) {}
    phase("SquareUpAreas");
    squareUpAreas(mesh, pool);
    phase("MarkStairAreas");
    markStairAreas(mesh, world, params, pool);
    phase("StitchAndRemoveJumpAreas");
    stitchAndRemoveJumpAreas(mesh, params, pool);
    phase("HandleObstacleTopAreas");
    if (params.generateFenceTops) handleObstacleTopAreas(mesh, params, pool);
    phase("FixUpGeneratedAreas");
    fixUpGeneratedAreas(mesh, world, params, pool);
}

} // namespace navfast
