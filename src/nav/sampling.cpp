#include "nav/sampling.h"
#include "job/thread_pool.h"

#include <algorithm>
#include <optional>
#include <string>
#include <tuple>

namespace navfast {
namespace {

bool stayOnFloor(const CollisionWorld& world, const GenerationParams& params, TraceResult& trace, float drop) {
    const Vec3 start = trace.endpos;
    trace = traceNav(world, start, start - Vec3{0,0,drop});
    return !trace.startSolid && trace.fraction < 1.0f && trace.normal.z >= params.slopeLimit;
}

bool isNodeOverlapped(const CollisionWorld& world, Vec3 pos, Vec3 offset) {
    // During full generation there is no existing area to overlap.
    constexpr Vec3 half{0.5f,0.5f,0.5f};
    const auto start = pos + Vec3{0,0,HalfHumanHeight};
    const auto end = start + offset * GenerationStepSize;
    const auto across = world.traceHull(start, end, -half, half, GenerationTraceMask);
    if (across.startSolid || across.allSolid || across.fraction < 0.1f) return true;
    const auto down = world.traceHull(across.endpos, end - Vec3{0,0,HumanHeight},
                                      -half, half, GenerationTraceMask);
    return down.startSolid || down.allSolid || down.fraction == 1.0f || down.normal.z < 0.7f;
}

struct StepResult {
    TraceResult ground;
    NodeLink link;
};

bool climbToNode(const CollisionWorld& world, const GenerationParams& params,
                 Vec3 from, Vec3 to, StepResult& result) {
    for (float height = StepHeight; height <= ClimbUpHeight; height += 1.0f) {
        const Vec3 lift{0,0,height};
        const auto start = from + lift;
        const auto end = to + lift;
        auto trace = traceNav(world, start, end);
        if (!trace.startSolid && trace.fraction == 1.0f) {
            if (!stayOnFloor(world, params, trace, DeathDrop) ||
                traceNav(world, from, start).fraction < 1.0f) return false;
            result.ground = trace;
            result.link.obstacleHeight = height;
            return true;
        }
        const auto obstacleStart = trace.endpos - start;
        if (obstacleStart.lengthSqr() > GenerationStepSize * GenerationStepSize) continue;
        trace = traceNav(world, end, start);
        if (trace.startSolid || trace.fraction == 1.0f) continue;
        const auto obstacleEnd = trace.endpos - start;
        if (obstacleEnd.lengthSqr() > GenerationStepSize * GenerationStepSize) continue;
        result.link.obstacleStart = obstacleStart.length();
        result.link.obstacleEnd = obstacleEnd.length();
        if (result.link.obstacleEnd == 0) result.link.obstacleEnd = GenerationStepSize;
    }
    return false;
}

std::optional<StepResult> sampleStep(const CollisionWorld& world, const GenerationParams& params,
                                     Vec3 from, Direction dir) {
    Vec3 target{snapToGrid(from.x), snapToGrid(from.y), from.z};
    target += directionVector[dir] * GenerationStepSize;
    StepResult result{};
    if (!traceAdjacentNode(world, params, from, target, result.ground) &&
        !climbToNode(world, params, from, target, result)) return {};
    if (result.ground.surfaceFlags & (source::SURF_SKY | source::SURF_SKY2D)) return {};

    const auto to = result.ground.endpos;
    if (std::ranges::all_of(cornerVector, [&](Vec3 offset) {
            return isNodeOverlapped(world, to, offset);
        })) return {};

    if (params.displacementTest > 0) {
        const auto up = traceNav(world, to, to + Vec3{0,0,static_cast<float>(params.displacementTest)});
        if (up.fraction > 0) {
            const auto down = traceNav(world, up.endpos, to);
            if (down.fraction < 1.0f && down.endpos.z > to.z + StepHeight) return {};
        }
    }
    if (result.link.obstacleHeight < StepHeight || to.z - from.z > result.link.obstacleHeight - 2.0f)
        result.link = {};
    return result;
}

std::optional<TraceResult> findGroundForNode(const CollisionWorld& world, Vec3 seed) {
    if (!std::isfinite(seed.x) || !std::isfinite(seed.y) || !std::isfinite(seed.z)) return {};
    seed.x = snapToGrid(seed.x);
    seed.y = snapToGrid(seed.y);
    const auto ground = traceNav(world, seed + Vec3{0,0,DuckHullHeight - 0.1f}, seed - Vec3{0,0,DeathDrop});
    if (ground.allSolid || ground.fraction >= 1.0f) return {};
    return ground;
}

} // namespace

bool traceAdjacentNode(const CollisionWorld& world, const GenerationParams& params,
                       Vec3 start, Vec3 end, TraceResult& trace, float drop) {
    for (bool first = true;; first = false) {
        trace = traceNav(world, start, end);
        if (trace.startSolid) return false;
        if (end.x == trace.endpos.x && end.y == trace.endpos.y)
            return stayOnFloor(world, params, trace, drop);
        const auto delta = trace.endpos - start;
        if (!first && delta.x * delta.x + delta.y * delta.y < 1.0f) return false;
        if (!stayOnFloor(world, params, trace, drop)) return false;

        start = traceNav(world, trace.endpos, trace.endpos + Vec3{0,0,StepHeight}).endpos;
        end.z = start.z;
        drop = DeathDrop;
    }
}

NavNodes sampleWalkableSpace(const CollisionWorld& world, std::span<const Vec3> seeds,
                             const GenerationParams& params, ThreadPool& pool, const Progress& progress) {
    NavNodes nodes;
    std::vector<std::optional<TraceResult>> grounds(seeds.size());
    pool.run(seeds.size(), [&](std::size_t i) { grounds[i] = findGroundForNode(world, seeds[i]); });
    std::vector<NodeId> frontier;
    std::vector<NodeId> next;
    std::vector<std::array<std::optional<StepResult>, 4>> results;
    float fraction = 0;
    for (std::size_t seed = 0; seed < grounds.size(); ++seed) {
        const auto& ground = grounds[seed];
        if (!ground || nodes.find(ground->endpos) != NodeId{}) continue;
        frontier.push_back(nodes.add(ground->endpos, ground->normal, false));
        while (!frontier.empty()) {
            const auto order = [&](NodeId id) {
                const auto [x,y,z] = nodes[id].position;
                return std::tuple{x,y,z,id};
            };
            std::ranges::sort(frontier, {}, order);
            results.resize(frontier.size());
            pool.run(frontier.size(), [&](std::size_t i) {
                const auto id = frontier[i];
                nodes.checkCrouch(id, world);
                const auto& node = nodes[id];
                for (const auto dir : directions)
                    results[i][dir] = node.visited[dir] ? std::nullopt : sampleStep(world, params, node.position, dir);
            });
            for (std::size_t i = 0; i < frontier.size(); ++i) {
                const auto from = frontier[i];
                for (const auto dir : directions) {
                    if (nodes[from].visited[dir]) continue;
                    nodes[from].visited[dir] = true;
                    auto& result = results[i][dir];
                    if (!result) continue;
                    const auto& hit = result->ground;
                    auto target = nodes.find(hit.endpos);
                    if (target == NodeId{}) {
                        target = nodes.add(hit.endpos, hit.normal, hit.hitDisplacement);
                        next.push_back(target);
                    }
                    result->link.target = target;
                    nodes.connect(from, dir, result->link, hit.endpos.z);
                }
            }
            frontier.swap(next);
            next.clear();
            const auto estimate = static_cast<float>(nodes.size());
            fraction = std::max(fraction, (static_cast<float>(seed) + estimate / (estimate + 4096.0f)) /
                                           static_cast<float>(grounds.size()));
            progress(fraction, std::to_string(nodes.size()) + " nodes");
        }
    }
    // TODO: Sample from ladder ends and connect generated ladders.
    progress(1, std::to_string(nodes.size()) + " nodes");
    return nodes;
}

} // namespace navfast
