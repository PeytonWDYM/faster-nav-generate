#include "nav/area_connections.h"
#include "nav/sampling.h"
#include "job/thread_pool.h"

#include <algorithm>

namespace navfast {
namespace {

bool testJumpDown(const CollisionWorld& world, Vec3 from, Vec3 to) {
    const float drop = from.z - to.z;
    if (drop <= JumpCrouchHeight || drop >= DeathDrop) return false;

    for (float height = 1; height <= ClimbUpHeight; height += 1.0f) {
        const auto up = traceNav(world, from, from + Vec3{0,0,height});
        if (up.fraction <= 0 || up.startSolid) continue;
        const Vec3 start{from.x, from.y, up.endpos.z - 0.5f};
        const Vec3 end{to.x, to.y, start.z};
        const auto across = traceNav(world, start, end);
        if (across.fraction != 1 || across.startSolid) continue;
        to.z += 2;
        const auto down = traceNav(world, end, to);
        return down.fraction > 0 && !down.startSolid && down.endpos.z <= to.z + StepHeight;
    }
    return false;
}

AreaId findJumpDownArea(const NavMesh& mesh, const CollisionWorld& world, Vec3 pos, Direction dir) {
    const auto step = directionVector[dir] * GenerationStepSize;
    const auto start = pos + Vec3{0,0,HalfHumanHeight} + step * 0.5f;
    auto end = start;
    for (int n = 0; n < 4; ++n) {
        end += step;
        if (traceNav(world, start, end).fraction < 1) break;
        const auto area = mesh.getNavArea(end, DeathDrop);
        if (area == AreaId{}) continue;
        end.z = mesh[area].getZ(end.x, end.y);
        return testJumpDown(world, pos, end) ? area : AreaId{};
    }
    return {};
}

using Connections = std::array<std::vector<AreaId>, 4>;

Connections findConnections(const NavMesh& mesh, AreaId id, const CollisionWorld& world, bool jumpConnections) {
    Connections result;
    const auto& area = mesh[id];
    for (const auto dir : {North, West, South, East}) {
        const auto [a,b] = edgeCorners[dir];
        const auto across = alongX(dir) ? South : East;
        const bool exterior = dir == South || dir == East;
        auto start = area.nodes[a];
        auto end = area.nodes[b];
        const auto record = [&](AreaId candidate) {
            if (candidate != id && candidate != AreaId{}) result[dir].push_back(candidate);
        };
        const auto jump = [&](NodeId node) {
            if (jumpConnections) record(findJumpDownArea(mesh, world, mesh.nodes[node].position, dir));
        };
        if (exterior) {
            start = mesh.nodes[start].links[opposite(dir)].target;
            end = mesh.nodes[end].links[opposite(dir)].target;
        }
        for (auto node = start; node != NodeId{} && node != end; node = mesh.nodes[node].links[across].target) {
            const auto neighbor = mesh.nodes[node].links[dir].target;
            if (neighbor != NodeId{} && mesh.nodes[neighbor].area != AreaId{} &&
                mesh.nodes[neighbor].links[opposite(dir)].target == node) record(mesh.nodes[neighbor].area);
            else jump(node);
        }
        if (!exterior || !jumpConnections) continue;
        for (auto node = area.nodes[a]; node != NodeId{} && node != area.nodes[b]; node = mesh.nodes[node].links[across].target) {
            const auto& source = mesh.nodes[node];
            const auto adjacent = source.links[dir].target;
            if (source.area != AreaId{} || source.blocked()) continue;
            if (adjacent != NodeId{} && (mesh.nodes[adjacent].blocked() || mesh.nodes[adjacent].area != AreaId{})) continue;
            jump(node);
        }
    }
    return result;
}

bool hasLink(const NavMesh& mesh, AreaId from, Direction dir, AreaId to) {
    const auto& list = mesh[from].outgoing[dir];
    return std::ranges::find(list, to) != list.end();
}

void fixCornerOnCornerAreas(NavMesh& mesh, const CollisionWorld& world, const GenerationParams& params,
                             ThreadPool& pool) {
    for (const auto id : mesh.areaIds()) {
        pool.checkCancelled();
        for (const auto corner : corners) {
            const auto right = directions[corner];
            const auto left = directions[(corner + 3) % 4];
            const auto& area = mesh[id];
            if (!area.outgoing[left].empty() || !area.outgoing[right].empty() ||
                !area.incoming[left].empty() || !area.incoming[right].empty()) continue;
            const auto pos = area.points[corner];
            for (const auto dir : {left, right}) {
                const auto alongOurEdge = dir == left ? opposite(right) : opposite(left);
                const auto a = pos + directionVector[dir] * (GenerationStepSize / 2);
                const auto b = pos + directionVector[alongOurEdge] * (GenerationStepSize / 2);
                const auto far = a + (b - pos);
                const auto other = mesh.getNavArea(a);
                if (other == AreaId{} || other == id || mesh[other].points[(corner + 2) % 4] != pos) continue;
                TraceResult trace;
                if (!traceAdjacentNode(world, params, pos, a, trace, StepHeight) ||
                    !traceAdjacentNode(world, params, a, far, trace, StepHeight) ||
                    !traceAdjacentNode(world, params, b, far, trace, StepHeight) ||
                    mesh.getNavArea(far) != AreaId{}) continue;
                NavArea bridge;
                bridge.attributes = mesh[id].attributes;
                const auto x = std::min(pos.x, far.x);
                const auto y = std::min(pos.y, far.y);
                for (const auto p : {pos,a,b,trace.endpos}) {
                    const auto index = p.y == y ? (p.x == x ? NW : NE) : (p.x == x ? SW : SE);
                    bridge.points[index] = p;
                }
                const auto link = mesh.add(std::move(bridge));
                mesh.connect(id, dir, link);
                mesh.connect(link, opposite(dir), id);
                mesh.connect(other, alongOurEdge, link);
                mesh.connect(link, opposite(alongOurEdge), other);
            }
        }
    }
}

void fixConnections(NavMesh& mesh, ThreadPool& pool) {
    for (const auto id : mesh.areaIds()) {
        pool.checkCancelled();
        const auto& area = mesh[id];
        if (!(area.attributes & Stairs) || !area.hasNodes()) continue;
        for (const auto dir : directions) {
            const auto [a,b] = edgeCorners[dir];
            if (std::abs(area.points[a].z - area.points[b].z) < StepHeight) continue;
            const auto neighbors = area.outgoing[dir];
            const auto back = opposite(dir);
            for (const auto to : neighbors) {
                if (!mesh[to].hasNodes()) continue;
                const auto portal = area.portal(mesh[to], dir).center;
                const auto fromNode = mesh.closestEdgeNode(id, dir, portal);
                const auto toNode = mesh.closestEdgeNode(to, back, mesh[to].closestPoint(portal));
                if (fromNode == NodeId{} || toNode == NodeId{}) continue;
                const auto& from = mesh.nodes[fromNode];
                const auto& dest = mesh.nodes[toNode];
                const auto [c,d] = edgeCorners[back];
                if (std::max(from.clearance[a].groundHeight, from.clearance[b].groundHeight) > StepHeight ||
                    dest.position.z + std::max(dest.clearance[c].groundHeight, dest.clearance[d].groundHeight) >
                        from.position.z + StepHeight) mesh.disconnect(id, to);
            }
        }
    }
    for (const auto id : mesh.areaIds()) {
        pool.checkCancelled();
        for (const auto dir : directions) {
            std::vector<AreaId> redundant;
            for (const auto adjacent : mesh[id].outgoing[dir])
                for (const auto far : mesh[adjacent].outgoing[dir])
                    if (hasLink(mesh, id, dir, far)) redundant.push_back(far);
            for (const auto far : redundant) mesh.disconnect(id, far);
        }
    }
}

} // namespace

void connectGeneratedAreas(NavMesh& mesh, const CollisionWorld& world, const GenerationParams& params,
                           ThreadPool& pool) {
    const auto ids = mesh.areaIds();
    std::vector<Connections> connections(ids.size());
    pool.run(ids.size(), [&](std::size_t i) {
        connections[i] = findConnections(mesh, ids[i], world, params.generateJumpConnections);
    });
    for (std::size_t i = 0; i < ids.size(); ++i)
        for (const auto dir : directions)
            for (const auto to : connections[i][dir]) mesh.connect(ids[i], dir, to);
}

void stitchAndRemoveJumpAreas(NavMesh& mesh, const GenerationParams& params, ThreadPool& pool) {
    const auto ids = mesh.areaIds();
    for (const auto id : ids) {
        pool.checkCancelled();
        const auto& jump = mesh[id];
        if (!(jump.attributes & Jump) || !params.generateJumpConnections) continue;
        for (const auto dir : directions) {
            const auto back = opposite(dir);
            for (const auto from : jump.incoming[back]) {
                if (mesh[from].attributes & Jump) continue;
                for (const auto to : jump.outgoing[dir]) {
                    if (mesh[to].attributes & Jump) continue;
                    const auto portal = mesh[from].portal(mesh[to], dir);
                    if (portal.halfWidth <= 0) continue;
                    const auto source = mesh[from].closestPoint(portal.center);
                    const auto dest = mesh[to].closestPoint(portal.center);
                    if ((mesh[from].attributes & Stairs) && source.z + StepHeight < dest.z) continue;
                    if (std::hypot(dest.x - source.x, dest.y - source.y) < GenerationStepSize * 3)
                        mesh.connect(from, dir, to);
                }
            }
        }
    }
    if (params.fixupJumpAreas)
        for (const auto id : ids)
            if (mesh[id].attributes & Jump) mesh.remove(id);
}

void fixUpGeneratedAreas(NavMesh& mesh, const CollisionWorld& world, const GenerationParams& params,
                         ThreadPool& pool) {
    fixCornerOnCornerAreas(mesh, world, params, pool);
    fixConnections(mesh, pool);
}

} // namespace navfast
