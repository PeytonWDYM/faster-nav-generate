#include "nav/obstacle_tops.h"
#include "job/thread_pool.h"

#include <algorithm>
#include <limits>
#include <optional>
#include <set>

namespace navfast {
namespace {

constexpr float MinObstacleAreaWidth = 10;

struct Obstacle {
    float start = GenerationStepSize;
    float end = 0;
    float height = 0;
    float leftZ = 0;
    float rightZ = 0;
    Direction direction{};
};

void adjustDistances(Obstacle& edge, float limit) {
    const float width = edge.end - edge.start;
    if (width >= MinObstacleAreaWidth) return;
    edge.start = std::clamp(edge.start - (MinObstacleAreaWidth - width) * 0.5f,
                            0.0f, limit - MinObstacleAreaWidth);
    edge.end = edge.start + MinObstacleAreaWidth;
}

std::array<Vec3, 4> rectangle(Vec3 center, float halfX, float halfY, float leftZ, float rightZ) {
    return {{{center.x - halfX,center.y - halfY,leftZ}, {center.x + halfX,center.y - halfY,rightZ},
             {center.x + halfX,center.y + halfY,rightZ}, {center.x - halfX,center.y + halfY,leftZ}}};
}

void raiseAreasWithInternalObstacles(NavMesh& mesh, ThreadPool& pool, const std::vector<AreaId>& ids) {
    std::vector<AreaId> remove;
    for (const auto id : ids) {
        pool.checkCancelled();
        auto& area = mesh[id];
        if (area.width() != GenerationStepSize || area.height() != GenerationStepSize || !area.hasNodes()) continue;
        std::optional<Obstacle> highest;
        bool stairNeighbor = false;
        for (const auto dir : directions) {
            auto current = Obstacle{.direction = dir};
            const auto& left = mesh.nodes[area.nodes[(dir + 3) % 4]];
            const auto& right = mesh.nodes[area.nodes[(dir + 2) % 4]];
            const bool stairs = std::ranges::any_of(area.outgoing[dir], [&](AreaId to) {
                return (mesh[to].attributes & Stairs) != 0;
            });
            stairNeighbor |= stairs;
            const auto& a = left.links[dir];
            const auto& b = right.links[dir];
            const bool blockedLeft = a.obstacleHeight > StepHeight;
            const bool blockedRight = b.obstacleHeight > StepHeight;
            if (!(blockedLeft && blockedRight) && !(stairs && (blockedLeft || blockedRight))) continue;
            current.start = std::min(a.obstacleStart, b.obstacleStart);
            current.end = current.start;
            current.leftZ = left.position.z + a.obstacleHeight;
            current.rightZ = right.position.z + b.obstacleHeight;
            current.height = std::max(current.leftZ, current.rightZ);
            if (!highest || current.height > highest->height) highest = current;
        }
        if (!highest) continue;
        if (stairNeighbor) {
            remove.push_back(id);
            continue;
        }
        auto& top = *highest;
        const auto dir = top.direction;
        adjustDistances(top, GenerationStepSize);
        const auto edge = edgeCorners[opposite(dir)];
        const auto center = (area.points[edge[0]] + area.points[edge[1]]) * 0.5f +
                            directionVector[dir] * ((top.start + top.end) * 0.5f);
        const auto half = (top.end - top.start) * 0.5f;
        if (dir == South || dir == West) std::swap(top.leftZ, top.rightZ);
        const auto points = rectangle(center, alongX(dir) ? half : GenerationStepSize * 0.5f,
                                      alongX(dir) ? GenerationStepSize * 0.5f : half,
                                      top.leftZ, top.rightZ);
        mesh.reshape(id, points);
        area.attributes = (area.attributes | ObstacleTop | NoMerge) & ~Jump;
        for (const auto side : directions) {
            if (alongX(side) == alongX(dir)) continue;
            while (!area.outgoing[side].empty()) {
                const auto adjacent = area.outgoing[side].back();
                mesh.disconnect(id, adjacent);
                mesh.disconnect(adjacent, id);
            }
        }
    }
    for (const auto id : remove) mesh.remove(id);
}

bool contains(const NavMesh& mesh, AreaId id, Vec3 point) {
    const auto& area = mesh[id];
    if (!area.overlaps(point)) return false;
    const auto z = area.getZ(point.x,point.y);
    if (point.z < z - StepHeight) return false;
    for (const auto neighbor : mesh.nearby({point,point})) {
        const auto& other = mesh[neighbor];
        if (neighbor == id || !other.overlaps(point)) continue;
        const auto otherZ = other.getZ(point.x,point.y);
        if (otherZ <= point.z && otherZ > z) return false;
    }
    return true;
}

bool contains(const NavArea& area, const NavArea& other) {
    return area.overlaps(other.points[NW]) && area.overlaps(other.points[SE]) &&
           area.points[NW].z <= other.points[NW].z && area.points[SE].z >= other.points[SE].z;
}

NodeLink obstacleFor(const NavMesh& mesh, AreaId dest, const NavNode& node, Direction dir, bool multi) {
    auto link = node.links[dir];
    if (!multi) return link;
    link.obstacleHeight = 0;
    if (contains(mesh, dest, node.position) || link.target == NodeId{}) return link;
    const auto& peak = mesh.nodes[link.target];
    const auto delta = peak.position.z - node.position.z;
    const auto next = peak.links[dir].target;
    if (delta <= StepHeight || next == NodeId{} ||
        (contains(mesh, dest, peak.position) &&
         std::abs(peak.position.z - mesh[dest].getZ(peak.position.x,peak.position.y)) < 2.0f))
        return link;
    const auto landing = mesh.nodes[next].position;
    if (contains(mesh, dest, landing) && std::abs(landing.z - node.position.z) <= StepHeight) {
        link.obstacleHeight = delta;
        link.obstacleStart = GenerationStepSize - MinObstacleAreaWidth / 2;
        link.obstacleEnd = GenerationStepSize + MinObstacleAreaWidth / 2;
    }
    return link;
}

bool createObstacleTopArea(NavMesh& mesh, AreaId from, AreaId to, Direction dir, bool multi) {
    const auto& source = mesh[from];
    const auto& dest = mesh[to];
    const auto portal = source.portal(dest, dir);
    if (portal.halfWidth <= 0) return false;

    Obstacle obstacle;
    float minimum = std::numeric_limits<float>::infinity();
    float firstHeight = 0, lastHeight = 0;
    const auto across = directions[(dir + 1) % 4];
    const auto finish = source.nodes[(dir + 1) % 4];
    for (auto id = source.nodes[dir]; id != NodeId{}; id = mesh.nodes[id].links[across].target) {
        const auto& node = mesh.nodes[id];
        const auto distance = node.position - portal.center;
        if (std::hypot(distance.x,distance.y) < portal.halfWidth + 1) {
            const auto link = obstacleFor(mesh, to, node, dir, multi);
            const auto height = link.obstacleHeight;
            minimum = std::min(minimum, height);
            obstacle.height = std::max(obstacle.height, height);
            obstacle.start = std::min(obstacle.start, link.obstacleStart);
            obstacle.end = std::max(obstacle.end, link.obstacleEnd);
            if (firstHeight == 0) {
                firstHeight = height;
                obstacle.leftZ = node.position.z;
            }
            lastHeight = height;
            obstacle.rightZ = node.position.z;
        }
        if (id == finish) break;
    }
    if (minimum <= StepHeight || obstacle.height <= StepHeight) return false;
    if (obstacle.height > std::max(firstHeight, lastHeight)) firstHeight = lastHeight = obstacle.height;
    obstacle.leftZ += firstHeight;
    obstacle.rightZ += lastHeight;
    if (dir == South || dir == West) std::swap(obstacle.leftZ, obstacle.rightZ);
    adjustDistances(obstacle, GenerationStepSize * (multi ? 2 : 1));
    const auto half = (obstacle.end - obstacle.start) * 0.5f;
    const auto center = portal.center + directionVector[dir] * (obstacle.start + half);
    NavArea top;
    top.points = rectangle(center, alongX(dir) ? half : portal.halfWidth,
                            alongX(dir) ? portal.halfWidth : half, obstacle.leftZ, obstacle.rightZ);
    top.attributes = source.attributes | NoMerge | ObstacleTop;
    const auto back = opposite(dir);
    const bool reciprocal = std::ranges::find(dest.outgoing[back], from) != dest.outgoing[back].end();
    const auto id = mesh.add(std::move(top));
    mesh.disconnect(from, to);
    mesh.connect(from, dir, id);
    mesh.connect(id, back, from);
    mesh.connect(id, dir, to);
    if (reciprocal) {
        mesh.disconnect(to, from);
        mesh.connect(to, back, id);
    }
    return true;
}

void removeOverlappingObstacleTops(NavMesh& mesh, const GenerationParams& params, ThreadPool& pool) {
    std::set<AreaId> remove;
    for (const auto id : mesh.areaIds()) {
        pool.checkCancelled();
        const auto& area = mesh[id];
        if (!(area.attributes & ObstacleTop)) continue;
        const auto bounds = area.bounds();
        if (std::min(area.normal().z, area.normal(true).z) < params.slopeLimit) remove.insert(id);
        for (const auto other : mesh.nearby(bounds)) {
            if (other <= id) continue;
            const auto& neighbor = mesh[other];
            if (!(neighbor.attributes & ObstacleTop) || !area.overlaps(neighbor)) continue;
            if (contains(area, neighbor)) remove.insert(other);
            else if (contains(neighbor, area)) remove.insert(id);
            else remove.insert(area.width() * area.height() > neighbor.width() * neighbor.height() ? other : id);
        }
    }
    for (const auto id : remove) mesh.remove(id);
}

} // namespace

void handleObstacleTopAreas(NavMesh& mesh, const GenerationParams& params, ThreadPool& pool) {
    const auto ids = mesh.areaIds();
    raiseAreasWithInternalObstacles(mesh, pool, ids);
    for (const auto from : ids) {
        pool.checkCancelled();
        if (!mesh[from].alive || (mesh[from].attributes & (Jump | ObstacleTop))) continue;
        for (const auto dir : directions) {
            const auto neighbors = mesh[from].outgoing[dir];
            for (const auto to : neighbors) {
                if (std::ranges::find(mesh[from].outgoing[dir], to) == mesh[from].outgoing[dir].end()) continue;
                if (mesh[to].attributes & (Jump | ObstacleTop)) continue;
                if (!createObstacleTopArea(mesh, from, to, dir, false))
                    createObstacleTopArea(mesh, from, to, dir, true);
            }
        }
    }
    removeOverlappingObstacleTops(mesh, params, pool);
}

} // namespace navfast
