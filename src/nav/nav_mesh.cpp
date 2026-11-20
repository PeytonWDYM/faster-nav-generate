#include "nav/nav_mesh.h"

#include <algorithm>
#include <limits>

namespace navfast {
namespace {

constexpr float CellSize = 300.0f;
int cell(float coordinate) { return static_cast<int>(std::floor(coordinate / CellSize)); }
std::uint64_t key(int x, int y) {
    return (std::uint64_t{static_cast<std::uint32_t>(x)} << 32) | static_cast<std::uint32_t>(y);
}

} // namespace

void NavMesh::index(AreaId id, bool insert) {
    const auto& box = (*this)[id].points;
    for (int y = cell(box[NW].y); y <= cell(box[SE].y); ++y) {
        for (int x = cell(box[NW].x); x <= cell(box[SE].x); ++x) {
            auto& bucket = grid_[key(x,y)];
            if (insert) bucket.push_back(id);
            else std::erase(bucket, id);
        }
    }
}

void NavMesh::assignNodes(AreaId id) {
    const auto& area = (*this)[id];
    if (!area.hasNodes()) return;
    auto rowEnd = area.nodes[NE];
    for (auto row = area.nodes[NW]; row != NodeId{} && row != area.nodes[SW]; row = nodes[row].links[South].target) {
        for (auto node = row; node != NodeId{} && node != rowEnd &&
             nodes[node].position.x < nodes[area.nodes[NE]].position.x; node = nodes[node].links[East].target)
            nodes[node].area = id;
        rowEnd = nodes[rowEnd].links[South].target;
    }
}

AreaId NavMesh::add(NavArea area) {
    const auto id = static_cast<AreaId>(areas_.size());
    areas_.push_back(std::move(area));
    index(id, true);
    assignNodes(id);
    return id;
}

void NavMesh::connect(AreaId from, Direction dir, AreaId to) {
    if (from == to || from == AreaId{} || to == AreaId{}) return;
    auto& list = (*this)[from].outgoing[dir];
    if (std::ranges::find(list, to) != list.end()) return;
    list.push_back(to);
    (*this)[to].incoming[opposite(dir)].push_back(from);
}

void NavMesh::disconnect(AreaId from, Direction dir, AreaId to) {
    std::erase((*this)[from].outgoing[dir], to);
    std::erase((*this)[to].incoming[opposite(dir)], from);
}

void NavMesh::disconnect(AreaId from, AreaId to) {
    for (const auto dir : directions) disconnect(from, dir, to);
}

void NavMesh::reshape(AreaId id, const std::array<Vec3, 4>& points) {
    index(id, false);
    (*this)[id].points = points;
    (*this)[id].nodes = {};
    index(id, true);
}

void NavMesh::remove(AreaId id) {
    auto& area = (*this)[id];
    for (const auto dir : directions) {
        while (!area.outgoing[dir].empty()) disconnect(id, dir, area.outgoing[dir].back());
        while (!area.incoming[dir].empty()) disconnect(area.incoming[dir].back(), opposite(dir), id);
    }
    index(id, false);
    area.alive = false;
}

void NavMesh::merge(AreaId keep, AreaId discarded, const std::array<NodeId, 4>& combinedNodes) {
    index(keep, false);
    auto& area = (*this)[keep];
    area.nodes = combinedNodes;
    for (const auto corner : corners) area.points[corner] = nodes[area.nodes[corner]].position;
    for (const auto dir : directions)
        for (const auto to : (*this)[discarded].outgoing[dir]) connect(keep, dir, to);
    for (const auto dir : directions) {
        auto sources = (*this)[discarded].incoming[opposite(dir)];
        for (const auto from : sources) {
            if (from == keep) continue;
            disconnect(from, discarded);
            disconnect(from, keep);
            connect(from, dir, keep);
        }
    }
    remove(discarded);
    index(keep, true);
    assignNodes(keep);
}

std::pair<AreaId, AreaId> NavMesh::split(AreaId id, bool splitY, float edge) {
    const auto& original = (*this)[id];
    const auto coord = [splitY](Vec3 p) { return splitY ? p.y : p.x; };
    if (edge <= coord(original.points[NW]) + 1 || edge >= coord(original.points[SE]) - 1) return {};
    const auto forward = splitY ? South : East;
    std::array<NavArea, 2> pieces;
    for (std::size_t i = 0; i < pieces.size(); ++i) {
        auto& area = pieces[i];
        area.points = original.points;
        area.nodes = original.nodes;
        area.attributes = original.attributes;
        const auto inside = i == 0 ? forward : opposite(forward);
        const auto [a,b] = edgeCorners[inside];
        for (const auto corner : {a,b}) {
            auto& point = area.points[corner];
            (splitY ? point.y : point.x) = edge;
            point.z = original.getZ(point.x, point.y);
        }
        if (area.hasNodes()) {
            while (area.nodes[a] != NodeId{} && area.nodes[b] != NodeId{} &&
                   !area.overlaps(nodes[area.nodes[a]].position, GenerationStepSize / 2)) {
                area.nodes[a] = nodes[area.nodes[a]].links[opposite(inside)].target;
                area.nodes[b] = nodes[area.nodes[b]].links[opposite(inside)].target;
            }
            if (area.nodes[a] == NodeId{} || area.nodes[b] == NodeId{}) area.nodes = {};
            else for (const auto corner : corners) area.points[corner].z = nodes[area.nodes[corner]].position.z;
        }
    }
    const auto a = add(std::move(pieces[0]));
    const auto b = add(std::move(pieces[1]));
    connect(a, forward, b);
    connect(b, opposite(forward), a);
    for (const auto piece : {a,b}) {
        const auto inner = piece == a ? forward : opposite(forward);
        for (const auto dir : directions) {
            if (dir == inner) continue;
            for (const auto to : (*this)[id].outgoing[dir])
                if ((*this)[piece].overlapAlong((*this)[to], dir) > 0) connect(piece, dir, to);
            for (const auto from : (*this)[id].incoming[dir])
                if ((*this)[piece].overlapAlong((*this)[from], dir) > 0) connect(from, opposite(dir), piece);
        }
    }
    remove(id);
    return {a,b};
}

std::vector<AreaId> NavMesh::areaIds() const {
    std::vector<AreaId> ids;
    for (std::size_t i = 1; i < areas_.size(); ++i)
        if (areas_[i].alive) ids.push_back(static_cast<AreaId>(i));
    return ids;
}

AreaId NavMesh::getNavArea(Vec3 pos, float beneathLimit) const {
    const auto found = grid_.find(key(cell(pos.x),cell(pos.y)));
    if (found == grid_.end()) return {};
    float highest = -std::numeric_limits<float>::infinity();
    AreaId id{};
    for (const auto candidate : found->second) {
        const auto& area = (*this)[candidate];
        if (!area.overlaps(pos)) continue;
        const auto z = area.getZ(pos.x, pos.y);
        if (z > pos.z + 5 || z < pos.z - beneathLimit || z <= highest) continue;
        highest = z;
        id = candidate;
    }
    return id;
}

std::vector<AreaId> NavMesh::nearby(Aabb extent) const {
    std::vector<AreaId> ids;
    for (int y = cell(extent.mins.y); y <= cell(extent.maxs.y); ++y) {
        for (int x = cell(extent.mins.x); x <= cell(extent.maxs.x); ++x) {
            const auto found = grid_.find(key(x,y));
            if (found != grid_.end()) ids.insert(ids.end(), found->second.begin(), found->second.end());
        }
    }
    std::ranges::sort(ids);
    ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
    return ids;
}

std::size_t NavMesh::connectionCount() const {
    std::size_t count = 0;
    for (const auto id : areaIds())
        for (const auto& side : (*this)[id].outgoing) count += side.size();
    return count;
}

NodeId NavMesh::closestEdgeNode(AreaId id, Direction dir, Vec3 pos) const {
    const auto& area = (*this)[id];
    const auto [a,b] = edgeCorners[dir];
    const auto forward = alongX(dir) ? South : East;
    float best = std::numeric_limits<float>::infinity();
    NodeId result{};
    for (auto node = area.nodes[a]; node != NodeId{}; node = nodes[node].links[forward].target) {
        const auto distance = (nodes[node].position - pos).lengthSqr();
        if (distance < best) {
            best = distance;
            result = node;
        }
        if (node == area.nodes[b] || !area.overlaps(nodes[node].position, GenerationStepSize / 2)) break;
    }
    return result;
}

} // namespace navfast
