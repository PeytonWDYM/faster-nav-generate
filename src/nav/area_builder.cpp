#include "nav/area_builder.h"
#include "job/thread_pool.h"

#include <algorithm>
#include <string>

namespace navfast {
namespace {

bool checkObstacles(const NavNode& node, int width, int height, int x, int y) {
    if (width == 1 && height == 1) return true;
    return !(x > 0 && node.links[West].obstacleHeight > StepHeight) &&
           !(y > 0 && node.links[North].obstacleHeight > StepHeight) &&
           !(x < width - 1 && node.links[East].obstacleHeight > StepHeight) &&
           !(y < height - 1 && node.links[South].obstacleHeight > StepHeight);
}

bool cellClearance(const NavNode& node, int x, int y, bool crouch) {
    bool requiresCrouch = false;
    for (const auto corner : corners) {
        if (x == 0 && cornerVector[corner].x < 0) continue;
        if (y == 0 && cornerVector[corner].y < 0) continue;
        const auto& clearance = node.clearance[corner];
        if (clearance.blocked) return false;
        requiresCrouch |= clearance.crouch;
    }
    return requiresCrouch == crouch;
}

struct Cell {
    bool closed = false;
    bool validCrouch = true;
    int eastRun = 0;
    int southRun = 0;
};

std::array<NodeId, 4> testArea(const NavNodes& nodes, const std::vector<Cell>& cells,
                               NodeId id, int width, int height) {
    const auto& origin = nodes[id];
    if (origin.clearance[SE].blocked || !cells[static_cast<std::size_t>(id)].closed) return {};
    const bool crouch = origin.clearance[SE].crouch;
    const bool large = width > 1 || height > 1;
    const float planeDistance = origin.normal.dot(origin.position);
    const auto onPlane = [&](NodeId node) {
        return std::abs(origin.normal.dot(nodes[node].position) - planeDistance) <= 5.0f;
    };

    std::array<NodeId, 4> result{};
    result[NW] = id;
    auto row = id;
    for (int y = 0; y < height; ++y) {
        auto current = row;
        for (int x = 0; x < width; ++x) {
            const auto& node = nodes[current];
            const auto& cell = cells[static_cast<std::size_t>(current)];
            if (node.area != AreaId{} || !cell.closed || (crouch && !cell.validCrouch) ||
                !cellClearance(node, x, y, crouch) || !checkObstacles(node, width, height, x, y))
                return {};
            current = node.links[East].target;
            if (large && !onPlane(current)) return {};
        }
        if (!checkObstacles(nodes[current], width, height, width, y)) return {};
        if (y == 0) result[NE] = current;
        row = nodes[row].links[South].target;
        if (large && !onPlane(row)) return {};
    }
    result[SW] = row;
    for (int x = 0; x < width; ++x) {
        if (!checkObstacles(nodes[row], width, height, x, height)) return {};
        row = nodes[row].links[East].target;
        if (row == NodeId{} || (large && !onPlane(row))) return {};
    }
    if (!checkObstacles(nodes[row], width, height, width, height)) return {};
    result[SE] = row;
    return result;
}

NavArea buildArea(const NavNodes& nodes, const std::array<NodeId, 4>& ids) {
    NavArea area;
    area.nodes = ids;
    if (nodes[ids[NW]].clearance[SE].crouch) area.attributes |= Crouch;
    for (const auto corner : corners) {
        const auto& node = nodes[ids[corner]];
        area.points[corner] = node.position;
        if (node.links[directions[(corner + 1) % 4]].obstacleHeight > StepHeight ||
            node.links[directions[(corner + 2) % 4]].obstacleHeight > StepHeight) area.attributes |= NoMerge;
    }
    return area;
}

} // namespace

void createNavAreasFromNodes(NavMesh& mesh, const CollisionWorld& world, const GenerationParams& params,
                             ThreadPool& pool, const Progress& progress) {
    auto& nodes = mesh.nodes;
    std::vector<Cell> cells(nodes.size() + 1);
    pool.run(nodes.size(), [&](std::size_t i) {
        ++i;
        const auto id = static_cast<NodeId>(i);
        cells[i].closed = nodes.closedCell(id);
        if (cells[i].closed && !nodes[id].clearance[SE].blocked &&
            std::ranges::any_of(nodes[id].clearance, &NodeClearance::crouch)) {
            const auto pos = nodes[id].position;
            const Vec3 maxs{GenerationStepSize, GenerationStepSize, HumanCrouchHeight};
            cells[i].validCrouch = !world.traceHull(pos, pos + Vec3{0,0,JumpCrouchHeight}, {}, maxs,
                                                    GenerationTraceMask).allSolid;
        }
    });

    std::vector<NodeId> uncovered;
    uncovered.reserve(nodes.size());
    for (auto i = nodes.size(); i > 0; --i) {
        const auto id = static_cast<NodeId>(i);
        if (cells[i].closed && !nodes[id].clearance[SE].blocked) uncovered.push_back(id);
    }

    // Any valid rectangle needs these runs on its north and west edges.
    // East and south neighbors precede their source in descending X+Y order.
    auto byPosition = uncovered;
    std::ranges::sort(byPosition, std::ranges::greater{}, [&](NodeId id) {
        const auto p = nodes[id].position;
        return p.x + p.y;
    });
    for (const auto id : byPosition) {
        auto& cell = cells[static_cast<std::size_t>(id)];
        const auto& links = nodes[id].links;
        cell.eastRun = 1 + cells[static_cast<std::size_t>(links[East].target)].eastRun;
        cell.southRun = 1 + cells[static_cast<std::size_t>(links[South].target)].southRun;
    }

    const auto size = std::min(params.areaMaxSize, static_cast<int>(std::sqrt(static_cast<double>(uncovered.size()))) + 1);
    int width = size;
    int height = size;
    std::size_t areaCount = 0;
    while (width > 0 && height > 0 && !uncovered.empty()) {
        const bool enoughNodes = static_cast<std::size_t>(width) * static_cast<std::size_t>(height) <= uncovered.size();
        if (enoughNodes) {
            for (const auto id : uncovered) {
                pool.checkCancelled();
                const auto& cell = cells[static_cast<std::size_t>(id)];
                if (width > cell.eastRun || height > cell.southRun || nodes[id].area != AreaId{}) continue;
                const auto areaNodes = testArea(nodes, cells, id, width, height);
                if (areaNodes[NW] == NodeId{}) continue;
                mesh.add(buildArea(nodes, areaNodes));
                ++areaCount;
            }
            std::erase_if(uncovered, [&](NodeId id) { return nodes[id].area != AreaId{}; });
        }
        if (width >= height) --width;
        else --height;
        progress(1.0f - static_cast<float>(width + height) / static_cast<float>(size * 2),
                 std::to_string(areaCount) + " areas");
    }
    progress(1, std::to_string(areaCount) + " areas");
}

} // namespace navfast
