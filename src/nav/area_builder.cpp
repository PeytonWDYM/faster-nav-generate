#include "nav/area_builder.h"
#include "job/thread_pool.h"

#include <algorithm>
#include <string>

namespace navfast {
namespace {

bool edgeOpen(const NavNode& node, Direction direction) {
    return !(node.links[direction].obstacleHeight > StepHeight);
}

constexpr unsigned Standing = 1;
constexpr unsigned Crouching = 2;
constexpr unsigned cornerBit(Corner corner) { return 1u << corner; }

// Runs count consecutive cells along East. Only free changes when an area is added.
struct RowRuns {
    int free = 0;
    std::array<int, 4> clearance{};
    int eastOpen = 0;
    int westOpen = 0;
    int northOpen = 0;
    int southOpen = 0;
    int level = 0;
};

struct Cell {
    bool eligible = false;
    // The NW slot is the origin; NE and SW denote the north and west edges.
    std::array<unsigned char, 4> clearance{};
    int eastRun = 0;
    int southRun = 0;
    int planarRank = 0;
    RowRuns row;
};

Cell prepareCell(const NavNodes& nodes, NodeId id, const CollisionWorld& world) {
    const auto& node = nodes[id];
    if (node.clearance[SE].blocked || !nodes.closedCell(id)) return {};

    unsigned blocked = 0;
    unsigned crouch = 0;
    for (const auto corner : corners) {
        if (node.clearance[corner].blocked) blocked |= cornerBit(corner);
        if (node.clearance[corner].crouch) crouch |= cornerBit(corner);
    }
    bool validCrouch = true;
    if (crouch != 0) {
        const auto pos = node.position;
        const Vec3 maxs{GenerationStepSize, GenerationStepSize, HumanCrouchHeight};
        validCrouch = !world.traceHull(pos, pos + Vec3{0,0,JumpCrouchHeight}, {}, maxs,
                                     GenerationTraceMask).allSolid;
    }
    const auto clearance = [&](unsigned mask) -> unsigned char {
        if (blocked & mask) return 0;
        if (crouch & mask) return validCrouch ? Crouching : 0;
        return Standing;
    };

    Cell cell;
    cell.eligible = true;
    cell.clearance[NW] = clearance(cornerBit(SE));
    cell.clearance[NE] = clearance(cornerBit(SE) | cornerBit(SW));
    cell.clearance[SW] = clearance(cornerBit(SE) | cornerBit(NE));
    cell.clearance[SE] = clearance(cornerBit(NW) | cornerBit(NE) | cornerBit(SE) | cornerBit(SW));
    return cell;
}

unsigned clearanceRunIndex(int y, unsigned stance) {
    return (y == 0 ? 0u : 2u) + (stance == Crouching ? 1u : 0u);
}

RowRuns prepareRowRuns(const NavNodes& nodes, NodeId id, const Cell& cell, const RowRuns& eastRuns, int maxWidth) {
    const auto& node = nodes[id];
    const auto& east = nodes[node.links[East].target];
    RowRuns runs;
    runs.free = node.area == AreaId{} ? std::min(maxWidth, 1 + eastRuns.free) : 0;
    for (const auto stance : {Standing, Crouching}) {
        const auto top = clearanceRunIndex(0, stance);
        const auto interior = clearanceRunIndex(1, stance);
        if (cell.clearance[NE] == stance) runs.clearance[top] = 1 + eastRuns.clearance[top];
        if (cell.clearance[SE] == stance) runs.clearance[interior] = 1 + eastRuns.clearance[interior];
    }
    runs.eastOpen = edgeOpen(node, East) ? 1 + eastRuns.eastOpen : 0;
    // Shift West onto the preceding cell. North and South include the east border.
    runs.westOpen = edgeOpen(east, West) ? 1 + eastRuns.westOpen : 0;
    runs.northOpen = edgeOpen(node, North) && edgeOpen(east, North) ? 1 + eastRuns.northOpen : 0;
    runs.southOpen = edgeOpen(node, South) && edgeOpen(east, South) ? 1 + eastRuns.southOpen : 0;
    runs.level = node.position.z == east.position.z ? 1 + eastRuns.level : 0;
    return runs;
}

// The existing size order is (n,n), (n-1,n), (n-1,n-1), ... (1,1).
int widthAt(int rank) { return (rank + 1) / 2; }
int heightAt(int rank) { return rank / 2 + 1; }

// Resolve the plane limit once, when an origin first passes the row tests.
// Rows with varying height and sloped origin normals can require exact point tests.
int findPlanarRank(const NavNodes& nodes, std::span<const Cell> cells, NodeId id, int rank) {
    const auto& origin = nodes[id];
    const float distance = origin.normal.dot(origin.position);
    const bool verticalNormal = origin.normal.x == 0 && origin.normal.y == 0;
    const auto onPlane = [&](NodeId node) {
        return std::abs(origin.normal.dot(nodes[node].position) - distance) <= 5.0f;
    };
    const int bottom = heightAt(rank);
    auto row = id;
    for (int y = 0; rank > 1 && y <= heightAt(rank); ++y) {
        const int width = widthAt(rank);
        const int firstX = y == 0 ? 1 : 0;
        auto current = y == 0 ? nodes[row].links[East].target : row;
        // For a vertical normal, a level row has the same plane distance at every point.
        if (verticalNormal && y < bottom && cells[static_cast<std::size_t>(row)].row.level >= width &&
            onPlane(current)) {
            row = nodes[row].links[South].target;
            continue;
        }
        for (int x = firstX; x <= width; ++x) {
            if (!onPlane(current)) {
                // This point also fails every larger rectangle that contains it. Search
                // the remaining rows only for failures that constrain the rank further.
                rank = std::max({2, 2 * x - 1, 2 * y - 2}) - 1;
                break;
            }
            current = nodes[current].links[East].target;
        }
        row = nodes[row].links[South].target;
    }
    return rank;
}

std::array<NodeId, 4> testAreaRows(const NavNodes& nodes, std::span<const Cell> cells,
                                  NodeId id, int width, int height, unsigned stance) {
    std::array<NodeId, 4> result{};
    result[NW] = id;
    if (width == 1 && height == 1) {
        result[NE] = nodes[id].links[East].target;
        result[SW] = nodes[id].links[South].target;
        result[SE] = nodes[result[SW]].links[East].target;
        return result;
    }

    auto row = id;
    for (int y = 0; y < height; ++y) {
        const auto& cell = cells[static_cast<std::size_t>(row)];
        const auto& runs = cell.row;
        if (runs.free < width || cell.clearance[y == 0 ? NW : SW] != stance ||
            runs.eastOpen < width - 1 || runs.westOpen < width ||
            (y > 0 && runs.northOpen < width) || (y < height - 1 && runs.southOpen < width))
            return {};
        if (width > 1) {
            const auto east = nodes[row].links[East].target;
            const auto& eastRuns = cells[static_cast<std::size_t>(east)].row;
            if (eastRuns.clearance[clearanceRunIndex(y, stance)] < width - 1) return {};
        }
        row = nodes[row].links[South].target;
    }
    result[SW] = row;
    auto north = id;
    // The bottom border has obstacle checks but need not contain eligible cells.
    for (int x = 0; x < width; ++x) {
        const auto& node = nodes[row];
        if (!edgeOpen(node, North) || (x > 0 && !edgeOpen(node, West)) ||
            (x < width - 1 && !edgeOpen(node, East))) return {};
        row = node.links[East].target;
        north = nodes[north].links[East].target;
    }
    if (!edgeOpen(nodes[row], North) || !edgeOpen(nodes[row], West)) return {};
    result[NE] = north;
    result[SE] = row;
    return result;
}

void updateCoverage(const NavNodes& nodes, std::span<Cell> cells, NodeId row, int width, int height) {
    for (int y = 0; y < height; ++y) {
        auto current = row;
        // NavMesh::add owns coverage. Update the cache from its assigned row prefix.
        for (int x = 0; x < width && nodes[current].area != AreaId{}; ++x) {
            cells[static_cast<std::size_t>(current)].row.free = 0;
            current = nodes[current].links[East].target;
        }
        if (current != row) {
            current = row;
            for (int distance = 1;; ++distance) {
                const auto west = nodes[current].links[West].target;
                auto& runs = cells[static_cast<std::size_t>(west)].row;
                if (runs.free <= distance || nodes[west].links[East].target != current) break;
                runs.free = distance;
                current = west;
            }
        }
        row = nodes[row].links[South].target;
    }
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
    constexpr std::size_t batch = 64;
    pool.run((nodes.size() + batch - 1) / batch, [&](std::size_t chunk) {
        const auto end = std::min((chunk + 1) * batch, nodes.size());
        for (auto i = chunk * batch + 1; i <= end; ++i)
            cells[i] = prepareCell(nodes, static_cast<NodeId>(i), world);
    });

    std::vector<NodeId> uncovered;
    uncovered.reserve(nodes.size());
    for (auto i = nodes.size(); i > 0; --i) {
        const auto id = static_cast<NodeId>(i);
        if (cells[i].eligible) uncovered.push_back(id);
    }

    const auto size = std::min(params.areaMaxSize,
                               static_cast<int>(std::sqrt(static_cast<double>(uncovered.size()))) + 1);

    // Preserve the original candidate limits on the north and west edges.
    auto byPosition = uncovered;
    std::ranges::sort(byPosition, std::ranges::greater{}, [&](NodeId id) {
        const auto p = nodes[id].position;
        return p.x + p.y;
    });
    int maxRank = 0;
    for (const auto id : byPosition) {
        auto& cell = cells[static_cast<std::size_t>(id)];
        const auto& links = nodes[id].links;
        cell.eastRun = 1 + cells[static_cast<std::size_t>(links[East].target)].eastRun;
        cell.southRun = 1 + cells[static_cast<std::size_t>(links[South].target)].southRun;
        maxRank = std::max(maxRank, std::min(2 * std::min(cell.eastRun, size),
                                            2 * std::min(cell.southRun, size) - 1));
    }

    // East advances on the sampling grid, and eligible cells have reciprocal
    // East/West links. Build row runs from the end of each eligible chain.
    for (const auto id : uncovered) {
        if (cells[static_cast<std::size_t>(nodes[id].links[East].target)].eligible) continue;
        RowRuns eastRuns;
        for (auto current = id; cells[static_cast<std::size_t>(current)].eligible;) {
            auto& cell = cells[static_cast<std::size_t>(current)];
            cell.row = eastRuns = prepareRowRuns(nodes, current, cell, eastRuns, size);
            const auto west = nodes[current].links[West].target;
            if (nodes[west].links[East].target != current) break;
            current = west;
        }
    }

    std::size_t areaCount = 0;
    for (int rank = maxRank; rank > 0 && !uncovered.empty(); --rank) {
        const auto width = widthAt(rank);
        const auto height = heightAt(rank);
        const bool enoughNodes = static_cast<std::size_t>(width) * static_cast<std::size_t>(height) <= uncovered.size();
        if (enoughNodes) {
            const auto previousAreaCount = areaCount;
            for (const auto id : uncovered) {
                pool.checkCancelled();
                auto& cell = cells[static_cast<std::size_t>(id)];
                const unsigned stance = cell.clearance[NW];
                if (width > cell.eastRun || width > cell.row.free || height > cell.southRun ||
                    stance == 0 || (cell.planarRank != 0 && rank > cell.planarRank)) continue;
                const auto areaNodes = testAreaRows(nodes, cells, id, width, height, stance);
                if (areaNodes[NW] == NodeId{}) continue;
                if (rank > 1 && cell.planarRank == 0) {
                    cell.planarRank = findPlanarRank(nodes, cells, id, rank);
                    if (rank > cell.planarRank) continue;
                }
                mesh.add(buildArea(nodes, areaNodes));
                updateCoverage(nodes, cells, id, width, height);
                ++areaCount;
            }
            if (areaCount != previousAreaCount)
                std::erase_if(uncovered, [&](NodeId id) { return nodes[id].area != AreaId{}; });
        }
        progress(1.0f - static_cast<float>(rank) / static_cast<float>(size * 2),
                 std::to_string(areaCount) + " areas");
    }
    progress(1, std::to_string(areaCount) + " areas");
}

} // namespace navfast
