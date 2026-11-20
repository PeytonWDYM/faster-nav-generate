#include "nav/exact/heightfield.h"

#include "job/thread_pool.h"
#include "nav/exact/clearance.h"
#include "nav/sampling.h"

#include <algorithm>
#include <limits>
#include <tuple>

namespace navfast::exact {
namespace {

int tileOrigin(int cell) {
    return cell & ~(TileWidth - 1);
}

int columnIndex(const Tile& tile, GridPoint point) {
    return (point.y - tile.origin.y) * TileWidth + point.x - tile.origin.x;
}

// Convert face extents to the range of cell centers the polygon can cover.
int firstCenter(float coordinate) {
    return static_cast<int>(std::ceil(coordinate / CellSize - 0.5f - 0.0001f));
}

int lastCenter(float coordinate) {
    return static_cast<int>(std::floor(coordinate / CellSize - 0.5f + 0.0001f));
}

void rasterizeFace(const FloorFace& face, std::uint32_t index, const Tile& tile, std::vector<Cell>& cells) {
    for (int row = 0; row < TileWidth; ++row) {
        const auto y = center({0, tile.origin.y + row}).y;
        float left = std::numeric_limits<float>::infinity(), right = -left;
        auto previous = face.points.back();
        for (const auto vertex : face.points) {
            auto a = previous, b = vertex;
            if (a.y > b.y) std::swap(a, b);
            if (a.y <= y && y <= b.y) {
                const auto x = [&](Vec3 p) { left = std::min(left, p.x); right = std::max(right, p.x); };
                if (a.y == b.y) {
                    x(a);
                    x(b);
                } else {
                    x(a + (b - a) * ((y - a.y) / (b.y - a.y)));
                }
            }
            previous = vertex;
        }
        if (left > right) continue;
        const auto from = std::max(firstCenter(left), tile.origin.x);
        const auto to = std::min(lastCenter(right), tile.origin.x + TileWidth - 1);
        for (int x = from; x <= to; ++x) {
            const auto p = center({x, tile.origin.y + row});
            cells.push_back({.z = face.plane.heightAt(p), .face = index,
                             .column = static_cast<std::uint16_t>(columnIndex(tile, {x, tile.origin.y + row}))});
        }
    }
}

bool coveredByDisplacement(const CollisionWorld& world, Vec3 p, int distance) {
    p.z += FloorSkin;
    const auto up = traceNav(world, p, p + Vec3{0, 0, static_cast<float>(distance)});
    if (up.fraction == 0) return false;
    const auto down = traceNav(world, up.endpos, p);
    return down.hitDisplacement && down.endpos.z > p.z + StepHeight;
}

} // namespace

GridPoint gridPoint(Vec3 p) {
    return {static_cast<int>(std::floor(p.x / CellSize)), static_cast<int>(std::floor(p.y / CellSize))};
}

GridPoint moved(GridPoint p, Direction dir, int distance) {
    p.x += static_cast<int>(directionVector[dir].x) * distance;
    p.y += static_cast<int>(directionVector[dir].y) * distance;
    return p;
}

Vec3 center(GridPoint p, float z) {
    return {(static_cast<float>(p.x) + 0.5f) * CellSize, (static_cast<float>(p.y) + 0.5f) * CellSize, z};
}

Heightfield::Heightfield(std::vector<FloorFace> surfaces, ThreadPool& pool) : faces(std::move(surfaces)) {
    for (std::uint32_t i = 0; i < faces.size(); ++i) {
        pool.checkCancelled();
        const auto& points = faces[i].points;
        auto min = points.front(), max = min;
        for (const auto p : points) {
            min.x = std::min(min.x, p.x);
            min.y = std::min(min.y, p.y);
            max.x = std::max(max.x, p.x);
            max.y = std::max(max.y, p.y);
        }
        for (int y = tileOrigin(firstCenter(min.y)); y <= tileOrigin(lastCenter(max.y)); y += TileWidth) {
            for (int x = tileOrigin(firstCenter(min.x)); x <= tileOrigin(lastCenter(max.x)); x += TileWidth) {
                const auto [it, inserted] = tileIds_.try_emplace(GridPoint{x, y}, static_cast<std::uint32_t>(tiles.size()));
                if (inserted) tiles.push_back(Tile{.origin = {x, y}, .faces = {}, .cells = {}, .columns = {}});
                tiles[it->second].faces.push_back(i);
            }
        }
    }
    for (auto& tile : std::span(tiles).subspan(1))
        for (const auto dir : directions)
            tile.neighbors[dir] = tileAt(moved(tile.origin, dir, TileWidth));
}

void Heightfield::rasterize(std::uint32_t id, const CollisionWorld& world, const GenerationParams& params) {
    auto& tile = tiles[id];
    tile.columns.resize(TileColumns + 1);
    std::vector<Cell> candidates;
    for (const auto i : tile.faces) rasterizeFace(faces[i], i, tile, candidates);
    std::ranges::sort(candidates, {}, [](const Cell& c) { return std::tuple{c.column, -c.z, c.face}; });
    auto it = candidates.begin();
    for (int i = 0; i < TileColumns; ++i) {
        tile.columns[i] = static_cast<std::uint32_t>(tile.cells.size());
        float previousZ = std::numeric_limits<float>::infinity();
        float displacementZ = -previousZ;
        for (; it != candidates.end() && it->column == i; ++it) {
            auto cell = *it;
            const auto& face = faces[cell.face];
            if (face.displacement) displacementZ = std::max(displacementZ, cell.z);
            if (previousZ - cell.z < HumanCrouchHeight) continue;
            const auto p = center({tile.origin.x + i % TileWidth, tile.origin.y + i / TileWidth}, cell.z);
            const auto clearance = floorClearance(world, p, face.plane.normal, params.slopeLimit);
            if (clearance.fit == Fit::Blocked) continue;
            if (!face.displacement && displacementZ > cell.z + StepHeight && params.displacementTest > 0 &&
                coveredByDisplacement(world, p, params.displacementTest)) continue;
            cell.attributes = clearance.fit == Fit::Crouching ? Crouch : 0;
            cell.lift = clearance.lift;
            tile.cells.push_back(cell);
            previousZ = cell.z;
        }
    }
    tile.columns.back() = static_cast<std::uint32_t>(tile.cells.size());
    tile.rasterized = true;
}

std::uint32_t Heightfield::tileAt(GridPoint p) const {
    const auto it = tileIds_.find({tileOrigin(p.x), tileOrigin(p.y)});
    return it == tileIds_.end() ? 0 : it->second;
}

std::span<const Cell> Heightfield::column(std::uint32_t id, GridPoint p) const {
    const auto& tile = tiles[id];
    const auto i = columnIndex(tile, p);
    return std::span(tile.cells).subspan(tile.columns[i], tile.columns[i + 1] - tile.columns[i]);
}

CellRef Heightfield::ref(std::uint32_t tile, const Cell& cell) const {
    return {tile, static_cast<std::uint32_t>(&cell - tiles[tile].cells.data())};
}

GridPoint Heightfield::point(CellRef ref) const {
    const auto& origin = tiles[ref.tile].origin;
    const auto column = (*this)[ref].column;
    return {origin.x + column % TileWidth, origin.y + column / TileWidth};
}

} // namespace navfast::exact
