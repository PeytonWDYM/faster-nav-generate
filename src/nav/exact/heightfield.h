#pragma once

#include "nav/nav_mesh.h"
#include "world/collision_world.h"

#include <compare>
#include <map>

namespace navfast { class ThreadPool; }

namespace navfast::exact {

inline constexpr float CellSize = 8;
inline constexpr int TileWidth = 32;
inline constexpr int TileColumns = TileWidth * TileWidth;

struct GridPoint {
    int x, y;
    auto operator<=>(const GridPoint&) const = default;
};

struct CellRef {
    std::uint32_t tile = 0, offset = 0;
    explicit operator bool() const { return tile != 0; }
    bool operator==(const CellRef&) const = default;
};

struct Cell {
    float z;
    float lift = 0;
    std::uint32_t face;
    std::uint32_t attributes = 0;
    std::uint32_t region = 0;
    AreaId area{};
    std::uint16_t column;
    bool reachable = false;
    std::array<CellRef, 4> links{};
};

struct Tile {
    GridPoint origin;
    std::vector<std::uint32_t> faces;
    std::vector<Cell> cells;
    std::vector<std::uint32_t> columns;
    std::array<std::uint32_t, 4> neighbors{};
    bool rasterized = false;
};

GridPoint gridPoint(Vec3 point);
GridPoint moved(GridPoint point, Direction dir, int distance = 1);
Vec3 center(GridPoint point, float z = 0);

class Heightfield {
public:
    Heightfield(std::vector<FloorFace> faces, ThreadPool& pool);

    void rasterize(std::uint32_t tile, const CollisionWorld& world, const GenerationParams& params);
    std::uint32_t tileAt(GridPoint point) const;
    std::span<const Cell> column(std::uint32_t tile, GridPoint point) const;
    CellRef ref(std::uint32_t tile, const Cell& cell) const;
    GridPoint point(CellRef cell) const;
    Vec3 position(CellRef cell) const { return center(point(cell), (*this)[cell].z); }
    Cell& operator[](CellRef cell) { return tiles[cell.tile].cells[cell.offset]; }
    const Cell& operator[](CellRef cell) const { return tiles[cell.tile].cells[cell.offset]; }

    std::vector<FloorFace> faces;
    std::vector<Tile> tiles{1}; // Index zero is the missing-tile sentinel.

private:
    std::map<GridPoint, std::uint32_t> tileIds_;
};

} // namespace navfast::exact
