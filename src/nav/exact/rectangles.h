#pragma once

#include "nav/exact/heightfield.h"

namespace navfast::exact {

struct Rectangle {
    GridPoint min, max; // Inclusive cell coordinates.
    FloorPlane plane;
    std::uint32_t attributes;
    std::vector<CellRef> cells;
};

std::vector<Rectangle> partitionFloors(Heightfield& field, std::size_t reachable,
                                       const GenerationParams& params, ThreadPool& pool, const Progress& progress);
void packRegion(std::vector<Rectangle>& rectangles, const Heightfield& field, std::span<const CellRef> cells,
                const GenerationParams& params, int maxCells, ThreadPool& pool);
NavArea refineRectangle(const Rectangle& rect, const Heightfield& field, const CollisionWorld& world,
                        const GenerationParams& params);

} // namespace navfast::exact
