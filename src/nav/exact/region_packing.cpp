#include "nav/exact/rectangles.h"

#include "job/thread_pool.h"
#include "nav/exact/clearance.h"

#include <algorithm>
#include <limits>
#include <numeric>
#include <optional>

namespace navfast::exact {
namespace {

struct Box {
    GridPoint min, max;
    int width() const { return max.x - min.x + 1; }
    int height() const { return max.y - min.y + 1; }
    std::size_t size() const { return static_cast<std::size_t>(width()) * height(); }
    explicit operator bool() const { return width() > 0 && height() > 0; }
};

Box bounds(const Heightfield& field, std::span<const CellRef> cells) {
    const auto point = field.point(cells.front());
    Box box{point, point};
    for (const auto cell : cells) {
        const auto [x, y] = field.point(cell);
        box.min.x = std::min(box.min.x, x);
        box.min.y = std::min(box.min.y, y);
        box.max.x = std::max(box.max.x, x);
        box.max.y = std::max(box.max.y, y);
    }
    return box;
}

struct RegionGrid {
    Box box;
    std::vector<CellRef> cells;

    CellRef& at(int x, int y) { return cells[index(x, y)]; }
    CellRef at(int x, int y) const { return cells[index(x, y)]; }

private:
    std::size_t index(int x, int y) const {
        return static_cast<std::size_t>(y - box.min.y) * box.width() + (x - box.min.x);
    }
};

struct Run { int x, height; };

// Monotone histograms find the best filled rectangle, including interior corners.
Box largest(const RegionGrid& grid, Box search, int limit, ThreadPool& pool) {
    std::vector<int> heights(static_cast<std::size_t>(search.width()) + 1);
    std::vector<Run> runs;
    runs.reserve(heights.size());
    Box result{{0, 0}, {-1, -1}};
    float best = 0;
    for (int y = search.min.y; y <= search.max.y; ++y) {
        if (y % 32 == 0) pool.checkCancelled();
        for (int i = 0; i < search.width(); ++i) {
            auto& height = heights[i];
            height = grid.at(search.min.x + i, y) ? std::min(height + 1, limit) : 0;
        }
        for (int i = 0; i <= search.width(); ++i) {
            const auto height = heights[i];
            int left = i;
            while (!runs.empty() && runs.back().height > height) {
                const auto run = runs.back();
                runs.pop_back();
                left = run.x;
                const auto width = std::min(limit, i - left);
                const auto aspect = static_cast<float>(std::max(width, run.height)) / std::min(width, run.height);
                const auto score = static_cast<float>(width * run.height) / (1 + 0.08f * std::max(0.0f, aspect - 3));
                if (score > best) {
                    best = score;
                    result = {{search.min.x + left, y - run.height + 1},
                              {search.min.x + left + width - 1, y}};
                }
            }
            if (height && (runs.empty() || runs.back().height < height)) runs.push_back({left, height});
        }
    }
    return result;
}

// Pick the cut orientation that divides the fewest remaining neighboring cells.
bool splitHorizontally(const RegionGrid& grid, Box search, Box filled) {
    int horizontal = 0, vertical = 0;
    for (const auto y : {filled.min.y, filled.max.y + 1}) {
        if (y <= search.min.y || y > search.max.y) continue;
        for (int x = search.min.x; x <= search.max.x; ++x) {
            if (x >= filled.min.x && x <= filled.max.x) continue;
            if (grid.at(x, y - 1) && grid.at(x, y)) ++horizontal;
        }
    }
    for (const auto x : {filled.min.x, filled.max.x + 1}) {
        if (x <= search.min.x || x > search.max.x) continue;
        for (int y = search.min.y; y <= search.max.y; ++y) {
            if (y >= filled.min.y && y <= filled.max.y) continue;
            if (grid.at(x - 1, y) && grid.at(x, y)) ++vertical;
        }
    }
    return horizontal <= vertical;
}

// Fit the two slopes on the complete Cartesian grid, then center the worst-case
// height error. Every cell must agree with the resulting plane at its corners.
std::optional<FloorPlane> fit(const RegionGrid& grid, const Heightfield& field, Box box,
                              const GenerationParams& params) {
    const auto& first = field.faces[field[grid.at(box.min.x, box.min.y)].face].plane;
    if (box.size() == 1) return first;
    const auto middle = (center(box.min) + center(box.max)) * 0.5f;
    float xx = 0, yy = 0, xz = 0, yz = 0;
    for (int y = box.min.y; y <= box.max.y; ++y) {
        for (int x = box.min.x; x <= box.max.x; ++x) {
            const auto d = center({x, y}, field[grid.at(x, y)].z) - middle;
            xx += d.x * d.x;
            yy += d.y * d.y;
            xz += d.x * (d.z - first.distance);
            yz += d.y * (d.z - first.distance);
        }
    }
    const auto dx = xx ? xz / xx : -first.normal.x / first.normal.z;
    const auto dy = yy ? yz / yy : -first.normal.y / first.normal.z;
    const auto n = Vec3{-dx, -dy, 1};
    const auto normal = n * (1.0f / n.length());
    if (normal.z < params.slopeLimit) return {};
    float min = std::numeric_limits<float>::infinity(), max = -min;
    for (int y = box.min.y; y <= box.max.y; ++y) {
        for (int x = box.min.x; x <= box.max.x; ++x) {
            const auto& cell = field[grid.at(x, y)];
            const auto actual = field.faces[cell.face].plane.normal;
            if (std::abs(actual.z - normal.z) > std::max(0.0f, params.slopeTolerance)) return {};
            const auto d = center({x, y}, cell.z) - middle;
            const auto z = d.z - dx * d.x - dy * d.y;
            const auto margin = (std::abs(dx + actual.x / actual.z) + std::abs(dy + actual.y / actual.z)) * CellSize * 0.5f;
            min = std::min(min, z - margin);
            max = std::max(max, z + margin);
        }
    }
    if ((max - min) * 0.5f > planeHeightTolerance(params)) return {};
    return FloorPlane{normal, normal.dot(middle + Vec3{0, 0, (min + max) * 0.5f})};
}

void appendRectangle(std::vector<Rectangle>& rectangles, const RegionGrid& grid, const Heightfield& field,
                     Box box, FloorPlane plane, std::uint32_t attributes, bool terrain, const GenerationParams& params) {
    if (terrain) {
        const auto fitted = fit(grid, field, box, params);
        if (!fitted) {
            auto first = box, second = box;
            if (box.width() >= box.height()) {
                first.max.x = std::midpoint(box.min.x, box.max.x);
                second.min.x = first.max.x + 1;
            } else {
                first.max.y = std::midpoint(box.min.y, box.max.y);
                second.min.y = first.max.y + 1;
            }
            appendRectangle(rectangles, grid, field, first, plane, attributes, terrain, params);
            appendRectangle(rectangles, grid, field, second, plane, attributes, terrain, params);
            return;
        }
        plane = *fitted;
    }
    auto& rect = rectangles.emplace_back(Rectangle{box.min, box.max, plane, attributes, {}});
    rect.cells.reserve(box.size());
    for (int y = box.min.y; y <= box.max.y; ++y)
        for (int x = box.min.x; x <= box.max.x; ++x) rect.cells.push_back(grid.at(x, y));
}

void packGrid(std::vector<Rectangle>& rectangles, const Heightfield& field, std::span<const CellRef> cells,
              Box box, const GenerationParams& params, int limit, ThreadPool& pool) {
    const auto& first = field[cells.front()];
    const auto& face = field.faces[first.face];
    if (!face.displacement && box.size() == cells.size() && box.width() <= limit && box.height() <= limit) {
        rectangles.push_back({box.min, box.max, face.plane, first.attributes, {cells.begin(), cells.end()}});
        return;
    }
    RegionGrid grid{box, std::vector<CellRef>(box.size())};
    for (const auto cell : cells) {
        const auto [x, y] = field.point(cell);
        grid.at(x, y) = cell;
    }
    std::vector<Box> pending{box};
    while (!pending.empty()) {
        pool.checkCancelled();
        const auto search = pending.back();
        pending.pop_back();
        const auto filled = largest(grid, search, limit, pool);
        if (!filled) continue;
        appendRectangle(rectangles, grid, field, filled, face.plane, first.attributes, face.displacement, params);

        const bool horizontal = splitHorizontally(grid, search, filled);
        const auto x0 = horizontal ? search.min.x : filled.min.x;
        const auto x1 = horizontal ? search.max.x : filled.max.x;
        const auto y0 = horizontal ? filled.min.y : search.min.y;
        const auto y1 = horizontal ? filled.max.y : search.max.y;
        for (const auto remaining : {Box{{x0, search.min.y}, {x1, filled.min.y - 1}},
                                     Box{{x0, filled.max.y + 1}, {x1, search.max.y}},
                                     Box{{search.min.x, y0}, {filled.min.x - 1, y1}},
                                     Box{{filled.max.x + 1, y0}, {search.max.x, y1}}})
            if (remaining) pending.push_back(remaining);
    }
}

} // namespace

void packRegion(std::vector<Rectangle>& rectangles, const Heightfield& field, std::span<const CellRef> cells,
                const GenerationParams& params, int maxCells, ThreadPool& pool) {
    const auto box = bounds(field, cells);
    if (box.size() <= cells.size() * 8) {
        packGrid(rectangles, field, cells, box, params, maxCells, pool);
        return;
    }
    // Avoid a dense bounding box for a sparse region such as a diagonal catwalk.
    std::map<std::uint32_t, std::vector<CellRef>> byTile;
    for (const auto cell : cells) byTile[cell.tile].push_back(cell);
    for (const auto& [tile, part] : byTile) {
        static_cast<void>(tile);
        packGrid(rectangles, field, part, bounds(field, part), params, maxCells, pool);
    }
}

} // namespace navfast::exact
