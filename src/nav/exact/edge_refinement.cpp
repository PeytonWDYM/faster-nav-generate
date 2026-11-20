#include "nav/exact/rectangles.h"

#include "nav/exact/clearance.h"

#include <algorithm>

namespace navfast::exact {
namespace {

struct Edge {
    GridPoint start;
    int count;
    Direction along;
};

Edge edgeCells(const Rectangle& rect, Direction dir) {
    if (alongX(dir)) return {{dir == East ? rect.max.x : rect.min.x, rect.min.y},
                             rect.max.y - rect.min.y + 1, South};
    return {{rect.min.x, dir == South ? rect.max.y : rect.min.y},
             rect.max.x - rect.min.x + 1, East};
}

bool hasSharedEdge(const Rectangle& rect, const Heightfield& field, Direction dir) {
    const auto edge = edgeCells(rect, dir);
    for (int i = 0; i < edge.count; ++i) {
        const auto p = moved(moved(edge.start, edge.along, i), dir);
        const auto tile = field.tileAt(p);
        if (!tile || !field.tiles[tile].rasterized) continue;
        const auto z = rect.plane.heightAt(center(p));
        if (std::ranges::any_of(field.column(tile, p), [z](const Cell& c) {
                return c.reachable && std::abs(c.z - z) <= 2;
            })) return true;
    }
    return false;
}

bool supported(const CollisionWorld& world, FloorPlane plane, Vec3 point, std::uint32_t attributes,
               const GenerationParams& params) {
    point.z = plane.heightAt(point);
    const float tolerance = planeHeightTolerance(params) + 0.25f;
    const Vec3 search{0, 0, tolerance};
    const auto floor = world.traceHull(point + search, point - search,
                                       {-FloorSkin, -FloorSkin, 0}, {FloorSkin, FloorSkin, FloorSkin},
                                       GenerationTraceMask);
    if (floor.startSolid || !floor.didHit() || floor.normal.z < params.slopeLimit ||
        (floor.surfaceFlags & (source::SURF_SKY | source::SURF_SKY2D))) return false;
    const auto clearance = floorClearance(world, floor.endpos, floor.normal, params.slopeLimit).fit;
    return clearance == Fit::Standing || ((attributes & Crouch) && clearance == Fit::Crouching);
}

// Test a whole side at half-cell intervals, including the two current corners.
bool edgeFits(const CollisionWorld& world, FloorPlane plane, Direction dir, float coordinate,
              Aabb extent, std::uint32_t attributes, const GenerationParams& params) {
    const auto a = alongX(dir) ? extent.mins.y : extent.mins.x;
    const auto b = alongX(dir) ? extent.maxs.y : extent.maxs.x;
    const auto intervals = std::max(1, static_cast<int>(std::ceil((b - a) / (CellSize * 0.5f))));
    for (int i = 0; i <= intervals; ++i) {
        const auto t = std::lerp(a, b, static_cast<float>(i) / static_cast<float>(intervals));
        const auto p = alongX(dir) ? Vec3{coordinate, t, 0} : Vec3{t, coordinate, 0};
        if (!supported(world, plane, p, attributes, params)) return false;
    }
    return true;
}

float refineEdge(const CollisionWorld& world, FloorPlane plane, Direction dir, float coordinate,
                 float reach, Aabb extent, std::uint32_t attributes, const GenerationParams& params) {
    const float sign = dir == North || dir == West ? -1.0f : 1.0f;
    const auto fits = [&](float offset) {
        return edgeFits(world, plane, dir, coordinate + offset * sign, extent, attributes, params);
    };
    float near = 0, far = std::min(reach, CellSize * 0.5f);
    if (fits(far)) {
        near = far;
        far = reach;
        if (near == far || fits(far)) return coordinate + far * sign;
    }
    for (int i = 0; i < 5; ++i) {
        const auto middle = (near + far) * 0.5f;
        if (fits(middle)) near = middle;
        else far = middle;
    }
    return coordinate + near * sign;
}

} // namespace

NavArea refineRectangle(const Rectangle& rect, const Heightfield& field, const CollisionWorld& world,
                        const GenerationParams& params) {
    constexpr float halfCell = CellSize * 0.5f;
    Aabb extent{center(rect.min), center(rect.max)};
    const auto [min, max] = extent;
    std::array<float*, 4> side{&extent.mins.y, &extent.maxs.x, &extent.maxs.y, &extent.mins.x};
    // Preserve shared portals if a side also borders an exposed stretch.
    std::array<bool, 4> shared{};
    for (const auto dir : directions) {
        shared[dir] = hasSharedEdge(rect, field, dir);
        if (shared[dir])
            *side[dir] += dir == North || dir == West ? -halfCell : halfCell;
    }
    const auto maxSize = GenerationStepSize * params.areaMaxSize;
    for (const auto dir : directions) {
        if (shared[dir]) continue;
        const auto centerSpan = alongX(dir) ? max.x - min.x : max.y - min.y;
        const auto reach = std::min(CellSize, (maxSize - centerSpan) * 0.5f);
        *side[dir] = refineEdge(world, rect.plane, dir, *side[dir], reach, extent, rect.attributes, params);
    }

    const auto [nw, se] = extent;
    NavArea area;
    area.points = {nw, {se.x, nw.y, 0}, se, {nw.x, se.y, 0}};
    for (auto& p : area.points) p.z = rect.plane.heightAt(p);
    area.attributes = rect.attributes;
    return area;
}

} // namespace navfast::exact
