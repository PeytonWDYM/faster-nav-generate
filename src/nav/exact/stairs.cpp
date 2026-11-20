#include "nav/exact/stairs.h"

#include "job/thread_pool.h"
#include "nav/exact/clearance.h"

#include <algorithm>
#include <map>
#include <optional>

namespace navfast::exact {
namespace {

struct Span {
    float min, max;
    float size() const { return max - min; }
};

Span overlap(Span a, Span b) {
    return {std::max(a.min, b.min), std::min(a.max, b.max)};
}

Span extent(const NavArea& area, bool x) {
    return x ? Span{area.points[NW].x, area.points[SE].x} : Span{area.points[NW].y, area.points[SE].y};
}

int axis(GridPoint p, bool x) { return x ? p.x : p.y; }
float axis(Vec3 p, bool x) { return x ? p.x : p.y; }

Vec3 point(bool x, float along, float across, float z) {
    return x ? Vec3{along, across, z} : Vec3{across, along, z};
}

struct Run {
    std::vector<std::size_t> treads;
    Direction uphill;
    Span width;
    int firstColumn, lastColumn;
};

using EdgeIndex = std::array<std::map<int, std::vector<std::size_t>>, 4>;

bool positive(Direction dir) { return dir == East || dir == South; }

int lowEdge(const Rectangle& rect, Direction dir) {
    return axis(positive(dir) ? rect.min : rect.max, alongX(dir));
}

// Follow adjacent cell rows uphill, narrowing the width to the span shared by every tread.
Run findRun(std::size_t first, Direction dir, std::span<const Rectangle> rectangles,
            std::span<const NavArea> areas, const EdgeIndex& edges, const std::vector<bool>& used,
            float sizeLimit) {
    const bool x = alongX(dir);
    const auto firstSpan = extent(areas[first], x);
    Run run{{first}, dir, extent(areas[first], !x),
            axis(rectangles[first].min, !x), axis(rectangles[first].max, !x)};
    while (true) {
        const auto last = run.treads.back();
        const auto nextRow = lowEdge(rectangles[last], opposite(dir)) + (positive(dir) ? 1 : -1);
        const auto found = edges[dir].find(nextRow);
        if (found == edges[dir].end()) break;

        auto best = areas.size();
        Span bestWidth{};
        for (const auto next : found->second) {
            if (used[next] || areas[next].attributes != areas[first].attributes) continue;
            const auto rise = areas[next].center().z - areas[last].center().z;
            if (rise <= FloorSkin || rise > StepHeight) continue;
            const auto length = extent(areas[next], x);
            if (std::max(length.max, firstSpan.max) - std::min(length.min, firstSpan.min) > sizeLimit) continue;
            if (axis(rectangles[next].min, !x) > run.lastColumn ||
                axis(rectangles[next].max, !x) < run.firstColumn) continue;
            const auto width = overlap(run.width, extent(areas[next], !x));
            if (width.size() < 1 || width.size() <= bestWidth.size()) continue;
            best = next;
            bestWidth = width;
        }
        if (best == areas.size()) break;
        run.treads.push_back(best);
        run.width = bestWidth;
        run.firstColumn = std::max(run.firstColumn, axis(rectangles[best].min, !x));
        run.lastColumn = std::min(run.lastColumn, axis(rectangles[best].max, !x));
    }
    return run;
}

TraceResult sweep(const CollisionWorld& world, Vec3 from, Vec3 to) {
    constexpr Vec3 mins{-HalfHumanWidth, -HalfHumanWidth, 0};
    constexpr Vec3 maxs{HalfHumanWidth, HalfHumanWidth, HumanHeight};
    return world.traceHull(from, to, mins, maxs, GenerationTraceMask);
}

std::optional<Vec3> land(const CollisionWorld& world, Vec3 from, float distance, float slopeLimit) {
    const auto ground = sweep(world, from, from - Vec3{0, 0, distance});
    if (ground.startSolid || !ground.didHit() || ground.normal.z < slopeLimit ||
        (ground.surfaceFlags & (source::SURF_SKY | source::SURF_SKY2D))) return {};
    return ground.endpos;
}

// Walk straight, or take the raised StepMove path if a riser blocks the sweep.
std::optional<Vec3> stepMove(const CollisionWorld& world, Vec3 from, Vec3 to, float slopeLimit) {
    from.z += FloorSkin;
    to.z = from.z;
    if (!sweep(world, from, to).didHit())
        return land(world, to, StepHeight + FloorSkin, slopeLimit);

    const auto up = sweep(world, from, from + Vec3{0, 0, StepHeight});
    if (up.startSolid) return {};
    to.z = up.endpos.z;
    if (sweep(world, up.endpos, to).didHit()) return {};
    return land(world, to, StepHeight + FloorSkin, slopeLimit);
}

// Walk the lane in short moves so each riser gets its own ground and ceiling checks.
bool walkLane(const CollisionWorld& world, Vec3 from, Vec3 to, float slopeLimit) {
    constexpr float moveLength = CellSize * 0.5f;
    const auto clearance = floorClearance(world, from, {0, 0, 1}, slopeLimit);
    if (clearance.fit != Fit::Standing) return false;
    const auto lift = clearance.lift + FloorSkin;
    auto ground = land(world, from + Vec3{0, 0, lift}, lift + FloorSkin, slopeLimit);
    if (!ground) return false;

    const auto moves = static_cast<int>(std::ceil(std::hypot(to.x - from.x, to.y - from.y) / moveLength));
    const auto sign = to.z > from.z ? 1.0f : -1.0f;
    for (int i = 1; i <= moves; ++i) {
        const auto t = static_cast<float>(i) / moves;
        const auto next = stepMove(world, *ground, from + (to - from) * t, slopeLimit);
        if (!next || sign * (next->z - ground->z) < -FloorSkin) return false;
        ground = next;
    }
    return std::abs(ground->z - to.z) <= StepHeight + FloorSkin;
}

bool traversable(const Run& run, std::span<const NavArea> areas, const CollisionWorld& world, float slopeLimit) {
    const bool x = alongX(run.uphill);
    const bool forward = positive(run.uphill);
    const auto& bottom = areas[run.treads.front()];
    const auto& top = areas[run.treads.back()];
    const auto a = extent(bottom, x), b = extent(top, x);
    const auto insetA = std::min(CellSize * 0.5f, a.size() * 0.5f);
    const auto insetB = std::min(CellSize * 0.5f, b.size() * 0.5f);
    const auto start = forward ? a.min + insetA : a.max - insetA;
    const auto end = forward ? b.max - insetB : b.min + insetB;

    // Put a lane near each edge and at the center, with extra lanes for wide stairs.
    const auto inset = std::min(CellSize * 0.5f, run.width.size() * 0.25f);
    const auto pairs = std::max(1, static_cast<int>(std::ceil(run.width.size() / (4 * HalfHumanWidth))));
    for (int i = 0; i <= pairs * 2; ++i) {
        const auto across = std::lerp(run.width.min + inset, run.width.max - inset,
                                      static_cast<float>(i) / (pairs * 2));
        auto from = point(x, start, across, 0);
        auto to = point(x, end, across, 0);
        from.z = bottom.getZ(from.x, from.y);
        to.z = top.getZ(to.x, to.y);
        if (!walkLane(world, from, to, slopeLimit) || !walkLane(world, to, from, slopeLimit)) return false;
    }
    return true;
}

NavArea stairArea(const Run& run, std::span<const NavArea> areas) {
    const auto& first = areas[run.treads.front()];
    const auto& last = areas[run.treads.back()];
    const bool x = alongX(run.uphill);
    const bool forward = positive(run.uphill);
    const auto low = extent(forward ? first : last, x).min;
    const auto high = extent(forward ? last : first, x).max;
    const auto nw = point(x, low, run.width.min, 0);
    const auto se = point(x, high, run.width.max, 0);
    const auto middle = (run.width.min + run.width.max) * 0.5f;
    const auto lowEdge = point(x, low, middle, 0);
    const auto highEdge = point(x, high, middle, 0);
    const auto lowZ = (forward ? first : last).getZ(lowEdge.x, lowEdge.y);
    const auto highZ = (forward ? last : first).getZ(highEdge.x, highEdge.y);
    NavArea area;
    area.points = {{{nw.x, nw.y, lowZ}, {se.x, nw.y, x ? highZ : lowZ},
                    {se.x, se.y, highZ}, {nw.x, se.y, x ? lowZ : highZ}}};
    area.attributes = first.attributes | Stairs;
    return area;
}

// Keep each side strip's original plane. The cell links will become the portals.
void emitRun(const Run& run, NavMesh& mesh, Heightfield& field,
             std::span<const Rectangle> rectangles, std::span<const NavArea> areas) {
    const auto id = mesh.add(stairArea(run, areas));
    const bool crossX = !alongX(run.uphill);
    for (const auto tread : run.treads) {
        std::array<AreaId, 2> strips{};
        for (const auto cell : rectangles[tread].cells) {
            const auto p = axis(field.position(cell), crossX);
            if (p >= run.width.min && p <= run.width.max) {
                field[cell].area = id;
                continue;
            }
            const bool low = p < run.width.min;
            auto& stripId = strips[low ? 0 : 1];
            if (stripId == AreaId{}) {
                auto strip = areas[tread];
                for (auto& corner : strip.points) {
                    auto& across = crossX ? corner.x : corner.y;
                    across = low ? std::min(across, run.width.min) : std::max(across, run.width.max);
                    corner.z = areas[tread].getZ(corner.x, corner.y);
                }
                stripId = mesh.add(std::move(strip));
            }
            field[cell].area = stripId;
        }
    }
}

} // namespace

std::size_t mergeStairRuns(NavMesh& mesh, Heightfield& field, std::span<const Rectangle> rectangles,
                           std::span<const NavArea> areas, const CollisionWorld& world,
                           const GenerationParams& params, ThreadPool& pool) {
    EdgeIndex edges;
    std::vector<std::size_t> order;
    for (std::size_t i = 0; i < areas.size(); ++i) {
        const auto& area = areas[i];
        if (!(area.attributes & Stairs) || (area.attributes & Crouch) ||
            area.width() < 1 || area.height() < 1) continue;
        const auto bounds = area.bounds();
        if (bounds.maxs.z - bounds.mins.z > FloorSkin) continue;
        order.push_back(i);
        for (const auto dir : directions) edges[dir][lowEdge(rectangles[i], dir)].push_back(i);
    }

    std::ranges::stable_sort(order, {}, [&](std::size_t i) { return areas[i].center().z; });
    const auto sizeLimit = std::min(32768.0f, GenerationStepSize * params.areaMaxSize);
    std::vector<bool> used(areas.size());
    std::size_t merged = 0;
    for (const auto first : order) {
        pool.checkCancelled();
        if (used[first]) continue;
        for (const auto dir : directions) {
            const auto run = findRun(first, dir, rectangles, areas, edges, used, sizeLimit);
            if (run.treads.size() < 2 || run.width.size() > sizeLimit ||
                !traversable(run, areas, world, params.slopeLimit)) continue;
            emitRun(run, mesh, field, rectangles, areas);
            for (const auto tread : run.treads) used[tread] = true;
            ++merged;
            break;
        }
    }
    return merged;
}

} // namespace navfast::exact
