#include "world/bvh.h"

#include "world/brush_clip.h"
#include "world/convex_sweep.h"

namespace navfast::detail {
namespace {

constexpr std::size_t leafSize = 4;
constexpr float miss = std::numeric_limits<float>::infinity();

struct Slabs {
    Vec3 start, inverse, padding;
    std::array<bool, 3> stationary;

    Slabs(const Sweep& sweep)
        : start(sweep.start),
          inverse{reciprocal(sweep.delta.x), reciprocal(sweep.delta.y), reciprocal(sweep.delta.z)},
          padding(sweep.extents + Vec3{traceEpsilon, traceEpsilon, traceEpsilon}),
          stationary{sweep.delta.x == 0, sweep.delta.y == 0, sweep.delta.z == 0} {}

    float entry(Aabb box, float limit) const {
        float near = 0, far = limit;
        if (!clip(box.mins.x, box.maxs.x, start.x, inverse.x, padding.x, stationary[0], near, far) ||
            !clip(box.mins.y, box.maxs.y, start.y, inverse.y, padding.y, stationary[1], near, far) ||
            !clip(box.mins.z, box.maxs.z, start.z, inverse.z, padding.z, stationary[2], near, far)) return miss;
        return near;
    }

private:
    static float reciprocal(float delta) { return delta == 0 ? 0 : 1.0f / delta; }

    static bool clip(float min, float max, float origin, float inverse, float pad, bool stationary,
                     float& near, float& far) {
        min = min - origin - pad;
        max = max - origin + pad;
        if (stationary) return min <= 0 && max >= 0;
        const auto a = min * inverse, b = max * inverse;
        near = std::max(near, std::min(a, b));
        far = std::min(far, std::max(a, b));
        return near <= far;
    }
};

} // namespace

Bvh::Bvh(std::span<Solid> solids) {
    if (!solids.empty()) {
        nodes_.reserve(2 * solids.size() / leafSize + 1);
        build(solids, 0);
    }
}

void Bvh::build(std::span<Solid> solids, std::uint32_t first) {
    auto bounds = emptyBox(), centers = emptyBox();
    std::uint32_t contents = 0;
    for (const auto& solid : solids) {
        bounds = unite(bounds, solid.bounds);
        const auto center = (solid.bounds.mins + solid.bounds.maxs) * 0.5f;
        centers = unite(centers, {center, center});
        contents |= solid.contents;
    }

    const auto index = nodes_.size();
    nodes_.push_back({bounds, contents, first, static_cast<std::uint32_t>(solids.size())});
    if (solids.size() <= leafSize) return;

    const auto span = centers.maxs - centers.mins;
    const auto axis = span.x >= span.y && span.x >= span.z ? axes[0] : (span.y >= span.z ? axes[1] : axes[2]);
    const auto middle = solids.size() / 2;
    std::nth_element(solids.begin(), solids.begin() + middle, solids.end(), [axis](const Solid& a, const Solid& b) {
        return (a.bounds.mins + a.bounds.maxs).dot(axis) < (b.bounds.mins + b.bounds.maxs).dot(axis);
    });
    build(solids.first(middle), first);
    nodes_[index].first = static_cast<std::uint32_t>(nodes_.size());
    nodes_[index].count = 0;
    build(solids.subspan(middle), first + static_cast<std::uint32_t>(middle));
}

TraceResult Bvh::trace(const Geometry& geometry, const Sweep& sweep, std::uint32_t mask) const {
    if (nodes_.empty() || (nodes_[0].contents & mask) == 0) return {};
    const Slabs slabs(sweep);
    SweepHit hit;

    struct Visit { std::uint32_t index; float entry; };
    // Median splitting bounds the depth by the number of bits in a solid index.
    std::array<Visit, 32> stack;
    std::size_t pending = 0;
    auto visit = Visit{0, slabs.entry(nodes_[0].bounds, hit.contact)};
    for (;;) {
        if (visit.entry <= hit.contact) {
            const auto& node = nodes_[visit.index];
            if (node.count != 0) {
                for (const auto& solid : std::span(geometry.solids).subspan(node.first, node.count)) {
                    if ((solid.contents & mask) == 0 || slabs.entry(solid.bounds, hit.contact) == miss) continue;
                    if (solid.planeCount != 0) {
                        traceConvex(sweep, solid, geometry.planes.data() + solid.first, hit);
                    } else {
                        traceDisplacement(sweep, geometry.triangles[solid.first], hit);
                    }
                    if (hit.result.allSolid) return hit.result;
                }
            } else {
                const auto child = [&](std::uint32_t index) {
                    const auto& n = nodes_[index];
                    return Visit{index, (n.contents & mask) != 0 ? slabs.entry(n.bounds, hit.contact) : miss};
                };
                auto a = child(visit.index + 1), b = child(node.first);
                if (a.entry > b.entry) std::swap(a, b);
                if (b.entry != miss) stack[pending++] = b;
                if (a.entry != miss) {
                    visit = a;
                    continue;
                }
            }
        }
        if (pending == 0) return hit.result;
        visit = stack[--pending];
    }
}

std::uint32_t Bvh::pointContents(const Geometry& geometry, Vec3 point) const {
    if (nodes_.empty()) return 0;
    std::uint32_t contents = 0;
    std::array<std::uint32_t, 32> stack{0};
    std::size_t pending = 1;
    while (pending != 0) {
        const auto index = stack[--pending];
        const auto& node = nodes_[index];
        if ((node.contents & ~contents) == 0 || !contains(node.bounds, point)) continue;
        if (node.count == 0) {
            stack[pending++] = node.first;
            stack[pending++] = index + 1;
            continue;
        }
        for (const auto& solid : std::span(geometry.solids).subspan(node.first, node.count)) {
            if (solid.faceCount != 0 && (solid.contents & ~contents) != 0 && contains(solid.bounds, point) &&
                pointInConvex(point, std::span(geometry.planes).subspan(solid.first, solid.faceCount))) {
                contents |= solid.contents;
            }
        }
    }
    return contents;
}

} // namespace navfast::detail
