#include "world/world_stats.h"

#include <charconv>
#include <stdexcept>
#include <utility>

#include "world/bsp_geometry.h"
#include "world/bvh.h"

namespace navfast {
namespace {

class FrozenWorld final : public CollisionWorld {
public:
    FrozenWorld(Aabb bounds, detail::Geometry geometry, WorldStats& stats)
        : bounds_(bounds), geometry_(std::move(geometry)), bvh_(geometry_.solids) {
        stats = geometry_.stats;
        stats.bvhNodes = bvh_.size();
    }

    TraceResult traceHull(Vec3 start, Vec3 end, Vec3 mins, Vec3 maxs, std::uint32_t mask) const override {
        const auto offset = (mins + maxs) * 0.5f;
        const detail::Sweep sweep{start + offset, end + offset, end - start, (maxs - mins) * 0.5f};
        auto result = bvh_.trace(geometry_, sweep, mask);
        result.endpos = start + sweep.delta * result.fraction;
        return result;
    }

    std::uint32_t pointContents(Vec3 point) const override { return bvh_.pointContents(geometry_, point); }
    Aabb bounds() const override { return bounds_; }

private:
    Aabb bounds_;
    detail::Geometry geometry_;
    detail::Bvh bvh_;
};

} // namespace

std::vector<std::string> listStaticPropModels(std::span<const std::byte> bsp) {
    return formats::BspReader(bsp).staticProps().models;
}

std::unique_ptr<CollisionWorld> buildCollisionWorld(const WorldInputs& inputs) {
    WorldStats stats;
    return buildCollisionWorldWithStats(inputs, stats);
}

std::unique_ptr<CollisionWorld> buildCollisionWorldWithStats(const WorldInputs& inputs, WorldStats& stats) {
    const formats::BspReader bsp(inputs.bsp);
    const auto props = bsp.staticProps();
    const auto world = bsp.model(0);
    if (!detail::finite(world.bounds.mins) || !detail::finite(world.bounds.maxs) ||
        !detail::contains(world.bounds, world.bounds.mins)) throw std::runtime_error("invalid BSP world bounds");
    detail::Geometry geometry;
    detail::addBrushModel(bsp, world, {}, {}, geometry);
    for (const auto& entity : inputs.entities) {
        if (!entity.model.starts_with('*')) continue;
        const auto* end = entity.model.data() + entity.model.size();
        std::size_t index = 0;
        const auto parsed = std::from_chars(entity.model.data() + 1, end, index);
        if (parsed.ec != std::errc{} || parsed.ptr != end || index == 0) {
            throw std::runtime_error("invalid BSP entity model " + entity.model);
        }
        detail::addBrushModel(bsp, bsp.model(index), entity.origin, entity.angles, geometry);
    }
    detail::addDisplacements(bsp, geometry);
    detail::addProps(inputs, props, geometry);
    return std::make_unique<FrozenWorld>(world.bounds, std::move(geometry), stats);
}

} // namespace navfast
