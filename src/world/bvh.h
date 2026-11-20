#pragma once

#include "world/geometry.h"

namespace navfast::detail {

class Bvh {
public:
    // Spatially reorder the solids and build fixed-size, mask-aware BVH leaves.
    explicit Bvh(std::span<Solid> solids);

    TraceResult trace(const Geometry& geometry, const Sweep& sweep, std::uint32_t mask) const;
    std::uint32_t pointContents(const Geometry& geometry, Vec3 point) const;
    std::size_t size() const { return nodes_.size(); }

private:
    struct Node {
        Aabb bounds;
        std::uint32_t contents;
        std::uint32_t first;  // First solid for leaves, second child for branches.
        std::uint32_t count;  // Zero for a branch. Its first child follows it in preorder.
    };

    void build(std::span<Solid> solids, std::uint32_t first);

    std::vector<Node> nodes_;
};

} // namespace navfast::detail
