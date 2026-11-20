#include "nav/nav_node.h"

#include <algorithm>
#include <bit>

namespace navfast {
namespace {

std::uint64_t xyKey(Vec3 pos) {
    const auto bits = [](float value) { return std::bit_cast<std::uint32_t>(value == 0 ? 0.0f : value); };
    return (std::uint64_t{bits(pos.x)} << 32) | bits(pos.y);
}

} // namespace

bool NavNode::blocked() const {
    return std::ranges::any_of(clearance, &NodeClearance::blocked);
}

NodeId NavNodes::find(Vec3 pos) const {
    const auto it = columns_.find(xyKey(pos));
    if (it == columns_.end()) return {};
    const auto& stack = it->second;
    for (auto node = stack.rbegin(); node != stack.rend(); ++node)
        if (std::abs((*this)[*node].position.z - pos.z) < 0.45f * GenerationStepSize) return *node;
    return {};
}

NodeId NavNodes::add(Vec3 position, Vec3 normal, bool displacement) {
    const auto id = static_cast<NodeId>(nodes_.size());
    nodes_.push_back({position, normal, displacement});
    columns_[xyKey(position)].push_back(id);
    return id;
}

void NavNodes::connect(NodeId from, Direction dir, NodeLink link, float sampledZ) {
    auto& source = (*this)[from];
    source.links[dir] = link;
    const float dz = source.position.z - sampledZ;
    if (std::abs(dz) < 50.0f) {
        auto& dest = (*this)[link.target];
        const auto back = opposite(dir);
        dest.links[back] = {from, link.obstacleHeight > 0 ? std::max(0.0f, link.obstacleHeight + dz) : 0,
                            GenerationStepSize - link.obstacleEnd, GenerationStepSize - link.obstacleStart};
        dest.visited[back] = true;
    }
}

bool NavNodes::closedCell(NodeId id) const {
    const auto east = (*this)[id].links[East].target;
    const auto south = (*this)[id].links[South].target;
    if (east == NodeId{} || south == NodeId{} ||
        (*this)[east].links[West].target != id || (*this)[south].links[North].target != id)
        return false;
    const auto se = (*this)[east].links[South].target;
    return se != NodeId{} && se == (*this)[south].links[East].target &&
           (*this)[se].links[North].target == east && (*this)[se].links[West].target == south;
}

void NavNodes::checkCrouch(NodeId id, const CollisionWorld& world) {
    auto& node = (*this)[id];
    const auto pos = node.position;
    const auto ceiling = world.traceHull(pos, pos + Vec3{0,0,JumpCrouchHeight},
                                         NavTraceMins, NavTraceMaxs, GenerationTraceMask);
    for (const auto corner : corners) {
        const auto sign = cornerVector[corner];
        const Vec3 mins{std::min(0.0f, sign.x * HalfHumanWidth), std::min(0.0f, sign.y * HalfHumanWidth), 0};
        Vec3 maxs{std::max(0.0f, sign.x * HalfHumanWidth), std::max(0.0f, sign.y * HalfHumanWidth), 0};
        auto& clearance = node.clearance[corner];
        clearance = {true, true, JumpCrouchHeight};
        for (float height = 0; height <= ceiling.endpos.z - pos.z; height += 1.0f) {
            const auto start = pos + Vec3{0,0,height};
            maxs.z = HumanCrouchHeight;
            if (world.traceHull(start, start, mins, maxs, GenerationTraceMask).startSolid) continue;
            maxs.z = HumanHeight;
            clearance = {world.traceHull(start, start, mins, maxs, GenerationTraceMask).startSolid,
                         false, start.z - pos.z};
            break;
        }
    }
}

} // namespace navfast
