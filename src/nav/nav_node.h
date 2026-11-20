#pragma once

#include "nav/generation.h"
#include "world/collision_world.h"

#include <unordered_map>
#include <vector>

namespace navfast {

enum class NodeId : std::uint32_t {};
enum class AreaId : std::uint32_t {};

struct NodeLink {
    NodeId target{};
    float obstacleHeight = 0;
    float obstacleStart = 0;
    float obstacleEnd = GenerationStepSize;
};

struct NodeClearance {
    bool crouch = false;
    bool blocked = false;
    float groundHeight = 0;
};

struct NavNode {
    Vec3 position;
    Vec3 normal;
    bool displacement = false;
    std::array<NodeLink, 4> links{};
    std::array<NodeClearance, 4> clearance{};
    std::array<bool, 4> visited{};
    AreaId area{};

    bool blocked() const;
};

/// Owns sampled nodes and exact-XY stacks, with the SDK's strict Z tolerance.
class NavNodes {
public:
    NodeId find(Vec3 position) const;
    NodeId add(Vec3 position, Vec3 normal, bool displacement);
    void connect(NodeId from, Direction dir, NodeLink link, float sampledZ);
    bool closedCell(NodeId id) const;
    void checkCrouch(NodeId id, const CollisionWorld& world);

    NavNode& operator[](NodeId id) { return nodes_[static_cast<std::size_t>(id)]; }
    const NavNode& operator[](NodeId id) const { return nodes_[static_cast<std::size_t>(id)]; }
    std::size_t size() const { return nodes_.size() - 1; }

private:
    std::vector<NavNode> nodes_{1};
    std::unordered_map<std::uint64_t, std::vector<NodeId>> columns_;
};

} // namespace navfast
