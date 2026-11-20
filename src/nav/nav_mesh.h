#pragma once

#include "nav/nav_area.h"

#include <span>
#include <utility>

namespace navfast {

/// Owns all nodes, area IDs, directed edges, and the spatial area grid for one generation.
class NavMesh {
public:
    NavNodes nodes;

    AreaId add(NavArea area);
    void remove(AreaId id);
    void connect(AreaId from, Direction dir, AreaId to);
    void disconnect(AreaId from, Direction dir, AreaId to);
    void disconnect(AreaId from, AreaId to);
    void reshape(AreaId id, const std::array<Vec3, 4>& points);
    void merge(AreaId keep, AreaId discarded, const std::array<NodeId, 4>& combinedNodes);
    std::pair<AreaId, AreaId> split(AreaId id, bool splitY, float coordinate);

    NavArea& operator[](AreaId id) { return areas_[static_cast<std::size_t>(id)]; }
    const NavArea& operator[](AreaId id) const { return areas_[static_cast<std::size_t>(id)]; }
    std::vector<AreaId> areaIds() const;
    AreaId getNavArea(Vec3 pos, float beneathLimit = 120.0f) const;
    std::vector<AreaId> nearby(Aabb extent) const;
    std::size_t connectionCount() const;
    NodeId closestEdgeNode(AreaId id, Direction dir, Vec3 point) const;

private:
    void index(AreaId id, bool insert);
    void assignNodes(AreaId id);

    std::vector<NavArea> areas_{1};
    std::unordered_map<std::uint64_t, std::vector<AreaId>> grid_;
};

} // namespace navfast
