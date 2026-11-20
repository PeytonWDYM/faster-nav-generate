#include "world/bsp_geometry.h"

#include <cstdio>
#include <unordered_set>

#include "formats/phy_reader.h"
#include "world/convex_sweep.h"
#include "world/source_flags.h"

namespace navfast::detail {
namespace {

void addPropHull(const formats::PhyConvex& convex, const Transform& transform, Geometry& geometry) {
    auto bounds = emptyBox();
    Vec3 center;
    std::vector<Vec3> points;
    points.reserve(convex.vertices.size());
    for (const auto point : convex.vertices) {
        const auto rotated = transform.rotate(point);
        const auto world = rotated + transform.origin;
        center += rotated;
        bounds = unite(bounds, {world, world});
        points.push_back(rotated);
    }
    center = center * (1.0f / static_cast<float>(points.size()));

    std::vector<Plane> hull;
    std::vector<Vec3> edges;
    for (const auto& face : convex.faces) {
        const auto a = points[face[0]], b = points[face[1]], c = points[face[2]];
        auto normal = (b - a).cross(c - a);
        const auto length = normal.length();
        if (length == 0) continue;
        normal = normal * (1.0f / length);
        if (normal.dot(a - center) < 0) normal = -normal;
        if (std::ranges::none_of(hull, [normal](const Plane& p) {
            return p.normal.dot(normal) > 0 && p.normal.cross(normal).lengthSqr() < 1e-12f;
        })) {
            auto distance = normal.dot(a);
            for (const auto point : points) distance = std::max(distance, normal.dot(point));
            hull.push_back({normal, distance});
        }
        edges.push_back(b - a);
        edges.push_back(c - b);
        edges.push_back(a - c);
    }
    const auto faceCount = hull.size();
    if (faceCount < 4) throw std::runtime_error(".phy ledge has no closed convex hull");
    appendBoxBevels(hull, points, edges);
    for (auto& plane : hull) plane.distance += plane.normal.dot(transform.origin);
    geometry.addConvex(bounds, source::CONTENTS_SOLID, hull, faceCount);
    ++geometry.stats.propHulls;
}

} // namespace

void addProps(const WorldInputs& inputs, const formats::StaticProps& props, Geometry& geometry) {
    std::unordered_map<std::string, std::vector<formats::PhyConvex>> cache;
    std::unordered_set<std::string> missing;
    const auto addModel = [&](const std::string& model, Vec3 origin, Vec3 angles) {
        const auto phy = inputs.phyFiles.find(model);
        if (phy == inputs.phyFiles.end()) {
            missing.insert(model);
            ++geometry.stats.propsSkipped;
            return;
        }
        auto [entry, added] = cache.try_emplace(model);
        if (added) {
            try {
                entry->second = formats::readPhy(phy->second);
            } catch (const std::runtime_error& error) {
                throw std::runtime_error(model + ": " + error.what());
            }
        }
        const auto transform = angleTransform(origin, angles);
        for (const auto& convex : entry->second) addPropHull(convex, transform, geometry);
        ++geometry.stats.propsLoaded;
    };

    for (const auto& prop : props.instances) {
        if (prop.solid == formats::solidVPhysics) addModel(props.models[prop.model], prop.origin, prop.angles);
    }
    for (const auto& entity : inputs.entities) {
        if (!entity.model.starts_with('*')) {
            addModel(formats::normalizeModelPath(entity.model), entity.origin, entity.angles);
        }
    }
    if (!missing.empty()) {
        std::fprintf(stderr, "navfast: skipped %zu prop instance(s) across %zu model(s) with missing .phy files\n",
                     geometry.stats.propsSkipped, missing.size());
    }
}

} // namespace navfast::detail
