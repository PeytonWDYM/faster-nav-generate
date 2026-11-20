#include "world/bsp_geometry.h"

#include <stdexcept>

#include "world/convex_sweep.h"

namespace navfast::detail {
namespace {

Aabb axialBounds(std::span<const Plane> planes) {
    auto bounds = emptyBox();
    bounds = {bounds.maxs, bounds.mins};
    for (const auto& plane : planes) {
        const auto n = plane.normal;
        if (n == axes[0]) bounds.maxs.x = std::min(bounds.maxs.x, plane.distance);
        if (n == axes[1]) bounds.maxs.y = std::min(bounds.maxs.y, plane.distance);
        if (n == axes[2]) bounds.maxs.z = std::min(bounds.maxs.z, plane.distance);
        if (n == -axes[0]) bounds.mins.x = std::max(bounds.mins.x, -plane.distance);
        if (n == -axes[1]) bounds.mins.y = std::max(bounds.mins.y, -plane.distance);
        if (n == -axes[2]) bounds.mins.z = std::max(bounds.mins.z, -plane.distance);
    }
    return bounds;
}

Vec3 intersection(const Plane& a, const Plane& b, const Plane& c) {
    const auto bc = b.normal.cross(c.normal);
    const auto det = static_cast<double>(a.normal.dot(bc));
    if (std::abs(det) <= 1e-7) return emptyBox().mins;
    const auto ca = c.normal.cross(a.normal), ab = a.normal.cross(b.normal);
    const double da = a.distance / det, db = b.distance / det, dc = c.distance / det;
    return {static_cast<float>(da * bc.x + db * ca.x + dc * ab.x),
            static_cast<float>(da * bc.y + db * ca.y + dc * ab.y),
            static_cast<float>(da * bc.z + db * ca.z + dc * ab.z)};
}

std::vector<Vec3> brushVertices(std::span<const Plane> faces) {
    std::vector<Vec3> vertices;
    for (std::size_t i = 0; i < faces.size(); ++i) {
        for (std::size_t j = i + 1; j < faces.size(); ++j) {
            for (std::size_t k = j + 1; k < faces.size(); ++k) {
                const auto p = intersection(faces[i], faces[j], faces[k]);
                if (!finite(p)) continue;
                if (std::ranges::any_of(faces, [p](const Plane& face) {
                    return p.dot(face.normal) - face.distance > 0.02f;
                })) continue;
                if (std::ranges::any_of(vertices, [p](Vec3 v) {
                    return (v - p).lengthSqr() < 1e-6f;
                })) continue;
                vertices.push_back(p);
            }
        }
    }
    if (vertices.empty()) throw std::runtime_error("BSP brush has no convex vertices");
    return vertices;
}

std::vector<Vec3> brushEdges(std::span<const Plane> faces, std::span<const Vec3> points) {
    // In a convex polyhedron, two vertices on the same two faces share an edge.
    std::vector<Vec3> edges;
    for (std::size_t i = 0; i < points.size(); ++i) {
        for (std::size_t j = i + 1; j < points.size(); ++j) {
            unsigned shared = 0;
            for (const auto& face : faces) {
                if (std::abs(points[i].dot(face.normal) - face.distance) < 0.025f &&
                    std::abs(points[j].dot(face.normal) - face.distance) < 0.025f &&
                    ++shared == 2) {
                    edges.push_back(points[j] - points[i]);
                    break;
                }
            }
        }
    }
    return edges;
}

void addBrush(const formats::BspReader& bsp, formats::Bytes sides, formats::Bytes planes,
              const formats::BspBrush& brush, const Transform& transform, bool rotated,
              Geometry& geometry) {
    using formats::BspBrushSide;
    using formats::BspPlane;
    constexpr int maxBrushSides = 128; // Source's MAX_BRUSH_SIDES.
    if (brush.sideCount > maxBrushSides) throw std::runtime_error("brush exceeds MAX_BRUSH_SIDES (128)");
    if (brush.sideCount == 0 || brush.contents == 0) return;
    const auto records = sides.records(static_cast<std::size_t>(brush.firstSide) * sizeof(BspBrushSide),
                                       static_cast<std::size_t>(brush.sideCount), sizeof(BspBrushSide));
    std::vector<Plane> hull, bevels;
    hull.reserve(static_cast<std::size_t>(brush.sideCount));
    for (std::size_t byte = 0; byte < records.size(); byte += sizeof(BspBrushSide)) {
        const auto side = records.read<BspBrushSide>(byte);
        const auto plane = planes.read<BspPlane>(static_cast<std::size_t>(side.plane) * sizeof(BspPlane));
        if (!finite(plane.normal) || std::abs(plane.normal.lengthSqr() - 1) > 0.01f ||
            !std::isfinite(plane.distance)) planes.fail();
        if (side.bevel && rotated) continue;
        (side.bevel ? bevels : hull).push_back(
            {plane.normal, plane.distance, bsp.surfaceFlags(side.texInfo)});
    }

    const auto faceCount = hull.size();
    if (faceCount < 4) sides.fail();
    auto bounds = axialBounds(hull);
    // Compiler bevels serve hull clipping only. Some are tighter than the true
    // faces, so they cannot bound the brush for rays or point queries.
    if (!rotated) hull.insert(hull.end(), bevels.begin(), bevels.end());
    if (rotated || !finite(bounds.mins) || !finite(bounds.maxs)) {
        auto vertices = brushVertices(std::span(hull).first(faceCount));
        auto edges = rotated ? brushEdges(std::span(hull).first(faceCount), vertices) : std::vector<Vec3>{};
        bounds = emptyBox();
        for (auto& vertex : vertices) {
            vertex = transform.point(vertex);
            bounds = unite(bounds, {vertex, vertex});
        }
        for (auto& plane : hull) {
            plane.normal = transform.rotate(plane.normal);
            plane.distance += plane.normal.dot(transform.origin);
        }
        if (rotated) {
            for (auto& edge : edges) edge = transform.rotate(edge);
            appendBoxBevels(hull, vertices, edges);
        }
    } else {
        bounds.mins += transform.origin;
        bounds.maxs += transform.origin;
        for (auto& plane : hull) plane.distance += plane.normal.dot(transform.origin);
    }
    if (!contains(bounds, bounds.mins)) sides.fail();
    geometry.addConvex(bounds, brush.contents, hull, faceCount);
    ++geometry.stats.brushes;
}

} // namespace

void addBrushModel(const formats::BspReader& bsp, const formats::BspModel& model,
                   Vec3 origin, Vec3 angles, Geometry& geometry) {
    using namespace formats;
    const auto sides = bsp.table(Lump::BrushSides, sizeof(BspBrushSide));
    const auto planes = bsp.table(Lump::Planes, sizeof(BspPlane));
    const auto brushes = bsp.table(Lump::Brushes, sizeof(BspBrush));
    const auto transform = angleTransform(origin, angles);
    const bool rotated = angles != Vec3{};
    for (const auto index : bsp.modelBrushes(model)) {
        try {
            addBrush(bsp, sides, planes, brushes.read<BspBrush>(index * sizeof(BspBrush)),
                     transform, rotated, geometry);
        } catch (const std::runtime_error& error) {
            throw std::runtime_error("BSP brush " + std::to_string(index) + ": " + error.what());
        }
    }
}

} // namespace navfast::detail
