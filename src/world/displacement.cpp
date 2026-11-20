#include "world/bsp_geometry.h"

#include <array>

namespace navfast::detail {
namespace {

constexpr unsigned removeTriangle = 1u << 5;
constexpr std::uint32_t hasSurfaceFlags = 0x80000000;
constexpr unsigned noRayCollision = 0x8, noHullCollision = 0x4;

Vec3 lerp(Vec3 a, Vec3 b, float t) {
    return a + (b - a) * t;
}

std::array<Vec3, 4> faceCorners(const formats::BspReader& bsp, const formats::BspFace& face,
                               Vec3 start) {
    const auto vertices = bsp.table(formats::Lump::Vertices, sizeof(Vec3));
    const auto edges = bsp.table(formats::Lump::Edges, 4);
    const auto surfEdges = bsp.table(formats::Lump::SurfEdges, 4);
    if (face.edgeCount != 4) surfEdges.fail();
    const auto indices = surfEdges.records(static_cast<std::size_t>(face.firstEdge) * 4, 4, 4);
    std::array<Vec3, 4> points;
    std::size_t first = 0;
    for (std::size_t i = 0; i < points.size(); ++i) {
        const auto edge = static_cast<std::int64_t>(indices.read<std::int32_t>(4 * i));
        const auto vertex = edges.read<std::uint16_t>(
            static_cast<std::size_t>(std::abs(edge)) * 4 + (edge < 0 ? 2 : 0));
        points[i] = vertices.read<Vec3>(vertex * sizeof(Vec3));
        if (!finite(points[i])) vertices.fail();
        if ((points[i] - start).lengthSqr() < (points[first] - start).lengthSqr()) first = i;
    }
    std::rotate(points.begin(), points.begin() + first, points.end());
    return points;
}

} // namespace

void addDisplacements(const formats::BspReader& bsp, Geometry& geometry) {
    using namespace formats;
    const auto info = bsp.table(Lump::DispInfo, sizeof(BspDispInfo));
    if (info.size() == 0) return;
    const auto faces = bsp.table(Lump::Faces, sizeof(BspFace));
    const auto verts = bsp.table(Lump::DispVerts, sizeof(BspDispVertex));
    const auto tris = bsp.table(Lump::DispTris, sizeof(std::uint16_t));

    for (std::size_t offset = 0; offset < info.size(); offset += sizeof(BspDispInfo)) {
        const auto disp = info.read<BspDispInfo>(offset);
        if (disp.power < 2 || disp.power > 4 || !finite(disp.start)) info.fail();
        const auto minTess = static_cast<std::uint32_t>(disp.minTess);
        // VBSP uses the high bit to distinguish surface flags from legacy tessellation data.
        const auto dispFlags = (minTess & hasSurfaceFlags) != 0 ? minTess : 0;
        const bool rayCollision = (dispFlags & noRayCollision) == 0;
        const bool hullCollision = (dispFlags & noHullCollision) == 0;
        if (!rayCollision && !hullCollision) continue;
        const auto side = std::size_t{1} << disp.power;
        const auto width = side + 1;
        const auto face = faces.read<BspFace>(disp.face * sizeof(BspFace));
        const auto corners = faceCorners(bsp, face, disp.start);
        const auto surface = bsp.surfaceFlags(face.texInfo);
        const auto vertRecords = verts.records(static_cast<std::size_t>(disp.firstVertex) * sizeof(BspDispVertex),
                                               width * width, sizeof(BspDispVertex));
        const auto flags = tris.records(static_cast<std::size_t>(disp.firstTriangle) * 2, 2 * side * side, 2);
        std::array<Vec3, 17 * 17> points;
        for (std::size_t y = 0; y < width; ++y) {
            const auto t = static_cast<float>(y) / static_cast<float>(side);
            const auto left = lerp(corners[0], corners[1], t);
            const auto right = lerp(corners[3], corners[2], t);
            for (std::size_t x = 0; x < width; ++x) {
                const auto i = y * width + x;
                const auto vert = vertRecords.read<BspDispVertex>(i * sizeof(BspDispVertex));
                points[i] = lerp(left, right, static_cast<float>(x) / static_cast<float>(side)) +
                            vert.direction * vert.distance;
                if (!finite(points[i])) verts.fail();
            }
        }

        std::size_t tag = 0;
        const auto add = [&](std::size_t a, std::size_t b, std::size_t c) {
            const auto triFlags = flags.read<std::uint16_t>(2 * tag++);
            // The remaining tags describe walkability and surface materials, not collision.
            if ((triFlags & removeTriangle) == 0) {
                geometry.addDisplacement({points[a], points[c], points[b]}, surface, rayCollision, hullCollision);
            }
        };
        for (std::size_t y = 0; y < side; ++y) {
            for (std::size_t x = 0; x < side; ++x) {
                const auto a = y * width + x, b = a + 1, c = a + width, d = c + 1;
                // Source alternates the diagonal by flattened vertex index.
                if (a % 2 != 0) {
                    add(a, c, b);
                    add(b, c, d);
                } else {
                    add(a, c, d);
                    add(a, d, b);
                }
            }
        }
    }
}

} // namespace navfast::detail
