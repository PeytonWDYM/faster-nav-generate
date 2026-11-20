#include "formats/phy_reader.h"

#include <cmath>
#include <limits>
#include <unordered_set>

namespace navfast::formats {
namespace {

constexpr std::uint32_t vphy = 0x59485056;
constexpr std::uint32_t ivps = 0x53505649;
constexpr std::size_t surfaceHeaderSize = 48, nodeSize = 28, ledgeHeaderSize = 16;
constexpr std::size_t triangleSize = 16, pointSize = 16;
constexpr float inchesPerMeter = 1.0f / 0.0254f;

std::size_t relativeOffset(Bytes surface, std::size_t from, std::int32_t delta) {
    const auto offset = static_cast<std::int64_t>(from) + delta;
    if (offset < static_cast<std::int64_t>(surfaceHeaderSize) ||
        offset >= static_cast<std::int64_t>(surface.size())) surface.fail();
    return static_cast<std::size_t>(offset);
}

PhyConvex readLedge(Bytes surface, std::size_t offset) {
    const auto pointBase = relativeOffset(surface, offset, surface.read<std::int32_t>(offset));
    const auto count = surface.read<std::uint16_t>(offset + 12);
    const auto triangles = surface.records(offset + ledgeHeaderSize, count, triangleSize);
    PhyConvex result;
    result.faces.reserve(count);
    constexpr auto unassigned = std::numeric_limits<std::uint16_t>::max();
    std::vector<std::uint16_t> remap;
    for (std::size_t i = 0; i < count; ++i) {
        std::array<std::uint16_t, 3> face;
        for (std::size_t j = 0; j < face.size(); ++j) {
            // IVP packs the 16-bit start point index and an opposite half-edge in each edge word.
            const auto index = triangles.read<std::uint16_t>(i * triangleSize + 4 + 4 * j);
            if (index >= remap.size()) remap.resize(static_cast<std::size_t>(index) + 1, unassigned);
            auto& mapped = remap[index];
            if (mapped == unassigned) {
                const auto point = surface.records(pointBase, static_cast<std::size_t>(index) + 1, pointSize)
                    .read<Vec3>(pointSize * index);
                if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
                    result.vertices.size() >= unassigned) surface.fail();
                mapped = static_cast<std::uint16_t>(result.vertices.size());
                result.vertices.push_back(Vec3{point.x, point.z, -point.y} * inchesPerMeter);
            }
            face[j] = mapped;
        }
        result.faces.push_back(face);
    }
    if (result.vertices.size() < 4) surface.fail();
    return result;
}

void readSurface(Bytes surface, std::vector<PhyConvex>& hulls) {
    const auto magic = surface.read<std::uint32_t>(44);
    // Older compact surfaces stored zero in the signature word.
    if (magic != ivps && magic != 0) surface.fail();

    std::vector<std::size_t> pending{relativeOffset(surface, 0, surface.read<std::int32_t>(32))};
    std::unordered_set<std::size_t> seen;
    while (!pending.empty()) {
        const auto node = pending.back();
        pending.pop_back();
        if (!seen.insert(node).second) surface.fail();
        const auto record = surface.slice(node, nodeSize);
        const auto right = record.read<std::int32_t>(0);
        if (right != 0) {
            // The left child follows the node. Any ledge on an internal node is a bounding hull.
            pending.push_back(relativeOffset(surface, node, right));
            pending.push_back(node + nodeSize);
        } else {
            const auto ledge = relativeOffset(surface, node, record.read<std::int32_t>(4));
            hulls.push_back(readLedge(surface, ledge));
        }
    }
}

} // namespace

std::vector<PhyConvex> readPhy(Bytes::Span bytes) {
    const Bytes file(bytes, "malformed compact .phy surface");
    const auto header = file.slice(0, 16);
    std::size_t offset = header.read<std::uint32_t>(0);
    if (offset < header.size()) file.fail();
    const auto count = header.read<std::uint32_t>(8);
    file.records(offset, count, 4);
    std::vector<PhyConvex> hulls;
    for (std::uint32_t i = 0; i < count; ++i) {
        const auto solid = file.slice(offset + 4, file.read<std::uint32_t>(offset));
        offset += 4 + solid.size();
        if (solid.read<std::uint32_t>(0) == vphy) {
            if (solid.read<std::uint16_t>(6) != 0) {
                throw std::runtime_error("unsupported .phy surface type (expected VPHY compact surface)");
            }
            readSurface(solid.slice(28, solid.read<std::uint32_t>(8)), hulls);
        } else {
            readSurface(solid, hulls);
        }
    }
    return hulls;
}

} // namespace navfast::formats
