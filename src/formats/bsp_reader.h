#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "core/vec3.h"
#include "formats/bytes.h"

namespace navfast::formats {

enum class Lump {
    Planes = 1, Vertices = 3, Nodes = 5, TexInfo = 6, Faces = 7, Leaves = 10,
    Edges = 12, SurfEdges = 13, Models = 14, LeafBrushes = 17, Brushes = 18,
    BrushSides = 19, DispInfo = 26, DispVerts = 33, Game = 35, DispTris = 48
};

struct BspModel {
    Aabb bounds;
    Vec3 origin;
    std::int32_t headNode, firstFace, faceCount;
};

struct BspPlane {
    Vec3 normal;
    float distance;
    std::int32_t type;
};

struct BspBrush {
    std::int32_t firstSide, sideCount;
    std::uint32_t contents;
};

struct BspBrushSide {
    std::uint16_t plane;
    std::int16_t texInfo, displacement;
    std::uint8_t bevel, thin;
};

struct BspFace {
    std::uint16_t plane;
    std::uint8_t side, onNode;
    std::int32_t firstEdge;
    std::int16_t edgeCount, texInfo, displacement;
    std::byte rest[42];
};

struct BspDispInfo {
    Vec3 start;
    std::int32_t firstVertex, firstTriangle, power, minTess;
    float smoothingAngle;
    std::uint32_t contents;
    std::uint16_t face;
    std::byte rest[138];
};

struct BspDispVertex {
    Vec3 direction;
    float distance, alpha;
};

struct StaticProp {
    Vec3 origin, angles;
    std::uint16_t model, firstLeaf, leafCount;
    std::uint8_t solid, flags;
};

inline constexpr std::uint8_t solidVPhysics = 6;

struct StaticProps {
    std::vector<std::string> models;
    std::vector<StaticProp> instances;
};

static_assert(sizeof(BspModel) == 48 && sizeof(BspPlane) == 20 && sizeof(BspBrush) == 12);
static_assert(sizeof(BspBrushSide) == 8 && sizeof(BspFace) == 56 && sizeof(BspDispInfo) == 176);
static_assert(sizeof(BspDispVertex) == 20 && sizeof(StaticProp) == 32);

class BspReader {
public:
    // Validate a VBSP header and its lump directory. The caller owns the bytes.
    explicit BspReader(Bytes::Span bytes);

    Bytes lump(Lump id) const;
    Bytes table(Lump id, std::size_t stride) const;
    BspModel model(std::size_t index) const;
    std::uint32_t surfaceFlags(std::int16_t texInfo) const;

    // Return unique brush indices reachable from the given model's BSP tree.
    std::vector<std::uint16_t> modelBrushes(const BspModel& model) const;

    // Decode the shared prefix of Source's versioned static-prop records.
    StaticProps staticProps() const;

private:
    struct Entry { std::uint32_t offset, size, version, uncompressedSize; };

    Bytes bytes_;
    std::array<Entry, 64> entries_;
};

std::string normalizeModelPath(std::string_view name);

} // namespace navfast::formats
