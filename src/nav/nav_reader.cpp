#include "nav/nav_file.h"
#include "job/binary_io.h"

#include <algorithm>
#include <limits>

namespace navfast {
namespace {

void include(Aabb& bounds, const Aabb& area) {
    bounds.mins = {std::min(bounds.mins.x,area.mins.x), std::min(bounds.mins.y,area.mins.y), std::min(bounds.mins.z,area.mins.z)};
    bounds.maxs = {std::max(bounds.maxs.x,area.maxs.x), std::max(bounds.maxs.y,area.maxs.y), std::max(bounds.maxs.z,area.maxs.z)};
}

void readArea(BinaryReader& reader, NavInspection& info) {
    reader.skip(8); // ID and attributes
    const auto nw = reader.vec();
    const auto se = reader.vec();
    const auto ne = reader.get<float>();
    const auto sw = reader.get<float>();
    include(info.bounds, {{nw.x,nw.y,std::min({nw.z,se.z,ne,sw})}, {se.x,se.y,std::max({nw.z,se.z,ne,sw})}});
    for (auto dir : directions) {
        static_cast<void>(dir);
        const auto count = reader.get<std::uint32_t>();
        info.connections += count;
        reader.skip(count, 4);
    }
    reader.skip(reader.get<std::uint8_t>(), 17); // hiding spot ID, position, and flags
    const auto encounters = reader.get<std::uint32_t>();
    for (std::uint32_t i = 0; i < encounters; ++i) {
        reader.skip(10); // from/to IDs and directions
        reader.skip(reader.get<std::uint8_t>(), 5);
    }
    reader.skip(2); // place
    reader.skip(reader.get<std::uint32_t>(), 4); // ladders up
    reader.skip(reader.get<std::uint32_t>(), 4); // ladders down
    reader.skip(24); // earliest occupy and light
    reader.skip(reader.get<std::uint32_t>(), 5); // visibility
    reader.skip(4); // inherit visibility
}

} // namespace

NavInspection inspectNav(const std::filesystem::path& path) {
    const auto bytes = readBinaryFile(path);
    BinaryReader reader(bytes);
    if (reader.get<std::uint32_t>() != 0xFEEDFACE) throw std::runtime_error("Invalid .nav signature");
    NavInspection info;
    info.version = reader.get<std::uint32_t>();
    if (info.version != 16) throw std::runtime_error("Unsupported .nav version " + std::to_string(info.version));
    info.subVersion = reader.get<std::uint32_t>();
    info.bspSize = reader.get<std::uint32_t>();
    info.analyzed = reader.get<std::uint8_t>() != 0;
    const auto places = reader.get<std::uint16_t>();
    for (std::uint16_t i = 0; i < places; ++i) reader.skip(reader.get<std::uint16_t>());
    reader.skip(1); // has unnamed areas
    info.areas = reader.get<std::uint32_t>();
    if (info.areas) {
        constexpr auto inf = std::numeric_limits<float>::infinity();
        info.bounds = {{inf,inf,inf}, {-inf,-inf,-inf}};
    }
    for (std::uint32_t i = 0; i < info.areas; ++i) readArea(reader, info);
    info.ladders = reader.get<std::uint32_t>();
    reader.skip(info.ladders, 60);
    return info;
}

} // namespace navfast
