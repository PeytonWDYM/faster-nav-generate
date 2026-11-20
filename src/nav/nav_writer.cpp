#include "nav/nav_file.h"
#include "job/binary_io.h"

namespace navfast {

void writeNav(const std::filesystem::path& path, const NavMesh& mesh, std::uint32_t bspSize) {
    BinaryWriter writer;
    writer.put<std::uint32_t>(0xFEEDFACE);
    writer.put<std::uint32_t>(16);
    writer.put<std::uint32_t>(0); // subversion
    writer.put(bspSize);
    writer.put<std::uint8_t>(0); // analyzed
    writer.put<std::uint16_t>(0); // place directory

    const auto ids = mesh.areaIds();
    writer.put<std::uint8_t>(!ids.empty());
    writer.put(binaryCount(ids.size()));
    for (const auto id : ids) {
        const auto& area = mesh[id];
        writer.put(static_cast<std::uint32_t>(id));
        writer.put(area.attributes);
        writer.vec(area.points[NW]);
        writer.vec(area.points[SE]);
        writer.put(area.points[NE].z);
        writer.put(area.points[SW].z);
        for (const auto& neighbors : area.outgoing) {
            writer.put(binaryCount(neighbors.size()));
            for (const auto to : neighbors) writer.put(static_cast<std::uint32_t>(to));
        }
        writer.put<std::uint8_t>(0); // hiding spots
        writer.put<std::uint32_t>(0); // encounter paths
        writer.put<std::uint16_t>(0); // place index
        writer.put<std::uint32_t>(0); // ladders up
        writer.put<std::uint32_t>(0); // ladders down
        writer.put(0.0f); // earliest occupy for each of two teams
        writer.put(0.0f);
        for (const auto corner : corners) {
            static_cast<void>(corner);
            writer.put(1.0f);
        }
        writer.put<std::uint32_t>(0); // visible areas
        writer.put<std::uint32_t>(0); // inherited visibility
    }
    writer.put<std::uint32_t>(0); // ladder records
    publishBinaryFile(path, writer.bytes(), true);
}

} // namespace navfast
