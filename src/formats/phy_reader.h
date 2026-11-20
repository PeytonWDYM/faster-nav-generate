#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "core/vec3.h"
#include "formats/bytes.h"

namespace navfast::formats {

struct PhyConvex {
    std::vector<Vec3> vertices;
    std::vector<std::array<std::uint16_t, 3>> faces;
};

// Read terminal convex ledges from a model's compact collision surfaces, in Source coordinates.
std::vector<PhyConvex> readPhy(Bytes::Span bytes);

} // namespace navfast::formats
