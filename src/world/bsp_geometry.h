#pragma once

#include "formats/bsp_reader.h"
#include "world/geometry.h"

namespace navfast::detail {

void addBrushModel(const formats::BspReader& bsp, const formats::BspModel& model,
                   Vec3 origin, Vec3 angles, Geometry& geometry);
void addDisplacements(const formats::BspReader& bsp, Geometry& geometry);
void addProps(const WorldInputs& inputs, const formats::StaticProps& props, Geometry& geometry);

} // namespace navfast::detail
