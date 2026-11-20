#include "world/brush_clip.h"

namespace navfast::detail {
namespace {

template<bool Ray>
void clip(const Sweep& sweep, const Solid& solid, const Plane* planes, SweepHit& trace) {
    const auto count = Ray ? solid.faceCount : solid.planeCount;
    float enter = 0, leave = 1;
    float hit = -std::numeric_limits<float>::infinity();
    const Plane* impact = nullptr;
    bool startOut = false, endOut = false;
    for (const auto& plane : std::span(planes, count)) {
        float distance = plane.distance;
        if constexpr (!Ray) distance += absVec(plane.normal).dot(sweep.extents);
        const float d1 = plane.normal.dot(sweep.start) - distance;
        const float d2 = plane.normal.dot(sweep.end) - distance;
        if (d1 > 0) {
            if (d2 > 0) return;
            startOut = true;
            const float inverse = 1.0f / (d1 - d2);
            enter = std::max(enter, d1 * inverse);
            const float backedOff = (d1 - traceEpsilon) * inverse;
            if (backedOff > hit) {
                hit = backedOff;
                impact = &plane;
            }
        } else if (d2 > 0) {
            endOut = true;
            leave = std::min(leave, d1 / (d1 - d2));
        }
        if (enter > leave) return;
    }

    // Keep the exact overlap interval so a grazing entry cannot disappear in one direction.
    // As in CM_ClipBoxToBrush, report an impact slightly before the entering plane.
    if (!startOut) {
        auto& result = trace.result;
        result.startSolid = true;
        if (!endOut) {
            result.allSolid = true;
            result.fraction = 0;
            result.normal = {};
            result.surfaceFlags = 0;
            result.hitDisplacement = false;
            result.contents = solid.contents;
        } else if (result.fraction == 1) {
            result.contents |= solid.contents;
        }
        return;
    }

    trace.consider(enter, std::max(0.0f, hit), impact->normal, solid.contents, impact->surfaceFlags, false);
}

} // namespace

void traceConvex(const Sweep& sweep, const Solid& solid, const Plane* planes, SweepHit& trace) {
    if (sweep.ray()) clip<true>(sweep, solid, planes, trace);
    else clip<false>(sweep, solid, planes, trace);
}

} // namespace navfast::detail
