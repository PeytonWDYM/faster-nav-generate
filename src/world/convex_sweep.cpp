#include "world/convex_sweep.h"

#include "world/source_flags.h"

namespace navfast::detail {
namespace {

bool hasNormal(std::span<const Plane> planes, Vec3 normal) {
    return std::ranges::any_of(planes, [normal](const Plane& plane) {
        return plane.normal.dot(normal) > 0 && plane.normal.cross(normal).lengthSqr() < 1e-12f;
    });
}

void addSupport(std::vector<Plane>& planes, std::span<const Vec3> points, Vec3 normal) {
    if (hasNormal(planes, normal)) return;
    float distance = -std::numeric_limits<float>::infinity();
    for (const auto point : points) distance = std::max(distance, point.dot(normal));
    planes.push_back({normal, distance});
}

struct TriangleInterval {
    float enter = 0, leave = 1;
    float hit = -std::numeric_limits<float>::infinity();
    Vec3 normal;

    bool clip(Vec3 axis, const Triangle& tri, const Sweep& sweep) {
        const float radius = absVec(axis).dot(sweep.extents);
        const float d0 = (tri.points[0] - sweep.start).dot(axis);
        const float d1 = (tri.points[1] - sweep.start).dot(axis);
        const float d2 = (tri.points[2] - sweep.start).dot(axis);
        const float min = std::min({d0, d1, d2}) - radius;
        const float max = std::max({d0, d1, d2}) + radius;
        const float speed = sweep.delta.dot(axis);
        if (speed == 0) return min <= 0 && max >= 0;
        const float inverse = 1.0f / speed;
        const float first = (speed > 0 ? min : max) * inverse;
        const float last = (speed > 0 ? max : min) * inverse;
        enter = std::max(enter, first);
        leave = std::min(leave, last);
        if (first >= 0) {
            const float backedOff = first - traceEpsilon * std::abs(inverse);
            if (backedOff > hit) {
                hit = backedOff;
                normal = speed > 0 ? -axis : axis;
            }
        }
        return enter <= leave;
    }
};

void traceTriangleLine(const Sweep& sweep, const Triangle& tri, SweepHit& trace) {
    // Accept either determinant sign so world line traces see both sides of the surface.
    const auto edge1 = tri.points[1] - tri.points[0];
    const auto edge2 = tri.points[2] - tri.points[0];
    const auto p = sweep.delta.cross(edge2);
    const auto determinant = edge1.dot(p);
    if (determinant == 0) return;
    const auto inverse = 1.0f / determinant;
    const auto relative = sweep.start - tri.points[0];
    const auto u = relative.dot(p) * inverse;
    const auto q = relative.cross(edge1);
    const auto v = sweep.delta.dot(q) * inverse;
    constexpr float edgeTolerance = 1e-6f;
    if (u < -edgeTolerance || v < -edgeTolerance || u + v > 1 + edgeTolerance) return;

    const auto fraction = edge2.dot(q) * inverse;
    if (fraction >= 0 && fraction < 1) {
        trace.consider(fraction, fraction, tri.normal, source::CONTENTS_SOLID, tri.surfaceFlags, true);
    }
}

} // namespace

void appendBoxBevels(std::vector<Plane>& planes, std::span<const Vec3> points,
                     std::span<const Vec3> edges) {
    for (const auto axis : axes) {
        addSupport(planes, points, axis);
        addSupport(planes, points, -axis);
    }
    for (const auto edge : edges) {
        for (const auto axis : axes) {
            const auto cross = edge.cross(axis);
            const auto length = cross.length();
            if (length == 0) continue;
            const auto normal = cross * (1.0f / length);
            addSupport(planes, points, normal);
            addSupport(planes, points, -normal);
        }
    }
}

void traceDisplacement(const Sweep& sweep, const Triangle& tri, SweepHit& trace) {
    if (sweep.ray()) {
        if (tri.rayCollision) traceTriangleLine(sweep, tri, trace);
        return;
    }

    // Source hull sweeps ignore a displacement approached from its back face.
    if (!tri.hullCollision || tri.normal.dot(sweep.delta) > traceEpsilon) return;
    TriangleInterval interval;
    if (!interval.clip(tri.normal, tri, sweep)) return;
    for (const auto axis : axes) {
        if (!interval.clip(axis, tri, sweep)) return;
    }
    for (std::size_t i = 0; i < tri.points.size(); ++i) {
        const auto edge = tri.points[(i + 1) % tri.points.size()] - tri.points[i];
        for (const auto axis : axes) {
            const auto cross = edge.cross(axis);
            const auto length = cross.length();
            if (length != 0 && !interval.clip(cross * (1.0f / length), tri, sweep)) return;
        }
    }

    if (sweep.stationary()) {
        trace.result = {0, {}, {}, true, true, source::CONTENTS_SOLID, tri.surfaceFlags, true};
    } else if (std::isfinite(interval.hit)) {
        trace.consider(interval.enter, std::max(0.0f, interval.hit), interval.normal,
                       source::CONTENTS_SOLID, tri.surfaceFlags, true);
    }
}

} // namespace navfast::detail
