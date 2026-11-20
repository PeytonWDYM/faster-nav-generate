#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <unordered_map>
#include <vector>

#include "core/vec3.h"

// Contract between map loading and nav generation.
//
// The collision world is a frozen snapshot of the map's static collision (BSP brushes,
// displacements, static props) plus the solid entities the caller decided are obstacles.
// After construction it is immutable, so every query is safe to call from any thread.
namespace navfast {

// A solid entity to bake into the world (doors and other "walkable" entities are filtered out
// by the caller before it gets here, mirroring Source's IsEntityWalkable()).
struct SolidEntity {
    std::string model;  // "*N" for a BSP brush submodel, otherwise a studio model path ("models/x.mdl")
    Vec3 origin;
    Vec3 angles;        // pitch, yaw, roll in degrees (Source convention)
};

struct WorldInputs {
    std::span<const std::byte> bsp;                              // whole .bsp file
    // Raw .phy files keyed by lower-case model path with forward slashes ("models/props/x.mdl").
    std::unordered_map<std::string, std::vector<std::byte>> phyFiles;
    std::vector<SolidEntity> entities;
};

// Result of a swept box (or ray, when mins == maxs == 0) against the world.
// Semantics follow Source's trace_t.
struct TraceResult {
    float fraction = 1.0f;          // 0..1 along start->end where the sweep stopped
    Vec3 endpos;                    // start + (end - start) * fraction
    Vec3 normal;                    // plane normal of the hit surface (zero if nothing hit)
    bool startSolid = false;        // box started inside solid
    bool allSolid = false;          // box never left solid
    std::uint32_t contents = 0;     // contents of what was hit
    std::uint32_t surfaceFlags = 0; // SURF_* of the hit surface (SURF_SKY etc.)
    bool hitDisplacement = false;   // hit a displacement surface (trace_t::IsDispSurface())

    bool didHit() const { return fraction < 1.0f || startSolid; }
};

class CollisionWorld {
public:
    virtual ~CollisionWorld() = default;

    // Sweep an axis-aligned box from start to end. `mask` is a CONTENTS_* mask (see source_flags.h).
    virtual TraceResult traceHull(Vec3 start, Vec3 end, Vec3 mins, Vec3 maxs, std::uint32_t mask) const = 0;

    // Ray trace (zero-extent hull).
    TraceResult traceLine(Vec3 start, Vec3 end, std::uint32_t mask) const {
        return traceHull(start, end, {}, {}, mask);
    }

    // Union of CONTENTS_* of everything containing `point`.
    virtual std::uint32_t pointContents(Vec3 point) const = 0;

    // World bounds (worldspawn model bounds).
    virtual Aabb bounds() const = 0;
};

// Lower-case static prop model paths referenced by the BSP's static prop game lump.
// The caller uses this to fetch the matching .phy files before building the world.
std::vector<std::string> listStaticPropModels(std::span<const std::byte> bsp);

// Parse everything and build acceleration structures. Throws std::runtime_error on malformed input.
// A missing .phy for a model is not an error: that model is skipped (and counted in a warning).
std::unique_ptr<CollisionWorld> buildCollisionWorld(const WorldInputs& inputs);

} // namespace navfast
