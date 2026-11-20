#pragma once

#include "core/vec3.h"
#include "world/source_flags.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <string_view>

namespace navfast {

struct GenerationParams {
    float slopeLimit = 0.7f;
    float slopeTolerance = 0.1f;
    int displacementTest = 10000;
    bool generateFenceTops = true;
    bool fixupJumpAreas = true;
    bool generateJumpConnections = true;
    int areaMaxSize = 50;
    bool exact = false;
};

using Progress = std::function<void(float, std::string_view)>;
using Phase = std::function<void(std::string_view)>;

inline constexpr float GenerationStepSize = 25.0f;
inline constexpr float StepHeight = 18.0f;
inline constexpr float JumpCrouchHeight = 64.0f;
inline constexpr float DeathDrop = 400.0f;
inline constexpr float ClimbUpHeight = 200.0f;
inline constexpr float HalfHumanWidth = 16.0f;
inline constexpr float HalfHumanHeight = 35.5f;
inline constexpr float HumanHeight = 71.0f;
inline constexpr float HumanCrouchHeight = 55.0f;
inline constexpr float DuckHullHeight = 36.0f;
inline constexpr Vec3 NavTraceMins{-0.45f, -0.45f, 0};
inline constexpr Vec3 NavTraceMaxs{0.45f, 0.45f, HumanCrouchHeight};
inline constexpr auto GenerationTraceMask = source::MASK_NPCSOLID_BRUSHONLY;

enum Direction : unsigned { North, East, South, West };
inline constexpr std::array directions{North, East, South, West};
inline constexpr std::array<Vec3, 4> directionVector{{{0,-1,0}, {1,0,0}, {0,1,0}, {-1,0,0}}};
constexpr Direction opposite(Direction dir) { return directions[(dir + 2) % 4]; }
constexpr bool alongX(Direction dir) { return dir == East || dir == West; }

enum Corner : unsigned { NW, NE, SE, SW };
inline constexpr std::array corners{NW, NE, SE, SW};
inline constexpr std::array<Vec3, 4> cornerVector{{{-1,-1,0}, {1,-1,0}, {1,1,0}, {-1,1,0}}};
inline constexpr std::array<std::array<Corner, 2>, 4> edgeCorners{{
    {NW, NE}, {NE, SE}, {SW, SE}, {NW, SW}
}};

enum Attribute : std::uint32_t {
    Crouch = 0x01, Jump = 0x02, Stairs = 0x1000, NoMerge = 0x2000, ObstacleTop = 0x4000
};

inline float snapToGrid(float coordinate) {
    return std::round(coordinate / GenerationStepSize) * GenerationStepSize;
}

} // namespace navfast
