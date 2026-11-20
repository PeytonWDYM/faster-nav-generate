#pragma once

#include <cstdint>

// Source engine contents / surface flag values (bspflags.h), used by traces and BSP parsing.
namespace navfast::source {

inline constexpr std::uint32_t CONTENTS_EMPTY = 0;
inline constexpr std::uint32_t CONTENTS_SOLID = 0x1;
inline constexpr std::uint32_t CONTENTS_WINDOW = 0x2;
inline constexpr std::uint32_t CONTENTS_AUX = 0x4;
inline constexpr std::uint32_t CONTENTS_GRATE = 0x8;
inline constexpr std::uint32_t CONTENTS_SLIME = 0x10;
inline constexpr std::uint32_t CONTENTS_WATER = 0x20;
inline constexpr std::uint32_t CONTENTS_OPAQUE = 0x80;
inline constexpr std::uint32_t CONTENTS_MOVEABLE = 0x4000;
inline constexpr std::uint32_t CONTENTS_PLAYERCLIP = 0x10000;
inline constexpr std::uint32_t CONTENTS_MONSTERCLIP = 0x20000;
inline constexpr std::uint32_t CONTENTS_MONSTER = 0x2000000;
inline constexpr std::uint32_t CONTENTS_DEBRIS = 0x4000000;
inline constexpr std::uint32_t CONTENTS_LADDER = 0x20000000;

inline constexpr std::uint32_t MASK_WATER = CONTENTS_WATER | CONTENTS_MOVEABLE | CONTENTS_SLIME;
inline constexpr std::uint32_t MASK_NPCSOLID_BRUSHONLY =
    CONTENTS_SOLID | CONTENTS_MOVEABLE | CONTENTS_MONSTERCLIP | CONTENTS_WINDOW | CONTENTS_GRATE;
inline constexpr std::uint32_t MASK_PLAYERSOLID_BRUSHONLY =
    CONTENTS_SOLID | CONTENTS_MOVEABLE | CONTENTS_WINDOW | CONTENTS_PLAYERCLIP | CONTENTS_GRATE;

inline constexpr std::uint32_t SURF_SKY2D = 0x2;
inline constexpr std::uint32_t SURF_SKY = 0x4;
inline constexpr std::uint32_t SURF_NODRAW = 0x80;

} // namespace navfast::source
