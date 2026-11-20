#pragma once

#include "job/job.h"

#include <GarrysMod/Lua/Interface.h>

namespace navfast::lua {

using GarrysMod::Lua::ILuaBase;

std::span<const std::byte> bytes(ILuaBase& lua, int index);
Vec3 vector(ILuaBase& lua, int index);
std::uint32_t mask(ILuaBase& lua, int index);
OwnedJobInputs readJobInputs(ILuaBase& lua);

} // namespace navfast::lua
