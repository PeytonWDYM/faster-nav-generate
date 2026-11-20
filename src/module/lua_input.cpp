#include "lua_input.h"

#include <cmath>
#include <limits>

namespace navfast::lua {
namespace {

namespace Type = GarrysMod::Lua::Type;

void expect(ILuaBase& lua, int index, int type) {
    if (!lua.IsType(index, type))
        throw std::runtime_error(std::string("Expected ") + lua.GetTypeName(type));
}

std::string_view string(ILuaBase& lua, int index) {
    expect(lua, index, Type::String);
    unsigned size = 0;
    const auto* data = lua.GetString(index, &size);
    return {data, size};
}

void field(ILuaBase& lua, int index, const char* key) {
    if (index < 0) index = lua.Top() + index + 1;
    lua.PushString(key);
    lua.RawGet(index);
}

double number(ILuaBase& lua, int index) {
    expect(lua, index, Type::Number);
    const auto value = lua.GetNumber(index);
    if (!std::isfinite(value)) throw std::runtime_error("Expected a finite number");
    return value;
}

template <typename T>
T integer(ILuaBase& lua, int index) {
    const auto value = number(lua, index);
    if (value < static_cast<double>(std::numeric_limits<T>::lowest()) ||
        value > static_cast<double>(std::numeric_limits<T>::max()) || value != std::trunc(value))
        throw std::runtime_error("Integer argument out of range");
    return static_cast<T>(value);
}

Vec3 finiteComponents(float x, float y, float z) {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
        throw std::runtime_error("Expected finite vector or angle components");
    return {x, y, z};
}

Vec3 angle(ILuaBase& lua, int index) {
    expect(lua, index, Type::Angle);
    const auto& angle = lua.GetAngle(index);
    return finiteComponents(angle.x, angle.y, angle.z);
}

template <typename T>
void assign(ILuaBase& lua, const char* key, T& value) {
    field(lua, 5, key);
    if (!lua.IsType(-1, Type::Nil)) {
        if constexpr (std::is_same_v<T, std::string>) value = string(lua, -1);
        else if constexpr (std::is_same_v<T, bool>)
            value = lua.IsType(-1, Type::Bool) ? lua.GetBool(-1) : number(lua, -1) != 0;
        else if constexpr (std::is_integral_v<T>) value = integer<T>(lua, -1);
        else value = static_cast<T>(number(lua, -1));
    }
    lua.Pop();
}

std::vector<std::byte> copyBytes(ILuaBase& lua, int index) {
    const auto data = bytes(lua, index);
    return {data.begin(), data.end()};
}

template <typename Read>
void array(ILuaBase& lua, int index, Read read) {
    expect(lua, index, Type::Table);
    const auto length = lua.ObjLen(index);
    for (int i = 1; i <= length; ++i) {
        lua.PushNumber(i);
        lua.RawGet(index);
        read();
        lua.Pop();
    }
}

} // namespace

std::span<const std::byte> bytes(ILuaBase& lua, int index) {
    const auto data = string(lua, index);
    return std::as_bytes(std::span{data.data(), data.size()});
}

Vec3 vector(ILuaBase& lua, int index) {
    expect(lua, index, Type::Vector);
    const auto& v = lua.GetVector(index);
    return finiteComponents(v.x, v.y, v.z);
}

Aabb hullBounds(ILuaBase& lua, int minsIndex, int maxsIndex) {
    const auto mins = vector(lua, minsIndex);
    const auto maxs = vector(lua, maxsIndex);
    if (mins.x > maxs.x || mins.y > maxs.y || mins.z > maxs.z)
        throw std::runtime_error("Trace hull mins must be at or below maxs on each axis");
    return {mins, maxs};
}

std::uint32_t mask(ILuaBase& lua, int index) {
    const auto value = number(lua, index);
    if (value < std::numeric_limits<std::int32_t>::min() || value > std::numeric_limits<std::uint32_t>::max() ||
        value != std::trunc(value)) throw std::runtime_error("Invalid trace mask");
    return static_cast<std::uint32_t>(static_cast<std::int64_t>(value));
}

OwnedJobInputs readJobInputs(ILuaBase& lua) {
    OwnedJobInputs owned;
    owned.bsp = copyBytes(lua, 1);
    auto& inputs = owned.inputs;
    expect(lua, 2, Type::Table);
    lua.PushNil();
    while (lua.Next(2)) {
        const auto model = std::string(string(lua, -2));
        inputs.world.phyFiles.emplace(model, copyBytes(lua, -1));
        lua.Pop();
    }
    array(lua, 3, [&] {
        expect(lua, -1, Type::Table);
        field(lua, -1, "model");
        auto model = std::string(string(lua, -1));
        lua.Pop();
        field(lua, -1, "pos");
        const auto pos = vector(lua, -1);
        lua.Pop();
        field(lua, -1, "ang");
        const auto ang = angle(lua, -1);
        lua.Pop();
        inputs.world.entities.push_back({std::move(model), pos, ang});
    });
    array(lua, 4, [&] { inputs.seedPositions.push_back(vector(lua, -1)); });

    expect(lua, 5, Type::Table);
    assign(lua, "threads", inputs.threads);
    assign(lua, "outputPath", inputs.outputPath);
    assign(lua, "dumpPath", inputs.dumpPath);
    std::string mode = "generate";
    assign(lua, "mode", mode);
    if (mode == "world") inputs.mode = JobInputs::WorldOnly;
    else if (mode != "generate") throw std::runtime_error("Invalid navfast job mode");
    auto& params = inputs.params;
    assign(lua, "slope_limit", params.slopeLimit);
    assign(lua, "slope_tolerance", params.slopeTolerance);
    assign(lua, "displacement_test", params.displacementTest);
    assign(lua, "generate_fencetops", params.generateFenceTops);
    assign(lua, "generate_fixup_jump_areas", params.fixupJumpAreas);
    assign(lua, "generate_jump_connections", params.generateJumpConnections);
    assign(lua, "area_max_size", params.areaMaxSize);
    assign(lua, "exact", params.exact);
    return owned;
}

} // namespace navfast::lua
