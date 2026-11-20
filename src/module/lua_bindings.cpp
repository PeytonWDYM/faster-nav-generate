#include "lua_input.h"

#include <memory>

namespace {

using namespace navfast;
using namespace GarrysMod::Lua;

std::unique_ptr<Job> job;

void field(ILuaBase& lua, const char* name, const std::string& value) {
    lua.PushString(value.c_str());
    lua.SetField(-2, name);
}

void field(ILuaBase& lua, const char* name, double value) {
    lua.PushNumber(value);
    lua.SetField(-2, name);
}

void field(ILuaBase& lua, const char* name, bool value) {
    lua.PushBool(value);
    lua.SetField(-2, name);
}

void field(ILuaBase& lua, const char* name, Vec3 value) {
    Vector vector;
    vector.x = value.x;
    vector.y = value.y;
    vector.z = value.z;
    lua.PushVector(vector);
    lua.SetField(-2, name);
}

template <typename Body>
int call(ILuaBase& lua, Body body) {
    try {
        return body();
    } catch (const std::exception& failure) {
        lua.PushString(failure.what());
    }
    // GMod raises a Lua error via longjmp. All owning C++ locals have already unwound.
    lua.ThrowError(lua.GetString(-1));
    return 0;
}

LUA_FUNCTION(StaticPropModels) {
    return call(*LUA, [&] {
        const auto models = listStaticPropModels(lua::bytes(*LUA, 1));
        LUA->CreateTable();
        for (std::size_t i = 0; i < models.size(); ++i) {
            LUA->PushNumber(static_cast<double>(i + 1));
            LUA->PushString(models[i].c_str());
            LUA->RawSet(-3);
        }
        return 1;
    });
}

LUA_FUNCTION(Start) {
    return call(*LUA, [&] {
        if (job->running()) throw std::runtime_error("A navfast job is already running");
        job->start(lua::readJobInputs(*LUA));
        return 0;
    });
}

LUA_FUNCTION(Poll) {
    return call(*LUA, [&] {
        const auto status = job->poll();
        LUA->CreateTable();
        field(*LUA, "running", status.running);
        field(*LUA, "done", status.done);
        field(*LUA, "phase", status.phase);
        field(*LUA, "progress", static_cast<double>(status.progress));
        field(*LUA, "message", status.message);
        if (!status.error.empty()) field(*LUA, "error", status.error);
        if (status.stats) {
            const auto& stats = *status.stats;
            LUA->CreateTable();
            field(*LUA, "nodes", static_cast<double>(stats.nodes));
            field(*LUA, "areas", static_cast<double>(stats.areas));
            field(*LUA, "connections", static_cast<double>(stats.connections));
            field(*LUA, "threads", static_cast<double>(stats.threads));
            field(*LUA, "seconds", stats.seconds);
            field(*LUA, "worldMins", stats.worldBounds.mins);
            field(*LUA, "worldMaxs", stats.worldBounds.maxs);
            LUA->CreateTable();
            for (const auto& phase : stats.phases) field(*LUA, phase.name.c_str(), phase.seconds);
            LUA->SetField(-2, "phaseSeconds");
            LUA->SetField(-2, "stats");
        }
        return 1;
    });
}

LUA_FUNCTION(Cancel) {
    job->cancel();
    static_cast<void>(LUA);
    return 0;
}

LUA_FUNCTION(TraceHull) {
    return call(*LUA, [&] {
        const auto start = lua::vector(*LUA, 1);
        const auto end = lua::vector(*LUA, 2);
        const auto mins = lua::vector(*LUA, 3);
        const auto maxs = lua::vector(*LUA, 4);
        const auto mask = lua::mask(*LUA, 5);
        const auto world = job->world();
        if (!world) throw std::runtime_error("No navfast collision world is ready");
        const auto trace = world->traceHull(start, end, mins, maxs, mask);
        LUA->CreateTable();
        field(*LUA, "Fraction", static_cast<double>(trace.fraction));
        field(*LUA, "HitPos", trace.endpos);
        field(*LUA, "HitNormal", trace.normal);
        field(*LUA, "StartSolid", trace.startSolid);
        field(*LUA, "AllSolid", trace.allSolid);
        field(*LUA, "Contents", static_cast<double>(trace.contents));
        field(*LUA, "SurfaceFlags", static_cast<double>(trace.surfaceFlags));
        field(*LUA, "HitDisplacement", trace.hitDisplacement);
        return 1;
    });
}

} // namespace

GMOD_MODULE_OPEN() {
    job = std::make_unique<Job>();
    LUA->PushSpecial(GarrysMod::Lua::SPECIAL_GLOB);
    LUA->CreateTable();
    for (const auto& [name, function] : {std::pair{"StaticPropModels", StaticPropModels},
         std::pair{"Start", Start}, std::pair{"Poll", Poll}, std::pair{"Cancel", Cancel},
         std::pair{"TraceHull", TraceHull}}) {
        LUA->PushCFunction(function);
        LUA->SetField(-2, name);
    }
    LUA->SetField(-2, "navfast");
    LUA->Pop();
    return 0;
}

GMOD_MODULE_CLOSE() {
    job.reset();
    static_cast<void>(LUA);
    return 0;
}
