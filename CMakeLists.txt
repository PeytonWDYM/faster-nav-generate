cmake_minimum_required(VERSION 3.25)
project(navfast VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Static CRT on Windows so the module has no VC++ redistributable dependency.
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

include(FetchContent)
# Facepunch's Lua C API headers (header-only). Pinned for reproducible builds.
FetchContent_Declare(gmod_module_base
    GIT_REPOSITORY https://github.com/Facepunch/gmod-module-base.git
    GIT_TAG e0156e5f96cd0bad8eed738f0d20e2c9a14b2a0d
    SOURCE_SUBDIR include)
FetchContent_MakeAvailable(gmod_module_base)

find_package(Threads REQUIRED)

if(MSVC)
    set(NAVFAST_WARNINGS /W4 /permissive-)
else()
    set(NAVFAST_WARNINGS -Wall -Wextra)
endif()

# Engine-free core: map parsing, collision world, nav generation, .nav writing, job runner.
file(GLOB_RECURSE NAVFAST_CORE_SOURCES CONFIGURE_DEPENDS
    src/core/*.cpp src/formats/*.cpp src/world/*.cpp src/nav/*.cpp src/job/*.cpp)
add_library(navfast_core STATIC ${NAVFAST_CORE_SOURCES})
target_include_directories(navfast_core PUBLIC src)
target_link_libraries(navfast_core PUBLIC Threads::Threads)
target_compile_options(navfast_core PRIVATE ${NAVFAST_WARNINGS})

# Garry's Mod binary module: gmsv_navfast_<platform>.dll
file(GLOB_RECURSE NAVFAST_MODULE_SOURCES CONFIGURE_DEPENDS src/module/*.cpp)
add_library(navfast_module SHARED ${NAVFAST_MODULE_SOURCES})
target_link_libraries(navfast_module PRIVATE navfast_core gmod-module-base)
target_compile_options(navfast_module PRIVATE ${NAVFAST_WARNINGS})
set_gmod_suffix_prefix(navfast_module)
set_target_properties(navfast_module PROPERTIES OUTPUT_NAME navfast)

# Offline tool: loads a .bsp from disk and probes the collision world (trace sanity checks, timing).
file(GLOB_RECURSE NAVFAST_PROBE_SOURCES CONFIGURE_DEPENDS tools/world_probe/*.cpp)
add_executable(world_probe ${NAVFAST_PROBE_SOURCES})
target_link_libraries(world_probe PRIVATE navfast_core)
target_compile_options(world_probe PRIVATE ${NAVFAST_WARNINGS})

# Offline tool: replays a dumped .navjob (same inputs the module saw) without running the game.
file(GLOB_RECURSE NAVFAST_CLI_SOURCES CONFIGURE_DEPENDS tools/navfast_cli/*.cpp)
add_executable(navfast_cli ${NAVFAST_CLI_SOURCES})
target_link_libraries(navfast_cli PRIVATE navfast_core)
target_compile_options(navfast_cli PRIVATE ${NAVFAST_WARNINGS})
