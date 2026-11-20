<img src="assets/icon.svg" width="64" height="64" alt="">

# Faster nav_generate

`nav_generate_fast` builds a Garry's Mod navigation mesh for the current map using every CPU core,
without freezing the server.

It follows the same generation steps as Valve's `nav_generate` (Source SDK 2013): sample walkable
space on a 25-unit grid, build areas from the samples, then merge, connect and fix them up. The
difference is where it runs. Vanilla generation runs on the game thread in small time slices.
`navfast` loads the map's collision into its own thread-safe collision world and runs generation
on worker threads, while the game keeps ticking.

## How it works

1. The addon's Lua reads the map's `.bsp` and every needed `.phy` file through the game filesystem,
   so workshop maps and mounted content work. It also collects spawn points and the solid entities
   that should block walking.
2. The binary module parses the BSP brushes, displacements and static props into a frozen
   collision world, then runs the generator across all cores at below-normal priority.
3. The result is written to `garrysmod/maps/<map>.nav` (any existing file is kept as
   `<map>.nav.bak`) and loaded with `navmesh.Load()`.

The module only uses the Lua C API. It has no engine signatures or offsets, so GMod updates
don't break it.

The hiding, encounter and sniper spot analysis from vanilla is skipped. Only CS-style bots use
that data; NextBots only need the areas and connections.

## Install

Download `navfast.zip` from Releases and merge its `garrysmod/` folder into your server's
`garrysmod/` folder:

```
garrysmod/lua/bin/gmsv_navfast_win64.dll    (64-bit Windows srcds, srcds_win64.exe)
garrysmod/lua/bin/gmsv_navfast_win32.dll    (32-bit Windows srcds, srcds.exe)
garrysmod/lua/bin/gmsv_navfast_linux64.dll  (64-bit Linux srcds)
garrysmod/lua/bin/gmsv_navfast_linux.dll    (32-bit Linux srcds)
garrysmod/addons/navfast/                    (Lua: commands, progress UI, viewer)
```

GMod only loads the module matching the server process's bitness; shipping all four is harmless.

## Usage

Run in the server console, or as a superadmin from the in-game console:

```
nav_generate_fast
```

Progress is printed to the server console, and superadmins in game see a small progress panel.

| Command | Meaning |
| --- | --- |
| `nav_generate_fast` | Generate and load the current map's nav mesh. Seeds come from every spawn point, your own position, and any marks. |
| `nav_generate_fast_mark` | Mark where you stand as an extra starting point (like `nav_mark_walkable`), e.g. a floor below a big drop. Cleared on map change. |
| `nav_generate_fast_clear_marks` | Remove all marks. |
| `nav_generate_fast_cancel` | Cancel a running job. |
| `nav_generate_fast_verify [count]` | Compare the module's traces against the engine's and save a report to `data/navfast/`. |
| `nav_view_fast 1` / `0` | Client-side viewer for the loaded mesh, styled after `nav_edit 1`. No `sv_cheats` needed. `nav_view_fast_range` sets the draw distance. |

| Convar | Default | Meaning |
| --- | --- | --- |
| `nav_generate_fast_threads` | `0` | Worker threads. `0` = every logical core. |
| `nav_generate_fast_dump` | `0` | `1` also saves the job inputs to `garrysmod/data/navfast/<map>.navjob` for offline replay. |

The vanilla tuning convars (`nav_slope_limit`, `nav_area_max_size`, `nav_generate_fencetops`, ...)
are read and respected.

## Building

Requires CMake 3.25+ and a C++20 compiler.

```
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
```

Targets: `navfast_module` (the GMod module), `navfast_cli` (replays a `.navjob` offline) and
`world_probe` (collision world checks against a `.bsp` on disk).

CI builds all four modules on a private self-hosted Linux runner (clang-cl + xwin for Windows,
clang for Linux, `-m32` via g++-multilib for 32-bit Linux).
Pushing a `v*` tag publishes a release zip. The workflow has no `pull_request` trigger, so forks
can never run jobs on that runner.

## License note

The generation logic is ported from Valve's Source SDK 2013 and is distributed under the
[Source SDK license](https://github.com/ValveSoftware/source-sdk-2013/blob/master/LICENSE):
free of charge, for use with Source engine games.
