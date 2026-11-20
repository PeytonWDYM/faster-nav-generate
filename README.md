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
garrysmod/lua/bin/gmsv_navfast_win64.dll    (64-bit Windows server, x86-64 branch)
garrysmod/lua/bin/gmsv_navfast_linux64.dll  (64-bit Linux server)
garrysmod/addons/navfast/                    (Lua: command, progress UI)
```

## Usage

Run in the server console, or as a superadmin from the in-game console:

```
nav_generate_fast
```

Progress is printed to the server console, and superadmins in game see a small progress panel.

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

CI builds on a private self-hosted Linux runner (clang-cl + xwin for Windows, clang for Linux).
Pushing a `v*` tag publishes a release zip. The workflow has no `pull_request` trigger, so forks
can never run jobs on that runner.

## License note

The generation logic is ported from Valve's Source SDK 2013 and is distributed under the
[Source SDK license](https://github.com/ValveSoftware/source-sdk-2013/blob/master/LICENSE):
free of charge, for use with Source engine games.
