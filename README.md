# CppTemplate

A starting point for C++20 games and simulations that runs on Windows, Linux, and in the browser. It comes with raylib rendering (lighting, shadows, sky, terrain) and audio, Dear ImGui tools, Cyclone physics, game AI, an event-driven game architecture, and netcode: TCP, UDP with LAN discovery, and a WebSocket relay that works the same on desktop and web.

**Status:** builds on Windows (MSVC) and for the web (Emscripten, WebGL2). The game in `app/` is a short starting point; the sandbox example shows every library working together; and the unit tests cover core, net, ai, engine, and Cyclone.

## Getting started

Windows 10/11, from the repo folder in File Explorer:

| File | What it does |
| --- | --- |
| `install.bat` | Installs whatever is missing (Git, CMake, Ninja, Python, the MSVC build tools, emsdk). Run once; safe to re-run. Nothing needs restarting afterwards. |
| `run.bat` | Builds the game and starts it. The first build downloads and compiles the libraries (several minutes, once). |
| `run-web.bat` | Builds the browser version and opens it at http://localhost:8080. Closing the window stops the server. |

## Layout

| Folder | Contents |
| --- | --- |
| `app/src/` | The game: `Game.cpp` drops physics boxes on a floor (Space drops another, R resets) and is where game code goes |
| `app/assets/` | Images and sounds, loaded as `"assets/name.png"` |
| `app/config/` | Runtime settings in `game.yaml`; changes apply without a rebuild |
| `examples/sandbox/` | A demo of every library: lighting and shadows, terrain, the physics playground, boids, audio, networking. Reference for how to use each one. |
| `libs/cyclone/` | Cyclone physics (Millington, *Game Physics Engine Development*) |
| `libs/engine/` | The game loop, rendering, lighting, audio, input, animation |
| `libs/core/` | Events, components, memory pools, config, timers |
| `libs/ai/`, `net/`, `gpu/` | Game AI, networking, WebGPU compute (optional) |
| `libs/vendor/` | Third-party code: raylib, Dear ImGui, rlImGui, FastNoiseLite |
| `tests/` | Unit tests for every library |
| `server/` | The WebSocket relay server |
| `tools/` | The install script |

## Working together

- Pull before starting work, so the partner's latest changes are included.
- Commit small, self-contained changes with a message saying what changed.
- Push once the project builds and runs.
- A conflict means both people changed the same lines; resolve it together rather than discarding either side. Agreeing on who edits which file avoids most of them.
- `build/` and `.cpm-cache/` are build output and are ignored by git.

<details>
<summary><b>Troubleshooting</b></summary>

- **"was not found" or "emsdk was not found":** a tool is missing or failed to install. `install.bat` lists what is still missing.
- **The first build seems stuck:** it is downloading and compiling the networking libraries; allow up to 10 minutes.
- **A build fails:** the last red lines in the window name the step and the error. `install.bat` reports anything still missing.
- **The browser page is blank:** refresh once the window says "Serving".

</details>

<details>
<summary><b>Playing over the network</b></summary>

Start `build\windows-debug\bin\relay_server.exe`, open two copies of the sandbox (`.\build.ps1 -Run -Sandbox`, or two tabs of http://localhost:8080/sandbox.html), press **Host** in one and **Join** with its code in the other. Pressing R in either one resets both scenes.

</details>

<details>
<summary><b>Command line</b></summary>

From PowerShell in the repo root:

| Command | What it does |
| --- | --- |
| `.\build.ps1` | Debug build, then the unit tests |
| `.\build.ps1 -Run` | Also starts the game |
| `.\build.ps1 -Run -Sandbox` | Starts the sandbox example instead |
| `.\build.ps1 -Config Release` | Release build |
| `.\build.ps1 -SkipTests` | Skips the unit tests |
| `.\build.ps1 -Web -Run` | Browser build, served on :8080 |
| `.\build.ps1 -GpuCompute` | Also builds libs/gpu (WebGPU compute); stays on until set OFF |
| `.\tools\install-windows.ps1 -CheckOnly` | Reports what's installed without changing anything |
| `.\tools\install-windows.ps1 -NoWeb` | Installs everything except emsdk (desktop builds only) |

</details>

<details>
<summary><b>CLion</b></summary>

Open the folder and enable the `Windows Debug`, `Windows Release`, or `Web` profile when CLion offers the presets. They use CLion's bundled Ninja and its default "Visual Studio" toolchain, and share build folders with `build.ps1`, so the libraries are only compiled once.

</details>

<details>
<summary><b>Linux (Ubuntu 24.04)</b></summary>

```bash
sudo apt install build-essential cmake ninja-build git libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev libgl1-mesa-dev
```

```bash
cmake --preset release && cmake --build --preset release && ctest --preset release
```

</details>

<details>
<summary><b>Licenses</b></summary>

Vendored and downloaded libraries keep their own licenses: raylib (zlib), Dear ImGui and rlImGui (MIT), FastNoiseLite (MIT), Asio (Boost), nlohmann_json (MIT), yaml-cpp (MIT), doctest (MIT), MbedTLS (Apache-2.0), libdatachannel (MPL-2.0). Choose a license for your own code before publishing.

</details>
