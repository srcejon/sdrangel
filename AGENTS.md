# Build Instructions

Use CMake presets by default.

## Linux
Default preset: `default`

Configure:
`cmake --preset default`

Build:
`cmake --build --preset default`

## Windows
Default preset: `default-qt6-windows`

Before configuring, make sure the `external/windows` submodule is checked out so bundled tools such as `pkg-config.exe` and libraries such as `OpenCV` are present.

Always run Windows configure and build commands from `vcvars64.bat` (or an already-open VS developer shell that has run it) so `cl.exe`, the Windows SDK tools, and standard libraries such as `kernel32.lib` are available in the environment.

Example:
`cmd /c "\"C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat\" && cmake --preset default-qt6-windows -G Ninja"`

Submodule checkout:
`git -c safe.directory=<repo-path> submodule update --init --recursive external/windows`

Configure:
`cmd /c "\"C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat\" && cmake --preset default-qt6-windows -G Ninja"`

Build:
`cmd /c "\"C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat\" && cmake --build --preset default-qt6-windows --parallel"`

Allow a longer timeout for Windows builds when running through Codex tools, as dependency and generated-code targets can take several minutes before the first actionable compiler error appears.

Always use `--parallel` for CMake builds, including subtarget builds with `--target`. Do not use `-j1` unless the user explicitly asks for a single-job build or parallelism is being debugged.

Subtarget build example:
`cmd /c "\"C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat\" && cmake --build --preset default-qt6-windows --target featurecamera --parallel"`

Current known caveat:
- The bundled Windows OpenCV package may be detected but still marked unusable; if that happens, targets including camera post-processing code can fail with missing `opencv2/...` headers until `OpenCV_DIR` points to a compatible build.

Working OpenCV override for this machine:
`-DOpenCV_DIR="C:/Users/jon/source/repos/sdrangel-windows-libraries/opencv4/x64/vc17"`

If Qt autogen fails with `libuv process spawn failed: operation not permitted` while running through Codex tools, rerun the Windows build or `cmake -E cmake_autogen` step outside the sandbox from the same VS developer environment.

## Validation
- There is no single top-level unit test runner
- Validate by building the relevant target or plugin for the area you changed
- If CMake or dependency configuration changes, rerun the configure step before building

## Architecture Notes
- SDRangel is a Qt-based SDR application with both GUI and server modes
- Core shared code lives in `sdrbase/`, GUI shared code in `sdrgui/`, and server shared code in `sdrsrv/`
- Most product functionality is plugin-based under `plugins/`
- Device integrations are implemented as sample source, sample sink, or sample MIMO plugins
- Channel processing is implemented under `plugins/channelrx/`, `plugins/channeltx/`, and `plugins/channelmimo/`
- Additional non-I/Q features live under `plugins/feature/`
- DSP-heavy code often lives in `sdrbase/dsp/`

## Repo Conventions
- Follow the presets defined in `CMakePresets.json`
- Prefer the default presets above unless the task explicitly requires a different preset
- Many subsystems use `Message`-derived classes and `Msg*` naming for commands and events
- Long-running hardware or DSP work is commonly implemented in `*worker` classes
- Preserve compatibility with existing `*settings.h/.cpp` patterns when changing saved configuration behavior

## Generated Code
- Do not edit generated files under `swagger/sdrangel/code/` manually
- For WebAPI changes, update the OpenAPI spec or generator configuration instead
- Main OpenAPI spec: `swagger/sdrangel/api/swagger/swagger.yaml`

## Useful Places To Look
- `plugins/channelrx/wdsprx/` is a good reference plugin for channel lifecycle, settings, and documentation
- `scriptsapi/` contains example Python scripts for WebAPI usage
- `plugins/*/*/readme.md` files are primary references for device- or plugin-specific behavior

## Notes
- Follow the presets defined in `CMakePresets.json`
- External dependency paths are configured through preset cache variables such as SDR SDK and Qt locations
