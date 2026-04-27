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

Before configuring, make sure the `external/windows` submodule is checked out so bundled tools such as `pkg-config.exe` are present.

Submodule checkout:
`git -c safe.directory=<repo-path> submodule update --init --recursive external/windows`

Configure:
`cmake --preset default-qt6-windows`

Build:
`cmake --build --preset default-qt6-windows`

Allow a longer timeout for Windows builds when running through Codex tools, as dependency and generated-code targets can take several minutes before the first actionable compiler error appears.

If the Visual Studio generator produces broken project files or MSBuild fails before reaching the changed target, use Ninja from a Visual Studio developer command prompt instead of the preset-generated Visual Studio solution.

Example configure from a VS developer shell:
`cmake -S . -B build-qt6-ninja-vsdev -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo -DDEBUG_OUTPUT=ON -DRX_SAMPLE_24BIT=ON -DARCH_OPT=SSE4_2 -DHIDE_CONSOLE=OFF -DENABLE_AIRSPY=ON -DENABLE_AIRSPYHF=ON -DENABLE_BLADERF=ON -DENABLE_HACKRF=ON -DENABLE_IIO=ON -DENABLE_MIRISDR=OFF -DENABLE_PERSEUS=ON -DENABLE_RTLSDR=ON -DENABLE_SDRPLAY=ON -DENABLE_SOAPYSDR=ON -DENABLE_XTRX=ON -DENABLE_USRP=ON -DBUILD_SERVER=OFF -DENABLE_QT6=ON -DCMAKE_PREFIX_PATH="C:/Qt/6.11.0/msvc2022_64;C:/Applications/boost_1_81_0"`

Example build from a VS developer shell:
`cmake --build build-qt6-ninja-vsdev --target <target> -j1`

The Ninja path should be run from `VsDevCmd.bat` so `cl.exe`, the Windows SDK tools, and standard libraries such as `kernel32.lib` are available in the environment.

Current known caveat:
- The bundled Windows OpenCV package may be detected but still marked unusable; if that happens, targets including camera post-processing code can fail with missing `opencv2/...` headers until `OpenCV_DIR` points to a compatible build.

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
- DSP-heavy code often lives in `wdsp/`

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
