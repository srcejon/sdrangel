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

Configure:
`cmake --preset default-qt6-windows`

Build:
`cmake --build --preset default-qt6-windows`

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
