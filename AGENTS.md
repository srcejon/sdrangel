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

Before configuring or building on Windows, check that `external/windows/pkg-config-lite/bin/pkg-config.exe` exists.
If it does not, initialize the bundled dependency submodule first:
`git -c safe.directory=<repo-path> submodule update --init --recursive external/windows`

Do not rely on the sandbox for Windows configure/build steps. Run Windows CMake configure and build commands outside the sandbox by default so compiler detection, Qt autogen, and other spawned tools can run normally.

Always run Windows configure and build commands from `vcvars64.bat` (or an already-open VS developer shell that has run it) so `cl.exe`, the Windows SDK tools, and standard libraries such as `kernel32.lib` are available in the environment.

Example:
`cmd /c "C:\PROGRA~1\MICROS~3\2022\COMMUN~1\VC\AUXILI~1\Build\vcvars64.bat && cmake --preset default-qt6-windows -G Ninja"`

Submodule checkout, if `pkg-config.exe` is missing:
`git -c safe.directory=<repo-path> submodule update --init --recursive external/windows`

Configure:
`cmd /c "C:\PROGRA~1\MICROS~3\2022\COMMUN~1\VC\AUXILI~1\Build\vcvars64.bat && cmake --preset default-qt6-windows -G Ninja"`

Build:
`cmd /c "C:\PROGRA~1\MICROS~3\2022\COMMUN~1\VC\AUXILI~1\Build\vcvars64.bat && cmake --build --preset default-qt6-windows --parallel"`

Allow a longer timeout for Windows configure/build commands when running through Codex tools, as dependency and generated-code targets can take several minutes before the first actionable compiler error appears.

Always use `--parallel` for CMake builds, including subtarget builds with `--target`. Do not use `-j1` unless the user explicitly asks for a single-job build or parallelism is being debugged.

Subtarget build example:
`cmd /c "C:\PROGRA~1\MICROS~3\2022\COMMUN~1\VC\AUXILI~1\Build\vcvars64.bat && cmake --build --preset default-qt6-windows --target featurecamera --parallel"`

Current known caveat:
- The bundled Windows OpenCV package may be detected but still marked unusable; if that happens, targets including camera post-processing code can fail with missing `opencv2/...` headers until `OpenCV_DIR` points to a compatible build.

Working OpenCV override for this machine:
`-DOpenCV_DIR="C:/Users/jon/source/repos/sdrangel-windows-libraries/opencv4/x64/vc17/lib"`

If Windows configure/build is being run from Codex tools, prefer requesting escalated execution immediately rather than retrying in the sandbox first.

If Qt autogen fails with `libuv process spawn failed: operation not permitted`, rerun the Windows build or `cmake -E cmake_autogen` step outside the sandbox from the same VS developer environment.

## Validation
- There is no single top-level unit test runner
- Validate by building the relevant target or plugin for the area you changed
- If CMake or dependency configuration changes, rerun the configure step before building

### Camera Star Spectrum Tests
Build and run the standalone optical spectrum extractor tests on Windows with:
`cmd /c "C:\PROGRA~1\MICROS~3\2022\COMMUN~1\VC\AUXILI~1\Build\vcvars64.bat && cmake --build --preset default-qt6-windows --target featurecamera_spectrum_tests --parallel"`
`cmd /c "set PATH=C:\Qt\6.11.1\msvc2022_64\bin;%PATH% && build-qt6\bin\plugins\featurecamera_spectrum_tests.exe"`
The tests are self-contained (synthetic images, no CSV or catalog data) and exit non-zero on failure.

Use the same Qt version the build was configured against (currently 6.11.1). An older Qt on `PATH` usually still runs, but its install may be missing plugin sets the newer one has - a partial 6.11.0 install with no `plugins\imageformats`, for instance, makes any test that reads or writes a JPEG fail with "Unsupported image format".

### Camera Star/Plate Solver Tests
Build the standalone camera star test target on Windows with:
`cmd /c "C:\PROGRA~1\MICROS~3\2022\COMMUN~1\VC\AUXILI~1\Build\vcvars64.bat && cmake --build --preset default-qt6-windows --target featurecamera_star_tests --parallel"`

Profile the standalone camera star/plate solver tests with Visual Studio Diagnostics Tools on Windows with:
`cmd /c "C:\PROGRA~1\MICROS~3\2022\COMMUN~1\VC\AUXILI~1\Build\vcvars64.bat && cmake --build --preset default-qt6-windows --target featurecamera_star_tests_vs_profile --parallel"`

The profiling target runs one unprofiled warmup pass, then writes a `.diagsession` under `build-qt6/plugins/feature/camera/test/vs-profile/`. Open that file in Visual Studio's Performance Profiler.

Run the test outside the sandbox so it uses the real user's `%APPDATA%\f4exb\SDRangel\camera` catalog cache. Set the Qt plugin path and runtime DLL paths explicitly:
`cmd /c "set QT_PLUGIN_PATH=C:\Qt\6.11.1\msvc2022_64\plugins && set PATH=C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v13.2\bin;C:\Users\jon\source\repos\srcejon_sdrangel_fix\external\windows\fftw-3;C:\Users\jon\source\repos\srcejon_sdrangel_fix\external\windows\libsigmf\lib;C:\Users\jon\source\repos\srcejon_sdrangel_fix\build-qt6\bin;C:\Qt\6.11.1\msvc2022_64\bin;C:\Users\jon\source\repos\srcejon_sdrangel_fix\external\windows\opencv4\x64\vc17\bin;%PATH% && build-qt6\bin\plugins\featurecamera_star_tests.exe plugins\feature\camera\test\star-tests.csv"`

When running from a Codex worktree, replace `C:\Users\jon\source\repos\srcejon_sdrangel_fix` with the current worktree path. If catalog/network behavior is being investigated, add `set QT_FORCE_STDERR_LOGGING=1 && set QT_LOGGING_RULES=*.debug=true;*.warning=true &&` before the `PATH` assignment to show Qt TLS and plate-solver diagnostics.

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
