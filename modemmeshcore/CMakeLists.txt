project (modemmeshcore)

set(modemmeshcore_SOURCES
    meshcorepacket.cpp
    meshcore_crypto.cpp
    meshcore_builders.cpp
    meshcore_command.cpp
    meshcore_decoder.cpp
    meshcore_identity.cpp
    monocypher.c
    monocypher-ed25519.c
)

set(modemmeshcore_HEADERS
    meshcorepacket.h
    meshcore_crypto.h
    meshcore_builders.h
    meshcore_command.h
    meshcore_decoder.h
    meshcore_identity.h
    monocypher.h
    monocypher-ed25519.h
    tiny_aes.h
)

include_directories(
    ${CMAKE_SOURCE_DIR}/exports
)

add_library(modemmeshcore
    ${modemmeshcore_SOURCES}
)

# Vendored Monocypher (BSD-2-Clause OR CC0-1.0) is intentionally untouched;
# silence sdrangel's strict-warning policy for the two .c files.
if (NOT MSVC)
    set_source_files_properties(monocypher.c monocypher-ed25519.c
        PROPERTIES COMPILE_FLAGS "-Wno-unused-function -Wno-unused-parameter"
    )
endif()

target_link_libraries(modemmeshcore
    Qt::Core
)

install(TARGETS modemmeshcore DESTINATION ${INSTALL_LIB_DIR})

# Smoke test executable. Not installed; run from build dir to validate the
# crypto + builder round-trip after edits to the lib.
add_executable(modemmeshcore_smoke EXCLUDE_FROM_ALL test/smoke.cpp)
target_link_libraries(modemmeshcore_smoke PRIVATE modemmeshcore Qt::Core)

# CLI tool: prints wire packet hex for a MESHCORE: command line. Used for
# golden-vector parity tests against the gr4-lora python meshcore_tx.py.
add_executable(modmeshcore_cli EXCLUDE_FROM_ALL test/cli.cpp)
target_link_libraries(modmeshcore_cli PRIVATE modemmeshcore Qt::Core)
