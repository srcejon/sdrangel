#
# Cppcheck static analysis support
#

option(BUILD_CPPCHECK "Build cppcheck target" OFF)

if(NOT BUILD_CPPCHECK)
    return()
endif()

find_program(CPPCHECK_EXECUTABLE cppcheck)

if(NOT CPPCHECK_EXECUTABLE)
    message(FATAL_ERROR
        "BUILD_CPPCHECK is ON but cppcheck was not found")
endif()

message(STATUS "cppcheck found: ${CPPCHECK_EXECUTABLE}")

# Number of parallel jobs
include(ProcessorCount)
ProcessorCount(CPPCHECK_JOBS)

if(CPPCHECK_JOBS EQUAL 0)
    set(CPPCHECK_JOBS 1)
endif()

set(CPPCHECK_OUTPUT_DIR
    ${CMAKE_BINARY_DIR}/cppcheck)

file(MAKE_DIRECTORY ${CPPCHECK_OUTPUT_DIR})

set(CPPCHECK_XML_OUTPUT
    ${CPPCHECK_OUTPUT_DIR}/cppcheck.xml)

# Optional suppressions
set(CPPCHECK_SUPPRESSIONS
    ${CMAKE_SOURCE_DIR}/cppcheck.suppress)

#style and performance back in later
set(CPPCHECK_OPTIONS
    --project=${CMAKE_BINARY_DIR}/compile_commands.json
    --check-level=exhaustive
    --enable=warning,portability
    --library=qt
    --inconclusive
    --inline-suppr
    --xml
    --xml-version=2
    -j${CPPCHECK_JOBS}
    -i ${CMAKE_BINARY_DIR}
    --suppress=missingIncludeSystem
    --relative-paths=${CMAKE_SOURCE_DIR}
    --cppcheck-build-dir=${CMAKE_BINARY_DIR}/cppcheck-build
    --checkers-report=${CMAKE_BINARY_DIR}/cppcheck-report-${CMAKE_PROJECT_NAME}.txt
    --output-file=${CPPCHECK_XML_OUTPUT}
)

if(EXISTS ${CPPCHECK_SUPPRESSIONS})
    list(APPEND CPPCHECK_OPTIONS
        --suppressions-list=${CPPCHECK_SUPPRESSIONS})
endif()

add_custom_target(cppcheck
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/cppcheck-build
    # Ensure all generated Qt AUTOGEN sources referenced by
    # compile_commands.json exist before invoking cppcheck.
    COMMAND ${CMAKE_COMMAND} --build .
    COMMAND ${CPPCHECK_EXECUTABLE}
            ${CPPCHECK_OPTIONS}
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Running cppcheck static analysis"
    VERBATIM
)

# These targets are excluded from the default build but still contribute
# generated sources referenced by compile_commands.json.
add_dependencies(cppcheck modemmeshcore_smoke modmeshcore_cli)

#
# Optional HTML report
#
find_program(CPPCHECK_HTMLREPORT_EXECUTABLE cppcheck-htmlreport)

if(CPPCHECK_HTMLREPORT_EXECUTABLE)

    add_custom_command(TARGET cppcheck POST_BUILD

        COMMAND ${CPPCHECK_HTMLREPORT_EXECUTABLE}
            --file=${CPPCHECK_XML_OUTPUT}
            --report-dir=${CPPCHECK_OUTPUT_DIR}/html
            --source-dir=${CMAKE_SOURCE_DIR}

        COMMENT "Generating cppcheck HTML report"

        VERBATIM
    )

endif()
