set(meteor_demod_sink_test_SOURCES
    meteordemodsinktest.cpp
    ../meteorsettings.cpp
    ../meteordemodsink.cpp
    ../meteorbaseband.cpp
)

set(meteor_demod_sink_test_HEADERS
    ../meteorsettings.h
    ../meteordemodsink.h
    ../meteorbaseband.h
)

add_executable(meteor_demod_sink_test EXCLUDE_FROM_ALL
    ${meteor_demod_sink_test_SOURCES}
    ${meteor_demod_sink_test_HEADERS}
)

set_target_properties(meteor_demod_sink_test PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${SDRANGEL_BINARY_BIN_DIR}
    RUNTIME_OUTPUT_DIRECTORY_DEBUG ${SDRANGEL_BINARY_BIN_DIR}
    RUNTIME_OUTPUT_DIRECTORY_RELEASE ${SDRANGEL_BINARY_BIN_DIR}
    RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO ${SDRANGEL_BINARY_BIN_DIR}
)

target_include_directories(meteor_demod_sink_test PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/..
)

target_compile_definitions(meteor_demod_sink_test PRIVATE
    METEOR_TEST_DATA_DIR="${CMAKE_CURRENT_LIST_DIR}/tests"
)

target_link_libraries(meteor_demod_sink_test PRIVATE
    Qt::Core
    Qt::Gui
    sdrbase
)
