set(meteor_demod_sink_test_SOURCES
    meteordemodsinktest.cpp
    ../meteorsettings.cpp
    ../meteormapgeometry.cpp
    ../movingtargetmatcher.cpp
    ../meteordemodsink.cpp
    ../meteorblobdetector.cpp
    ../meteorbaseband.cpp
    ../rmobreport.cpp
)

set(meteor_demod_sink_test_HEADERS
    ../meteorsettings.h
    ../meteormapgeometry.h
    ../movingtargetmatcher.h
    ../meteordemodsink.h
    ../meteorblobdetector.h
    ../meteorbaseband.h
    ../rmobreport.h
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

add_custom_target(meteor_demod_sink_regression
    COMMAND $<TARGET_FILE:meteor_demod_sink_test>
        --test-dir "${CMAKE_CURRENT_LIST_DIR}/tests"
    COMMAND $<TARGET_FILE:meteor_demod_sink_test>
        --test-dir "${CMAKE_CURRENT_LIST_DIR}/tests"
        --chunk-samples 127
    DEPENDS meteor_demod_sink_test
    USES_TERMINAL
    VERBATIM
)
