project(sdrplay)

set(sdrplayv3_SOURCES
    sdrplayv3input.cpp
    sdrplayv3plugin.cpp
    sdrplayv3settings.cpp
    sdrplayv3webapiadapter.cpp
    sdrplayv3thread.cpp
)

set(sdrplayv3_HEADERS
    sdrplayv3input.h
    sdrplayv3plugin.h
    sdrplayv3settings.h
    sdrplayv3webapiadapter.h
    sdrplayv3thread.h
)

include_directories(
    ${CMAKE_SOURCE_DIR}/swagger/sdrangel/code/qt5/client
    ${SDRPLAY_INCLUDE_DIR}
)

if(NOT SERVER_MODE)
    set(sdrplayv3_SOURCES
        ${sdrplayv3_SOURCES}
        sdrplayv3gui.cpp
        sdrplayv3gui.ui
    )
    set(sdrplayv3_HEADERS
        ${sdrplayv3_HEADERS}
        sdrplayv3gui.h
    )

    set(TARGET_NAME ${PLUGINS_PREFIX}inputsdrplayv3)
    set(TARGET_LIB "Qt::Widgets")
    set(TARGET_LIB_GUI "sdrgui")
    set(INSTALL_FOLDER ${INSTALL_PLUGINS_DIR})
else()
    set(TARGET_NAME ${PLUGINSSRV_PREFIX}inputsdrplayv3srv)
    set(TARGET_LIB "")
    set(TARGET_LIB_GUI "")
    set(INSTALL_FOLDER ${INSTALL_PLUGINSSRV_DIR})
endif()

if(NOT Qt6_FOUND)
    add_library(${TARGET_NAME} ${sdrplayv3_SOURCES})
else()
    qt_add_plugin(${TARGET_NAME} CLASS_NAME SDRPlayV3Plugin)
    target_sources(${TARGET_NAME} PRIVATE ${sdrplayv3_SOURCES})
endif()

if(NOT BUILD_SHARED_LIBS)
    set_property(GLOBAL APPEND PROPERTY STATIC_PLUGINS_PROPERTY ${TARGET_NAME})
endif()

target_link_libraries(${TARGET_NAME} PRIVATE
        Qt::Core
        ${TARGET_LIB}
	sdrbase
	${TARGET_LIB_GUI}
        swagger
        ${SDRPLAY_LIBRARIES}
)

# Fix path to library
if (APPLE AND BUILD_SHARED_LIBS AND SDRPLAY_LIBRARIES)
    find_program(OTOOL_EXECUTABLE otool)
    execute_process(
        COMMAND ${OTOOL_EXECUTABLE} -D ${SDRPLAY_LIBRARIES}
        OUTPUT_VARIABLE SDRPLAY_OTOOL_OUTPUT
        RESULT_VARIABLE SDRPLAY_OTOOL_RESULT
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    # Output is "<file>:" followed by the install name (repeated per architecture for universal binaries)
    string(REPLACE "\n" ";" SDRPLAY_OTOOL_LINES "${SDRPLAY_OTOOL_OUTPUT}")
    list(LENGTH SDRPLAY_OTOOL_LINES SDRPLAY_OTOOL_LINE_COUNT)
    if (SDRPLAY_OTOOL_RESULT EQUAL 0 AND SDRPLAY_OTOOL_LINE_COUNT GREATER 1)
        list(GET SDRPLAY_OTOOL_LINES -1 SDRPLAY_INSTALL_NAME)
        string(STRIP "${SDRPLAY_INSTALL_NAME}" SDRPLAY_INSTALL_NAME)
    else()
        set(SDRPLAY_INSTALL_NAME "")
    endif()
    if (SDRPLAY_INSTALL_NAME STREQUAL "")
        message(WARNING "Couldn't get install name of ${SDRPLAY_LIBRARIES}. SDRplay plugin may fail to load from app bundle.")
    elseif (NOT IS_ABSOLUTE "${SDRPLAY_INSTALL_NAME}")
        get_filename_component(SDRPLAY_LIB_DIR "${SDRPLAY_LIBRARIES}" DIRECTORY)
        get_filename_component(SDRPLAY_LIB_NAME "${SDRPLAY_INSTALL_NAME}" NAME)
        set(SDRPLAY_ABSOLUTE_INSTALL_NAME "${SDRPLAY_LIB_DIR}/${SDRPLAY_LIB_NAME}")
        message(STATUS "SDRplay plugin: changing ${SDRPLAY_INSTALL_NAME} to ${SDRPLAY_ABSOLUTE_INSTALL_NAME}")
        add_custom_command(TARGET ${TARGET_NAME}
            POST_BUILD COMMAND
            ${CMAKE_INSTALL_NAME_TOOL} -change ${SDRPLAY_INSTALL_NAME} ${SDRPLAY_ABSOLUTE_INSTALL_NAME} $<TARGET_FILE:${TARGET_NAME}>)
    endif()
endif()

install(TARGETS ${TARGET_NAME} DESTINATION ${INSTALL_FOLDER})

# Install debug symbols
if (WIN32)
    install(FILES $<TARGET_PROPERTY:${TARGET_NAME},RUNTIME_OUTPUT_DIRECTORY>/${TARGET_NAME}stripped.pdb CONFIGURATIONS Release DESTINATION ${INSTALL_FOLDER} RENAME ${TARGET_NAME}.pdb )
    install(FILES $<TARGET_PDB_FILE:${TARGET_NAME}> CONFIGURATIONS Debug RelWithDebInfo DESTINATION ${INSTALL_FOLDER} )
endif()
