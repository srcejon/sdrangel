project(audiocatsiso)

set(audiocatsiso_SOURCES
    audiocatinputworker.cpp
    audiocatoutputworker.cpp
	audiocatsiso.cpp
	audiocatsisoplugin.cpp
    audiocatsisosettings.cpp
    audiocatsisowebapiadapter.cpp
    audiocatsisohamlib.cpp
    audiocatsisocatworker.cpp
)

set(audiocatsiso_HEADERS
    audiocatinputworker.h
    audiocatoutputworker.h
    audiocatsiso.h
	audiocatsisoplugin.h
    audiocatsisosettings.h
    audiocatsisowebapiadapter.h
    audiocatsisohamlib.h
    audiocatsisocatworker.h
)

include_directories(
    ${CMAKE_SOURCE_DIR}/swagger/sdrangel/code/qt5/client
    ${CMAKE_SOURCE_DIR}/devices
    ${hamlib_INCLUDE_DIRS}
)

if (NOT SERVER_MODE)
    set (audiocatsiso_SOURCES
        ${audiocatsiso_SOURCES}
        audiocatsisogui.cpp
        audiocatsisogui.ui
        audiocatsisocatdialog.cpp
        audiocatsisocatdialog.ui
    )
    set(audiocatsiso_HEADERS
        ${audiocatsiso_HEADERS}
        audiocatsisogui.h
        audiocatsisocatdialog.h
    )
    set(TARGET_NAME ${PLUGINS_PREFIX}mimoaudiocatsiso)
    set(TARGET_LIB "Qt::Widgets")
    set(TARGET_LIB_GUI "sdrgui")
    set(INSTALL_FOLDER ${INSTALL_PLUGINS_DIR})
else()
    set(TARGET_NAME ${PLUGINSSRV_PREFIX}mimoaudiocatsisosrv)
    set(TARGET_LIB "")
    set(TARGET_LIB_GUI "")
    set(INSTALL_FOLDER ${INSTALL_PLUGINSSRV_DIR})
endif()

if(NOT Qt6_FOUND)
    add_library(${TARGET_NAME} ${audiocatsiso_SOURCES})
else()
    qt_add_plugin(${TARGET_NAME} CLASS_NAME AudioCATSISOPlugin ${audiocatsiso_SOURCES})
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
    ${hamlib_LIBRARIES}
)

install(TARGETS ${TARGET_NAME} DESTINATION ${INSTALL_FOLDER})

# Install debug symbols
if (WIN32)
    install(FILES $<TARGET_PROPERTY:${TARGET_NAME},RUNTIME_OUTPUT_DIRECTORY>/${TARGET_NAME}stripped.pdb CONFIGURATIONS Release DESTINATION ${INSTALL_FOLDER} RENAME ${TARGET_NAME}.pdb )
    install(FILES $<TARGET_PDB_FILE:${TARGET_NAME}> CONFIGURATIONS Debug RelWithDebInfo DESTINATION ${INSTALL_FOLDER} )
endif()
