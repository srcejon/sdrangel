project(xtrxinput)

set(xtrxinput_SOURCES
	xtrxinput.cpp
	xtrxinputplugin.cpp
    xtrxinputsettings.cpp
    xtrxinputwebapiadapter.cpp
	xtrxinputthread.cpp
)

set(xtrxinput_HEADERS
	xtrxinput.h
	xtrxinputplugin.h
    xtrxinputsettings.h
    xtrxinputwebapiadapter.h
	xtrxinputthread.h
)

include_directories(
        ${CMAKE_SOURCE_DIR}/swagger/sdrangel/code/qt5/client
	${CMAKE_SOURCE_DIR}/devices
	${LIBXTRX_INCLUDE_DIRS}
)

if(NOT SERVER_MODE)
    set(xtrxinput_SOURCES
        ${xtrxinput_SOURCES}
        xtrxinputgui.cpp

        xtrxinputgui.ui
    )
    set(xtrxinput_HEADERS
        ${xtrxinput_HEADERS}
        xtrxinputgui.h
    )

    set(TARGET_NAME ${PLUGINS_PREFIX}inputxtrx)
    set(TARGET_LIB "Qt::Widgets")
    set(TARGET_LIB_GUI "sdrgui")
    set(INSTALL_FOLDER ${INSTALL_PLUGINS_DIR})
else()
    set(TARGET_NAME ${PLUGINSSRV_PREFIX}inputxtrxsrv)
    set(TARGET_LIB "")
    set(TARGET_LIB_GUI "")
    set(INSTALL_FOLDER ${INSTALL_PLUGINSSRV_DIR})
endif()

if(NOT Qt6_FOUND)
    add_library(${TARGET_NAME} ${xtrxinput_SOURCES})
else()
    qt_add_plugin(${TARGET_NAME} CLASS_NAME XTRXInputPlugin ${xtrxinput_SOURCES})
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
        ${LIBXTRX_LIBRARY}
        xtrxdevice
)

install(TARGETS ${TARGET_NAME} DESTINATION ${INSTALL_FOLDER})

# Install debug symbols
if (WIN32)
    install(FILES $<TARGET_PROPERTY:${TARGET_NAME},RUNTIME_OUTPUT_DIRECTORY>/${TARGET_NAME}stripped.pdb CONFIGURATIONS Release DESTINATION ${INSTALL_FOLDER} RENAME ${TARGET_NAME}.pdb )
    install(FILES $<TARGET_PDB_FILE:${TARGET_NAME}> CONFIGURATIONS Debug RelWithDebInfo DESTINATION ${INSTALL_FOLDER} )
endif()
