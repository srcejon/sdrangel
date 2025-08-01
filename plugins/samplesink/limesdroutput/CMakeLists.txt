project(limesdroutput)

set(limesdroutput_SOURCES
	limesdroutput.cpp
	limesdroutputplugin.cpp
    limesdroutputsettings.cpp
    limesdroutputwebapiadapter.cpp
	limesdroutputthread.cpp
)

set(limesdroutput_HEADERS
	limesdroutput.h
	limesdroutputplugin.h
    limesdroutputsettings.h
    limesdroutputwebapiadapter.h
	limesdroutputthread.h
)

include_directories(
        ${CMAKE_SOURCE_DIR}/swagger/sdrangel/code/qt5/client
	${CMAKE_SOURCE_DIR}/devices
	${LIMESUITE_INCLUDE_DIR}
)

if(NOT SERVER_MODE)
    set(limesdroutput_SOURCES
        ${limesdroutput_SOURCES}
        limesdroutputgui.cpp

        limesdroutputgui.ui
    )
    set(limesdroutput_HEADERS
        ${limesdroutput_HEADERS}
        limesdroutputgui.h
    )

    set(TARGET_NAME ${PLUGINS_PREFIX}outputlimesdr)
    set(TARGET_LIB "Qt::Widgets")
    set(TARGET_LIB_GUI "sdrgui")
    set(INSTALL_FOLDER ${INSTALL_PLUGINS_DIR})
else()
    set(TARGET_NAME ${PLUGINSSRV_PREFIX}outputlimesdrsrv)
    set(TARGET_LIB "")
    set(TARGET_LIB_GUI "")
    set(INSTALL_FOLDER ${INSTALL_PLUGINSSRV_DIR})
endif()

if(NOT Qt6_FOUND)
    add_library(${TARGET_NAME} ${limesdroutput_SOURCES})
else()
    qt_add_plugin(${TARGET_NAME} CLASS_NAME LimeSDROutputPlugin ${limesdroutput_SOURCES})
endif()

if(NOT BUILD_SHARED_LIBS)
    set_property(GLOBAL APPEND PROPERTY STATIC_PLUGINS_PROPERTY ${TARGET_NAME})
endif()

if(LIMESUITE_EXTERNAL)
    add_dependencies(${TARGET_NAME} limesuite)
endif()

target_link_libraries(${TARGET_NAME} PRIVATE
        Qt::Core
        ${TARGET_LIB}
	sdrbase
	${TARGET_LIB_GUI}
        swagger
        ${LIMESUITE_LIBRARY}
        limesdrdevice
)

install(TARGETS ${TARGET_NAME} DESTINATION ${INSTALL_FOLDER})

# Install debug symbols
if (WIN32)
    install(FILES $<TARGET_PROPERTY:${TARGET_NAME},RUNTIME_OUTPUT_DIRECTORY>/${TARGET_NAME}stripped.pdb CONFIGURATIONS Release DESTINATION ${INSTALL_FOLDER} RENAME ${TARGET_NAME}.pdb )
    install(FILES $<TARGET_PDB_FILE:${TARGET_NAME}> CONFIGURATIONS Debug RelWithDebInfo DESTINATION ${INSTALL_FOLDER} )
endif()
