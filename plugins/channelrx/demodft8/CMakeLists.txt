project(demodft8)

set(demodft8_SOURCES
	ft8demod.cpp
    ft8demodsettings.cpp
    ft8demodsink.cpp
    ft8demodbaseband.cpp
    ft8demodwebapiadapter.cpp
	ft8plugin.cpp
    ft8buffer.cpp
    ft8demodworker.cpp
)

set(demodft8_HEADERS
	ft8demod.h
    ft8demodsettings.h
    ft8demodsink.h
    ft8demodbaseband.h
    ft8demodwebapiadapter.h
	ft8plugin.h
    ft8buffer.h
    ft8demodworker.h;
)

include_directories(
    ${CMAKE_SOURCE_DIR}/swagger/sdrangel/code/qt5/client
    ${CMAKE_SOURCE_DIR}/ft8
    ${FFTW3F_INCLUDE_DIRS}
)

if(NOT SERVER_MODE)
    set(demodft8_SOURCES
        ${demodft8_SOURCES}
        ft8demodgui.cpp
        ft8demodfilterproxy.cpp
        ft8demodgui.ui
        ft8demodsettingsdialog.cpp
        ft8demodsettingsdialog.ui
    )
    set(demodft8_HEADERS
        ${demodft8_HEADERS}
        ft8demodgui.h
        ft8demodfilterproxy.h
        ft8demodsettingsdialog.h
    )
    set(TARGET_NAME ${PLUGINS_PREFIX}demodft8)
    set(TARGET_LIB "Qt::Widgets")
    set(TARGET_LIB_GUI "sdrgui")
    set(INSTALL_FOLDER ${INSTALL_PLUGINS_DIR})
else()
    set(TARGET_NAME ${PLUGINSSRV_PREFIX}demodft8srv)
    set(TARGET_LIB "")
    set(TARGET_LIB_GUI "")
    set(INSTALL_FOLDER ${INSTALL_PLUGINSSRV_DIR})
endif()

if(NOT Qt6_FOUND)
    add_library(${TARGET_NAME} ${demodft8_SOURCES})
else()
    qt_add_plugin(${TARGET_NAME} CLASS_NAME FT8Plugin ${demodft8_SOURCES})
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
    ft8
)

install(TARGETS ${TARGET_NAME} DESTINATION ${INSTALL_FOLDER})

# Install debug symbols
if (WIN32)
    install(FILES $<TARGET_PROPERTY:${TARGET_NAME},RUNTIME_OUTPUT_DIRECTORY>/${TARGET_NAME}stripped.pdb CONFIGURATIONS Release DESTINATION ${INSTALL_FOLDER} RENAME ${TARGET_NAME}.pdb )
    install(FILES $<TARGET_PDB_FILE:${TARGET_NAME}> CONFIGURATIONS Debug RelWithDebInfo DESTINATION ${INSTALL_FOLDER} )
endif()
