project(demodm17)

set(m17_SOURCES
    m17demod.cpp
    m17demodbaseband.cpp
    m17demodsink.cpp
    m17demodsettings.cpp
    m17demodwebapiadapter.cpp
	m17demodplugin.cpp
    m17demodbaudrates.cpp
    m17demodprocessor.cpp
    m17demodfilters.cpp
)

set(m17_HEADERS
	m17demod.h
    m17demodbaseband.h
    m17demodsink.h
    m17demodsettings.h
    m17demodwebapiadapter.h
	m17demodplugin.h
    m17demodbaudrates.h
    m17demodprocessor.h
    m17demodfilters.h
)

include_directories(
    ${CMAKE_SOURCE_DIR}/swagger/sdrangel/code/qt5/client
    ${CODEC2_INCLUDE_DIR}
    ${CMAKE_SOURCE_DIR}/modemm17
)

if(NOT SERVER_MODE)
    set(m17_SOURCES
        ${m17_SOURCES}
        m17demodgui.cpp
	    m17demodgui.ui
        m17statustextdialog.cpp
        m17statustextdialog.ui
    )
    set(m17_HEADERS
        ${m17_HEADERS}
        m17demodgui.h
        m17statustextdialog.h
    )
    set(TARGET_NAME ${PLUGINS_PREFIX}demodm17)
    set(TARGET_LIB "Qt::Widgets" Qt::Charts)
    set(TARGET_LIB_GUI "sdrgui")
    set(INSTALL_FOLDER ${INSTALL_PLUGINS_DIR})
else()
    set(TARGET_NAME ${PLUGINSSRV_PREFIX}demodm17srv)
    set(TARGET_LIB "")
    set(TARGET_LIB_GUI "")
    set(INSTALL_FOLDER ${INSTALL_PLUGINSSRV_DIR})
endif()

if(NOT Qt6_FOUND)
    add_library(${TARGET_NAME} ${m17_SOURCES})
else()
    qt_add_plugin(${TARGET_NAME} CLASS_NAME M17DemodPlugin ${m17_SOURCES})
endif()

if(NOT BUILD_SHARED_LIBS)
    set_property(GLOBAL APPEND PROPERTY STATIC_PLUGINS_PROPERTY ${TARGET_NAME})
endif()

if(CODEC2_EXTERNAL)
    add_dependencies(${TARGET_NAME} codec2)
endif()

target_link_libraries(${TARGET_NAME} PRIVATE
    Qt::Core
    ${TARGET_LIB}
	sdrbase
	${TARGET_LIB_GUI}
    swagger
    ${CODEC2_LIBRARIES}
    modemm17
)

install(TARGETS ${TARGET_NAME} DESTINATION ${INSTALL_FOLDER})

# Install debug symbols
if (WIN32)
    install(FILES $<TARGET_PROPERTY:${TARGET_NAME},RUNTIME_OUTPUT_DIRECTORY>/${TARGET_NAME}stripped.pdb CONFIGURATIONS Release DESTINATION ${INSTALL_FOLDER} RENAME ${TARGET_NAME}.pdb )
    install(FILES $<TARGET_PDB_FILE:${TARGET_NAME}> CONFIGURATIONS Debug RelWithDebInfo DESTINATION ${INSTALL_FOLDER} )
endif()
