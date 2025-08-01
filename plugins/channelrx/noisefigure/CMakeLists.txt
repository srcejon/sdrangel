project(noisefigure)

set(noisefigure_SOURCES
    noisefigure.cpp
    noisefiguresettings.cpp
    noisefigurebaseband.cpp
    noisefiguresink.cpp
    noisefigureplugin.cpp
    noisefigurewebapiadapter.cpp
)

set(noisefigure_HEADERS
    noisefigure.h
    noisefiguresettings.h
    noisefigurebaseband.h
    noisefiguresink.h
    noisefigureplugin.h
    noisefigurewebapiadapter.h
)

include_directories(
    ${CMAKE_SOURCE_DIR}/swagger/sdrangel/code/qt5/client
	${Boost_INCLUDE_DIRS}
)

if(NOT SERVER_MODE)
    set(noisefigure_SOURCES
        ${noisefigure_SOURCES}
        noisefiguregui.cpp
        noisefiguregui.ui
        noisefigurecontroldialog.ui
        noisefigurecontroldialog.cpp
        noisefigureenrdialog.ui
        noisefigureenrdialog.cpp
    )
    set(noisefigure_HEADERS
        ${noisefigure_HEADERS}
        noisefiguregui.h
        noisefigurecontroldialog.h
        noisefigureenrdialog.h
    )

    set(TARGET_NAME ${PLUGINS_PREFIX}noisefigure)
    set(TARGET_LIB "Qt::Widgets" Qt::Charts)
    set(TARGET_LIB_GUI "sdrgui")
    set(INSTALL_FOLDER ${INSTALL_PLUGINS_DIR})
else()
    set(TARGET_NAME ${PLUGINSSRV_PREFIX}noisefiguresrv)
    set(TARGET_LIB "")
    set(TARGET_LIB_GUI "")
    set(INSTALL_FOLDER ${INSTALL_PLUGINSSRV_DIR})
endif()

if(NOT Qt6_FOUND)
    add_library(${TARGET_NAME} ${noisefigure_SOURCES})
else()
    qt_add_plugin(${TARGET_NAME} CLASS_NAME NoiseFigurePlugin ${noisefigure_SOURCES})
endif()

if(NOT BUILD_SHARED_LIBS)
    set_property(GLOBAL APPEND PROPERTY STATIC_PLUGINS_PROPERTY ${TARGET_NAME})
endif()

target_link_libraries(${TARGET_NAME} PRIVATE
    Qt::Core
    ${TARGET_LIB}
    sdrbase
    ${TARGET_LIB_GUI}
)

install(TARGETS ${TARGET_NAME} DESTINATION ${INSTALL_FOLDER})

# Install debug symbols
if (WIN32)
    install(FILES $<TARGET_PROPERTY:${TARGET_NAME},RUNTIME_OUTPUT_DIRECTORY>/${TARGET_NAME}stripped.pdb CONFIGURATIONS Release DESTINATION ${INSTALL_FOLDER} RENAME ${TARGET_NAME}.pdb )
    install(FILES $<TARGET_PDB_FILE:${TARGET_NAME}> CONFIGURATIONS Debug RelWithDebInfo DESTINATION ${INSTALL_FOLDER} )
endif()
