project(fcdproplus)

set(fcdproplus_SOURCES
	fcdproplusinput.cpp
	fcdproplusplugin.cpp
    fcdproplussettings.cpp
    fcdpropluswebapiadapter.cpp
	fcdproplusthread.cpp
)

set(fcdproplus_HEADERS
	fcdproplusinput.h
	fcdproplusplugin.h
    fcdproplussettings.h
    fcdpropluswebapiadapter.h
	fcdproplusthread.h
)

include_directories(
    ${HIDAPI_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/swagger/sdrangel/code/qt5/client
	${CMAKE_SOURCE_DIR}/fcdhid
	${CMAKE_SOURCE_DIR}/fcdlib
)

if(NOT SERVER_MODE)
    set(fcdproplus_SOURCES
        ${fcdproplus_SOURCES}
        fcdproplusgui.cpp
        fcdproplusgui.ui
    )
    set(fcdproplus_HEADERS
        ${fcdproplus_HEADERS}
        fcdproplusgui.h
    )

    set(TARGET_NAME ${PLUGINS_PREFIX}inputfcdproplus)
    set(TARGET_LIB "Qt::Widgets")
    set(TARGET_LIB_GUI "sdrgui")
    set(INSTALL_FOLDER ${INSTALL_PLUGINS_DIR})
else()
    set(TARGET_NAME ${PLUGINSSRV_PREFIX}inputfcdproplussrv)
    set(TARGET_LIB "")
    set(TARGET_LIB_GUI "")
    set(INSTALL_FOLDER ${INSTALL_PLUGINSSRV_DIR})
endif()

if(NOT Qt6_FOUND)
    add_library(${TARGET_NAME} ${fcdproplus_SOURCES})
else()
    qt_add_plugin(${TARGET_NAME} CLASS_NAME FCDProPlusPlugin ${fcdproplus_SOURCES})
endif()

if(NOT BUILD_SHARED_LIBS)
    set_property(GLOBAL APPEND PROPERTY STATIC_PLUGINS_PROPERTY ${TARGET_NAME})
endif()

target_link_libraries(${TARGET_NAME} PRIVATE
    ${HIDAPI_LIBRARIES}
    Qt::Core
    ${TARGET_LIB}
	sdrbase
	${TARGET_LIB_GUI}
    swagger
    fcdhid
    fcdlib
)

install(TARGETS ${TARGET_NAME} DESTINATION ${INSTALL_FOLDER})

# Install debug symbols
if (WIN32)
    install(FILES $<TARGET_PROPERTY:${TARGET_NAME},RUNTIME_OUTPUT_DIRECTORY>/${TARGET_NAME}stripped.pdb CONFIGURATIONS Release DESTINATION ${INSTALL_FOLDER} RENAME ${TARGET_NAME}.pdb )
    install(FILES $<TARGET_PDB_FILE:${TARGET_NAME}> CONFIGURATIONS Debug RelWithDebInfo DESTINATION ${INSTALL_FOLDER} )
endif()
