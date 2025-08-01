project(perseus)

set(perseus_SOURCES
    perseusinput.cpp
    perseusplugin.cpp
    perseussettings.cpp
    perseuswebapiadapter.cpp
    perseusworker.cpp
)

set(perseus_HEADERS
    perseusinput.h
    perseusplugin.h
    perseussettings.h
    perseuswebapiadapter.h
    perseusworker.h
)

include_directories(
    ${CMAKE_SOURCE_DIR}/swagger/sdrangel/code/qt5/client
    ${CMAKE_SOURCE_DIR}/devices
    ${LIBUSB_INCLUDE_DIR}
    ${LIBPERSEUS_INCLUDE_DIR}
)

if(NOT SERVER_MODE)
    set(perseus_SOURCES
        ${perseus_SOURCES}
        perseusgui.cpp

        perseusgui.ui
    )
    set(perseus_HEADERS
        ${perseus_HEADERS}
        perseusgui.h
    )

    set(TARGET_NAME ${PLUGINS_PREFIX}inputperseus)
    set(TARGET_LIB "Qt::Widgets")
    set(TARGET_LIB_GUI "sdrgui")
    set(INSTALL_FOLDER ${INSTALL_PLUGINS_DIR})
else()
    set(TARGET_NAME ${PLUGINSSRV_PREFIX}inputperseussrv)
    set(TARGET_LIB "")
    set(TARGET_LIB_GUI "")
    set(INSTALL_FOLDER ${INSTALL_PLUGINSSRV_DIR})
endif()

if(NOT Qt6_FOUND)
    add_library(${TARGET_NAME} ${perseus_SOURCES})
else()
    qt_add_plugin(${TARGET_NAME} CLASS_NAME PerseusPlugin ${perseus_SOURCES})
endif()

if(NOT BUILD_SHARED_LIBS)
    set_property(GLOBAL APPEND PROPERTY STATIC_PLUGINS_PROPERTY ${TARGET_NAME})
endif()

if(LIBPERSEUS_EXTERNAL)
    add_dependencies(${TARGET_NAME} perseus)
endif()
target_link_libraries(${TARGET_NAME} PRIVATE
        Qt::Core
        ${TARGET_LIB}
	sdrbase
	${TARGET_LIB_GUI}
        swagger
        ${LIBUSB_LIBRARIES}
        ${LIBPERSEUS_LIBRARIES}
        perseusdevice
)

install(TARGETS ${TARGET_NAME} DESTINATION ${INSTALL_FOLDER})

# Install debug symbols
if (WIN32)
    install(FILES $<TARGET_PROPERTY:${TARGET_NAME},RUNTIME_OUTPUT_DIRECTORY>/${TARGET_NAME}stripped.pdb CONFIGURATIONS Release DESTINATION ${INSTALL_FOLDER} RENAME ${TARGET_NAME}.pdb )
    install(FILES $<TARGET_PDB_FILE:${TARGET_NAME}> CONFIGURATIONS Debug RelWithDebInfo DESTINATION ${INSTALL_FOLDER} )
endif()
