project(udpsource)

set(udpsource_SOURCES
    udpsource.cpp
    udpsourcebaseband.cpp
    udpsourcesource.cpp
	udpsourceplugin.cpp
	udpsourceudphandler.cpp
	udpsourcemsg.cpp
    udpsourcesettings.cpp
    udpsourcewebapiadapter.cpp
)

set(udpsource_HEADERS
	udpsource.h
    udpsourcebaseband.h
    udpsourcesource.h
	udpsourceplugin.h
	udpsourceudphandler.h
	udpsourcemsg.h
    udpsourcesettings.h
    udpsourcewebapiadapter.h
)

include_directories(
	${CMAKE_SOURCE_DIR}/swagger/sdrangel/code/qt5/client
)

if(NOT SERVER_MODE)
    set(udpsource_SOURCES
        ${udpsource_SOURCES}
        udpsourcegui.cpp

        udpsourcegui.ui
    )
    set(udpsource_HEADERS
        ${udpsource_HEADERS}
        udpsourcegui.h
    )

    set(TARGET_NAME ${PLUGINS_PREFIX}udpsource)
    set(TARGET_LIB "Qt::Widgets")
    set(TARGET_LIB_GUI "sdrgui")
    set(INSTALL_FOLDER ${INSTALL_PLUGINS_DIR})
else()
    set(TARGET_NAME ${PLUGINSSRV_PREFIX}udpsourcesrv)
    set(TARGET_LIB "")
    set(TARGET_LIB_GUI "")
    set(INSTALL_FOLDER ${INSTALL_PLUGINSSRV_DIR})
endif()

if(NOT Qt6_FOUND)
    add_library(${TARGET_NAME} ${udpsource_SOURCES})
else()
    qt_add_plugin(${TARGET_NAME} CLASS_NAME UDPSourcePlugin ${udpsource_SOURCES})
endif()

if(NOT BUILD_SHARED_LIBS)
    set_property(GLOBAL APPEND PROPERTY STATIC_PLUGINS_PROPERTY ${TARGET_NAME})
endif()

target_link_libraries(${TARGET_NAME} PRIVATE
        Qt::Core
        ${TARGET_LIB}
        Qt::Network
	sdrbase
	${TARGET_LIB_GUI}
        swagger
)

install(TARGETS ${TARGET_NAME} DESTINATION ${INSTALL_FOLDER})

# Install debug symbols
if (WIN32)
    install(FILES $<TARGET_PROPERTY:${TARGET_NAME},RUNTIME_OUTPUT_DIRECTORY>/${TARGET_NAME}stripped.pdb CONFIGURATIONS Release DESTINATION ${INSTALL_FOLDER} RENAME ${TARGET_NAME}.pdb )
    install(FILES $<TARGET_PDB_FILE:${TARGET_NAME}> CONFIGURATIONS Debug RelWithDebInfo DESTINATION ${INSTALL_FOLDER} )
endif()
