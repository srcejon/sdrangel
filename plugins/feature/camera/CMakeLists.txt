project(camera)

set(camera_SOURCES
    camera.cpp
    camerapostprocessor.cpp
    camerafinder.cpp
    camerasettings.cpp
    cameraplugin.cpp
    cameraworker.cpp
    camerawebapiadapter.cpp
)

set(camera_HEADERS
    camera.h
    camerapostprocessor.h
    camerafinder.h
    camerasettings.h
    cameraplugin.h
    cameraworker.h
    camerawebapiadapter.h
)

include_directories(
    ${CMAKE_SOURCE_DIR}/swagger/sdrangel/code/qt5/client
    ${OpenCV_INCLUDE_DIRS}
)

if(NOT SERVER_MODE)
    set(camera_SOURCES
        ${camera_SOURCES}
        cameragui.cpp
        cameragui.ui
        camerasettingsdialog.cpp
        camerasettingsdialog.ui
        camerahistogramdialog.cpp
        cameraobjectdevicesettingsgui.cpp
        cameraicons.qrc
    )
    set(camera_HEADERS
        ${camera_HEADERS}
        cameragui.h
        camerasettingsdialog.h
        camerahistogramdialog.h
        cameraobjectdevicesettingsgui.h
    )

    set(TARGET_NAME ${PLUGINS_PREFIX}featurecamera)
    set(TARGET_LIB "Qt::Widgets" Qt::Charts)
    set(TARGET_LIB_GUI "sdrgui")
    set(INSTALL_FOLDER ${INSTALL_PLUGINS_DIR})
else()
    set(TARGET_NAME ${PLUGINSSRV_PREFIX}featurecamerasrv)
    set(TARGET_LIB "")
    set(TARGET_LIB_GUI "")
    set(INSTALL_FOLDER ${INSTALL_PLUGINSSRV_DIR})
endif()

if(NOT Qt6_FOUND)
    add_library(${TARGET_NAME} ${camera_SOURCES})
else()
    qt_add_plugin(${TARGET_NAME} CLASS_NAME CameraPlugin ${camera_SOURCES})
endif()

if(NOT BUILD_SHARED_LIBS)
    set_property(GLOBAL APPEND PROPERTY STATIC_PLUGINS_PROPERTY ${TARGET_NAME})
endif()

target_link_libraries(${TARGET_NAME} PRIVATE
    Qt::Core
    Qt::Network
    Qt::Gui
    Qt::Multimedia
    ${TARGET_LIB}
    "${OpenCV_LIBS}"
    sdrbase
    ${TARGET_LIB_GUI}
)

install(TARGETS ${TARGET_NAME} DESTINATION ${INSTALL_FOLDER})

if (WIN32)
    install(FILES $<TARGET_PDB_FILE:${TARGET_NAME}> CONFIGURATIONS Debug RelWithDebInfo DESTINATION ${INSTALL_FOLDER} )
endif()
