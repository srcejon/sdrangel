project(camera)

set(camera_SOURCES
    camera.cpp
    cameradetector.cpp
    cameraframealigner.cpp
    cameraframestacker.cpp
    cameraimageprocessor.cpp
    cameraplatesolver.cpp
    camerapostprocessor.cpp
    camerafinder.cpp
    camerasettings.cpp
    cameraplugin.cpp
    cameraworker.cpp
    camerawebapiadapter.cpp
)

set(camera_HEADERS
    camera.h
    cameradetector.h
    cameraframealigner.h
    cameraframestacker.h
    cameraimageprocessor.h
    cameraplatesolver.h
    camerapipelineframe.h
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
    ${ASICAMERA_INCLUDE_DIR}
    ${ZLIB_INCLUDE_DIRS}
    ${ZLIB_INCLUDE_DIR}
)

if(NOT SERVER_MODE)
    set(camera_SOURCES
        ${camera_SOURCES}
        cameragui.cpp
        cameragui.ui
        cameradetectionhistory.cpp
        camerasettingsdialog.cpp
        camerasettingsdialog.ui
        camerahistogramdialog.cpp
        cameraobjectdevicesettingsgui.cpp
        cameraicons.qrc
    )
    set(camera_HEADERS
        ${camera_HEADERS}
        cameragui.h
        cameradetectionhistory.h
        cameradetectionhistoryentry.h
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

if(Qt${QT_DEFAULT_MAJOR_VERSION}TextToSpeech_FOUND)
    list(APPEND TARGET_LIB Qt::TextToSpeech)
endif()

set(camera_OpenCV_LIBS ${OpenCV_LIBS})
if(NOT OpenCV_bgsegm_FOUND)
    list(REMOVE_ITEM camera_OpenCV_LIBS opencv_bgsegm)
endif()
if(NOT OpenCV_ximgproc_FOUND)
    list(REMOVE_ITEM camera_OpenCV_LIBS opencv_ximgproc)
endif()

if(NOT Qt6_FOUND)
    add_library(${TARGET_NAME} ${camera_SOURCES})
else()
    qt_add_plugin(${TARGET_NAME} CLASS_NAME CameraPlugin)
    target_sources(${TARGET_NAME} PRIVATE ${camera_SOURCES})
endif()

if(ASICAMERA_FOUND)
    add_compile_definitions(ASICAMERA_FOUND)
endif()

if(OpenCV_cudaarithm_FOUND AND OpenCV_cudaimgproc_FOUND AND OpenCV_cudawarping_FOUND AND OpenCV_cudafilters_FOUND)
    target_compile_definitions(${TARGET_NAME} PRIVATE CAMERA_OPENCV_CUDA_IMAGE_PROCESSING)
    target_compile_definitions(${TARGET_NAME} PRIVATE CAMERA_OPENCV_CUDA_STACKING)
endif()
if(OpenCV_cudaarithm_FOUND AND OpenCV_cudaimgproc_FOUND AND OpenCV_cudawarping_FOUND AND OpenCV_cudafilters_FOUND AND OpenCV_cudabgsegm_FOUND)
    target_compile_definitions(${TARGET_NAME} PRIVATE CAMERA_OPENCV_CUDA_MOTION_DETECTION)
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
    ${camera_OpenCV_LIBS}
    ${ASICAMERA_LIBRARIES}
    ${ZLIB_LIBRARIES}
    sdrbase
    ${TARGET_LIB_GUI}
)

install(TARGETS ${TARGET_NAME} DESTINATION ${INSTALL_FOLDER})

if (WIN32)
    install(FILES $<TARGET_PDB_FILE:${TARGET_NAME}> CONFIGURATIONS Debug RelWithDebInfo DESTINATION ${INSTALL_FOLDER} )
endif()
