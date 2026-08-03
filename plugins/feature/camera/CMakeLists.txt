project(camera)
option(ENABLE_CAMERA_TENSORRT "Enable optional TensorRT YOLO inference in the camera feature" OFF)
option(ENABLE_CAMERA_LITERT "Enable optional Android LiteRT YOLO inference" ${ANDROID})
set(LITERT_ROOT "" CACHE PATH "Root of the Android LiteRT CompiledModel SDK and libraries")

set(camera_SOURCES
    camera.cpp
    cameradrawing.cpp
    cameradetector.cpp
    cameraclearskyreference.cpp
    cameraclouddetector.cpp
    cameramotiondetector.cpp
    camerastardetector.cpp
    cameraobjectdetector.cpp
    camerayololitert.cpp
    cameradiffdetector.cpp
    camerathermaldecoder.cpp
    camerathermalprocessor.cpp
    cameraframepreprocessor.cpp
    cameraframealigner.cpp
    camerahdrfusion.cpp
    cameraframestacker.cpp
    cameraimageutils.cpp
    cameraimagepool.cpp
    camerameteorphotometer.cpp
    cameraimageprocessor.cpp
    cameraopticalspectrum.cpp
    cameraopticalspectrumlibrary.cpp
    cameramediaplaybackstate.cpp
    cameramediaplaybackcontroller.cpp
    cameramediametadata.cpp
    cameraffmpegaudio.cpp
    camerarecorder.cpp
    cameravideowriter.cpp
    cameravideofiledecoder.cpp
    camerayoutubestreamer.cpp
    camerapostprocessor.cpp
    camerafinder.cpp
    cameraqtaudiocontroller.cpp
    cameraalpacacontroller.cpp
    cameraasicontroller.cpp
    camerasettings.cpp
    cameraplugin.cpp
    cameraworker.cpp
    camerawebapiadapter.cpp
)

set(camera_HEADERS
    camera.h
    cameradrawing.h
    cameradetector.h
    cameraclearskyreference.h
    cameraclouddetector.h
    cameramotiondetector.h
    camerastardetector.h
    cameraobjectdetector.h
    camerayololitert.h
    camerayolotensorrt.h
    cameradiffdetector.h
    camerathermaldecoder.h
    camerathermalprocessor.h
    cameraframepreprocessor.h
    cameraframealigner.h
    camerahdrfusion.h
    cameraframestacker.h
    cameraimageutils.h
    cameraimagepool.h
    camerameteorphotometer.h
    cameraaudiobytequeue.h
    cameraimageprocessor.h
    cameraopticalspectrum.h
    cameraopticalspectrumlibrary.h
    cameramediaplaybackstate.h
    cameramediaplaybackcontroller.h
    cameramediametadata.h
    cameraffmpegaudio.h
    cameraffmpegcompat.h
    camerarecorder.h
    cameravideowriter.h
    cameravideofiledecoder.h
    camerayoutubestreamer.h
    cameraplatesolver.h
    camerapipelineframe.h
    camerapostprocessor.h
    camerafinder.h
    cameraqtaudiocontroller.h
    cameraalpacacontroller.h
    cameraasicontroller.h
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
        camerafilesequencedialog.cpp
        cameradetectionhistory.cpp
        camerasettingsdialog.cpp
        camerasettingsdialog.ui
        camerahistogramdialog.cpp
        cameraopticalspectrumdialog.cpp
        cameraopticalspectrumlinesdialog.cpp
        cameraopticalspectrumreferencedialog.cpp
        cameraclearskyreferencedialog.cpp
        camerastellariumclient.cpp
        cameraicons.qrc
    )
    set(camera_HEADERS
        ${camera_HEADERS}
        cameragui.h
        camerafilesequencedialog.h
        cameradetectionhistory.h
        cameradetectionhistoryentry.h
        camerasettingsdialog.h
        camerahistogramdialog.h
        cameraopticalspectrumdialog.h
        cameraopticalspectrumlinesdialog.h
        cameraopticalspectrumreferencedialog.h
        cameraclearskyreferencedialog.h
        camerastellariumclient.h
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
if (Qt${QT_DEFAULT_MAJOR_VERSION}Sensors_FOUND)
    list(APPEND TARGET_LIB Qt::Sensors)
endif()

# OpenGL viewport for the preview QGraphicsView (cameragui.cpp). Qt6 split
# QOpenGLWidget into its own module; only needed for the GUI build.
if(NOT SERVER_MODE AND Qt6_FOUND)
    list(APPEND TARGET_LIB Qt::OpenGLWidgets)
endif()

set(camera_OpenCV_LIBS ${OpenCV_LIBS})
if(NOT OpenCV_bgsegm_FOUND)
    list(REMOVE_ITEM camera_OpenCV_LIBS opencv_bgsegm)
endif()
if(NOT OpenCV_ximgproc_FOUND)
    list(REMOVE_ITEM camera_OpenCV_LIBS opencv_ximgproc)
endif()

# WS1a (cross-build determinism): compile the plate solver exactly ONCE into a static library
# linked by BOTH the featurecamera plugin and the featurecamera_star_tests harness. Linking the
# same final-code object into both binaries makes the harness exercise byte-identical solver
# machine code as the shipped plugin, eliminating the cross-build (DLL vs EXE) ULP divergence that
# flipped marginal wide-fisheye solves (see doc/camera/plate-solver-notes.md, the wide-7/8/9 saga).
# LTCG is disabled here so the .obj carries final codegen rather than IL that would be re-generated
# (and could re-diverge) at each consuming target's link. cameraplatesolver.cpp is
# feature-define-insensitive (no CAMERA_OPENCV_CUDA_*/FFmpeg #ifdef) and its public interface uses
# only ABI-stable types (CameraSettings, CameraPipelineStarDetection, CameraPlateSolveResult), so a
# single compilation is safe for both consumers regardless of their own CUDA/FFmpeg define sets.
# On Linux this directory is processed once per variant (GUI featurecamera + server
# featurecamerasrv), but camera_platesolver is a single global target shared by both, so guard
# its creation against the second pass. The lib is feature-define-insensitive (no CUDA/FFmpeg
# #ifdef) with an ABI-stable interface, so one instance is correct for both plugin variants and
# the test harness.
if(NOT TARGET camera_platesolver)
add_library(camera_platesolver STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/cameraplatesolver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cameraplatesolvercatalog.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cameraplatesolversiril.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cameraplatesolverrefine.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cameraplatesolveracceptance.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cameraplatesolvercore.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cameraplatesolverpipeline.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/cameraplatesolver.h
    ${CMAKE_CURRENT_SOURCE_DIR}/cameraplatesolverinternal.h
)
set_target_properties(camera_platesolver PROPERTIES INTERPROCEDURAL_OPTIMIZATION OFF)
target_include_directories(camera_platesolver PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/sdrbase
    ${CMAKE_SOURCE_DIR}/swagger/sdrangel/code/qt5/client
    ${OpenCV_INCLUDE_DIRS}
    ${ZLIB_INCLUDE_DIRS}
    ${ZLIB_INCLUDE_DIR}
)
target_link_libraries(camera_platesolver PUBLIC
    Qt::Core
    Qt::Gui
    Qt::Network
    sdrbase
    ${camera_OpenCV_LIBS}
    ${ZLIB_LIBRARIES}
)
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
    target_compile_definitions(${TARGET_NAME} PRIVATE CAMERA_OPENCV_CUDA_DETECTION)
    target_compile_definitions(${TARGET_NAME} PRIVATE CAMERA_OPENCV_CUDA_CLOUD_DETECTION)
endif()
if(OpenCV_cudaarithm_FOUND AND OpenCV_cudaimgproc_FOUND AND OpenCV_cudawarping_FOUND AND OpenCV_cudafilters_FOUND AND OpenCV_cudabgsegm_FOUND)
    target_compile_definitions(${TARGET_NAME} PRIVATE CAMERA_OPENCV_CUDA_MOTION_DETECTION)
endif()

find_package(FFmpeg COMPONENTS AVCODEC AVFORMAT AVUTIL SWSCALE SWRESAMPLE)
if(FFMPEG_FOUND)
    message(STATUS "FFmpeg found for camera: ${FFMPEG_VERSION_STRING}")
    target_compile_definitions(${TARGET_NAME} PRIVATE CAMERA_FFMPEG_STREAMING)
    target_include_directories(${TARGET_NAME} PRIVATE ${FFMPEG_INCLUDE_DIRS})
    if(DEFINED FFMPEG_DEPENDS)
        add_dependencies(${TARGET_NAME} ${FFMPEG_DEPENDS})
    endif()
else()
    message(STATUS "FFmpeg not found for camera")
endif()

if(ENABLE_CAMERA_TENSORRT)
    find_path(TENSORRT_INCLUDE_DIR NvInfer.h
        HINTS
            "$ENV{TENSORRT_ROOT}/include"
            "${TENSORRT_ROOT}/include"
            "${CUDA_TOOLKIT_ROOT_DIR}/include"
            "${CUDAToolkit_INCLUDE_DIRS}"
    )
    find_library(TENSORRT_INFER_LIBRARY
        NAMES nvinfer nvinfer_10
        HINTS
            "$ENV{TENSORRT_ROOT}/lib"
            "$ENV{TENSORRT_ROOT}/lib64"
            "$ENV{TENSORRT_ROOT}/lib/x64"
            "${TENSORRT_ROOT}/lib"
            "${TENSORRT_ROOT}/lib64"
            "${TENSORRT_ROOT}/lib/x64"
            "${CUDA_TOOLKIT_ROOT_DIR}/lib64"
            "${CUDA_TOOLKIT_ROOT_DIR}/lib/x64"
    )
    find_library(TENSORRT_ONNXPARSER_LIBRARY
        NAMES nvonnxparser nvonnxparser_10
        HINTS
            "$ENV{TENSORRT_ROOT}/lib"
            "$ENV{TENSORRT_ROOT}/lib64"
            "$ENV{TENSORRT_ROOT}/lib/x64"
            "${TENSORRT_ROOT}/lib"
            "${TENSORRT_ROOT}/lib64"
            "${TENSORRT_ROOT}/lib/x64"
            "${CUDA_TOOLKIT_ROOT_DIR}/lib64"
            "${CUDA_TOOLKIT_ROOT_DIR}/lib/x64"
    )
    find_library(CUDART_LIBRARY
        NAMES cudart cudart_static
        HINTS
            "${CUDA_TOOLKIT_ROOT_DIR}/lib64"
            "${CUDA_TOOLKIT_ROOT_DIR}/lib/x64"
    )

    if(TENSORRT_INCLUDE_DIR AND TENSORRT_INFER_LIBRARY AND TENSORRT_ONNXPARSER_LIBRARY AND CUDART_LIBRARY)
        target_sources(${TARGET_NAME} PRIVATE camerayolotensorrt.cpp)
        target_compile_definitions(${TARGET_NAME} PRIVATE CAMERA_TENSORRT_YOLO)
        target_include_directories(${TARGET_NAME} PRIVATE ${TENSORRT_INCLUDE_DIR})
        target_link_libraries(${TARGET_NAME} PRIVATE
            ${TENSORRT_INFER_LIBRARY}
            ${TENSORRT_ONNXPARSER_LIBRARY}
            ${CUDART_LIBRARY}
        )
        message(STATUS "Camera TensorRT YOLO support enabled")
    else()
        message(WARNING "ENABLE_CAMERA_TENSORRT is ON, but TensorRT or CUDA runtime libraries were not found; camera TensorRT support is disabled")
    endif()
endif()

if(ANDROID AND ENABLE_CAMERA_LITERT)
    find_path(LITERT_COMPILED_MODEL_INCLUDE_DIR litert/c/litert_compiled_model.h
        HINTS
            "$ENV{LITERT_ROOT}/litert_cc_sdk"
            "${LITERT_ROOT}/litert_cc_sdk"
            "${EXTERNAL_LIBRARY_FOLDER}/litert-2.1.6/litert_cc_sdk"
        NO_CMAKE_FIND_ROOT_PATH
    )
    find_file(LITERT_RUNTIME_LIBRARY
        NAMES libLiteRt.so
        HINTS
            "$ENV{LITERT_ROOT}/lib/${ANDROID_ABI}"
            "${LITERT_ROOT}/lib/${ANDROID_ABI}"
            "${EXTERNAL_LIBRARY_FOLDER}/litert-2.1.6/lib/${ANDROID_ABI}"
        NO_CMAKE_FIND_ROOT_PATH
    )
    find_file(LITERT_GPU_ACCELERATOR_LIBRARY
        NAMES libLiteRtClGlAccelerator.so
        HINTS
            "$ENV{LITERT_ROOT}/lib/${ANDROID_ABI}"
            "${LITERT_ROOT}/lib/${ANDROID_ABI}"
            "${EXTERNAL_LIBRARY_FOLDER}/litert-2.1.6/lib/${ANDROID_ABI}"
        NO_CMAKE_FIND_ROOT_PATH
    )

    if(LITERT_COMPILED_MODEL_INCLUDE_DIR AND LITERT_RUNTIME_LIBRARY AND LITERT_GPU_ACCELERATOR_LIBRARY)
        set(LITERT_GENERATED_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/litert-generated")
        file(MAKE_DIRECTORY "${LITERT_GENERATED_INCLUDE_DIR}/litert/build_common")
        set(LITERT_BUILD_CONFIG_DISABLE_GPU 0)
        set(LITERT_BUILD_CONFIG_DISABLE_NPU 1)
        configure_file(
            "${LITERT_COMPILED_MODEL_INCLUDE_DIR}/litert/build_common/build_config.h.in"
            "${LITERT_GENERATED_INCLUDE_DIR}/litert/build_common/build_config.h"
        )
        target_compile_definitions(${TARGET_NAME} PRIVATE CAMERA_LITERT_YOLO)
        target_include_directories(${TARGET_NAME} PRIVATE ${LITERT_COMPILED_MODEL_INCLUDE_DIR} ${LITERT_GENERATED_INCLUDE_DIR})
        target_link_libraries(${TARGET_NAME} PRIVATE ${LITERT_RUNTIME_LIBRARY} EGL GLESv3)
        set_property(GLOBAL APPEND PROPERTY SDRANGEL_ANDROID_EXTRA_LIBS
            ${LITERT_RUNTIME_LIBRARY}
            ${LITERT_GPU_ACCELERATOR_LIBRARY}
        )
        message(STATUS "Camera Android LiteRT 2.1 CompiledModel YOLO support enabled")
    else()
        message(WARNING "ENABLE_CAMERA_LITERT is ON, but the LiteRT CompiledModel SDK/runtime/GPU accelerator were not found; camera LiteRT support is disabled. Set LITERT_ROOT to an extracted LiteRT 2.1 package, or set LITERT_COMPILED_MODEL_INCLUDE_DIR, LITERT_RUNTIME_LIBRARY and LITERT_GPU_ACCELERATOR_LIBRARY explicitly.")
    endif()
endif()

if(NOT BUILD_SHARED_LIBS)
    set_property(GLOBAL APPEND PROPERTY STATIC_PLUGINS_PROPERTY ${TARGET_NAME})
endif()

target_link_libraries(${TARGET_NAME} PRIVATE
    camera_platesolver
    Qt::Core
    Qt::Network
    Qt::Gui
    Qt::Multimedia
    ${TARGET_LIB}
    ${camera_OpenCV_LIBS}
    ${ASICAMERA_LIBRARIES}
    ${ZLIB_LIBRARIES}
    ${AVCODEC_LIBRARIES}
    ${AVFORMAT_LIBRARIES}
    ${AVUTIL_LIBRARIES}
    ${SWSCALE_LIBRARIES}
    ${SWRESAMPLE_LIBRARIES}
    sdrbase
    ${TARGET_LIB_GUI}
)

if (WIN32)
    # winmm for timeBeginPeriod/timeEndPeriod (raise the present timer resolution while streaming)
    target_link_libraries(${TARGET_NAME} PRIVATE winmm)
endif()

install(TARGETS ${TARGET_NAME} DESTINATION ${INSTALL_FOLDER})

if(WIN32)
    # Run deployqt for QtSensors, which isn't used in other plugins
    include(DeployQt)
    windeployqt(${TARGET_NAME} ${SDRANGEL_BINARY_BIN_DIR} "")
endif()

if (WIN32)
    install(FILES $<TARGET_PDB_FILE:${TARGET_NAME}> CONFIGURATIONS Debug RelWithDebInfo DESTINATION ${INSTALL_FOLDER} )
endif()

if(NOT SERVER_MODE)
    add_subdirectory(test)
endif()
