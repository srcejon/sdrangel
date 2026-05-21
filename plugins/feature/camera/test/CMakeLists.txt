set(camera_star_tests_SOURCES
    camerastartests.cpp
    ../cameradetector.cpp
    ../camerastardetector.cpp
    ../cameraplatesolver.cpp
    ../camerasettings.cpp
    ../cameraicons.qrc
)

set(camera_star_tests_HEADERS
    ../cameradetector.h
    ../camerastardetector.h
    ../cameraplatesolver.h
    ../camerapipelineframe.h
    ../camerasettings.h
)

add_executable(featurecamera_star_tests EXCLUDE_FROM_ALL
    ${camera_star_tests_SOURCES}
    ${camera_star_tests_HEADERS}
)

target_compile_definitions(featurecamera_star_tests PRIVATE
    CAMERA_STAR_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

target_include_directories(featurecamera_star_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_SOURCE_DIR}/sdrbase
    ${CMAKE_SOURCE_DIR}/swagger/sdrangel/code/qt5/client
    ${OpenCV_INCLUDE_DIRS}
    ${ZLIB_INCLUDE_DIRS}
    ${ZLIB_INCLUDE_DIR}
)

target_link_libraries(featurecamera_star_tests PRIVATE
    Qt::Core
    Qt::Gui
    Qt::Network
    ${camera_OpenCV_LIBS}
    ${ZLIB_LIBRARIES}
    sdrbase
)
