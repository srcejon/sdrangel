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

if(WIN32)
    get_target_property(camera_star_tests_qmake Qt::qmake IMPORTED_LOCATION)
    get_filename_component(camera_star_tests_qt_bin_dir "${camera_star_tests_qmake}" DIRECTORY)
    get_filename_component(camera_star_tests_qt_dir "${camera_star_tests_qt_bin_dir}" DIRECTORY)
    set(camera_star_tests_qt_platforms_dir "${camera_star_tests_qt_dir}/plugins/platforms")
    if(EXISTS "${camera_star_tests_qt_platforms_dir}")
        add_custom_command(TARGET featurecamera_star_tests POST_BUILD
            COMMAND "${CMAKE_COMMAND}" -E make_directory "$<TARGET_FILE_DIR:featurecamera_star_tests>/platforms"
            COMMAND "${CMAKE_COMMAND}" -E copy_directory
                "${camera_star_tests_qt_platforms_dir}"
                "$<TARGET_FILE_DIR:featurecamera_star_tests>/platforms"
            COMMENT "Deploying Qt platform plugins for camera star tests"
        )
    endif()
    set(camera_star_tests_qt_tls_dir "${camera_star_tests_qt_dir}/plugins/tls")
    if(EXISTS "${camera_star_tests_qt_tls_dir}")
        add_custom_command(TARGET featurecamera_star_tests POST_BUILD
            COMMAND "${CMAKE_COMMAND}" -E make_directory "$<TARGET_FILE_DIR:featurecamera_star_tests>/tls"
            COMMAND "${CMAKE_COMMAND}" -E copy_directory
                "${camera_star_tests_qt_tls_dir}"
                "$<TARGET_FILE_DIR:featurecamera_star_tests>/tls"
            COMMENT "Deploying Qt TLS plugins for camera star tests"
        )
    endif()
endif()
