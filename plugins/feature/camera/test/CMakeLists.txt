set(camera_star_tests_SOURCES
    camerastartests.cpp
    ../cameradetector.cpp
    ../cameraimageutils.cpp
    ../cameraimagepool.cpp
    ../camerastardetector.cpp
    ../camerasettings.cpp
    ../cameraicons.qrc
)

add_executable(featurecamera_media_metadata_tests EXCLUDE_FROM_ALL
    cameramediametadatatests.cpp
    ../cameramediametadata.cpp
    ../cameramediametadata.h
    ../camerapipelineframe.h
    ../camerasettings.h
)
target_include_directories(featurecamera_media_metadata_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(featurecamera_media_metadata_tests PRIVATE
    Qt::Core
    Qt::Gui
    ${camera_OpenCV_LIBS}
)
# WS1a: cameraplatesolver.cpp is NOT compiled here. It comes from the camera_platesolver static
# library (defined in ../CMakeLists.txt) which the featurecamera plugin also links, so the harness
# runs the byte-identical solver object that ships in the GUI -- no cross-build ULP divergence.

set(camera_star_tests_HEADERS
    ../cameradetector.h
    ../cameraimageutils.h
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
    camera_platesolver
    Qt::Core
    Qt::Gui
    Qt::Network
    ${camera_OpenCV_LIBS}
    ${ZLIB_LIBRARIES}
    sdrbase
)

if(Qt${QT_DEFAULT_MAJOR_VERSION}TextToSpeech_FOUND)
    target_link_libraries(featurecamera_star_tests PRIVATE Qt::TextToSpeech)
endif()

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

    if(MSVC)
        find_program(camera_star_tests_powershell NAMES powershell.exe powershell)
        if(camera_star_tests_powershell)
            set(camera_star_tests_profile_runtime_paths
                "$<TARGET_FILE_DIR:featurecamera_star_tests>"
                "${CMAKE_BINARY_DIR}/bin"
                "${camera_star_tests_qt_bin_dir}"
            )

            foreach(camera_star_tests_runtime_dir IN ITEMS
                    "${OpenCV_LIBS_BIN_DIR}"
                    "${FFTW3F_DLL_DIR}"
                    "${LIBSIGMF_DLL_DIR}"
                    "${ASICAMERA_DLL_DIR}")
                if(camera_star_tests_runtime_dir)
                    list(APPEND camera_star_tests_profile_runtime_paths "${camera_star_tests_runtime_dir}")
                endif()
            endforeach()

            if(DEFINED CUDA_TOOLKIT_ROOT_DIR AND EXISTS "${CUDA_TOOLKIT_ROOT_DIR}/bin")
                list(APPEND camera_star_tests_profile_runtime_paths "${CUDA_TOOLKIT_ROOT_DIR}/bin")
            endif()
            list(JOIN camera_star_tests_profile_runtime_paths "|" camera_star_tests_profile_runtime_path_arg)

            add_custom_target(featurecamera_star_tests_vs_profile
                COMMAND "${CMAKE_COMMAND}" -E make_directory "${CMAKE_CURRENT_BINARY_DIR}/vs-profile"
                COMMAND "${camera_star_tests_powershell}"
                    -NoProfile
                    -ExecutionPolicy Bypass
                    -File "${CMAKE_CURRENT_SOURCE_DIR}/profile-camera-star-tests-vs.ps1"
                    -Executable "$<TARGET_FILE:featurecamera_star_tests>"
                    -Csv "${CMAKE_CURRENT_SOURCE_DIR}/star-tests.csv"
                    -OutputDirectory "${CMAKE_CURRENT_BINARY_DIR}/star-test-output"
                    -ProfileOutputDirectory "${CMAKE_CURRENT_BINARY_DIR}/vs-profile"
                    -QtPluginPath "${camera_star_tests_qt_dir}/plugins"
                    -RuntimePath "${camera_star_tests_profile_runtime_path_arg}"
                    -Warmup
                DEPENDS featurecamera_star_tests
                VERBATIM
                COMMENT "Profiling camera star/plate solver tests with Visual Studio Diagnostics"
            )
        else()
            message(WARNING "PowerShell was not found; featurecamera_star_tests_vs_profile will not be available")
        endif()
    endif()
endif()

set(camera_cloud_tests_SOURCES
    cameracloudtests.cpp
    ../cameradetector.cpp
    ../cameraimageutils.cpp
    ../cameraimagepool.cpp
    ../cameramediametadata.cpp
    ../cameraclearskyreference.cpp
    ../cameraclouddetector.cpp
    ../cameramotiondetector.cpp
    ../camerastardetector.cpp
    ../camerasettings.cpp
    ../cameraicons.qrc
)

set(camera_cloud_tests_HEADERS
    ../cameradetector.h
    ../cameraimageutils.h
    ../cameramediametadata.h
    ../cameraclearskyreference.h
    ../cameraclouddetector.h
    ../cameramotiondetector.h
    ../camerastardetector.h
    ../cameraplatesolver.h
    ../camerapipelineframe.h
    ../camerasettings.h
)

add_executable(featurecamera_cloud_tests EXCLUDE_FROM_ALL
    ${camera_cloud_tests_SOURCES}
    ${camera_cloud_tests_HEADERS}
)

target_compile_definitions(featurecamera_cloud_tests PRIVATE
    CAMERA_CLOUD_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

if(OpenCV_cudaarithm_FOUND AND OpenCV_cudaimgproc_FOUND AND OpenCV_cudawarping_FOUND AND OpenCV_cudafilters_FOUND)
    target_compile_definitions(featurecamera_cloud_tests PRIVATE CAMERA_OPENCV_CUDA_CLOUD_DETECTION)
endif()

add_executable(featurecamera_thermal_tests EXCLUDE_FROM_ALL
    camerathermaltests.cpp
    ../camerathermaldecoder.cpp
    ../camerathermaldecoder.h
    ../camerapipelineframe.h
    ../camerasettings.h
)
target_include_directories(featurecamera_thermal_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(featurecamera_thermal_tests PRIVATE
    Qt::Core
    Qt::Gui
    ${camera_OpenCV_LIBS}
)

target_include_directories(featurecamera_cloud_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_SOURCE_DIR}/sdrbase
    ${CMAKE_SOURCE_DIR}/swagger/sdrangel/code/qt5/client
    ${OpenCV_INCLUDE_DIRS}
    ${ZLIB_INCLUDE_DIRS}
    ${ZLIB_INCLUDE_DIR}
)

target_link_libraries(featurecamera_cloud_tests PRIVATE
    camera_platesolver
    Qt::Core
    Qt::Gui
    Qt::Network
    ${camera_OpenCV_LIBS}
    ${ZLIB_LIBRARIES}
    sdrbase
)

if(Qt${QT_DEFAULT_MAJOR_VERSION}TextToSpeech_FOUND)
    target_link_libraries(featurecamera_cloud_tests PRIVATE Qt::TextToSpeech)
endif()

if(WIN32)
    if(EXISTS "${camera_star_tests_qt_platforms_dir}")
        add_custom_command(TARGET featurecamera_cloud_tests POST_BUILD
            COMMAND "${CMAKE_COMMAND}" -E make_directory "$<TARGET_FILE_DIR:featurecamera_cloud_tests>/platforms"
            COMMAND "${CMAKE_COMMAND}" -E copy_directory
                "${camera_star_tests_qt_platforms_dir}"
                "$<TARGET_FILE_DIR:featurecamera_cloud_tests>/platforms"
            COMMENT "Deploying Qt platform plugins for camera cloud tests"
        )
    endif()
endif()

add_executable(featurecamera_spectrum_tests EXCLUDE_FROM_ALL
    cameraspectrumtests.cpp
    ../cameraopticalspectrum.cpp
    ../cameraopticalspectrum.h
    ../cameraopticalspectrumlibrary.cpp
    ../cameraopticalspectrumlibrary.h
)

target_include_directories(featurecamera_spectrum_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${ZLIB_INCLUDE_DIRS}
    ${ZLIB_INCLUDE_DIR}
)

target_link_libraries(featurecamera_spectrum_tests PRIVATE
    Qt::Core
    Qt::Gui
    ${ZLIB_LIBRARIES}
)

if(OpenCV_cudaarithm_FOUND AND OpenCV_cudaimgproc_FOUND AND OpenCV_cudawarping_FOUND AND OpenCV_cudafilters_FOUND)
    add_executable(featurecamera_hdr_tests EXCLUDE_FROM_ALL
        camerahdrtests.cpp
        ../camerahdrfusion.cpp
    )

    target_compile_definitions(featurecamera_hdr_tests PRIVATE
        CAMERA_HDR_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
        CAMERA_OPENCV_CUDA_STACKING
    )

    target_include_directories(featurecamera_hdr_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/..
        ${OpenCV_INCLUDE_DIRS}
    )

    target_link_libraries(featurecamera_hdr_tests PRIVATE
        Qt::Core
        ${camera_OpenCV_LIBS}
    )
endif()
