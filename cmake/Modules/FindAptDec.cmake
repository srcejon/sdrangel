IF(NOT APT_FOUND)
    find_package(PkgConfig)
    if(PkgConfig_FOUND)
        PKG_CHECK_MODULES(PC_APT apt)
    endif()

    FIND_PATH(
        APT_INCLUDE_DIR
        NAMES apt.h
        HINTS ${APT_DIR}/include/apt
        PATHS /usr/local/include/apt
              /usr/include/apt
    )

    FIND_LIBRARY(
        APT_LIBRARIES
        NAMES apt
        HINTS ${APT_DIR}/lib
        PATHS /usr/local/lib
              /usr/lib
              /usr/lib64
    )

    message(STATUS "APT LIBRARIES " ${APT_LIBRARIES})
    message(STATUS "APT INCLUDE DIRS " ${APT_INCLUDE_DIR})

    INCLUDE(FindPackageHandleStandardArgs)
    FIND_PACKAGE_HANDLE_STANDARD_ARGS(AptDec DEFAULT_MSG APT_LIBRARIES APT_INCLUDE_DIR)
    MARK_AS_ADVANCED(APT_LIBRARIES APT_INCLUDE_DIR)

ENDIF(NOT APT_FOUND)
