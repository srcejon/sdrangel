project(meteor)

include_directories(
    ${CMAKE_SOURCE_DIR}/swagger/sdrangel/code/qt5/client
)

set(meteor_SOURCES
    meteor.cpp
    meteorsettings.cpp
    meteorbaseband.cpp
    meteordemodsink.cpp
    meteorplugin.cpp
)

set(meteor_HEADERS
    meteor.h
    meteorsettings.h
    meteorbaseband.h
    meteordemodsink.h
    meteorplugin.h
)

if(NOT SERVER_MODE)
    set(meteor_SOURCES
        ${meteor_SOURCES}
        meteorgui.cpp
        meteorgui.ui
    )
    set(meteor_HEADERS
        ${meteor_HEADERS}
        meteorgui.h
    )

    set(TARGET_NAME ${PLUGINS_PREFIX}meteor)
    set(TARGET_LIB Qt::Widgets Qt::Charts)
    set(TARGET_LIB_GUI sdrgui)
    set(INSTALL_FOLDER ${INSTALL_PLUGINS_DIR})
else()
    set(TARGET_NAME ${PLUGINSSRV_PREFIX}meteorsrv)
    set(TARGET_LIB "")
    set(TARGET_LIB_GUI "")
    set(INSTALL_FOLDER ${INSTALL_PLUGINSSRV_DIR})
endif()

if(NOT Qt6_FOUND)
    add_library(${TARGET_NAME} ${meteor_SOURCES})
else()
    qt_add_plugin(${TARGET_NAME} CLASS_NAME MeteorPlugin)
    target_sources(${TARGET_NAME} PRIVATE ${meteor_SOURCES})
endif()

if(NOT BUILD_SHARED_LIBS)
    set_property(GLOBAL APPEND PROPERTY STATIC_PLUGINS_PROPERTY ${TARGET_NAME})
endif()

target_link_libraries(${TARGET_NAME} PRIVATE
    Qt::Core
    ${TARGET_LIB}
    sdrbase
    ${TARGET_LIB_GUI}
)

install(TARGETS ${TARGET_NAME} DESTINATION ${INSTALL_FOLDER})

if (WIN32)
    install(FILES $<TARGET_PROPERTY:${TARGET_NAME},RUNTIME_OUTPUT_DIRECTORY>/${TARGET_NAME}stripped.pdb CONFIGURATIONS Release DESTINATION ${INSTALL_FOLDER} RENAME ${TARGET_NAME}.pdb )
    install(FILES $<TARGET_PDB_FILE:${TARGET_NAME}> CONFIGURATIONS Debug RelWithDebInfo DESTINATION ${INSTALL_FOLDER} )
endif()
