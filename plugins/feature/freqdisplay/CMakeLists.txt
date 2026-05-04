project(freqdisplay)

set(freqdisplay_SOURCES
    freqdisplay.cpp
    freqdisplaysettings.cpp
    freqdisplayplugin.cpp
    freqdisplaywebapiadapter.cpp
)

set(freqdisplay_HEADERS
    freqdisplay.h
    freqdisplaysettings.h
    freqdisplayplugin.h
    freqdisplaywebapiadapter.h
)

include_directories(
    ${CMAKE_SOURCE_DIR}/swagger/sdrangel/code/qt5/client
)

if(NOT SERVER_MODE)
    set(freqdisplay_SOURCES
        ${freqdisplay_SOURCES}
        freqdisplaygui.cpp
        freqdisplaygui.ui
    )
    set(freqdisplay_HEADERS
        ${freqdisplay_HEADERS}
        freqdisplaygui.h
    )

    set(TARGET_NAME ${PLUGINS_PREFIX}featurefreqdisplay)
    set(TARGET_LIB "Qt::Widgets")
    set(TARGET_LIB_GUI "sdrgui")
    set(INSTALL_FOLDER ${INSTALL_PLUGINS_DIR})
    if(Qt${QT_DEFAULT_MAJOR_VERSION}TextToSpeech_FOUND)
        list(APPEND TARGET_LIB Qt::TextToSpeech)
    endif()
else()
    set(TARGET_NAME ${PLUGINSSRV_PREFIX}featurefreqdisplaysrv)
    set(TARGET_LIB "")
    set(TARGET_LIB_GUI "")
    set(INSTALL_FOLDER ${INSTALL_PLUGINSSRV_DIR})
endif()

if(NOT Qt6_FOUND)
    add_library(${TARGET_NAME} ${freqdisplay_SOURCES})
else()
    qt_add_plugin(${TARGET_NAME} CLASS_NAME FreqDisplayPlugin)
    target_sources(${TARGET_NAME} PRIVATE ${freqdisplay_SOURCES})
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
    install(FILES $<TARGET_PDB_FILE:${TARGET_NAME}> CONFIGURATIONS Debug RelWithDebInfo DESTINATION ${INSTALL_FOLDER})
endif()
