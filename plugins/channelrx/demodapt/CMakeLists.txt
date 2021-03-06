project(demodapt)

set(demodapt_SOURCES
    aptdemod.cpp
    aptdemodsettings.cpp
    aptdemodbaseband.cpp
    aptdemodsink.cpp
    aptdemodimageworker.cpp
    aptdemodplugin.cpp
    aptdemodwebapiadapter.cpp
)

set(demodapt_HEADERS
    aptdemod.h
    aptdemodsettings.h
    aptdemodbaseband.h
    aptdemodsink.h
    aptdemodimageworker.h
    aptdemodplugin.h
    aptdemodwebapiadapter.h
)

include_directories(
    ${CMAKE_SOURCE_DIR}/swagger/sdrangel/code/qt5/client
    ${APT_INCLUDE_DIR}
)

if(NOT SERVER_MODE)
    set(demodapt_SOURCES
        ${demodapt_SOURCES}
        aptdemodgui.cpp
        aptdemodgui.ui
        aptdemodsettingsdialog.cpp
        aptdemodsettingsdialog.ui
        icons.qrc
    )
    set(demodapt_HEADERS
        ${demodapt_HEADERS}
        aptdemodgui.h
        aptdemodsettingsdialog.h
    )

    set(TARGET_NAME demodapt)
    set(TARGET_LIB "Qt5::Widgets")
    set(TARGET_LIB_GUI "sdrgui")
    set(INSTALL_FOLDER ${INSTALL_PLUGINS_DIR})
else()
    set(TARGET_NAME demodaptsrv)
    set(TARGET_LIB "")
    set(TARGET_LIB_GUI "")
    set(INSTALL_FOLDER ${INSTALL_PLUGINSSRV_DIR})
endif()

add_library(${TARGET_NAME} SHARED
    ${demodapt_SOURCES}
)

if(APT_EXTERNAL)
    add_dependencies(${TARGET_NAME} apt)
endif()

target_link_libraries(${TARGET_NAME}
    Qt5::Core
    ${TARGET_LIB}
    sdrbase
    ${TARGET_LIB_GUI}
    ${APT_LIBRARIES}
)

install(TARGETS ${TARGET_NAME} DESTINATION ${INSTALL_FOLDER})
