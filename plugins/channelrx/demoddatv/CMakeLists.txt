project(datv)


set(datv_SOURCES
    datvdemod.cpp
    datvdemodplugin.cpp
    datvdemodsettings.cpp
    datvdemodwebapiadapter.cpp
    datvideostream.cpp
    datvudpstream.cpp
    datvdemodreport.cpp
    datvdemodsink.cpp
    datvdemodbaseband.cpp
    leansdr/dvb.cpp
    leansdr/filtergen.cpp
    leansdr/framework.cpp
    leansdr/math.cpp
    leansdr/sdr.cpp
    datvdemodgui.ui
    datvdvbs2ldpcdialog.ui
)

set(ldpc_SOURCES
    ldpctool/tables_handler.cpp
    ldpctool/ldpcworker.cpp
)

set(datv_HEADERS
    datvdemod.h
    datvdemodplugin.h
    datvdemodsettings.h
    datvdemodwebapiadapter.h
    datvideostream.h
    datvudpstream.h
    datvdemodreport.h
    datvdemodsink.h
    datvdemodbaseband.h
    leansdr/dvb.h
    leansdr/dvbs2.h
    leansdr/filtergen.h
    leansdr/framework.h
    leansdr/math.h
    leansdr/sdr.h
)

set(ldpc_HEADERS
    ldpctool/ldpc.h
    ldpctool/dvb_s2_tables.h
    ldpctool/dvb_s2x_tables.h
    ldpctool/dvb_t2_tables.h
    ldpctool/ldpcworker.h
)

if(NOT SERVER_MODE)
    set(datv_SOURCES 
        ${datv_SOURCES}
        datvdemodgui.ui
        datvdvbs2ldpcdialog.ui
        datvdemodgui.cpp
        datvdvbs2ldpcdialog.cpp
        datvideorender.cpp
    )
    set(datv_HEADERS ${datv_HEADERS} 
        datvdemodgui.h
        datvdvbs2ldpcdialog.h
        datvconstellation.h
        datvdvbs2constellation.h    
        datvideorender.h
        )
    set(TARGET_NAME ${PLUGINS_PREFIX}demoddatv)
    set(TARGET_LIB Qt::Widgets Qt::Multimedia Qt::MultimediaWidgets)
    set(TARGET_LIB_GUI "sdrgui")
    set(INSTALL_FOLDER ${INSTALL_PLUGINS_DIR})
else()
    set(TARGET_NAME ${PLUGINSSRV_PREFIX}demoddatvsrv)
    set(TARGET_LIB "")
    set(TARGET_LIB_GUI "")
    set(INSTALL_FOLDER ${INSTALL_PLUGINSSRV_DIR})
endif()

include_directories(
    ${AVCODEC_INCLUDE_DIRS}
    ${AVFORMAT_INCLUDE_DIRS}
    ${AVUTIL_INCLUDE_DIRS}
    ${SWSCALE_INCLUDE_DIRS}
    ${SWRESAMPLE_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/swagger/sdrangel/code/qt5/client
)

if(NOT Qt6_FOUND)
    add_library(${TARGET_NAME} ${datv_SOURCES} ${ldpc_SOURCES})
else()
    qt_add_plugin(${TARGET_NAME} CLASS_NAME DATVDemodPlugin ${datv_SOURCES} ${ldpc_SOURCES})
endif()

if(NOT BUILD_SHARED_LIBS)
    set_property(GLOBAL APPEND PROPERTY STATIC_PLUGINS_PROPERTY ${TARGET_NAME})
endif()

target_link_libraries(${TARGET_NAME} PRIVATE
    Qt::Core
    ${TARGET_LIB}    
    sdrbase
    ${TARGET_LIB_GUI}
    ${AVCODEC_LIBRARIES}
    ${AVFORMAT_LIBRARIES}
    ${AVUTIL_LIBRARIES}
    ${SWSCALE_LIBRARIES}
    ${SWRESAMPLE_LIBRARIES}
)

if(DEFINED FFMPEG_DEPENDS)
    add_dependencies(${TARGET_NAME} ${FFMPEG_DEPENDS})
endif()

install(TARGETS ${TARGET_NAME} DESTINATION ${INSTALL_FOLDER})

# Install debug symbols
if (WIN32)
    install(FILES $<TARGET_PROPERTY:${TARGET_NAME},RUNTIME_OUTPUT_DIRECTORY>/${TARGET_NAME}stripped.pdb CONFIGURATIONS Release DESTINATION ${INSTALL_FOLDER} RENAME ${TARGET_NAME}.pdb )
    install(FILES $<TARGET_PDB_FILE:${TARGET_NAME}> CONFIGURATIONS Debug RelWithDebInfo DESTINATION ${INSTALL_FOLDER} )
endif()
