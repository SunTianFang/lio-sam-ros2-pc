
project( Nav_ThirdParty )

include_directories(
    jsoncpp/include
    jsoncpp
    jsoncpp/src/lib_json
    yaml-cpp/include
    lcm/include
    yaml-cpp/include/yaml-cpp/node
    yaml-cpp/include/yaml-cpp/node/detail
    yaml-cpp/include/yaml-cpp
    yaml-cpp/include/yaml-cpp/contrib
    ${GLIB_DIR}  # Assuming GLIB_DIR is defined elsewhere
)

aux_source_directory(./yaml-cpp/src/ DIR_SRCS_YAML)


set(ThirdParty_SRCS
    jsoncpp/src/lib_json/json_reader.cpp
    jsoncpp/src/lib_json/json_value.cpp
    jsoncpp/src/lib_json/json_valueiterator.inl
    jsoncpp/src/lib_json/json_writer.cpp
    yaml-cpp/src/exceptions.cpp
    yaml-cpp/src/memory.cpp
    yaml-cpp/src/node_data.cpp
    lcm/src/channel_port_map_update_t.c
    lcm/src/channel_to_port_t.c
    lcm/src/eventlog.c
    lcm/src/lcm.c
    lcm/src/lcm_file.c
    lcm/src/lcm_memq.c
    lcm/src/lcm_mpudpm.c
    lcm/src/lcm_tcpq.c
    lcm/src/lcm_udpm.c
    lcm/src/ringbuffer.c
    lcm/src/udpm_util.c
)
	
	#aarch64-linux-gnu 下
    # set(GLIB_DIR "/usr/include/glib-2.0" "/usr/lib/aarch64-linux-gnu/glib-2.0/include")
    # set(GLIB_LIB_DIR "/usr/lib/aarch64-linux-gnu")
    # include_directories(${GLIB_DIR})
    # link_directories( ${GLIB_LIB_DIR})
    # add_library (Nav_ThirdParty SHARED ${ThirdParty_SRCS} ${DDS_SRC} ${DIR_SRCS_YAML})

    # 链接 GLib 库
    # target_link_libraries(Nav_ThirdParty  /usr/lib/aarch64-linux-gnu/libglib-2.0.so)

    #pc
    set(GLIB_DIR "/usr/include/glib-2.0" "/usr/lib/x86_64-linux-gnu/glib-2.0/include")
    set(GLIB_LIB_DIR "/usr/lib/x86_64-linux-gnu")
    include_directories(${GLIB_DIR})
    link_directories( ${GLIB_LIB_DIR})
    add_library ( Nav_ThirdParty SHARED ${ThirdParty_SRCS} ${DDS_SRC} ${DIR_SRCS_YAML})

    # 链接 GLib 库
    target_link_libraries(Nav_ThirdParty  /usr/lib/x86_64-linux-gnu/libglib-2.0.so)

    
#MRC3588
if(PLATFORM_MRC3588)
    add_compile_definitions(_RK3588_LINUX64)
    add_compile_definitions(NAV_APP)
    add_compile_definitions(_LINUX64)
    #glib
    set(GLIB_DIR "/opt/Mrc3588/glib/include/glib-2.0" "/opt/Mrc3588/glib/include")
    set(DDS_INCLUDE_DIR
    "/opt/Mrc3588/sysrootnew/usr/local/include"
    "/opt/Mrc3588/boost/include"
    )

    include_directories(${GLIB_DIR}
        ./DDS/include
        ../Diagnosis/include
        ../Scan/include
        ../Tools/include
        ../Geometry/include
        ../Csm/include
        ../Csm/include/csm
        ../Csm/src/csm
        eigen3
        ${DDS_INCLUDE_DIR}
        )

    file(GLOB DDS_SRC DDS/src/*.cxx DDS/src/*.cpp)
endif(PLATFORM_MRC3588)

if(PLATFORM_MRC3588)
    add_library ( Nav_ThirdParty SHARED ${ThirdParty_SRCS} ${DDS_SRC} ${DIR_SRCS_YAML})
endif(PLATFORM_MRC3588)


##MRC3588
if(PLATFORM_MRC3588)
    target_link_libraries(Nav_ThirdParty
        /opt/Mrc3588/sysrootnew/usr/local/lib/libfastcdr.so.1.0.26
        /opt/Mrc3588/sysrootnew/usr/local/lib/libfastrtps.so.2.9.0
        /opt/Mrc3588/sysrootnew/usr/local/lib/libfoonathan_memory-0.7.1.so
        /opt/Mrc3588/sysrootnew/usr/lib/aarch64-linux-gnu/libpcre.so.3
        /opt/Mrc3588/sysrootnew/usr/lib/aarch64-linux-gnu/libtinyxml2.so.8
        /opt/Mrc3588/sysrootnew/usr/lib/aarch64-linux-gnu/libssl.so.1.1
        /opt/Mrc3588/sysrootnew/usr/lib/aarch64-linux-gnu/libcrypto.so.1.1
        /opt/Mrc3588/sysrootnew/usr/lib/aarch64-linux-gnu/libboost_chrono.so.1.74.0
        /opt/Mrc3588/sysrootnew/usr/lib/aarch64-linux-gnu/libboost_thread.so.1.74.0
        )
endif()



