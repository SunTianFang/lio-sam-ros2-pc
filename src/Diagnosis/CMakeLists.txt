add_definitions(-DMG_ENABLE_HTTP_STREAMING_MULTIPART)


include_directories(include/
    ../Geometry/include
    ../World/include
   
    ../Common/Platform/include
    ../Scan/include
  
    ../Tools/include
    ../ThirdParty/eigen3/
    ../NavBase/Mapping/include
    ../NavBase/Protocol/include
    ../NavBase/BlackBox/include
    ../NavBase/Network/include
    ../NavBase/LaserSensor/include
    ../NavBase/LaserDrivers/pf_r2000/include
    ../NavBase/LaserDrivers/fr_R2/include
    ../NavBase/LaserDrivers/Hokuyo_uam05lp/include
    ../NavBase/LaserDrivers/Sick_nanoScan3/include
    ../NavBase/LaserDrivers/WJ/include
    ../NavBase/LaserDrivers/Leimou_f30/include
    ../NavBase/LaserDrivers/BlueSea/sdk
    ../NavBase/LaserDrivers/BlueSea/sdk/service
    ../ThirdParty/jsoncpp/include
    ../ThirdParty/lcm/include
    ../ThirdParty/DDS/include
    ../ThirdParty/yaml-cpp/include/yaml-cpp
    ../ThirdParty/yaml-cpp/include
    ../NavBase/Mapping/include
    )


set ( Diagnosis_SRCS
	src/LCMTask.cpp
	src/Diagnosis.cpp
	src/robot_control_t.c
    src/robot_control_t_new.c
	)



#MRC3588
if(PLATFORM_MRC3588)
    add_compile_definitions(_RK3588_LINUX64)
    add_compile_definitions(NAV_APP)
    add_compile_definitions(_LINUX64)

    set(PCL_DIR "/opt/Mrc3588/pcl/include/pcl-1.11")
    set(DDS_INCLUDE_DIR "/opt/Mrc3588/sysrootnew/usr/local/include")

    include_directories(${PCL_DIR}
        /opt/Mrc3588/boost/include
        /opt/Mrc3588/pcl/include
#        ../ThirdParty/DDS/include/
        ${DDS_INCLUDE_DIR}
        )
endif()


add_library ( Diagnosis SHARED ${Diagnosis_SRCS} )

