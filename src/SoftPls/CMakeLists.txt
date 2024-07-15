include_directories(include
    ../Scan/include
    ../Tools/include
    ../Geometry/include
    ../ThirdParty/eigen3
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
    )

set ( SoftPls_SRCS
	src/AvoidObstacle.cpp
	src/PLS.cpp
	)

#MRC3588
if(PLATFORM_MRC3588)
    add_compile_definitions(_RK3588_LINUX64)
    add_compile_definitions(NAV_APP)
    add_compile_definitions(_LINUX64)

    set(PCL_DIR "/opt/Mrc3588/pcl/include/pcl-1.11")
    include_directories(${PCL_DIR}
        /opt/Mrc3588/boost/include
        /opt/Mrc3588/pcl/include
        ../ThirdParty/yaml-cpp/include
        )
endif()

add_library ( SoftPls SHARED ${SoftPls_SRCS})


