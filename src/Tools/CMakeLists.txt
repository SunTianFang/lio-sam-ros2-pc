include_directories(include
    ../Tools/include
    ../Geometry/include
    include/RK3399

    )


set ( Tools_SRCS
	src/Archive.cpp
	src/Tools.cpp
	src/time_patch.cpp
	src/Combination.cpp
	src/AbstractParameter.cpp
	src/Gpio.cpp
	src/LED.cpp
	src/systemInfo.cpp
	src/WatchDog.cpp
	)

#MRC3588
if(PLATFORM_MRC3588)
    add_compile_definitions(_RK3588_LINUX64)
    add_compile_definitions(NAV_APP)
    add_compile_definitions(_LINUX64)
endif()



add_library ( Tools SHARED ${Tools_SRCS}  )
target_include_directories(Tools PUBLIC include)  # 确保Tools库的头文件可见