cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0048 NEW)
project(lio_sam)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()


set(PCL_DIR "/usr/lib/aarch64-linux-gnu/cmake/pcl")
set(PCL_INCLUDE_DIRS "/usr/include/pcl-1.10")  
# 设置运行时候查找动态库的路径
set(CMAKE_INSTALL_RPATH "/home/stf/lio-sam_all/ros2_pc/lio-sam-ros2-pc_add_communication_and_supported_livo_mid_360/install/lio_sam/lib/release")

# 导入Tools文件夹的CMakeLists.txt
add_subdirectory(src/ThirdParty)
add_subdirectory(src/Tools)
add_subdirectory(src/NavBase) 
add_subdirectory(src/Geometry)
add_subdirectory(src/Scan)
add_subdirectory(src/Diagnosis)
add_subdirectory(src/SoftPls)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(PCL 1.10 REQUIRED COMPONENTS common io)
#打印pcl路径
message(STATUS "PCL_INCLUDE_DIRS: ${PCL_INCLUDE_DIRS}")
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
set(OpenCV_STATIC ON)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Eigen REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(livox_ros_driver2 REQUIRED)

if(Boost_FOUND)
    # 打印Boost的包括目录和库目录
    message(STATUS "Found Boost: Yes")
    message(STATUS "Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")
    message(STATUS "Boost_LIBRARY_DIRS: ${Boost_LIBRARY_DIRS}")
    message(STATUS "Boost_LIBRARIES: ${Boost_LIBRARIES}")
endif()

include_directories(
  include/lio_sam
  "/usr/local/include"
  "/usr/local/include/pcl_conversions"
  "/usr/local/include/pcl"
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  src/ThirdParty/DDS/include
  src/ThirdParty/eigen3/include
  src/ThirdParty/jsoncpp/include
  src/ThirdParty/lcm/include
  src/ThirdParty/yaml-cpp/include
  src/Tools/include
  src/NavBase/Protocol/include
  src/NavBase/BlackBox/include
  src/NavBase/CanDev/include
  src/NavBase/LaserDrivers/include
  src/NavBase/LaserSensor/include
  src/NavBase/Mapping/include
  src/NavBase/Network/include 
  src/Geometry/include
  src/Scan/include
  src/Diagnosis/include
  src/SoftPls/include
)

rosidl_generate_interfaces(${PROJECT_NAME} "msg/CloudInfo.msg" "srv/SaveMap.srv" DEPENDENCIES std_msgs sensor_msgs geometry_msgs)


add_executable(${PROJECT_NAME}_featureExtraction src/featureExtraction.cpp)
ament_target_dependencies(${PROJECT_NAME}_featureExtraction rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL livox_ros_driver2)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_featureExtraction "${cpp_typesupport_target}" Nav_ThirdParty Tools NavBase Geometry Scan Diagnosis  SoftPls) 

add_executable(${PROJECT_NAME}_imageProjection src/imageProjection.cpp)
ament_target_dependencies(${PROJECT_NAME}_imageProjection rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_msgs visualization_msgs pcl_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL livox_ros_driver2)
target_link_libraries(${PROJECT_NAME}_imageProjection "${cpp_typesupport_target}" Nav_ThirdParty Tools NavBase Geometry Scan Diagnosis  SoftPls) 

add_executable(${PROJECT_NAME}_imuPreintegration src/imuPreintegration.cpp)
ament_target_dependencies(${PROJECT_NAME}_imuPreintegration rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL GTSAM Eigen livox_ros_driver2)
target_link_libraries(${PROJECT_NAME}_imuPreintegration gtsam "${cpp_typesupport_target}" Nav_ThirdParty Tools NavBase Geometry Scan Diagnosis  SoftPls)

add_executable(${PROJECT_NAME}_mapOptimization src/mapOptmization.cpp)
ament_target_dependencies(${PROJECT_NAME}_mapOptimization rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL GTSAM livox_ros_driver2)
if (OpenMP_CXX_FOUND)
  target_link_libraries(${PROJECT_NAME}_mapOptimization gtsam "${cpp_typesupport_target}" OpenMP::OpenMP_CXX Nav_ThirdParty Tools NavBase Geometry Scan Diagnosis  SoftPls)
else()
  target_link_libraries(${PROJECT_NAME}_mapOptimization gtsam "${cpp_typesupport_target}" Nav_ThirdParty Tools NavBase Geometry Scan Diagnosis  SoftPls)
endif()

# # 为 'lio_sam_mapOptimization' 设置RPATH
# set_target_properties(${PROJECT_NAME}_mapOptimization PROPERTIES
#   INSTALL_RPATH "/home/stf/lio-sam_all/ros2_pc/lio-sam-ros2-pc_add_communication_and_supported_livo_mid_360/install/lio_sam/lib/release"
#   BUILD_WITH_INSTALL_RPATH TRUE
# )
# set_target_properties(${PROJECT_NAME}_imuPreintegration
#   PROPERTIES
#   INSTALL_RPATH "/home/stf/lio-sam_all/ros2_pc/lio-sam-ros2-pc_add_communication_and_supported_livo_mid_360/install/lio_sam/lib/release"
#   BUILD_WITH_INSTALL_RPATH TRUE
# )
# set_target_properties(${PROJECT_NAME}_imageProjection
#   PROPERTIES
#   INSTALL_RPATH "/home/stf/lio-sam_all/ros2_pc/lio-sam-ros2-pc_add_communication_and_supported_livo_mid_360/install/lio_sam/lib/release"
#   BUILD_WITH_INSTALL_RPATH TRUE
# )
# set_target_properties(${PROJECT_NAME}_featureExtraction
#   PROPERTIES
#   INSTALL_RPATH "/home/stf/lio-sam_all/ros2_pc/lio-sam-ros2-pc_add_communication_and_supported_livo_mid_360/install/lio_sam/lib/release"
#   BUILD_WITH_INSTALL_RPATH TRUE
# )


install(
  TARGETS Nav_ThirdParty
  DESTINATION lib/release
)

install(
  TARGETS Tools
  DESTINATION lib/release
)

install(
  TARGETS NavBase
  DESTINATION lib/release
)

install(
  TARGETS Geometry
  DESTINATION lib/release
)

install(
  TARGETS Scan
  DESTINATION lib/release
)

install(
  TARGETS Diagnosis
  DESTINATION lib/release
)

install(
  TARGETS SoftPls
  DESTINATION lib/release
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(
  TARGETS ${PROJECT_NAME}_imageProjection
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}_imuPreintegration
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}_featureExtraction
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}_mapOptimization
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY "include/"
  DESTINATION include
)

ament_export_include_directories(include)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

