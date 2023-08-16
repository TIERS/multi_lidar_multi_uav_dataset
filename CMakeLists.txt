cmake_minimum_required(VERSION 3.0.2)
project(multi_lidar_multi_uav_dataset)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs 
  livox_ros_driver
)

find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  DEPENDS     
    roscpp 
    std_msgs
    sensor_msgs
    geometry_msgs
    pcl_conversions
    pcl_ros 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include 
  ${catkin_INCLUDE_DIRS} 
)
 
add_executable(avia_format_converter src/avia_format_converter.cpp)
target_link_libraries(avia_format_converter ${catkin_LIBRARIES})

add_executable(mid360_format_converter src/mid360_format_converter.cpp)
target_link_libraries(mid360_format_converter ${catkin_LIBRARIES})
 
add_executable(lidars_extrinsic_computation src/lidars_extrinsic_computation.cpp)
target_link_libraries(lidars_extrinsic_computation ${catkin_LIBRARIES})
 
 