cmake_minimum_required(VERSION 3.8)
project(asump_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Compilation flags
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(slam_toolbox REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

# Include PCL
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Add executable
add_executable(lidar_odometry_node
  src/lidar_odometry.cpp
)

# Link libraries
target_link_libraries(lidar_odometry_node
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES}
  ${PCL_LIBRARIES}
  ${pcl_ros_LIBRARIES}
  ${pcl_conversions_LIBRARIES}
)

# Dependencies
ament_target_dependencies(lidar_odometry_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
)

install(TARGETS lidar_odometry_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
