cmake_minimum_required(VERSION 3.8)
project(asump_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(asump_localization REQUIRED)

include_directories(include)

# Библиотека с логикой контроллера
add_library(path_follower_core SHARED
  src/path_follower.cpp
)
ament_target_dependencies(path_follower_core
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  nav2_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  asump_localization
)

# Исполняемый файл ноды
add_executable(path_follower_node
  src/path_follower_node.cpp
)
target_link_libraries(path_follower_node path_follower_core)
ament_target_dependencies(path_follower_node
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  nav2_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  asump_localization
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS path_follower_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS path_follower_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()