cmake_minimum_required(VERSION 3.5)
project(imu)

# Set C++14 as standard
set(CMAKE_CXX_STANDARD 14)

# Find ROS 2 Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial REQUIRED)

# Add header file directory
include_directories(include)

# Create the imu_cmd library
add_library(imu_cmd
  src/imu_cmd.cpp
)

# Set the dependencies of the imu_cmd library
ament_target_dependencies(imu_cmd
  rclcpp
  std_msgs
  serial
  sensor_msgs
)

# Create an imu_node executable file
add_executable(imu_node
  src/imu_node.cpp
)

# Link the imu_cmd library to the imu_node executable file
target_link_libraries(imu_node imu_cmd)

# Set the dependencies of imu_node
ament_target_dependencies(imu_node
  rclcpp
  std_msgs
  serial
  sensor_msgs
)

# Install executable files
install(TARGETS imu_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install library files
install(TARGETS imu_cmd
  DESTINATION lib/${PROJECT_NAME}
)

# Install other resources (such as launch files, configuration files, etc.)
install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# Create ROS 2 package
ament_package()
