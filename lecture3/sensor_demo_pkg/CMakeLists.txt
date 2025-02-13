cmake_minimum_required(VERSION 3.8)
project(sensor_demo_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include/sensor_demo_pkg)
# camera_demo
add_executable(camera_demo src/camera_demo.cpp)
ament_target_dependencies(camera_demo rclcpp sensor_msgs)

#lidar_demo
add_executable(lidar_demo src/lidar_demo.cpp)
ament_target_dependencies(lidar_demo rclcpp sensor_msgs)

#temperature_demo
add_executable(temperature_demo src/temperature_demo.cpp)
ament_target_dependencies(temperature_demo rclcpp sensor_msgs)

#processing_demo
add_executable(processing_demo src/processing_demo.cpp)
ament_target_dependencies(processing_demo rclcpp sensor_msgs)

# Install directories
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Install executables
install(TARGETS
  camera_demo lidar_demo temperature_demo processing_demo
  DESTINATION lib/${PROJECT_NAME})

ament_package()
