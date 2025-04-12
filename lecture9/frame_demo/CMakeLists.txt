cmake_minimum_required(VERSION 3.8)
project(frame_demo)

# Set C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(ariac_msgs REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_eigen_INCLUDE_DIRS}
  ${ariac_msgs_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${kdl_parser_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

# Create a single executable with all components
add_executable(frame_demo 
  src/main.cpp
  src/broadcaster_demo.cpp
  src/kdl_frame_demo.cpp
)

# Link dependencies to our single executable
ament_target_dependencies(frame_demo
  rclcpp
  geometry_msgs
  tf2_ros
  tf2_eigen
  ariac_msgs
  orocos_kdl
  Eigen3
)

#-----------------------------
# Python
#-----------------------------
# Install Python modules
ament_python_install_package(${PROJECT_NAME})

#-----------------------------
# Installation
#-----------------------------
# Install Python executables
install(PROGRAMS
  script/frame_broadcaster_demo.py
  script/frame_kdl_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install C++ executable
install(TARGETS
  frame_demo
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directory for both project code and headers
install(DIRECTORY include/
  DESTINATION include
)

# Export package dependencies
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  geometry_msgs
  tf2_ros
  tf2_eigen
  ariac_msgs
  orocos_kdl
  kdl_parser
  Eigen3
)

ament_package()