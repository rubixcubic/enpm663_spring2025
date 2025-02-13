cmake_minimum_required(VERSION 3.8)
project(pub_demo_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

#--------
# C++
#--------
include_directories(include)

add_executable(pub_demo_cpp src/pub_demo.cpp)
ament_target_dependencies(pub_demo_cpp rclcpp std_msgs)

install(TARGETS
  pub_demo_cpp
  DESTINATION lib/${PROJECT_NAME}
)

#--------
# Python
#--------
# Install Python modules
ament_python_install_package(${PROJECT_NAME} 
  SCRIPTS_DESTINATION lib/${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS 
  scripts/pub_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
