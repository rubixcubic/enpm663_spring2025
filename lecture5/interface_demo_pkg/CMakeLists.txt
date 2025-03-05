cmake_minimum_required(VERSION 3.8)
project(interface_demo_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(enpm663_interfaces REQUIRED)


# ---------------------------------------------
# CPP
# ---------------------------------------------
include_directories(include)

add_executable(interface_demo_cpp src/interface_demo.cpp)
ament_target_dependencies(interface_demo_cpp rclcpp enpm663_interfaces)

install(TARGETS
interface_demo_cpp
DESTINATION lib/${PROJECT_NAME}
)

# ---------------------------------------------
# Python
# ---------------------------------------------
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  script/interface_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
