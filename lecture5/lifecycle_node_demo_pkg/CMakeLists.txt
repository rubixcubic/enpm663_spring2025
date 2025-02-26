cmake_minimum_required(VERSION 3.8)
project(lifecycle_node_demo_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)


# ---------------------------------------------
# Python
# ---------------------------------------------
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  script/controller_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
