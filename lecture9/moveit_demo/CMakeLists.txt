cmake_minimum_required(VERSION 3.5)
project(moveit_demo)

# Set C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Add compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set package dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  moveit_ros_planning_interface
  ariac_msgs
  shape_msgs
  control_msgs
  diagnostic_msgs
  tf2_kdl
)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Add moveit_demo library
add_library(${PROJECT_NAME}_lib SHARED
  src/minimal_demo.cpp
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  include
)
ament_target_dependencies(${PROJECT_NAME}_lib ${THIS_PACKAGE_INCLUDE_DEPENDS})
set_target_properties(${PROJECT_NAME}_lib PROPERTIES
  OUTPUT_NAME ${PROJECT_NAME}
)

# Add moveit_demo executable
add_executable(moveit_demo_cpp src/minimal_demo.cpp)
target_include_directories(moveit_demo_cpp PRIVATE include)
target_link_libraries(moveit_demo_cpp
  ${PROJECT_NAME}_lib
)
ament_target_dependencies(moveit_demo_cpp ${THIS_PACKAGE_INCLUDE_DEPENDS})
set_target_properties(moveit_demo_cpp PROPERTIES
  OUTPUT_NAME moveit_demo_cpp
)

# Install targets
install(TARGETS moveit_demo_cpp
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}_lib
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install extra files
install(DIRECTORY
  config
  launch
  resources
  meshes
  DESTINATION share/${PROJECT_NAME}
)

# Export libraries
ament_export_libraries(
  ${PROJECT_NAME}_lib
)

# Export include directories
ament_export_include_directories(
  include
)

# Export dependencies
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION lib/${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  script/minimal_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()