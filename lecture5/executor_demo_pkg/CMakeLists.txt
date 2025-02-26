cmake_minimum_required(VERSION 3.8)
project(executor_demo_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# ---------------------------------------------
# C++
# ---------------------------------------------
include_directories(include)

#----------------------------------------------
# Singlethreaded Executor Example
add_executable(singlethreaded_executor_cpp src/singlethreaded_executor.cpp)
ament_target_dependencies(singlethreaded_executor_cpp rclcpp)

install(TARGETS
singlethreaded_executor_cpp
DESTINATION lib/${PROJECT_NAME}
)
#----------------------------------------------
# Multithreaded Executor with 2 Mutually Exclusive Callback Groups
add_executable(dual_mutually_exclusive_cpp src/dual_mutually_exclusive.cpp)
ament_target_dependencies(dual_mutually_exclusive_cpp rclcpp)

install(TARGETS
dual_mutually_exclusive_cpp
DESTINATION lib/${PROJECT_NAME}
)
#----------------------------------------------
# Multithreaded Executor with 1 Mutually Exclusive and 1 Reentrant Group
add_executable(exclusive_reentrant_cpp src/exclusive_reentrant.cpp)
ament_target_dependencies(exclusive_reentrant_cpp rclcpp)

install(TARGETS
exclusive_reentrant_cpp
DESTINATION lib/${PROJECT_NAME}
)
#----------------------------------------------
# Multithreaded Executor with 1 Reentrant Group
add_executable(reentrant_cpp src/reentrant.cpp)
ament_target_dependencies(reentrant_cpp rclcpp)

install(TARGETS
reentrant_cpp
DESTINATION lib/${PROJECT_NAME}
)
# ---------------------------------------------
# Python
# ---------------------------------------------
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  script/singlethreaded_executor.py
  script/dual_mutually_exclusive.py
  script/exclusive_reentrant.py
  script/reentrant.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
