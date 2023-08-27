cmake_minimum_required(VERSION 3.8)
project(wall_follow)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# Include cpp "include" directory
include_directories(include)

#Install Py modules
ament_python_install_package(${PROJECT_NAME})

#install Py executables
install(PROGRAMS
  scripts/wall_follow_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
