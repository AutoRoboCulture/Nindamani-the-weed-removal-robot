cmake_minimum_required(VERSION 3.5)
project(rpicam_ai_interface)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -fpermissive")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(sensor_msgs REQUIRED)

# DEPENDS: system dependencies of this project that dependent projects also need
ament_export_include_directories(include
  ${ament_INCLUDE_DIRS}
)

ament_package()
install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
  scripts/rpicam_ai_interface.py
  scripts/camera.py
  scripts/perspectiveImage.py
  DESTINATION lib/${PROJECT_NAME}
)


