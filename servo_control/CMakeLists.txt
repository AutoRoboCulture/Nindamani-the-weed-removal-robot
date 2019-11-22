cmake_minimum_required(VERSION 3.5)
project(servo_control)

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
find_package(rclcpp REQUIRED)
find_package(rcl REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

## Generate services in the 'srv' folder

set(msg_files
"srv/AutoCalibrate.srv"
"srv/GetAbsolutePosition.srv"
"srv/GetFeedbackGain.srv"
"srv/GetIntegralGain.srv"
"srv/GetMaxMotorSpeed.srv"
"srv/GetPositionEncoder.srv"
"srv/GetProportionateGain.srv"
"srv/LoadFactorySettings.srv"
"srv/ReadDamping.srv"
"srv/ReadMotorSpeed.srv"
"srv/SetAbsolutePosition.srv"
"srv/SetFeedbackGain.srv"
"srv/SetIntegralGain.srv"
"srv/SetMaxMotorSpeed.srv"
"srv/SetMotorSpeed.srv"
"srv/SetPositionEncoder.srv"
"srv/SetProportionateGain.srv"
"srv/SetRelativePosition.srv"
"srv/WriteDamping.srv"
"srv/OpenGripper.srv"
"srv/CloseGripper.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# DEPENDS: system dependencies of this project that dependent projects also need
ament_export_include_directories(include
  ${ament_INCLUDE_DIRS}
)


install(
  PROGRAMS
  scripts/servo_ros2_interface.py
  scripts/servo_driver.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()


