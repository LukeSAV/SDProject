cmake_minimum_required(VERSION 2.8.3)
project(serial)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES serial
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

include(CheckIncludeFiles)
check_include_files(linux/joystick.h HAVE_LINUX_JOYSTICK_H)



catkin_install_python(PROGRAMS
	src/serial_manager.py
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY migration_rules scripts config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
