cmake_minimum_required(VERSION 2.8.3)
project(fb)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  sensor_msgs
  rtk
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fb
 CATKIN_DEPENDS message_generation rospy rtk
#  DEPENDS system_lib
)

catkin_install_python(PROGRAMS
	src/fb_manager.py
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
