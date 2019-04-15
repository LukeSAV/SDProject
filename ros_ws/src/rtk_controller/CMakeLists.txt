cmake_minimum_required(VERSION 2.8.3)
project(rtk_controller)
set(CMAKE_CXX_FLAGS "-std=c++11")
find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  rtk
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES rtk_controller
  CATKIN_DEPENDS roscpp rospy std_msgs rtk
#  DEPENDS system_lib
)
include(CheckIncludeFiles)

catkin_install_python(PROGRAMS
	#src/rtk_manager.py
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenGL_INCLUDE_DIRS}
	${GLUT_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/main.cpp
  src/MapNode.cpp
  src/MapData.cpp
	src/xml_reader.cpp
  src/LocalOp.cpp
  src/Map.cpp
  src/Node.cpp
  src/GLDebug.cpp
)

add_dependencies(${PROJECT_NAME}_node
	${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node 
  ${catkin_LIBRARIES}
 	${OPENGL_LIBRARIES} 
	${GLUT_LIBRARY}
)

install(TARGETS ${PROJECT_NAME}_node
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY migration_rules scripts config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

add_compile_options(-lglut)
add_compile_options(-lGL)