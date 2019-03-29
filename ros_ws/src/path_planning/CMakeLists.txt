cmake_minimum_required(VERSION 2.8.3)
project(path_planning)
set(CMAKE_CXX_FLAGS "-std=c++11")
set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(Threads REQUIRED)
find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	nav_msgs
	rtk
)

catkin_package(
	INCLUDE_DIRS
	include

	CATKIN_DEPENDS
	roscpp
	rospy
	std_msgs
	nav_msgs
	rtk
)


include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${OpenGL_INCLUDE_DIRS}
	${GLUT_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
	src/main.cpp
	src/Map.cpp
	src/Node.cpp
	src/GlobalMap.cpp
	src/GlobalNode.cpp
	src/xml_reader.cpp
	src/GLDebug.cpp
	src/LocalOp.cpp
)

add_dependencies(${PROJECT_NAME}_node
	${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
	${PROJECT_NAME}_node 
	${catkin_LIBRARIES} 
	${OPENGL_LIBRARIES} 
	${GLUT_LIBRARY}
	Threads::Threads
)

install(TARGETS ${PROJECT_NAME}_node
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_compile_options(-lglut)
add_compile_options(-lGL)