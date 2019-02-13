cmake_minimum_required(VERSION 2.8.3)
project(path_planning)
set(CMAKE_CXX_FLAGS "-std=c++11")
set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(Threads REQUIRED)
find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
  nav_msgs
)

catkin_package(
	INCLUDE_DIRS
	include
	CATKIN_DEPENDS
	roscpp
	std_msgs
  nav_msgs
)


include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/main.cpp
  src/Map.cpp
  src/Node.cpp
)

add_dependencies(${PROJECT_NAME}_node
	${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} Threads::Threads)

install(TARGETS ${PROJECT_NAME}_node
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
