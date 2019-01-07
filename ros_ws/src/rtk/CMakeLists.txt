cmake_minimum_required(VERSION 2.8.3)
project(rtk)
set(CMAKE_CXX_FLAGS "-std=c++11")
set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(Threads REQUIRED)
find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
)

catkin_package(
	INCLUDE_DIRS
	include
	CATKIN_DEPENDS
	roscpp
	std_msgs
)


include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_gps_node
	src/Landmark.cpp
	src/NMEAData.cpp
	src/ntrip_helper.cpp
	src/serial.cpp
	src/ntripclient.cpp
	src/Node.cpp
	src/xml_reader.cpp
)
add_dependencies(${PROJECT_NAME}_gps_node
	${catkin_EXPORTED_TARGETS}
)

#set_target_properties(${PROJECT_NAME}_gps_node PROPERTIES OUTPUT_NAME gps_node PREFIX "")
target_link_libraries(${PROJECT_NAME}_gps_node ${catkin_LIBRARIES} Threads::Threads)

install(TARGETS ${PROJECT_NAME}_gps_node
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

