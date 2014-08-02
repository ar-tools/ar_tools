
cmake_minimum_required(VERSION 2.8)

project(ar_pose)

find_package(catkin REQUIRED
	roscpp
	visualization_msgs
	artoolkit
	geometry_msgs
	tf
	resource_retriever
	image_transport
	cv_bridge
	message_generation
)

include_directories(${catkin_INCLUDE_DIRS} include)

add_message_files(FILES
	ARMarker.msg
	ARMarkers.msg
)

generate_messages(DEPENDENCIES
	geometry_msgs
)

catkin_package()

add_executable(ar_single
	src/ar_single.cpp
	src/object.cpp
)
target_link_libraries(ar_single
	${catkin_LIBRARIES}
)
add_dependencies(ar_single
	${CMAKE_PROJECT_NAME}_generate_messages_cpp
)

add_executable(ar_multi
	src/ar_multi.cpp
	src/object.cpp
)
target_link_libraries(ar_multi
	${catkin_LIBRARIES}
)
add_dependencies(ar_multi
	${CMAKE_PROJECT_NAME}_generate_messages_cpp
)
