
cmake_minimum_required(VERSION 2.8)

project(artoolkit)

find_package(catkin REQUIRED)

catkin_package(
	LIBRARIES GL glut ARgsub AR ARMulti ARvideo GL glut
)

include(ExternalProject)
ExternalProject_Add(ARToolkit
	# Settings:
	#  out of source build is not supported
	BINARY_DIR ARToolkit-prefix/src/ARToolkit

	# Download
	URL http://csc.kth.se/~fevb/downloads/ARToolKit-SVN-r311.tar.gz
	URL_MD5 2f3537f782b5506133bcc79f2daba3bb 

	# Patch
	PATCH_COMMAND patch -p0 < ${CMAKE_CURRENT_SOURCE_DIR}/patch_auto_config
		COMMAND patch -p0 < ${CMAKE_CURRENT_SOURCE_DIR}/patch_v4l
		COMMAND sh ${CMAKE_CURRENT_SOURCE_DIR}/cleanup.sh

	# Configure (with V4L driver)
	CONFIGURE_COMMAND ./Configure 1 n n n

	# Build
	BUILD_COMMAND make

	INSTALL_COMMAND cp -r include/AR ${CATKIN_DEVEL_PREFIX}/include/
		COMMAND cp -r lib/libAR.a lib/libARgsub.a lib/libARgsub_lite.a lib/libARMulti.a lib/libARvideo.a lib/libARgsubUtil.a
			${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}
)

# add ARgsub library
add_custom_target(
  copy_ARgsub ALL
  COMMAND cmake -E copy ${CMAKE_CURRENT_BINARY_DIR}/ARToolkit-prefix/src/ARToolkit/lib/libARgsub.a ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/libARgsub.a)

add_library(ARgsub STATIC)
set_target_properties(ARgsub PROPERTIES LINKER_LANGUAGE CXX)

add_dependencies(ARgsub copy_ARgsub)
add_dependencies(copy_ARgsub ARToolkit)

# add AR library
add_custom_target(
  copy_AR ALL
  COMMAND cmake -E copy ${CMAKE_CURRENT_BINARY_DIR}/ARToolkit-prefix/src/ARToolkit/lib/libAR.a ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/libAR.a)

add_library(AR STATIC)
set_target_properties(AR PROPERTIES LINKER_LANGUAGE CXX)

add_dependencies(AR copy_AR)
add_dependencies(copy_AR ARToolkit)

# add ARMulti library
add_custom_target(
  copy_ARMulti ALL
  COMMAND cmake -E copy ${CMAKE_CURRENT_BINARY_DIR}/ARToolkit-prefix/src/ARToolkit/lib/libARMulti.a ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/libARMulti.a)

add_library(ARMulti STATIC)
set_target_properties(ARMulti PROPERTIES LINKER_LANGUAGE CXX)

add_dependencies(ARMulti copy_ARMulti)
add_dependencies(copy_ARMulti ARToolkit)

# add ARvideo library
add_custom_target(
  copy_ARvideo ALL
  COMMAND cmake -E copy ${CMAKE_CURRENT_BINARY_DIR}/ARToolkit-prefix/src/ARToolkit/lib/libARvideo.a ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/libARvideo.a)

add_library(ARvideo STATIC)
set_target_properties(ARvideo PROPERTIES LINKER_LANGUAGE CXX)

add_dependencies(ARvideo copy_ARvideo)
add_dependencies(copy_ARvideo ARToolkit)

# add ARgsubUtil library
add_custom_target(
  copy_ARgsubUtil ALL
  COMMAND cmake -E copy ${CMAKE_CURRENT_BINARY_DIR}/ARToolkit-prefix/src/ARToolkit/lib/libARgsubUtil.a ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/libARgsubUtil.a)

add_library(ARgsubUtil STATIC)
set_target_properties(ARgsubUtil PROPERTIES LINKER_LANGUAGE CXX)

add_dependencies(ARgsubUtil copy_ARgsubUtil)
add_dependencies(copy_ARgsubUtil ARToolkit)


# add ARgsub_lite library
add_custom_target(
  copy_ARgsub_lite ALL
  COMMAND cmake -E copy ${CMAKE_CURRENT_BINARY_DIR}/ARToolkit-prefix/src/ARToolkit/lib/libARgsub_lite.a ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/libARgsub_lite.a)

add_library(ARgsub_lite STATIC)
set_target_properties(ARgsub_lite PROPERTIES LINKER_LANGUAGE CXX)

add_dependencies(ARgsub_lite copy_ARgsub_lite)
add_dependencies(copy_ARgsub_lite ARToolkit)