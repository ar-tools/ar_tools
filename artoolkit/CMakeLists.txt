
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
	URL https://code.ros.org/svn/release/download/thirdparty/ARToolKit-SVN-r311.tar.gz
	URL_HASH MD5=2f3537f782b5506133bcc79f2daba3bb

	# Patch
	PATCH_COMMAND patch -p0 < ${CMAKE_CURRENT_SOURCE_DIR}/patch_auto_config
		COMMAND patch -p0 < ${CMAKE_CURRENT_SOURCE_DIR}/patch_v4l
		COMMAND sh ${CMAKE_CURRENT_SOURCE_DIR}/cleanup.sh

	# Configure (with V4L driver)
	CONFIGURE_COMMAND ./Configure 1 n n n

	# Build
	BUILD_COMMAND make

	INSTALL_COMMAND cp -r include/AR ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}
		COMMAND cp -r lib/libAR.a lib/libARgsub.a lib/libARgsub_lite.a lib/libARMulti.a lib/libARvideo.a lib/libARgsubUtil.a
			${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}
)
