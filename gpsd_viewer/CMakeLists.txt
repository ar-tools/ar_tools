cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

set(GPSDVIEWER       gpsd_viewer)
set(GPSDVIEWER_SRC   src/gpsd_viewer.cpp
					src/gui/AppData.cpp
	                src/gui/callbacks.cpp)

rosbuild_add_executable(${GPSDVIEWER}  ${GPSDVIEWER_SRC})

########## EXTERNAL DEPENDENCIES ##################

pkg_check_modules(GTK      REQUIRED gtk+-2.0)
pkg_check_modules(GMODULE  REQUIRED gmodule-2.0)
pkg_check_modules(LIBGLADE REQUIRED libglade-2.0)

include_directories(${GTK_INCLUDE_DIRS})
include_directories(${GMODULE_INCLUDE_DIRS})
include_directories(${LIBGLADE_INCLUDE_DIRS})

link_directories(${GTK_LIBRARY_DIRS})
link_directories(${GMODULE_LIBRARY_DIRS})
link_directories(${LIBGLADE_LIBRARY_DIRS})

target_link_libraries(${GPSDVIEWER} ${GTK_LIBRARIES})
target_link_libraries(${GPSDVIEWER} ${GMODULE_LIBRARIES})
target_link_libraries(${GPSDVIEWER} ${LIBGLADE_LIBRARIES})
target_link_libraries(${GPSDVIEWER} libosmgpsmap.so)

