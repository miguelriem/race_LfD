cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set (CMAKE_CXX_FLAGS "-frounding-math -Wunknown-pragmas -Wno-deprecated -Wno-deprecated-declarations -O3 ")

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})


rosbuild_add_library(${PROJECT_NAME} src/race_tabletop_segmentation.cpp)
target_link_libraries(${PROJECT_NAME} boost_system)
target_link_libraries(${PROJECT_NAME} /usr/lib/libCGAL.so)
target_link_libraries(${PROJECT_NAME} /usr/lib/libCGAL_Core.so)
target_link_libraries(${PROJECT_NAME} /usr/lib/x86_64-linux-gnu/libmpfr.so.4)
target_link_libraries(${PROJECT_NAME} /usr/lib/x86_64-linux-gnu/libgmp.so.10)


rosbuild_add_executable(race_tabletop_segmentation_node	src/race_tabletop_segmentation_node.cpp)
target_link_libraries(race_tabletop_segmentation_node ${PROJECT_NAME})


