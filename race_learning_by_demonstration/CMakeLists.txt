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

set (CMAKE_CXX_FLAGS "-frounding-math -Wno-deprecated -Wno-deprecated-declarations -O3")
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


#rosbuild_add_executable(gesture_detection_node src/gesture_detection_node.cpp)
#target_link_libraries(gesture_detection_node /usr/lib/libCGAL.so)
#target_link_libraries(gesture_detection_node /usr/lib/libCGAL_Core.so)
#target_link_libraries(gesture_detection_node /usr/lib/x86_64-linux-gnu/libmpfr.so.4)
#target_link_libraries(gesture_detection_node /usr/lib/x86_64-linux-gnu/libgmp.so.10)


rosbuild_add_executable(learning_by_demonstration src/main.cpp)
target_link_libraries(learning_by_demonstration /usr/lib/libCGAL.so)
target_link_libraries(learning_by_demonstration /usr/lib/libCGAL_Core.so)
target_link_libraries(learning_by_demonstration /usr/lib/x86_64-linux-gnu/libmpfr.so.4)
target_link_libraries(learning_by_demonstration /usr/lib/x86_64-linux-gnu/libgmp.so.10)

#rosbuild_add_executable(icp_test src/icp_test.cpp)

#rosbuild_add_executable(openni_tracking src/openni_tracking.cpp)
