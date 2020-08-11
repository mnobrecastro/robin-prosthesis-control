cmake_minimum_required(VERSION 3.1.0)

project(robin-prosthesis-control)

find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#set(REALSENSE_DIR "C:/Program Files (x86)/Intel RealSense SDK 2.0")
#include_directories("${REALSENSE_DIR}/include/librealsense2")
#link_directories("${REALSENSE_DIR}/lib/x64")

add_executable (main src/main.cpp)
target_link_libraries (main ${PCL_LIBRARIES})