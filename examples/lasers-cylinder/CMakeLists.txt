cmake_minimum_required(VERSION 3.1.0)

project(lasers-cylinder)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../dependencies)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../src)

set(REALSENSE_INSTALL_PATH CACHE PATH "C:/Program Files (x86)/Intel RealSense SDK 2.0")
include_directories(${REALSENSE_INSTALL_PATH}/include/)
link_directories(${REALSENSE_INSTALL_PATH}/lib/x64)
set(REALSENSE_LIBS "realsense2.lib")
link_libraries(${REALSENSE_LIBS})

set(ROYALE_INSTALL_PATH CACHE PATH "C:/Program Files/royale/3.23.0.86")
include_directories(${ROYALE_INSTALL_PATH}/include)
include_directories(${ROYALE_INSTALL_PATH}/samples/inc)
link_directories(${ROYALE_INSTALL_PATH}/lib)
set(ROYALE_LIBS "royale.lib")
link_libraries(${ROYALE_LIBS})

set(ENGACOUSTICS_PATH CACHE PATH ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/tdk-engacoustics/)
include_directories(${ENGACOUSTICS_PATH}/include)
link_directories(${ENGACOUSTICS_PATH}/lib)
#find_library(ENGACOUSTICS_LIB TactorInterface HINTS ${CMAKE_CURRENT_SOURCE_DIR}../../dependencies/tflite/lib/)
set(ENGACOUSTICS_LIB "TactorInterface.lib")
link_libraries(${ENGACOUSTICS_LIB})

add_executable (lasers-cylinder lasers-cylinder.cpp)
target_link_libraries (lasers-cylinder robin)

install(TARGETS lasers-cylinder DESTINATION bin)