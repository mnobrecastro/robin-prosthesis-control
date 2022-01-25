cmake_minimum_required(VERSION 3.1.0)

project(lccp-semiauto-control)

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
set(ENGACOUSTICS_LIB "TactorInterface.lib")
link_libraries(${ENGACOUSTICS_LIB})

add_executable (lccp-semiauto-control lccp-semiauto-control.cpp)
target_link_libraries (lccp-semiauto-control robin)

install(TARGETS lccp-semiauto-control DESTINATION bin)