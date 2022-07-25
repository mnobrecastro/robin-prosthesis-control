#Robin's Source folder
cmake_minimum_required(VERSION 3.1.0)

project(robin-lib)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(PCL 1.11 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_libraries(${PCL_LIBRARIES})

find_package(OpenCV REQUIRED PATHS "C:/opencv/build/")
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_DIR}/x64/vc15/lib)
link_libraries(${OpenCV_LIBS})

set(REALSENSE_INSTALL_PATH CACHE PATH "C:/Program Files (x86)/Intel RealSense SDK 2.0/")
include_directories(${REALSENSE_INSTALL_PATH}/include/)
link_directories(${REALSENSE_INSTALL_PATH}/lib/x64)
set(REALSENSE_LIB "realsense2.lib")
link_libraries(${REALSENSE_LIBS})

set(ROYALE_INSTALL_PATH CACHE PATH "C:/Program Files/royale/3.23.0.86/")
include_directories(${ROYALE_INSTALL_PATH}/include)
include_directories(${ROYALE_INSTALL_PATH}/samples/inc)
link_directories(${ROYALE_INSTALL_PATH}/lib)
set(ROYALE_LIB "royale.lib")
link_libraries(${ROYALE_LIB})

set(ENGACOUSTICS_PATH CACHE PATH ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/tdk-engacoustics/)
include_directories(${ENGACOUSTICS_PATH}/include)
link_directories(${ENGACOUSTICS_PATH}/lib)
#find_library(ENGACOUSTICS_LIB TactorInterface HINTS ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/tflite/lib/)
set(ENGACOUSTICS_LIB "TactorInterface.lib")
link_libraries(${ENGACOUSTICS_LIB})


set(robin_SOURCES "")
set(robin_INCS "")
set(robin_SRCS "")

add_subdirectory(robin)

include(robin/control/CMakeLists.txt)
include(robin/feedback/CMakeLists.txt)
include(robin/primitive/CMakeLists.txt)
include(robin/sensor/CMakeLists.txt)
include(robin/solver/CMakeLists.txt)
include(robin/utils/CMakeLists.txt)

add_library(robin STATIC ${robin_SRCS})
target_include_directories(robin PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

install(TARGETS robin DESTINATION lib)
install(FILES ${robin_INCS} DESTINATION include)

