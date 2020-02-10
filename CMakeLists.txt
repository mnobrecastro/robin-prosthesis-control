cmake_minimum_required(VERSION 2.6)

project(michelangelo-depth)

set(CMAKE_PREFIX_PATH "C:/Program Files/royale/3.20.0.62/share")
find_package(royale REQUIRED)
set(royale_LIB_DIR "C:/Program Files/royale/3.20.0.62/lib") #
link_directories(${royale_LIB_DIR})

find_package(PCL REQUIRED)
if(PCL_FOUND)
  include_directories(${PCL_INCLUDE_DIRS})
  add_definitions(${PCL_DEFINITIONS})
  link_directories(${PCL_LIBRARY_DIRS})
endif()

set(DEPENDENCIES royale ${PCL_LIBRARIES})

add_executable(main src/main.cpp)
set_property(TARGET main PROPERTY CXX_STANDARD 11)
target_link_libraries(main ${DEPENDENCIES})