#find_package(GTest REQUIRED HINTS "C:/git/googletest/")
#include_directories(SYSTEM ${GTEST_INCLUDE_DIRS} ${GTEST_SRC_DIR})

#add_library(gtest STATIC ${GTEST_SRC_DIR}/src/gtest-all.cc)

#enable_testing()

# VS needs -C config to run correct
#if(MSVC)
#  set(PCL_CTEST_ARGUMENTS ${PCL_CTEST_ARGUMENTS} -C $<$<CONFIG:Debug>:Debug>$<$<CONFIG:Release>:Release>)
#endif()


# Download and unpack googletest at configure time
#configure_file(CMakeLists.txt.in googletest-distribution/CMakeLists.txt)
#execute_process(COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" .
#    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/googletest-distribution"
#)
#execute_process(COMMAND "${CMAKE_COMMAND}" --build .
#    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/googletest-distribution"
#)

# Prevent GoogleTest from overriding compiler/linker options
# when building with Visual Studio
#set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

#add_subdirectory("${CMAKE_BINARY_DIR}/googletest-src"
#                 "${CMAKE_BINARY_DIR}/googletest-build"
#)


#add_subdirectory(control)

#add_subdirectory(feedback)

#add_subdirectory(primitive)
#add_subdirectory(sensor)
#add_subdirectory(solver)
#add_subdirectory(utils)