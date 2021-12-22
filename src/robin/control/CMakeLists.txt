# Robin's Control module source files

list(APPEND robin_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/control.cpp
    ${CMAKE_CURRENT_LIST_DIR}/control.h
    ${CMAKE_CURRENT_LIST_DIR}/control_simple.cpp
    ${CMAKE_CURRENT_LIST_DIR}/control_simple.h
    ${CMAKE_CURRENT_LIST_DIR}/control_sequential.cpp
    ${CMAKE_CURRENT_LIST_DIR}/control_sequential.h
    ${CMAKE_CURRENT_LIST_DIR}/control_pid.cpp
    ${CMAKE_CURRENT_LIST_DIR}/control_pid.h
)

list(APPEND robin_INCS
    ${CMAKE_CURRENT_LIST_DIR}/control.h
    ${CMAKE_CURRENT_LIST_DIR}/control_simple.h
    ${CMAKE_CURRENT_LIST_DIR}/control_sequential.h
    ${CMAKE_CURRENT_LIST_DIR}/control_pid.h
)

list(APPEND robin_SRCS
    ${CMAKE_CURRENT_LIST_DIR}/control.cpp
    ${CMAKE_CURRENT_LIST_DIR}/control_simple.cpp
    ${CMAKE_CURRENT_LIST_DIR}/control_sequential.cpp
    ${CMAKE_CURRENT_LIST_DIR}/control_pid.cpp
)