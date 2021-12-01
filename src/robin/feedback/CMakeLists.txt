# Robin's Feedback module source files

list(APPEND robin_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/feedback.cpp
    ${CMAKE_CURRENT_LIST_DIR}/feedback.h
    ${CMAKE_CURRENT_LIST_DIR}/feedback_aim.cpp
    ${CMAKE_CURRENT_LIST_DIR}/feedback_aim.h
    ${CMAKE_CURRENT_LIST_DIR}/feedback_grasp.cpp
    ${CMAKE_CURRENT_LIST_DIR}/feedback_grasp.h
)

list(APPEND robin_INCS
    ${CMAKE_CURRENT_LIST_DIR}/feedback.h
    ${CMAKE_CURRENT_LIST_DIR}/feedback_aim.h
    ${CMAKE_CURRENT_LIST_DIR}/feedback_grasp.h
)

list(APPEND robin_SRCS
    ${CMAKE_CURRENT_LIST_DIR}/feedback.cpp
    ${CMAKE_CURRENT_LIST_DIR}/feedback_aim.cpp
    ${CMAKE_CURRENT_LIST_DIR}/feedback_grasp.cpp
)