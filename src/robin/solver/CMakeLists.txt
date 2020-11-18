# Robin's Solver module source files

list(APPEND robin_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/solver.cpp
    ${CMAKE_CURRENT_LIST_DIR}/solver.h
	
    ${CMAKE_CURRENT_LIST_DIR}/solver1.cpp
    ${CMAKE_CURRENT_LIST_DIR}/solver1.h
	${CMAKE_CURRENT_LIST_DIR}/solver1_emg.cpp
	${CMAKE_CURRENT_LIST_DIR}/solver1_emg.h
	
##	${CMAKE_CURRENT_LIST_DIR}/solver2.cpp
##	${CMAKE_CURRENT_LIST_DIR}/solver2.h
	
	${CMAKE_CURRENT_LIST_DIR}/solver3.cpp
	${CMAKE_CURRENT_LIST_DIR}/solver3.h
	${CMAKE_CURRENT_LIST_DIR}/solver3_lccp.cpp
	${CMAKE_CURRENT_LIST_DIR}/solver3_lccp.h
	${CMAKE_CURRENT_LIST_DIR}/solver3_lasers.cpp
	${CMAKE_CURRENT_LIST_DIR}/solver3_lasers.h
)

list(APPEND robin_INCS
    ${CMAKE_CURRENT_LIST_DIR}/solver.h
	
    ${CMAKE_CURRENT_LIST_DIR}/solver1.h
	${CMAKE_CURRENT_LIST_DIR}/solver1_emg.h
	
	##	${CMAKE_CURRENT_LIST_DIR}/solver2.h
	
	${CMAKE_CURRENT_LIST_DIR}/solver3.h
	${CMAKE_CURRENT_LIST_DIR}/solver3_lccp.h
	${CMAKE_CURRENT_LIST_DIR}/solver3_lasers.h
)

list(APPEND robin_SRCS
    ${CMAKE_CURRENT_LIST_DIR}/solver.cpp
	
    ${CMAKE_CURRENT_LIST_DIR}/solver1.cpp
	${CMAKE_CURRENT_LIST_DIR}/solver1_emg.cpp
	
##	${CMAKE_CURRENT_LIST_DIR}/solver2.cpp
	
	${CMAKE_CURRENT_LIST_DIR}/solver3.cpp
	${CMAKE_CURRENT_LIST_DIR}/solver3_lccp.cpp
	${CMAKE_CURRENT_LIST_DIR}/solver3_lasers.cpp
)