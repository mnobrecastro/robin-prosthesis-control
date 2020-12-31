# Robin's Sensor module source files

list(APPEND robin_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/sensor.cpp
    ${CMAKE_CURRENT_LIST_DIR}/sensor.h
    ${CMAKE_CURRENT_LIST_DIR}/sensor_array.cpp
    ${CMAKE_CURRENT_LIST_DIR}/sensor_array.h

    ${CMAKE_CURRENT_LIST_DIR}/sensor1.cpp
    ${CMAKE_CURRENT_LIST_DIR}/sensor1.h

    ${CMAKE_CURRENT_LIST_DIR}/sensor2.cpp
    ${CMAKE_CURRENT_LIST_DIR}/sensor2.h

    ${CMAKE_CURRENT_LIST_DIR}/sensor3.cpp
    ${CMAKE_CURRENT_LIST_DIR}/sensor3.h

    ${CMAKE_CURRENT_LIST_DIR}/camera_depth.cpp
    ${CMAKE_CURRENT_LIST_DIR}/camera_depth.h
    ${CMAKE_CURRENT_LIST_DIR}/realsense_d400.cpp
    ${CMAKE_CURRENT_LIST_DIR}/realsense_d400.h
    ${CMAKE_CURRENT_LIST_DIR}/royale_picoflexx.cpp
    ${CMAKE_CURRENT_LIST_DIR}/royale_picoflexx.h

	${CMAKE_CURRENT_LIST_DIR}/laser_scanner.cpp
    ${CMAKE_CURRENT_LIST_DIR}/laser_scanner.h
	${CMAKE_CURRENT_LIST_DIR}/laser_array.cpp
    ${CMAKE_CURRENT_LIST_DIR}/laser_array.h

    ${CMAKE_CURRENT_LIST_DIR}/sensor4.cpp
    ${CMAKE_CURRENT_LIST_DIR}/sensor4.h

    ${CMAKE_CURRENT_LIST_DIR}/hand.cpp
    ${CMAKE_CURRENT_LIST_DIR}/hand.h
    ${CMAKE_CURRENT_LIST_DIR}/hand_direct.cpp
    ${CMAKE_CURRENT_LIST_DIR}/hand_direct.h
    ${CMAKE_CURRENT_LIST_DIR}/hand_udp.cpp
    ${CMAKE_CURRENT_LIST_DIR}/hand_udp.h
    ${CMAKE_CURRENT_LIST_DIR}/hand_michelangelo.cpp
    ${CMAKE_CURRENT_LIST_DIR}/hand_michelangelo.h
)

list(APPEND robin_INCS
    ${CMAKE_CURRENT_LIST_DIR}/sensor.h
    ${CMAKE_CURRENT_LIST_DIR}/sensor_array.h

    ${CMAKE_CURRENT_LIST_DIR}/sensor1.h

    ${CMAKE_CURRENT_LIST_DIR}/sensor2.h

    ${CMAKE_CURRENT_LIST_DIR}/sensor3.h

    ${CMAKE_CURRENT_LIST_DIR}/camera_depth.h
    ${CMAKE_CURRENT_LIST_DIR}/realsense_d400.h
    ${CMAKE_CURRENT_LIST_DIR}/royale_picoflexx.h

    ${CMAKE_CURRENT_LIST_DIR}/laser_scanner.h
    ${CMAKE_CURRENT_LIST_DIR}/laser_array.h

    ${CMAKE_CURRENT_LIST_DIR}/sensor4.h

    ${CMAKE_CURRENT_LIST_DIR}/hand.h
    ${CMAKE_CURRENT_LIST_DIR}/hand_direct.h
    ${CMAKE_CURRENT_LIST_DIR}/hand_udp.h
    ${CMAKE_CURRENT_LIST_DIR}/hand_michelangelo.h
)

list(APPEND robin_SRCS
    ${CMAKE_CURRENT_LIST_DIR}/sensor.cpp
    ${CMAKE_CURRENT_LIST_DIR}/sensor_array.cpp

    ${CMAKE_CURRENT_LIST_DIR}/sensor1.cpp

    ${CMAKE_CURRENT_LIST_DIR}/sensor2.cpp

    ${CMAKE_CURRENT_LIST_DIR}/sensor3.cpp

    ${CMAKE_CURRENT_LIST_DIR}/camera_depth.cpp
    ${CMAKE_CURRENT_LIST_DIR}/realsense_d400.cpp
    ${CMAKE_CURRENT_LIST_DIR}/royale_picoflexx.cpp

	${CMAKE_CURRENT_LIST_DIR}/laser_scanner.cpp
	${CMAKE_CURRENT_LIST_DIR}/laser_array.cpp

    ${CMAKE_CURRENT_LIST_DIR}/sensor4.cpp

    ${CMAKE_CURRENT_LIST_DIR}/hand.cpp
    ${CMAKE_CURRENT_LIST_DIR}/hand_direct.cpp
    ${CMAKE_CURRENT_LIST_DIR}/hand_udp.cpp
    ${CMAKE_CURRENT_LIST_DIR}/hand_michelangelo.cpp
)