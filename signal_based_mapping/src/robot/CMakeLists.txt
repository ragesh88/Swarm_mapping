#include_directories(${CMAKE_SOURCE_DIR}/controllers)



ADD_LIBRARY(robot MODULE robot.cpp ${CMAKE_SOURCE_DIR}/include/robot/robot.h )

# install in <prefix>/lib
install( TARGETS robot DESTINATION ${GENERATED_LIB})