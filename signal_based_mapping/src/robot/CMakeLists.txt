#include_directories(${CMAKE_SOURCE_DIR}/controllers)



ADD_LIBRARY(robot MODULE robot.cpp )

# install in <prefix>/lib
install( TARGETS robot DESTINATION ${GENERATED_LIB})
