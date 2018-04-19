



ADD_LIBRARY(forward_sensor_model MODULE forward_sensor_model.cpp)

ADD_LIBRARY(inverse_sensor_model MODULE inverse_sensor_model.cpp)

# install in <prefix>/lib
install( TARGETS forward_sensor_model inverse_sensor_model DESTINATION ${GENERATED_LIB})
