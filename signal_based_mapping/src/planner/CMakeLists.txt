


ADD_LIBRARY(planners MODULE base_planner.cpp)

# install in <prefix>/lib
install( TARGETS planners DESTINATION ${GENERATED_LIB})
