
# listing the tests required for compilation
SET( TESTS
        mySampleTest
        myPlannerTest
)


# create a library module for each plugin and link libstage to each

foreach( TEST ${TESTS} )
  ADD_EXECUTABLE( ${TEST} ${TEST}.cpp ../src/robot/robot.cpp ../src/planner/base_planner.cpp
                   ../src/occupancy_grid/inverse_sensor_model.cpp)
endforeach( TEST )




set_source_files_properties( ${TESTS} PROPERTIES
  COMPILE_FLAGS "${FLTK_CFLAGS}" 
)

foreach( TEST ${TESTS} )
  TARGET_LINK_LIBRARIES( ${TEST} stage ${OpenCV_USED_LIBS} ${Boost_LIBRARIES} )
endforeach( TEST )


# install in <prefix>/lib
install( TARGETS ${TESTS} DESTINATION ${PROJECT_BINARY_DIR})

