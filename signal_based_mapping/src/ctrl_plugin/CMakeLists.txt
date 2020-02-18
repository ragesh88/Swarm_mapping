
# listing the plugins required for compilation
SET( PLUGINS
  mylevyWalker
  myMILevywalker
  MILevywalker_alpha
  MILevywalker_Alberto_merger
)


# create a library module for each plugin and link libstage to each

foreach( PLUGIN ${PLUGINS} )
  ADD_LIBRARY( ${PLUGIN} MODULE ${PLUGIN}.cpp ../robot/robot.cpp ../planner/base_planner.cpp
          ../occupancy_grid/inverse_sensor_model.cpp)
endforeach( PLUGIN )
				
#ADD_LIBRARY( fasr2 MODULE fasr2.cc astar/findpath.cpp )
ADD_LIBRARY(lasernoise MODULE lasernoise.cpp)

# add extras to the list of plugins
#SET( PLUGINS ${PLUGINS} fasr2 )
SET( PLUGINS ${PLUGINS} lasernoise)

set_source_files_properties( ${PLUGINS} PROPERTIES 
  COMPILE_FLAGS "${FLTK_CFLAGS}" 
)


foreach( PLUGIN ${PLUGINS} )
  TARGET_LINK_LIBRARIES( ${PLUGIN} stage ${OPENGL_LIBRARIES} ${OpenCV_USED_LIBS} ${Boost_LIBRARIES})
endforeach( PLUGIN )

# delete the "lib" prefix from the plugin libraries
SET_TARGET_PROPERTIES( ${PLUGINS} PROPERTIES PREFIX "" )

# install in <prefix>/lib
install( TARGETS ${PLUGINS} DESTINATION ${PROJECT_PLUGIN_DIR}) 

