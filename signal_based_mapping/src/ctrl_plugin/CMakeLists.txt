
# listing the plugins required for compilation
SET( PLUGINS
  mylevyWalker
)


# create a library module for each plugin and link libstage to each

foreach( PLUGIN ${PLUGINS} )
  ADD_LIBRARY( ${PLUGIN} MODULE ${PLUGIN}.cpp ../robot/robot.cpp ../planner/base_planner.cpp
          ../occupancy_grid/forward_sensor_model.cpp ../occupancy_grid/occupancyGrid.cpp)
endforeach( PLUGIN )
				
#ADD_LIBRARY( fasr2 MODULE fasr2.cc astar/findpath.cpp )
ADD_LIBRARY(lasernoise MODULE lasernoise.cpp)

# add extras to the list of plugins
#SET( PLUGINS ${PLUGINS} fasr2 )
SET( PLUGINS ${PLUGINS} lasernoise)

set_source_files_properties( ${PLUGINS} PROPERTIES 
  COMPILE_FLAGS "${FLTK_CFLAGS}" 
)


# Find the Open CV package
find_package( OpenCV REQUIRED core imgproc)

# The OpenCV packages used here
SET(OpenCV_USED_LIBS opencv_core opencv_imgproc)

# Find the Boost package
find_package( Boost COMPONENTS date_time filesystem system program_options REQUIRED)

foreach( PLUGIN ${PLUGINS} )
  TARGET_LINK_LIBRARIES( ${PLUGIN} stage ${OPENGL_LIBRARIES} ${OpenCV_USED_LIBS} ${Boost_LIBRARIES})
endforeach( PLUGIN )

# delete the "lib" prefix from the plugin libraries
SET_TARGET_PROPERTIES( ${PLUGINS} PROPERTIES PREFIX "" )

# install in <prefix>/lib
install( TARGETS ${PLUGINS} DESTINATION ${PROJECT_PLUGIN_DIR}) 

