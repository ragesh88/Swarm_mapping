cmake_minimum_required(VERSION 3.5)
project(stage_ctrl_plugin)

# setting the c++ standard 
set (CMAKE_CXX_STANDARD 14)

# Setting to render documentation
set(DOC OFF)

# set the plugin install directory 
set(PROJECT_PLUGIN_DIR "${CMAKE_SOURCE_DIR}/ctrl_lib")

# set the generated library install directory 
set(GENERATED_LIB "${CMAKE_SOURCE_DIR}/gen_lib")


# Set the bin directory to install the executables
set(PROJECT_BINARY_DIR "${CMAKE_SOURCE_DIR}/bin")

# add the stage library 
link_directories(~/stg/lib)

# add the generated libraries
#link_directories(GENERATED_LIB)

# Find the Open CV package
find_package( OpenCV REQUIRED core imgproc highgui)


# The OpenCV packages used here
SET(OpenCV_USED_LIBS opencv_core opencv_imgproc opencv_highgui)


# Find the Boost package
find_package( Boost COMPONENTS date_time filesystem system program_options REQUIRED)



# include the header files directory
include_directories(${CMAKE_SOURCE_DIR}/include)

#include the stage library header files
include_directories(~/stg/include)

set(MY_HEADERS "${CMAKE_SOURCE_DIR}/include")

# Descend into the controller source directory
add_subdirectory(src)


# Descend into the doc directory for documentation generation using Doxygen
# The document is generated if DOC variable is "ON" or if it is build in release mode
IF(DOC OR CMAKE_BUILD_TYPE MATCHES "^[Rr]elease")
    message("Document generation command using Doxygen added to Makefile")
    message("The doc value is ${DOC}")
    add_subdirectory(doc)
ENDIF()

if ()
    # build the docs
endif()


# Descend into the test directory
#add_subdirectory(test)
