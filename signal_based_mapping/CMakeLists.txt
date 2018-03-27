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
link_directories(GENERATED_LIB)


# include the header files directory
include_directories(${CMAKE_SOURCE_DIR}/include)

set(MY_HEADERS "${CMAKE_SOURCE_DIR}/include")

# Descend into the controller source directory
add_subdirectory(src)


# Descend into the doc directory for documentation generation using Doxygen
IF(DOC)
    message("The doc value is ${DOC}")
    add_subdirectory(doc)
ENDIF()


# Descend into the test directory
add_subdirectory(test)
