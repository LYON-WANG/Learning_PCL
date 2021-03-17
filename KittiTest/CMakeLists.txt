# Requirement of cmake version
cmake_minimum_required(VERSION 3.2)

project(KittiTest) 

set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# Set directory of executable files
set(CMAKE_RUNTIME)

# set the directory of executable files
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

# Find and include Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# FInd and include PCL
find_package(PCL REQUIRED COMPONENT common io filters visualization segmentation)
# link_directories must be before add_executable
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions( ${PCL_DEFINITIONS} )

# Name of executable file and path of source file
add_executable(${PROJECT_NAME}
	${CMAKE_CURRENT_SOURCE_DIR} src/${PROJECT_NAME}.cpp)
target_link_libraries( ${PROJECT_NAME}  ${PCL_LIBRARIES} )


