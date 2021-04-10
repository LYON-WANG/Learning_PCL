# Requirement of cmake version
cmake_minimum_required(VERSION 3.2)

project(Lidar_odometry) 

set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# Set directory of executable files
set(CMAKE_RUNTIME)

# set the directory of executable files
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)
include_directories(SYSTEM ${CMAKE_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

# Find and include Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# FInd and include PCL
find_package(PCL REQUIRED)
# link_directories must be before add_executable
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions( ${PCL_DEFINITIONS} )

# Find and include Python.h
find_package(PythonInterp 2.7 REQUIRED)
find_package(PythonLibs 2.7 REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS} ${Python_NumPy_INCLUDE_DIRS})
include_directories(${PYTHONHOME}/include)
include_directories(${PYTHONHOME}/Lib/site-packages/numpy/core/include)
link_directories(${PYTHONHOME}/libs)
add_definitions(-DMATPLOTLIBCPP_PYTHON_HEADER=Python.h)
include_directories(SYSTEM ${matplotlib_SOURCE_DIR})

# Name of executable file and path of source file
add_executable(${PROJECT_NAME}
	${CMAKE_CURRENT_SOURCE_DIR} src/${PROJECT_NAME}.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/src/matplotlibcpp.h)
target_link_libraries( ${PROJECT_NAME}  ${PCL_LIBRARIES} ${PYTHON_LIBRARIES})




