# Author: Ankush Gupta
# Date  : 19th December, 2013

cmake_minimum_required(VERSION 2.6)
project(icp_align)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/CMakeModules")

# EXTERNAL LIBRARIES
find_package(Boost REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
		    ${CMAKE_SOURCE_DIR}/src)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_definitions("-DEIGEN_DEFAULT_TO_ROW_MAJOR")

add_executable(test_minimal test_minimal.cpp CloudViewer.cpp segment_object.cpp compute_alignment.cpp)
target_link_libraries(test_minimal ${PCL_LIBRARIES})
