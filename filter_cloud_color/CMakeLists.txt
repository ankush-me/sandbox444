cmake_minimum_required(VERSION 2.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules")
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

message(STATUS "here this thing" ${PROJECT_SOURCE_DIR})

set(BULLET_DIR ${PROJECT_SOURCE_DIR}/../lib/bullet-2.79)
set(BULLET_LIBS BulletDynamics BulletCollision LinearMath)

set(LOG4CPLUS_DIR ${PROJECT_SOURCE_DIR}/../lib/log4cplus-1.1.0-rc3)
set(LOG4CPLUS_INCLUDE_DIRS ${LOG4CPLUS_DIR}/include ${CMAKE_BINARY_DIR}/include)
set(LOG4CPLUS_LIBRARY "log4cplus")

# EXTERNAL LIBRARIES
find_package(Boost COMPONENTS program_options  REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

message(STATUS "here this thing" ${OpenCV_LIBS})

add_definitions("-DEIGEN_DEFAULT_TO_ROW_MAJOR")

include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
                    ${BULLET_DIR}/src
		    ${PROJECT_SOURCE_DIR}
		    ${PROJECT_SOURCE_DIR}/src
		    ${EIGEN3_INCLUDE_DIRS}
		    ${LOG4CPLUS_INCLUDE_DIRS}
)


link_directories(${PCL_LIBRARY_DIRS}
                 ${PROJECT_SOURCE_DIR}/src
                 ${PROJECT_SOURCE_DIR}/utils
)


add_subdirectory(${PROJECT_SOURCE_DIR}/utils)
add_subdirectory(${PROJECT_SOURCE_DIR}/src)
add_subdirectory(${PROJECT_SOURCE_DIR}/../lib ${CMAKE_CURRENT_BINARY_DIR}/lib)


#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

rosbuild_add_executable(test src/test.cpp)
target_link_libraries(test ${PCL_LIBRARIES}
                           ${OpenCV_LIBS} cv_bridge
			   FilterWriter cloudOps utils)

rosbuild_add_executable(find_color_points src/color_point_finder.cpp)
target_link_libraries(find_color_points ${PCL_LIBRARIES}
					cloudOps utils
 					boxFilter
 					${Boost_LIBRARIES})

rosbuild_add_executable(color_associater src/color_point_associater.cpp
                                         src/filter_config.cpp
					 					 src/surgical_extraction.cpp)
target_link_libraries(color_associater ${PCL_LIBRARIES}
                                       cloudOps 
                                       utils
                                       boxFilter
                                       lineFinder
                                       ${Boost_LIBRARIES})
