cmake_minimum_required(VERSION 2.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

#-------------------------- Generally useful function ------------------
function(echo_all_variables)
  # prints out all the CMAKE variables
  message(STATUS "")
  get_cmake_property(vs VARIABLES)
  foreach(v ${vs})
    message(STATUS "${v}='${${v}}'")
  endforeach(v)
  message(STATUS "")
endfunction(echo_all_variables)

#--- EXAMPLE USAGE ---
#echo_all_variables()



macro(find_libraries _libs _path lib_out)
  # finds the libraries in LIBS in PATH
  foreach(lib ${${_libs}})

    set(out_name ${lib}_out)
    find_library(${out_name} ${lib} ${${_path}})

    if (NOT ${${out_name}} STREQUAL ${out_name}-NOTFOUND)
      list(APPEND ${lib_out} ${${out_name}})
    else()
      message(FATAL_ERROR "${lib} not found in ${${_path}}")
    endif()

  endforeach(lib)

endmacro()

#--- EXAMPLE USAGE ---
#
# set(tf_libs "libtf.so" "libgtest.a")
# find_libraries(tf_libs TF_LIBRARY_DIRS out_tf_libs)
# message(STATUS  "${out_tf_libs}")

#------------------------------------------------------------------------

rosbuild_init()

set(FILTER_COLOR_CLOUD_DIR ${PROJECT_SOURCE_DIR})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${FILTER_COLOR_CLOUD_DIR}/CMakeModules")

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

message(STATUS "here this thing" ${PROJECT_SOURCE_DIR})

set(BULLET_DIR ${FILTER_COLOR_CLOUD_DIR}/../lib/bullet-2.79)
set(BULLET_PATH "/opt/ros/$ENV{ROS_DISTRO}/stacks/bullet")
set(BULLET_INCLUDE_DIRS "${BULLET_PATH}/include")
set(BULLET_LIBS BulletDynamics BulletCollision LinearMath)

set(LOG4CPLUS_DIR ${FILTER_COLOR_CLOUD_DIR}/../lib/log4cplus-1.1.0-rc3)
set(LOG4CPLUS_INCLUDE_DIRS ${LOG4CPLUS_DIR}/include ${CMAKE_BINARY_DIR}/include)
set(LOG4CPLUS_LIBRARY "log4cplus")

# Set path for OpenCV: This is compiling with ROS's openCV. More of a hack.
set(OpenCV_DIR "/opt/ros/$ENV{ROS_DISTRO}/share/OpenCV")
find_package(OpenCV REQUIRED)

#echo_all_variables()

# EXTERNAL LIBRARIES
find_package(Boost COMPONENTS program_options  REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

add_definitions("-DEIGEN_DEFAULT_TO_ROW_MAJOR")


include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
                    ${BULLET_INCLUDE_DIRS} ${BULLET_DIR}/src
		    ${FILTER_COLOR_CLOUD_DIR}
		    ${FILTER_COLOR_CLOUD_DIR}/src
		    ${EIGEN3_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS}
                 ${FILTER_COLOR_CLOUD_DIR}/src
                 ${FILTER_COLOR_CLOUD_DIR}/utils
)

add_subdirectory(${FILTER_COLOR_CLOUD_DIR}/utils)
add_subdirectory(${FILTER_COLOR_CLOUD_DIR}/src)
add_subdirectory(${FILTER_COLOR_CLOUD_DIR}/../lib ${CMAKE_CURRENT_BINARY_DIR}/lib)



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
                                            cloudOps utils
                                            boxFilter
                                            ${Boost_LIBRARIES})
