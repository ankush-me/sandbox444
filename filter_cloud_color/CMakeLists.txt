# Author: Ankush Gupta
# Date  : 15th July, 2012

cmake_minimum_required(VERSION 2.6)
project(Filter)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/CMakeModules")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

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


# Set the paths for TF (ROS PACKAGE) and BULLET (required by tf)
# ^ Unfortunately hardcoded.
set(TF_PATH "/opt/ros/$ENV{ROS_DISTRO}/stacks/geometry/tf")
set(TF_INCLUDE_DIRS "${TF_PATH}/include")
set(TF_LIBRARY_DIRS "${TF_PATH}/lib")

set(BULLET_PATH "/opt/ros/$ENV{ROS_DISTRO}/stacks/bullet")
set(BULLET_INCLUDE_DIRS "${BULLET_PATH}/include")
#set(BULLET_LIBRARY_DIRS "${BULLET_PATH}/lib")

# Set paths for CV_BRIDGE
set(CV_BRIDGE_PATH "/opt/ros/$ENV{ROS_DISTRO}/stacks/vision_opencv/cv_bridge")
set(CV_BRIDGE_INCLUDE_DIRS "${CV_BRIDGE_PATH}/include")
set(CV_BRIDGE_LIBRARY_DIRS "${CV_BRIDGE_PATH}/lib")

# Set path for OpenCV: This is compiling with ROS's openCV. More of a hack.
set(OpenCV_DIR "/opt/ros/$ENV{ROS_DISTRO}/share/OpenCV")
find_package(OpenCV REQUIRED)

# EXTERNAL LIBRARIES
find_package(Boost REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
 # find the ros stuff: this sets $ROS_INCLUDE_DIRS and $ROS_LIBRARIES
 # see '/opt/ros/fuerte/share/ros/cmake/ros-config.cmake' for details.
find_package(ROS
  COMPONENTS roscpp roscpp_serialization rostime rosbag
  REQUIRED)


include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
                    ${ROS_INCLUDE_DIRS}
                    ${TF_INCLUDE_DIRS} ${BULLET_INCLUDE_DIRS}
		    ${CMAKE_SOURCE_DIR}/src
		    ${CV_BRIDGE_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS} ${TF_LIBRARY_DIRS}
                 ${CMAKE_SOURCE_DIR}/src
		 ${CV_BRIDGE_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})
add_definitions("-DEIGEN_DEFAULT_TO_ROW_MAJOR")


#  Get the TF dynamic link library path
set(tf_libs "libtf.so")
find_libraries(tf_libs TF_LIBRARY_DIRS TF_LIBRARIES)

add_subdirectory(src)

add_executable(test test.cpp)
target_link_libraries(test ${PCL_LIBRARIES}
                           ${OpenCV_LIBS} cv_bridge
			   FilterWriter cloudOps utils)

#############################################################
 