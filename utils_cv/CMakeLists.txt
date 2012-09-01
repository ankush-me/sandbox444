# Author: Ankush Gupta
# Date  : 31th August, 2012

cmake_minimum_required(VERSION 2.6)

# Set path for OpenCV: This is compiling with ROS's openCV. More of a hack.
set(OpenCV_DIR "/opt/ros/$ENV{ROS_DISTRO}/share/OpenCV")
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS}
                    ${CMAKE_SOURCE_DIR}/include)

add_subdirectory(src)
