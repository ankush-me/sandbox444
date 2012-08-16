# Author: Ankush Gupta
# Date  : 15th August, 2012

cmake_minimum_required(VERSION 2.6)
project(Surgical)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/CMakeModules")



# Set path for OpenCV: This is compiling with ROS's openCV. More of a hack.
set(OpenCV_DIR "/opt/ros/$ENV{ROS_DISTRO}/share/OpenCV")
find_package(OpenCV REQUIRED)

# EXTERNAL LIBRARIES
find_package(Boost REQUIRED)
find_package(PCL REQUIRED)
find_package(Qt4 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
		    ${CMAKE_SOURCE_DIR}
		    ${CMAKE_CURRENT_BINARY_DIR})

link_directories(${PCL_LIBRARY_DIRS}
                 ${CMAKE_SOURCE_DIR}
		)

add_definitions(${PCL_DEFINITIONS})
add_definitions("-DEIGEN_DEFAULT_TO_ROW_MAJOR")
add_definitions(${QT_DEFINITIONS})
include(${QT_USE_FILE})


set(Surgical_SOURCES main.cpp SurgicalGUI.cpp)
set(Surgical_HEADERS SurgicalGUI.h)
set(Surgical_FORMS surgical_gui.ui)

qt4_wrap_cpp(Surgical_HEADERS_MOC ${Surgical_HEADERS})
qt4_wrap_ui(Surgical_FORMS_HEADERS ${Surgical_FORMS})

#add_library(lst ListInteractor.cpp)
#target_link_libraries(lst ${QT_LIBRARIES})

add_executable(surgical Hole.hpp Cut.hpp mouseHandler.h
                        mouseHandler.cpp
			ListInteractor.hpp
                        ${Surgical_SOURCES} 
		        ${Surgical_HEADERS_MOC}
			${Surgical_FORMS_HEADERS})

target_link_libraries(surgical ${PCL_LIBRARIES}
                               ${OpenCV_LIBS}
			       ${QT_LIBRARIES})

#############################################################
