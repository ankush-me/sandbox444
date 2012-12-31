# Author: Ankush Gupta
# Date  : 15th August, 2012

cmake_minimum_required(VERSION 2.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

#project(Surgical)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/CMakeModules")
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})
find_package(Qt4 REQUIRED)



include_directories(${PCL_INCLUDE_DIRS}
		    ${CMAKE_SOURCE_DIR}
		    ${CMAKE_CURRENT_BINARY_DIR})

link_directories(${PCL_LIBRARY_DIRS})


add_definitions(${QT_DEFINITIONS})
include(${QT_USE_FILE})
add_definitions("-DEIGEN_DEFAULT_TO_ROW_MAJOR")

add_subdirectory(${CMAKE_SOURCE_DIR}/utils)

set(Surgical_SOURCES src/main.cpp src/SurgicalGUI.cpp)
set(Surgical_HEADERS src/SurgicalGUI.h)
set(Surgical_FORMS src/surgical_gui.ui)

qt4_wrap_cpp(Surgical_HEADERS_MOC ${Surgical_HEADERS})
qt4_wrap_ui(Surgical_FORMS_HEADERS ${Surgical_FORMS})

rosbuild_add_executable(surgical src/mouseHandler.cpp
                                 ${Surgical_SOURCES} 
				 ${Surgical_HEADERS_MOC}
				 ${Surgical_FORMS_HEADERS} )

target_link_libraries(surgical utils
			       ${QT_LIBRARIES})

#############################################################
