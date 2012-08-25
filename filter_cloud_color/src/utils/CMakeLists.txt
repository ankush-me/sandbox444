include_directories(${LOG4CPLUS_INCLUDE_DIRS})

add_library(utils utils_vector.cpp utils_pcl.cpp config.cpp logging.cpp conversions.cpp)
target_link_libraries(utils ${OpenCV_LIBS} cv_bridge ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${LOG4CPLUS_LIBRARY})

add_executable(ColorSampler color_detector.cpp)
target_link_libraries(ColorSampler ${OpenCV_LIBS})
