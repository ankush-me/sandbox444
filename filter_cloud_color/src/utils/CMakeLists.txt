add_library(utils utils_vector.cpp utils_pcl.cpp)
target_link_libraries(utils ${OpenCV_LIBS} cv_bridge ${PCL_LIBRARIES})

add_executable(ColorSampler color_detector.cpp)
target_link_libraries(ColorSampler ${OpenCV_LIBS})
