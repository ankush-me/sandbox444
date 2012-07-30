add_subdirectory(utils)

add_library(FilterWriter filter_color.cpp)
target_link_libraries(FilterWriter ${PCL_LIBRARIES} ${ROS_LIBRARIES})

add_library(cloudOps cloud_ops.cpp)
target_link_libraries(cloudOps ${PCL_LIBRARIES} ${ROS_LIBRARIES}
                               ${OpenCV_LIBS} cv_bridge utils)

