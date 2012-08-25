add_subdirectory(utils)

add_library(FilterWriter filter_color.cpp)
target_link_libraries(FilterWriter ${PCL_LIBRARIES} ${ROS_LIBRARIES})

add_library(cloudOps cloud_ops.cpp)
target_link_libraries(cloudOps ${PCL_LIBRARIES} ${ROS_LIBRARIES}
                               ${OpenCV_LIBS} cv_bridge utils)

add_library(boxFilter geom.cpp plane_finding.cpp get_table2.cpp)
target_link_libraries(boxFilter ${PCL_LIBRARIES} ${ROS_LIBRARIES}
                               ${OpenCV_LIBS} ${BULLET_LIBS} utils)