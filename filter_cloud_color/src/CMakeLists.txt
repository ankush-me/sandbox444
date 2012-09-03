include_directories(${FILTER_COLOR_CLOUD_DIR}/lib)

add_library(FilterWriter filter_color.cpp)
target_link_libraries(FilterWriter ${PCL_LIBRARIES})

add_library(cloudOps cloud_ops.cpp)
target_link_libraries(cloudOps ${PCL_LIBRARIES} ${OpenCV_LIBS} 
                               cv_bridge utils)

message(STATUS "RANDOM" ${OpenCV_LIBS})

add_library(boxFilter geom.cpp plane_finding.cpp get_table.cpp)
target_link_libraries(boxFilter ${PCL_LIBRARIES}
                               ${OpenCV_LIBS} ${BULLET_LIBS} utils)
