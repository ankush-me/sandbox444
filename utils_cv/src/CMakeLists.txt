# Author: Ankush Gupta
# Date  : 31th August, 2012

add_library(utils_cv hueFilter.cpp)
target_link_libraries(utils_cv ${OpenCV_LIBS})