/** Name: Ankush Gupta
    Date: 20 July, 2012 */

#ifndef  _FILTER_POINTCLOUD_BAGFILE
#define  _FILTER_POINTCLOUD_BAGFILE

#include <rosbag/view.h>
#include <rosbag/bag.h>
#include <rosbag/query.h>

#include <boost/foreach.hpp>

#include <pcl/io/io.h>
#include <pcl/point_types.h>

#include<stdio.h>
#include<stdlib.h>


/** Saves a bagfile at OUTPUT_FILE, with the point-cloud
    on the topic CLOUD_TOPIC, filtered by the function FILTER,
    given, the input bagfile at INPUT_FILE.
    
    Topics other than cloud_topic are written to the output 
    bagfile unchanged.
 */
void filter_point_cloud_bagfile(void(*filter)(pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
					      pcl::PointCloud<pcl::PointXYZRGB>::Ptr),
				std::string input_file,
				std::string output_file,
				std::string cloud_topic);

#endif
