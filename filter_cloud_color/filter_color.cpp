/** Name: Ankush Gupta
    Date: 20 July, 2012 */

#include "filter_color.h"

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
				std::string cloud_topic) {

  rosbag::Bag bag_in;
  bag_in.open(input_file, rosbag::bagmode::Read);
  rosbag::View view(bag_in);

  rosbag::Bag bag_out;
  bag_out.open(output_file, rosbag::bagmode::Write);


  BOOST_FOREACH(rosbag::MessageInstance const msg, view) {
    
    if(msg.getTopic() == cloud_topic
       || ("/" + msg.getTopic() == cloud_topic)) {
      
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr
	cloud_pcl(new pcl::PointCloud<pcl::PointXYZRGB>);

      sensor_msgs::PointCloud2ConstPtr cloud_ros
	= msg.instantiate<sensor_msgs::PointCloud2>();

      if (cloud_ros != NULL) {
	pcl::fromROSMsg(*cloud_ros, *cloud_pcl);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr
	  cloud_pcl_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	(*filter)(cloud_pcl, cloud_pcl_filtered);
	
	sensor_msgs::PointCloud2 cloud_ros_filtered;
	pcl::toROSMsg(*cloud_pcl_filtered, cloud_ros_filtered);
	
	bag_out.write(msg.getTopic(), msg.getTime(),
		      cloud_ros_filtered, msg.getConnectionHeader());
      }
    } else {
      	bag_out.write(msg.getTopic(), msg.getTime(),
		      msg, msg.getConnectionHeader());
    }
  }
  bag_in.close();
  bag_out.close();
}
