#include <ros/topic.h>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include "filter_wrapper.h"

/*
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <pcl/common/transforms.h>

#include <geometry_msgs/Transform.h>
#include <sensor_msgs/PointCloud2.h>
#include <bulletsim_msgs/Initialization.h>

#include "utils_tracking.h"
#include "config_tracking.h"
#include "utils/conversions.h"
*/

static ColorCloudPtr cloud_pcl (new ColorCloud);
bool message_pending = false;

/*
  Callback to store last message.
*/
void callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  fromROSMsg(*msg, *cloud_pcl);
  message_pending = true;
}

/*
  Append filters to your cascader
*/
void createFilter (filter_cascader &cascader) {
  return;
}


int main (int argc, char* argv[]) {
  ros::init(argc, argv, "real_time_cloud_filter");

  string camNS = *(argv+1);

  ros::NodeHandle nh;
  ros::Subscriber pc_sub = 
    nh.subscribe(camNS + "depth_registered/points", 1, &callback);
  ros::Publisher pc_pub = 
    nh.advertise<sensor_msgs::PointCloud2>
    (camNS + "depth_registered/filtered_points", 

  filter_cascader cascader;
  createFilter(cascader);

  while (!pending) {
    ros::spinOnce();
    sleep(.001);
    if (!ros::ok()) 
      throw std::runtime_error("caught signal while waiting for first message");
  }

  while (ros::ok()) {        
    ColorCloudPtr cloud_pcl_filtered (new ColorCloud);
    cascader.filter(cloud_pcl, cloud_pcl_filtered);

    sensor_msgs::PointCloud2 cloud_ros_filtered;
    pcl::toROSMsg(*cloud_pcl_filtered, cloud_ros_filtered);
    
    pc_pub.publish(cloud_ros_filtered);

    pending = false;
    while (ros::ok() && !pending) {
      sleep(.01);
      ros::spinOnce();
    }
  }
}
