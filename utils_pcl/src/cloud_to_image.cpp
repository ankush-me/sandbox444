/** This node publishes an image extracted from a
    pointcloud on the given topic.

    Author: Ankush Gupta */

#include <ros/ros.h>
#include <utils_pcl/CloudToImage.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "cloud_to_point");
  ros::NodeHandle nh("~");

  std::string cloud_topic; 
  nh.param<std::string>("cloud_topic", cloud_topic,
			"/camera/depth_registered/points");
  ROS_INFO("Subscribing to %s for pointclouds.", cloud_topic.c_str());

  std::string image_topic;
  nh.param<std::string>("image_topic", image_topic, "/cloud_to_image");

  ROS_INFO("Publishing image to :%s", image_topic.c_str());
  
  CloudToImage cloud2img(&nh, image_topic, cloud_topic);

  ros::spin();
  return 0;
}
