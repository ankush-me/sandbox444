/** Class for publishing an image extracted from a point cloud on a topic.
    Author: Ankush Gupta */

#ifndef _CLOUD_TO_IMAGE_HPP_
#define _CLOUD_TO_IMAGE_HPP_

#include <ros/ros.h>
#include <utils_pcl/CloudImageComm.hpp>
#include <sensor_msgs/Image.h>

class CloudToImage : CloudImageComm {

  /** The name of the topic on which the image is to be advertised.*/
  std::string _img_out_topic;

  /** The image publisher. */
  ros::Publisher  _image_pub;

  /** This is called whenever a new point-cloud is recieved. */
  void process() {
    this->publish_image();
  }

  void publish_image() {
    _image_pub.publish(_img_ros);
  }

public:
  CloudToImage(ros::NodeHandle * nh_ptr,
	       std::string image_out_topic="/cloud_to_image/image_rect_color",
	       std::string cloud_topic="/camera/depth_registered/points") 
    : CloudImageComm(nh_ptr, cloud_topic),
      _img_out_topic(image_out_topic)
  { 
    _image_pub = _nh_ptr->advertise<sensor_msgs::Image>(_img_out_topic, 1);
  }
};

#endif
