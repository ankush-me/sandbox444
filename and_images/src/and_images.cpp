#include <ros/ros.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <sstream>

#include <utils_cv/ImageAND.hpp>
#include <utils_cv/CannyBlur.hpp>
#include <utils_cv/ImageProcessor.hpp>

using namespace cv;


std::string window_name;
ImageProcessor::Ptr cannyblur(new CannyBlur);
ImageAND ander(cannyblur,3);


void create_display_windows(std::string window_name="window") {
  namedWindow(window_name);
  waitKey(10);
}


void imageCB(const sensor_msgs::Image::ConstPtr img) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(img);
  } catch (cv_bridge::Exception& ex1) {
    ROS_ERROR("cv_bridge exception: %s", ex1.what());
    return;
  }

  ander.update(cv_ptr->image.clone());

  if (ander.is_ready()) {
    imshow(window_name, ander.get());
    waitKey(10);
  }
}


int main(int argc, char** argv) {

  ros::init(argc, argv, "and_images");
  ros::NodeHandle nh("and_images_node");

  window_name = "ANDed";
  create_display_windows(window_name);

  std::string topic; 
  nh.param<std::string>("topic", topic, "/camera/rgb/image_rect");
  ros::Subscriber sub = nh.subscribe(topic, 1, imageCB);

  ros::spin();
  return 0;
}
