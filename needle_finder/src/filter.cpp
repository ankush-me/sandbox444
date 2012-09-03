/** Author: Ankush Gupta
    Date  : 31st, August 2012. */

#include <ros/ros.h>
#include <utils_cv/hueFilter.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>

void imageCB(const sensor_msgs::Image::ConstPtr img) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(img);
  } catch (cv_bridge::Exception& ex1) {
    ROS_ERROR("cv_bridge exception: %s", ex1.what());
    return;
  }

  cv::Mat in_img  = cv_ptr->image.clone();
  cv::imshow("orig", in_img);
  cv::waitKey(5);
  
  //cv::Mat cvt_img;
  ///in_img.convertTo(cvt_img, -1, 1.01, 0);
  //cv::imshow("cvt", cvt_img);
  //cv::waitKey(5);


  hueFilter hFilter(80,85);
  cv::Mat dest;
  hFilter.filter(in_img, dest);

  cv::imshow("in", dest);
  cv::waitKey(5);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "hue_filter_node");
  ros::NodeHandle nh;
  ros::Subscriber sub;

  cv::namedWindow("in");
  cv::waitKey(100);

  std::string img_topic = "/wide_stereo/left/image_rect_color";
  sub = nh.subscribe(img_topic, 1, imageCB);
  ros::spin();

  return 0;
 }
