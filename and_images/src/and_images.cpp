#include <ros/ros.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <sstream>
#include <vector>

#include <utils_cv/ImageAND.hpp>
#include <utils_cv/CannyBlur.hpp>
#include <utils_cv/ImageProcessor.hpp>

using namespace cv;


ImageProcessor::Ptr cannyblur(new CannyBlur);
ImageAND ander(cannyblur,3);


void imageCB(const sensor_msgs::Image::ConstPtr img) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(img);
  } catch (cv_bridge::Exception& ex1) {
    ROS_ERROR("cv_bridge exception: %s", ex1.what());
    return;
  }
  cv::Mat cv_img = cv_ptr->image.clone();
  ander.update(cv_img);

  if (ander.is_ready()) {
    cv::Mat and_img = ander.get();

    cv::Mat debug_mat;
    cv::GaussianBlur(and_img, debug_mat, cv::Size(9,9), 2, 2);
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(debug_mat, circles, CV_HOUGH_GRADIENT,
		     2, debug_mat.rows/3, 200, 100,50,75);

    cv::Mat circular_mask = cv::Mat::zeros(cv_img.size(), cv_img.type());
    for( size_t i = 0; i < circles.size(); i++ ) {
      cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
      int radius = cvRound(circles[i][2]);
      // Draw the circle center
      cv::circle(circular_mask, center, 3, cv::Scalar(255,255,255), -1, 8, 0);
      // Draw the circle outline
      cv::circle(circular_mask, center, radius, cv::Scalar(255,255,255), 7, 8, 0);
    }
    cv::imshow("Hough Circles: ANDed", circular_mask);
    cv::waitKey(5);
  }
}


int main(int argc, char** argv) {

  ros::init(argc, argv, "and_image_node");
  ros::NodeHandle nh("and_image_node");

  std::string topic; 
  nh.param<std::string>("topic", topic, "/camera/rgb/image_rect");
  ros::Subscriber sub = nh.subscribe(topic, 1, imageCB);
  std::cout<<"Topic : "<<topic<<std::endl;

  ros::spin();
  return 0;
}
