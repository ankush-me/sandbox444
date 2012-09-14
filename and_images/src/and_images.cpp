#include <ros/ros.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <sstream>
#include <vector>

#include <utils_cv/ImageAND.hpp>
#include <utils_cv/CannyBlur.hpp>
#include <utils_cv/ImageProcessor.hpp>
#include <utils_cv/HueFilter.hpp>
#include <utils_pcl/CloudImageComm.hpp>

using namespace cv;

ImageProcessor::Ptr cannyblur(new CannyBlur);
ImageAND ander(cannyblur,3);
HueFilter hFilter(75, 85);

/** Class for finding a needle (macro sized, circular) in an organized point cloud. */
class NeedleFinder : CloudImageComm {

  /** This is called whenever a new point-cloud is recieved. */
  void process() {
    ander.update(_img_cv);
    if (ander.is_ready()) {
      cv::Mat and_img = ander.get();

      cv::Mat debug_mat;
      cv::GaussianBlur(and_img, debug_mat, cv::Size(9,9), 2, 2);
      std::vector<cv::Vec3f> circles;
      cv::HoughCircles(debug_mat, circles, CV_HOUGH_GRADIENT,
		       2, debug_mat.rows/3, 200, 100,50,75);

      cv::Mat circular_mask = cv::Mat::zeros(_img_cv.size(), _img_cv.type());
      for( size_t i = 0; i < circles.size(); i+=1 ) {
	cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
	int radius = cvRound(circles[i][2]);
	// Draw the circle outline
	cv::circle(circular_mask, center, radius, 255, 7, 8, 0);
      }

      cv::Mat hue_mask;
      hFilter.filter(_img_cv, hue_mask, true);

      cv::imshow("Original", _img_cv);
      cv::waitKey(5);
      cv::imshow("Hue Filtered", hue_mask);
      cv::waitKey(5);
      cv::imshow("Hough Circles: ANDed", circular_mask);
      cv::waitKey(5);
    }
  }

public:
  NeedleFinder(ros::NodeHandle * nh_ptr,
	       std::string cloud_topic="/camera/depth_registered/points") 
    : CloudImageComm(nh_ptr, cloud_topic) { }
};


int main(int argc, char** argv) {

  ros::init(argc, argv, "and_image_node");
  ros::NodeHandle nh("~");

  std::string topic; 
  nh.param<std::string>("cloud_topic", topic, "/camera/depth_registered/points");
  NeedleFinder n_finder(&nh, topic);

  ros::spin();
  return 0;
}
