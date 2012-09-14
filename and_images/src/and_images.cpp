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
HueFilter hFilter(20,40);

/** Class for publishing an image extracted from a point cloud on a topic. */
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
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    _image_pub = _nh_ptr->advertise<sensor_msgs::Image>(_img_out_topic, 1);
  }
};



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
      for( size_t i = 0; i < circles.size(); i++ ) {
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
  ros::NodeHandle nh("and_image_node");

  std::string topic; 
  nh.param<std::string>("topic", topic, "/camera/rgb/image_rect");
  NeedleFinder n_finder(&nh, topic);

  //ros::Subscriber sub = nh.subscribe(topic, 1, imageCB);

  ros::spin();
  return 0;
}
