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

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <utils_cv/ImageAND.hpp>
#include <utils_cv/CannyBlur.hpp>
#include <utils_cv/ImageProcessor.hpp>
#include <utils_cv/HueFilter.hpp>
#include <utils_cv/SaturationFilter.hpp>
#include <utils_pcl/CloudImageComm.hpp>

using namespace cv;

/** Class for finding a needle (macro sized, circular)
    in an organized point cloud. */
class NeedleFinder : CloudImageComm {

  ImageProcessor::Ptr cannyblur;
  ImageAND ander;
  HueFilter hFilter;
  SaturationFilter sFilter;
  pcl::visualization::PCLVisualizer viewer;

  /** Displays a point-cloud. */
  void visualize_data(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
    viewer.removePointCloud("needle points"); 
    viewer.addPointCloud<pcl::PointXYZRGB>(cloud, "needle points");
    ros::Rate r(60);
    while(!viewer.wasStopped()) {
      viewer.spinOnce(100);
      ros::spinOnce();
      r.sleep();
    }
  }

  /** Returns a pointcloud corresponding to the pixels in the
      IMG which are non-zero. Pixels which have NaN distance information
      are filtered out.
      The image is expected to be a single channel image. If not
      only the first channel is taken into consideration. */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr
  cloud_from_image(cv::Mat img, bool debug=false) {

    std::vector<cv::Mat> channels;
    cv::split(img, channels);
    cv::Mat ch0 =  channels[0];

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
      out_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    int non_zero_pixels = 0;

    for (int r = 0; r < img.rows; r+=1) {
      for (int c = 0; c < img.cols; c+=1) {
	if (ch0.at<uint8_t>(r,c) != 0) {
	  non_zero_pixels += 1;
	  pcl::PointXYZRGB pt = _cloud_ptr->at(c,r);
	  if (pcl::isFinite(pt))
	    out_cloud->points.push_back(pt);
	}
      }
    }

    if (debug) {
      ROS_INFO("Found %d non-zero pixels. Found %d finite points.",
	       non_zero_pixels, out_cloud->points.size());
      visualize_data(out_cloud);
    }

    return out_cloud;
  }




  /** This is called whenever a new point-cloud is recieved. */
  void process() {
    cv::Rect roi(100,50,420,350);
    cv::Mat img(_img_cv, roi);

    ander.update(img);

    if (ander.is_ready()) {
      cv::Mat and_img = ander.get();

      cv::Mat debug_mat;
      cv::GaussianBlur(and_img, debug_mat, cv::Size(9,9), 2, 2);
      std::vector<cv::Vec3f> circles;
      cv::HoughCircles(debug_mat, circles, CV_HOUGH_GRADIENT,
		       2, debug_mat.rows/3, 200, 100,50,80);

      cv::Mat circular_mask = cv::Mat::zeros(img.size(), CV_8UC1);
      for( size_t i = 0; i < circles.size(); i+=1 ) {
	cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
	int radius = cvRound(circles[i][2]);
	// Draw the circle outline
	cv::circle(circular_mask, center, radius, 255, 10, 8, 0);
      }

      cv::Mat hue_mask, sat_mask, color_mask;
      hFilter.filter(img, hue_mask, true);
      sFilter.filter(img, sat_mask, true);
      cv::bitwise_and(hue_mask, sat_mask, color_mask);
      
      // AND the color-space mask and the circular hough space mask
      cv::Mat needle_mask;
      cv::bitwise_and(color_mask, circular_mask, needle_mask);

      cv::imshow("Original : ROI", img);
      cv::waitKey(5);
      cv::imshow("Hue and Saturation Filtered", color_mask);
      cv::waitKey(5);
      cv::imshow("Needle Mask", needle_mask);
      cv::waitKey(5);

      cloud_from_image(needle_mask, true);
    }
  }

public:
  NeedleFinder(ros::NodeHandle * nh_ptr,
	       std::string cloud_topic="/camera/depth_registered/points",
	       uint8_t h_min=75, uint8_t h_max=90,
	       uint8_t s_min=50, uint8_t s_max=255) 
    : CloudImageComm(nh_ptr, cloud_topic),
      viewer("visualizer"),
      cannyblur(new CannyBlur),
      ander(cannyblur, 3),
      hFilter(h_min, h_max),
      sFilter(s_min, s_max){ }
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "and_image_node");
  ros::NodeHandle nh("~");

  std::string topic; 
  nh.param<std::string>("cloud_topic", topic, 
			"/camera/depth_registered/points");
  NeedleFinder n_finder(&nh, topic);

  ros::spin();
  return 0;
}
