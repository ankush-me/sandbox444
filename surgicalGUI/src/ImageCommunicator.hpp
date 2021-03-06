/** Author: Ankush Gupta
    Date  : 15th August, 2012. */

#ifndef _IMAGE_COMMUNICATOR_
#define _IMAGE_COMMUNICATOR_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <cv.h>
#include <list>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/point_types.h>

#include "Hole.hpp"
#include "Cut.hpp"
#include "mouseHandler.h"
#include "utils/utils_pcl.h"

#include <iostream>

class SurgicalGUI;

class ImageCommunicator {

private:
  /** Node handle of the ROS node, THIS object should attach to. */
  ros::NodeHandle* _nh_ptr;

  /** Subscriber for getting new point clouds. */
  ros::Subscriber  _cloud_sub;

  /** The latest point-cloud received. */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud_ptr;

  /** The latest point-cloud received [ROS FORMAT]. */
  sensor_msgs::PointCloud2::Ptr _cloud_ptr_ros;

  /** Image used for point-cloud->opencv::mat conversion. */
  sensor_msgs::Image _sensor_image;

  /** The main frame is updated, every time when a new frame is received. */
  cv::Mat _main_frame;

  /** This is a temporary frame on which HOLES and CUTS are drawn. */
  cv::Mat _working_frame;

  /** The name of the window to display the image in. */
  std::string _window_name;

  /** The name of the topic on which the point_cloud is being published. */
  std::string _cloud_topic;
  
  /** The GUI which displays the buttons for user interaction. */
  SurgicalGUI * _gui;

  /** True IFF user wants to fix a frame. */
  bool _is_fixed;

  /** Called when this receives a new point-cloud. */
  void cloudCB(const sensor_msgs::PointCloud2::ConstPtr cloud_ros) {
    if (!_is_fixed) {
      _cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::fromROSMsg(*cloud_ros, *_cloud_ptr);
 
      // get image from the point-cloud
      cv_bridge::CvImagePtr cv_ptr;
      try {
	pcl::toROSMsg (*cloud_ros, _sensor_image);
	try {
	  cv_ptr = cv_bridge::toCvCopy(_sensor_image);
	} catch (cv_bridge::Exception& ex1) {
	  ROS_ERROR("Surgic@lGUI.ImageCommunicator:cv_bridge exception: %s", ex1.what());
	  return;
	}
      } catch (std::runtime_error ex2) {
	ROS_ERROR("Surgic@lGUI.ImageCommunicator:Error in converting cloud to image message: %s", 
		  ex2.what());
	return;
      }

      cv::Mat temp_frame;
      temp_frame   = cv_ptr->image.clone();
      cv::resize(temp_frame, _main_frame, cv::Size(), ZOOM_FACTOR, ZOOM_FACTOR, CV_INTER_LINEAR);
      _working_frame = _main_frame.clone();
      cv::imshow(_window_name, _working_frame);
    }
  }

public:
  ImageCommunicator(ros::NodeHandle * nh_ptr,
		    std::string cloud_topic="/camera/depth_registered/points",
		    std::string window_name="SurgiC@l") 
  : _gui(NULL),
    _nh_ptr(nh_ptr),
    _cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>(480,640)),
    _cloud_ptr_ros(new sensor_msgs::PointCloud2),
    _sensor_image(),
    _main_frame(cv::Mat::zeros(ZOOM_FACTOR*480, ZOOM_FACTOR*640, CV_32FC3)),
    _working_frame(),
    _is_fixed(false),
    _window_name(window_name),
    _cloud_topic(cloud_topic)  {
    _cloud_sub = _nh_ptr->subscribe(_cloud_topic, 1,
				    &ImageCommunicator::cloudCB, this);
    cv::namedWindow(_window_name);
    cv::imshow(_window_name, _main_frame);
  }
  
  void set_gui(SurgicalGUI * gui) {
    _gui = gui;
    cv::setMouseCallback(_window_name, mouseHandler, (void *) _gui);
  }

  SurgicalGUI* get_gui() {
    return _gui;
  }

  /** Return the pointer to the last point-cloud received. */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_cloud_ptr() {return _cloud_ptr;}

  /** Return the pointer to the last point-cloud received [ROS FORMAT]. */
  sensor_msgs::PointCloud2::Ptr get_cloud_ptr_ros() {
    _cloud_ptr_ros.reset(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*_cloud_ptr, *_cloud_ptr_ros);  
    return _cloud_ptr_ros;
  }

  /** Arrests the last frame as the permanent one. */
  void fix_frame() { _is_fixed = true; }

  /** Refreshes the display and paints the HOLES and CUTS. */
  void repaint(std::list<Hole::Ptr> &holes, std::list<Cut::Ptr> &cuts) {
    _working_frame = _main_frame.clone();

    std::list<Hole::Ptr>::iterator holes_iter;
    for (holes_iter = holes.begin(); holes_iter != holes.end(); holes_iter++)
      (*holes_iter)->paint(_working_frame, ZOOM_FACTOR);

    std::list<Cut::Ptr>::iterator cuts_iter;
    for (cuts_iter = cuts.begin(); cuts_iter != cuts.end(); cuts_iter++)
      (*cuts_iter)->paint(_working_frame, ZOOM_FACTOR);

    cv::imshow(_window_name, _working_frame);
  }

};

#endif
