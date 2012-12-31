#ifndef _CLOUD_IMAGE_COMM_HPP_
#define _CLOUD_IMAGE_COMM_HPP_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <cv.h>
#include <opencv2/core/core.hpp>

#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>

#include <boost/shared_ptr.hpp>

class CloudImageComm {
protected:
  /** Shared pointer to an CloudImageComm. */
  typedef boost::shared_ptr<CloudImageComm> Ptr;

  /** Node handle of the ros node, this should attach to. */
  ros::NodeHandle* _nh_ptr;

  /** Subscriber for getting new point clouds. */
  ros::Subscriber  _cloud_sub;

  /** The name of the topic on which the point_cloud is being published. */
  std::string _cloud_topic;

  /** The latest point-cloud recieved. */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud_ptr;

  /** The latest point-cloud recieved [ROS FORMAT]. */
  sensor_msgs::PointCloud2::Ptr _cloud_ptr_ros;

  /** Image used for pointcloud -> opencv::mat conversion. */
  sensor_msgs::Image _img_ros;

  /** The latest image recieved in opencv format. */
  cv::Mat _img_cv;



  /** Called when this recieves a new point-cloud. */
  void cloudCB(const sensor_msgs::PointCloud2::ConstPtr cloud_ros) {
    _cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*cloud_ros, *_cloud_ptr);
 
    // get image from the point-cloud
    cv_bridge::CvImagePtr cv_ptr;
    try {
      pcl::toROSMsg (*cloud_ros, _img_ros);
      try {
	cv_ptr = cv_bridge::toCvCopy(_img_ros);
      } catch (cv_bridge::Exception& ex1) {
	ROS_ERROR("utils_cloud.CloudImageComm : cv_bridge exception: %s", ex1.what());
	return;
      }
    } catch (std::runtime_error ex2) {
      ROS_ERROR("utils_cloud.CloudImageComm : Error in converting cloud to image message: %s", 
		ex2.what());
      return;
    }

    _img_cv  = cv_ptr->image.clone(); 
    this->process();
  }

  /** The function which is called whenever a new point-cloud is recieved. */
  virtual void process() = 0;

public:
  CloudImageComm(ros::NodeHandle * nh_ptr,
		 std::string cloud_topic="/camera/depth_registered/points") 
    : _nh_ptr(nh_ptr),
      _cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>(480,640)),
      _cloud_ptr_ros(new sensor_msgs::PointCloud2),
      _img_ros(),
      _img_cv(cv::Mat::zeros(480,640,CV_32FC3)),
      _cloud_topic(cloud_topic)
  {
    _cloud_sub = _nh_ptr->subscribe(_cloud_topic, 1,
				    &CloudImageComm::cloudCB, this);
  }
  
  /** Return the pointer to the last point-cloud recieved. */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_cloud_ptr() {return _cloud_ptr;}

  /** Return the pointer to the last point-cloud recieved [ROS FORMAT]. */
  sensor_msgs::PointCloud2::Ptr get_cloud_ptr_ros() {
    _cloud_ptr_ros.reset(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*_cloud_ptr, *_cloud_ptr_ros);  
    return _cloud_ptr_ros;
  }
  
  /** Return the image from the last cloud recieved [ROS Format]. */
  sensor_msgs::Image get_image_ros() {return _img_ros;}

  /** Return the image from the last cloud recieved [OPENCV Format]. */
  cv::Mat get_image() {return _img_cv;}
};

#endif
