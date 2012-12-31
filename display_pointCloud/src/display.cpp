#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <surgical_msgs/InitInfo.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>


class visualizer {
  /** A simple class to help visualize the PCL point clouds.*/
public: 
    /** A PCL viewer instance.*/
  pcl::visualization::CloudViewer viewer;

  /** Initialize the viewer with the name as WINDOWNAME.*/
  visualizer(std::string *windowName =  new std::string("visualizer")) 
  : viewer(*windowName) { }

  /** Display the point_cloud CLOUD till the user exits.
      Holds the thread busy.*/
  void display(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) {
    viewer.showCloud(cloud);
  }

  /** Returns TRUE iff the gui of the viewer was stopped.*/
  bool wasStopped() {return viewer.wasStopped();}
};
visualizer viz;


void cloudCB(const sensor_msgs::PointCloud2::ConstPtr& cloud_data) {
  //ROS_INFO("DisplayCloud: Data recieved.");
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pcl (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*cloud_data, *cloud_pcl);
  viz.display(cloud_pcl);
}

int main(int argc, char** argv) {
  std::string cloud_topic = "/camera/depth_registered/points";
  ros::init(argc, argv, "display_cloud");
  ros::NodeHandle nh;

  ros::Subscriber  sub;
  sub = nh.subscribe(cloud_topic, 1, cloudCB);
  ros::spin();
  return 0;
}
