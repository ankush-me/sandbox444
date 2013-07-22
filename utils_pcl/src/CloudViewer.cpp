/** Simple class to visualize point-clouds in a new thread.
    Author: Ankush Gupta */

#include <ros/ros.h>
#include <utils_pcl/CloudViewer.hpp>

void CloudViewer::spin() {
   while (!_viewer->wasStopped()) {
     _viewer->spinOnce(100);
    //boost::this_thread::sleep (boost::posix_time::milliseconds (1));

    if(_updated) {
      std::cout<<"UPDATED"<<std::endl;
      _viewer->updatePointCloud(_cloud, "cloud");
      //_viewer->removePointCloud("cloud");
      //_viewer->addPointCloud(_cloud, "cloud");
      //_viewer->showCloud(_cloud, "cloud");
      _updated = false;
    }
  }
}

/** Initializes the point-cloud viewer. */
void CloudViewer::initialize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
  _viewer->initCameraParameters ();
  _viewer->setBackgroundColor(0, 0, 0);
  _viewer->addCoordinateSystem(1.0);

  pcl::PointXYZ sphere_center;
  sphere_center.x = 0;
  sphere_center.y = 0;
  sphere_center.z = 0;
  _viewer->addSphere(sphere_center, 1, "sphere");

  spin();
  //_thread = boost::thread(&CloudViewer::spin, this);
}


/** Call this to update the point-cloud displayed. */
void CloudViewer::view(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
  if (cloud->points.size() > 0) { 
    if (!_is_init) {
      initialize(cloud);
      _is_init = true;
    }

    _cloud   = cloud;
    _updated = true;
  }
}

CloudViewer::CloudViewer() :
  _viewer(new pcl::visualization::PCLVisualizer("Needle Points")),
  _is_init(false), _updated(false), _cloud() {
  int argc = 0;
  char** argv;
  ros::init(argc, argv, "Cloud Viewer",  ros::init_options::AnonymousName);
  ros::NodeHandle nh;
}

CloudViewer::~CloudViewer() {
  _thread.join();
}
