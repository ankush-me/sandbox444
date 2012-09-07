#include <ros/ros.h>
#include <utils_pcl/cloud_utils.hpp>

#include <iostream>
#include <pcl/point_types.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_circle");

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  // Fill in the cloud data
  cloud->width  = 15;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  // Generate the data
  for (size_t i = 0; i < cloud->points.size (); ++i) {
    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1.0;
    cloud->points[i].r = cloud->points[i].g = cloud->points[i].b =0;
  }

  // Set a few outliers
  cloud->points[0].z = 2.0;
  cloud->points[3].z = -2.0;
  cloud->points[6].z = 4.0;

  compute_circle3d(cloud, false);
  ros::spin();
  return 0;
}
