/** Tests fitting a 3D circle to a point-cloud.
    Author: Ankush Gupta .*/

#include <ros/ros.h>
#include <utils_pcl/cloud_utils.hpp>

#include <iostream>
#include <pcl/point_types.h>
#include <math.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_circle");
  ros::NodeHandle nh;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  // Fill in the cloud data
  cloud->width  = 15;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  // Generate the data
  for (size_t i = 0; i < cloud->points.size (); ++i) {
    cloud->points[i].x = -1.0f + float(i)*(2.0/cloud->points.size());
    cloud->points[i].y = ( sqrt(1.0 - cloud->points[i].x*cloud->points[i].x)
			   + 0.07*(rand () / (RAND_MAX + 1.0f)) );
    cloud->points[i].z = 1.0;

    //pt.r = pt.g = pt.b =0;
  }

  // Set a few outliers
  cloud->points[0].z = 2.0;
  cloud->points[3].z = -2.0;
  cloud->points[6].z = 4.0;

  ros::Duration d(0.5);
  for (int i = 0; i < 15; i +=1) {
  //int i = 10;
    Eigen::Vector3f pt(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
    std::cout<<"ref pt: \n"<<pt.transpose()<<std::endl;
    
    circle3d c3d(cloud);
    c3d.compute_circle3d(true);
    Eigen::MatrixXf frame = c3d.extend_circumference(pt,0,circle3d::CCW,true);
    std::cout<<"circum pt: \n"<<frame<<std::endl;
    d.sleep();
  }
  ros::spin();
  return 0;
}
