#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

#include "segment_object.h"
#include "alignment.cpp"
#include "CloudViewer.hpp"
#include <sstream>

#include<stdio.h>
#include<stdlib.h>

int main(int argc, char** argv) {

  // number of point-clouds to align:
  int N = 2;

  CloudViewer cv;
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> point_clouds;

  for (int i=0; i < N; i+=1){
    std::stringstream fname;
    fname << "cdata/cloud_" << i << ".pcd";
    std::cout << "Reading " << fname.str() << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile<pcl::PointXYZRGB> (fname.str(), *cloud);
    point_clouds.push_back(cloud);

    /** Code to segment the point-clouds and save them to pcd files. */
    //segment_object(cloud);
    //std::stringstream savename;
    //savename << "linked/cloud_segmented_" << i << ".pcd";
    //std::cout << "saving: " << savename.str() << std::endl;
    //pcl::io::savePCDFile(savename.str(), *cloud);

  }

  Eigen::Matrix4f total_tf = Eigen::Matrix4f::Identity();
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> transformed_point_clouds;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr combined_pc (new pcl::PointCloud<pcl::PointXYZRGB>);

  transformed_point_clouds.push_back(point_clouds[0]);
  *combined_pc += *point_clouds[0];

  for (int i=1; i < point_clouds.size(); i++) {
    std::cout << "Aligning " << i+1 << "/" << point_clouds.size() << std::endl;

    Eigen::Matrix4f t = align(point_clouds[i], point_clouds[i-1], 1, 1);
    std::cout << t << std::endl;

    Eigen::Matrix4f t_comp;
    t_comp   = total_tf * t;
    total_tf = t_comp;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_pc (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::transformPointCloud(*point_clouds[i], *transformed_pc, total_tf);
    transformed_point_clouds.push_back(transformed_pc);
    *combined_pc += *transformed_pc;
    //*combined_pc += *point_clouds[i];
  }
  std::cout << "DONE" << std::endl;

  //int k = 0;
  while (true) {
    cv.view2(combined_pc);
  }
  return 0;
}
