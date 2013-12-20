#ifndef  _COMPUTE_ALIGNMENT
#define  _COMPUTE_ALIGNMENT

#include <iostream>
#include <pcl/registration/icp.h>

#include <stdio.h>
#include <stdlib.h>

#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>


Eigen::Matrix4f compute_alignment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr target, pcl::PointCloud<pcl::PointXYZRGB>::Ptr input);
#endif
