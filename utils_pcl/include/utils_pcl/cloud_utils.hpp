#ifndef _UTILS_PCL_CLOUD_UTILS_H_
#define _UTILS_PCL_CLOUD_UTILS_H_

#include <boost/foreach.hpp>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/ransac.h>

#include <Eigen/Dense>

#include <vector>
#include <iostream>
#include <math.h>



/** Returns an Eigen MatrixXf corresponding to the 3D points
    in the input point-cloud.
    Number of rows    = number of points.
    Number of columns = 3 : x,y,z. */
Eigen::MatrixXf
pcl_to_eigen(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points);

Eigen::MatrixXf
pcl_to_eigen(pcl::PointCloud<pcl::PointXYZ>::Ptr points);


/** Returns a vector of coefficients of a plane which
    best fit the input CLOUD.
    Uses pcl's RANSAC segmentation.

    The returned coefficeints are : [A,B,C,D] : Ax + By + Cz + D = 0.*/
std::vector<float> get_plane_coeffs(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

/** Projects the input points onto the plane defined by PLANE_COEFFS.
    PLANE_COEFFS is expected to contain 4 coefficients [A,B,C,D], which
    define the plane Ax + By + Cz + D = 0. */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr
project_points_plane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud,
		     std::vector<float> plane_coeffs);

/** Finds three mutually perpendicular vectors
    given the first one, viz. V1. */
void perp_basis(const Eigen::Vector3f& z,
		Eigen::Vector3f& x, Eigen::Vector3f& y);

/** Fits a 3d circle to the given input point CLOUD.*/
void compute_circle3d(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		      bool verbose);

#endif
