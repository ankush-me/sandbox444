#ifndef _UTILS_PCL_CLOUD_UTILS_H_
#define _UTILS_PCL_CLOUD_UTILS_H_

#include <boost/foreach.hpp>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <vector>

/** Returns a vector of coefficients of a plane which
    best fits the input CLOUD.
    Uses pcl's RANSAC segmentation.

    The returned coefficeints are : A B C D : Ax + By + Cz = D. */
std::vector<float> get_plane_coeffs(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

/** Projects the input points onto the plane defined by PLANE_COEFFS.
    PLANE_COEFFS is expected to contain 4 coefficients [A,B,C,D], which
    define the plane Ax + By + Cz = D. */
template<typename pointT> pcl::PointCloud<pointT>::Ptr
project_points_plane(pcl::PointCloud<pointT>::Ptr src_cloud,
		     std::vector<float> plane_coeffs);

#endif
