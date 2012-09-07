#include <utils_pcl/cloud_utils.hpp>

using namespace std;

/** Returns a vector of coefficients of a plane which
    best fits the input CLOUD.
    Uses pcl's RANSAC segmentation.

    The returned coefficeints are : [A,B,C,D] : Ax + By + Cz = D. */
template<typename pointT>
vector<float> get_plane_coeffs(typename pcl::PointCloud<pointT>::Ptr cloud) {

  pcl::ModelCoefficients::Ptr coeffs (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

  typename pcl::SACSegmentation<pointT> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(100);
  seg.setDistanceThreshold(.0025);
  seg.setInputCloud(cloud);
  seg.segment(*inliers,*coeffs);

  return coeffs->values;
}


/** Projects the input points onto the plane defined by PLANE_COEFFS.
    PLANE_COEFFS is expected to contain 4 coefficients [A,B,C,D], which
    define the plane Ax + By + Cz = D. */
template<typename pointT> typename pcl::PointCloud<pointT>::Ptr
project_points_plane(typename pcl::PointCloud<pointT>::Ptr src_cloud,
		     vector<float> plane_coeffs) {
  if (plane_coeffs.size() != 4)
    throw("utils_pcl/cloud_utils/project_points_plane: Invalid number of plane coefficients.");

  typename pcl::PointCloud<pointT>::Ptr cloud_projected
    (new typename pcl::PointCloud<pointT>);
  typename pcl::ModelCoefficients::Ptr coefficients
    (new pcl::ModelCoefficients ());
  coefficients->values.resize (4);
  for(int i = 0; i < 4; i += 1)
    coefficients->values[i] = plane_coeffs[i];

  typename pcl::ProjectInliers<pointT> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (src_cloud);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_projected);

  return cloud_projected;
}
