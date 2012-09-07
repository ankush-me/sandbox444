#include <utils_pcl/cloud_utils.hpp>

using namespace std;

/** Returns an Eigen MatrixXf corresponding to the 3D points
		in the input point-cloud.*/
template<typename pointT>
vector<Eigen::Vector3d> pcl_to_eigen(typename pcl::PointCloud<pointT>::Ptr points) {


}



/** Returns a vector of coefficients of a plane which
    best fit the input CLOUD.
    Uses pcl's RANSAC segmentation.

    The returned coefficeints are : [A,B,C,D] : Ax + By + Cz + D = 0.*/
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
    define the plane Ax + By + Cz + D = 0. */
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

/** Finds three mutually perpendicular vectors
		given the first one, viz. V1. */
void perp_basis(const Eigen::Vector3f& z,
							 Eigen::Vector3f& x, Eigen::Vector3f& y) {
	x = Eigen::Vector3f(-z[1], z[0], 0);
	if (x.norm() == 0)
		x = Eigen::Vector3f(1, 0, 0);
	else
		x.normalize();
	y = z.cross(x);
}

/** Fits a 3d circle to the given input point CLOUD.*/
template<typename pointT>
void compute_circle3d(typename pcl::PointCloud<pointT>::Ptr cloud,
											bool verbose) {
  _normal = get_plane_normal(verbose);

  _x_axis = (_pt2-_pt1).normalized();
  _z_axis = _normal.normalized();
  _y_axis = (_z_axis.cross(_x_axis)).normalized();
  
  _rotation << _x_axis(0), _x_axis(1), _x_axis(2),
    _y_axis(0), _y_axis(1), _y_axis(2),
    _z_axis(0), _z_axis(1), _z_axis(2);
  _translation = -_pt1;
  
  // Transform the given 3 points in their plane's frame
  // WORLD -> LOCAL PLANE
  Eigen::Vector3d pt1_trans = _rotation*(_pt1 + _translation);
  Eigen::Vector3d pt2_trans = _rotation*(_pt2 + _translation);
  Eigen::Vector3d pt3_trans = _rotation*(_pt3 + _translation);

  // Reduce the dimensionality (z-component is always 0: by design)
  Eigen::Vector2d pt1_c(pt1_trans(0), pt1_trans(1));
  Eigen::Vector2d pt2_c(pt2_trans(0), pt2_trans(1));
  Eigen::Vector2d pt3_c(pt3_trans(0), pt3_trans(1));

  // Fit a circle to the points (in a plane).
  Eigen::Vector2d center;
  get_circle(pt1_c, pt2_c, pt3_c,center,_radius,verbose);

  // Increase the dimensionality
  Eigen::Vector3d center3(center(0), center(1), 0);

  // Do the inverse transformation: LOCAL PLANE -> WORLD
  _center = (_rotation.transpose()*center3) - _translation;
}
