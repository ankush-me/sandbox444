#include <utils_pcl/cloud_utils.hpp>

using namespace std;


pcl::PointCloud<pcl::PointXYZRGB>::Ptr eigen_to_pcl(Eigen::MatrixXf &points) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  cloud->width  = points.rows();
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);
  for (size_t i = 0; i < cloud->points.size (); ++i) {
    cloud->points[i].x = points(i,0);
    cloud->points[i].y = points(i,1);
    cloud->points[i].z = points(i,2);
    cloud->points[i].r = cloud->points[i].g = cloud->points[i].b =0;
  }
  return cloud;
}


/** Returns an Eigen MatrixXf corresponding to the 3D points
    in the input point-cloud.
    Number of rows    = number of points.
    Number of columns = 3 : x,y,z. */
Eigen::MatrixXf pcl_to_eigen(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
  return cloud->getMatrixXfMap(3,8,0);
}

Eigen::MatrixXf
pcl_to_eigen(pcl::PointCloud<pcl::PointXYZ>::Ptr points) {
  return points->getMatrixXfMap(3,4,0);
}


/** Returns a vector of coefficients of a plane which
    best fit the input CLOUD.
    Uses pcl's RANSAC segmentation.

    The returned coefficeints are : [A,B,C,D] : Ax + By + Cz + D = 0.*/
vector<float> get_plane_coeffs(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {

  pcl::ModelCoefficients::Ptr coeffs (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
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
pcl::PointCloud<pcl::PointXYZRGB>::Ptr
project_points_plane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_cloud,
		     vector<float> plane_coeffs) {
  if (plane_coeffs.size() != 4)
    throw("utils_pcl/cloud_utils/project_points_plane: Invalid number of plane coefficients.");

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected
    (new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::ModelCoefficients::Ptr coefficients
    (new pcl::ModelCoefficients ());

  coefficients->values.resize (4);
  for(int i = 0; i < 4; i += 1)
    coefficients->values[i] = plane_coeffs[i];
  
  pcl::ProjectInliers<pcl::PointXYZRGB> proj;
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
  y.normalize();
}


/** Fits a 3d circle to the given input point CLOUD.*/
void compute_circle3d(typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		      bool verbose) {

  vector<float> plane = get_plane_coeffs(cloud);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr proj_points = project_points_plane(cloud, plane);

  Eigen::Vector3f z_plane = Eigen::Vector3f(plane[0], plane[1], plane[2]);
  z_plane.normalize();
  Eigen::Vector3f y_plane, x_plane;
  perp_basis(z_plane, x_plane, y_plane);
  Eigen::Matrix3f rotation;
  rotation.col(0) = x_plane;
  rotation.col(1) = y_plane;
  rotation.col(2) = z_plane;

  Eigen::MatrixXf plane_pts_world = pcl_to_eigen(proj_points);
  Eigen::Vector3f origin_plane    = plane_pts_world.row(0);

  // Transform the given points in their plane's frame
  // WORLD -> LOCAL PLANE
    
  Eigen::MatrixXf plane_pts(plane_pts_world);
  plane_pts.rowwise() -= origin_plane;
  plane_pts *= rotation.transpose();

  cout<<"plane pts: \n"<<plane_pts<<endl;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cloud  = eigen_to_pcl(plane_pts);
  boost::shared_ptr<pcl::SampleConsensusModelCircle2D<pcl::PointXYZRGB> >
    model(new pcl::SampleConsensusModelCircle2D<pcl::PointXYZRGB>(plane_cloud));

  pcl::RandomSampleConsensus<pcl::PointXYZRGB> sac(model, .02);
  bool result = sac.computeModel(2);
  Eigen::VectorXf xyr;
  sac.getModelCoefficients (xyr);
  cout<<"circle : "<<xyr<<endl;
 
}
