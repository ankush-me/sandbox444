#include <utils_pcl/cloud_utils.hpp>

using namespace std;

#define PI 3.14159265

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


/** Finds a 3D circle which fits ALL the points in in_cloud. */
circle3d::circle3d(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud)
  : _cloud(in_cloud),
    _normal(),
    _a(0),_b(0),_c(0),_d(0),
    _rotation(3,3),
    _translation(),
    _center(),
    _viewer(new pcl::visualization::PCLVisualizer ("Visualizer")),
    _radius(0)
{
  _viewer->setBackgroundColor (0, 0, 0);
  _viewer->addPointCloud<pcl::PointXYZRGB>(_cloud, "input cloud");
  _viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "input cloud");
  _viewer->addCoordinateSystem (1.0);
  _viewer->initCameraParameters ();
}

/** Saves an orthogonal matrix corresponding to a basis defined
    on the "best fitting" plane to the given point-cloud in the matrix BASIS [input].
    The z-axis (col(3)) points in the direction of the normal.
    col(1) is the x-axis, and col(2) is the y-axis. 

    Returns the projections of the points on the plane found. */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr circle3d::get_plane_basis(Eigen::Matrix3f &basis,
								 Eigen::Vector4f &coeffs,
								 bool verbose) {
  vector<float> plane = get_plane_coeffs(_cloud);
  coeffs = Eigen::Vector4f(plane[0], plane[1], plane[2], plane[3]);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr proj_points = project_points_plane(_cloud, plane);

  Eigen::Vector3f z_plane = Eigen::Vector3f(plane[0], plane[1], plane[2]);
  z_plane.normalize();
  Eigen::Vector3f y_plane, x_plane;
  perp_basis(z_plane, x_plane, y_plane);
  Eigen::Matrix3f rotation;
  rotation.col(0) = x_plane;
  rotation.col(1) = y_plane;
  rotation.col(2) = z_plane;
  basis = rotation;

  return proj_points;
}


/** Fits a circle to the points in the given 3D  pointcloud
    and saves them in the CENTER and RADIUS.
    It assumes that the points lie on the SAME XY plane,
    i.e. their Z-component is the same.  */
void circle3d::get_circle(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points,
			  Eigen::Vector3f &center,
			  float &radius, bool verbose, float ransac_thresh) {

  boost::shared_ptr<pcl::SampleConsensusModelCircle2D<pcl::PointXYZRGB> >
    model(new pcl::SampleConsensusModelCircle2D<pcl::PointXYZRGB>(points));

  pcl::RandomSampleConsensus<pcl::PointXYZRGB> sac(model, ransac_thresh);
  sac.setMaxIterations(100);

  bool result = sac.computeModel();
  Eigen::VectorXf xyr;
  sac.getModelCoefficients (xyr);
  center(0) = xyr(0);
  center(1) = xyr(1);
  center(2) = points->points[0].z;
  radius    = xyr(2);
}

/** The main function : should be called before any other.
    Calculates a 3D circle which fits the threee given points.*/
void circle3d::compute_circle3d(bool verbose) {

  Eigen::Vector4f coeffs;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr proj_points = 
    get_plane_basis(_rotation, coeffs, verbose);
  _normal = _rotation.col(2);
  _a = coeffs(0);
  _b = coeffs(1);
  _c = coeffs(2);
  _d = coeffs(3);

  Eigen::MatrixXf plane_pts_world = pcl_to_eigen(proj_points);
  Eigen::Vector3f _translation    = plane_pts_world.row(0);

  // Transform the given points in their plane's frame
  // WORLD -> LOCAL PLANE  
  Eigen::MatrixXf plane_pts(plane_pts_world);
  plane_pts.rowwise() -= _translation;
  plane_pts *= _rotation.transpose();

  // Get the best fit circle
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cloud  = eigen_to_pcl(plane_pts);
  get_circle(plane_cloud, _center, _radius, verbose);
  _center = _rotation*_center;
  _center += _translation;

  if (verbose) {
    std::cout<<" 3D circle fitting:\n\t center : "<<_center.transpose()
	     <<"\n\t radius : "<<_radius<<std::endl;
  }
}



/** Returns whether PT2 is clockwise or counter-clockwise
    with respect to pt1 in the plane defined by the three points. */
circle3d::orientation circle3d::get_orientation(Eigen::Vector3f &pt1,
				      Eigen::Vector3f &pt2) {
  return _normal.dot((pt1 - _center).cross(pt2 - _center)) >= 0? CCW : CW;
}

/** Returns a point on this circle closest to the given PT. */
Eigen::Vector3f circle3d::snap_to_circle(Eigen::Vector3f pt) {
  Eigen::Vector3f center_to_pt = pt - _center;
  _normal.normalize();
  double comp = _normal.dot(center_to_pt);
  Eigen::Vector3f project_pt = center_to_pt - comp*_normal;
  return _center + _radius*((project_pt).normalized());
}


/** Displays:
    1. The input points.
    2. The best fitting 3D circle.
    3. The given coordinate frame and the coordinate frame at 0,0,0
    4. The best fitting plane. */
void circle3d::visualize_data(Eigen::Matrix3f &rotation, Eigen::Vector3f translation) {
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer
    (new pcl::visualization::PCLVisualizer ("Visualizer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZRGB> (_cloud, "input cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "input cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  

  pcl::PointXYZ sphere_center;
  sphere_center.x = _center(0);
  sphere_center.y = _center(1);
  sphere_center.z = _center(2);
  viewer->addSphere (sphere_center, _radius, "sphere");

  pcl::ModelCoefficients coeffs;
  coeffs.values.push_back (_a);
  coeffs.values.push_back (_b);
  coeffs.values.push_back (_c);
  coeffs.values.push_back (_d);
  viewer->addPlane (coeffs, "plane");

  pcl::PointXYZ x;
  x.x = translation.x() + rotation(0,0);
  x.y = translation.y() + rotation(1,0);
  x.z = translation.z() + rotation(2,0);
  
  pcl::PointXYZ y;
  y.x = translation.x() +  rotation(0,1);
  y.y = translation.y() + rotation(1,1);
  y.z = translation.z() + rotation(2,1);

  pcl::PointXYZ z;
  z.x = translation.x() + rotation(0,2);
  z.y = translation.y() + rotation(1,2);
  z.z = translation.z() + rotation(2,2);

  pcl::PointXYZ center;
  center.x = translation.x();
  center.y = translation.y();
  center.z = translation.z();

  viewer->addLine(center,x,"x-axis");
  viewer->addLine(center,y,"y-axis");
  viewer->addLine(center,z,"z-axis");

  while (!viewer->wasStopped ()) {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}

/** Adds a frame to the visualizer. */
void circle3d::add_frame(Eigen::MatrixXf & frame,
			      std::string frame_name) {
  
  Eigen::Vector3f translation(frame(0,3),frame(1,3),frame(2,3));
  Eigen::MatrixXf rotation = frame.block(0,0,3,3);

  pcl::PointXYZ x;
  x.x = translation.x() + rotation(0,0);
  x.y = translation.y() + rotation(1,0);
  x.z = translation.z() + rotation(2,0);
  
  pcl::PointXYZ y;
  y.x = translation.x() +  rotation(0,1);
  y.y = translation.y() + rotation(1,1);
  y.z = translation.z() + rotation(2,1);

  pcl::PointXYZ z;
  z.x = translation.x() + rotation(0,2);
  z.y = translation.y() + rotation(1,2);
  z.z = translation.z() + rotation(2,2);

  pcl::PointXYZ center;
  center.x = translation.x();
  center.y = translation.y();
  center.z = translation.z();

  stringstream ss1,ss2,ss3;
  ss1<<frame_name<<"_x";
  ss2<<frame_name<<"_y";
  ss3<<frame_name<<"_z";

  std::string x_axis = ss1.str();
  std::string y_axis = ss2.str();
  std::string z_axis = ss3.str();

  _viewer->addLine(center,x,x_axis);
  _viewer->addLine(center,y,y_axis);
  _viewer->addLine(center,z,z_axis);
}


void circle3d::spin_viewer() {
  _viewer->spinOnce(100);
}

/** Returns a frame at the PT on the circle.*/
Eigen::MatrixXf circle3d::get_frame(Eigen::Vector3f pt,
				    bool visualize) {
  Eigen::Vector3f reference_pt = snap_to_circle(pt);
  std::cout<<"Input pt: "<<pt.transpose()
	   <<"\n Pt being used: "<<reference_pt.transpose()<<std::endl;

  float pdist= fabs((_center - reference_pt).norm() - _radius);
  std::cout<<"\tPts distance from circle: "<<pdist<<std::endl;
  if (pdist > 0.005) {
    std::cout<<"Error: extend_circumference: Reference point, not on the circle."
	     <<"\n\t Distance : "<<pdist
	     <<std::endl;
    throw;
  }

  Eigen::Vector3f center_to_ref   = (reference_pt - _center).normalized();
  Eigen::Vector3f tangent = (center_to_ref.cross(_normal)).normalized();

  Eigen::Matrix3f world_to_target(3,3);
  world_to_target << tangent,
    center_to_ref,
    _normal;
  
  Eigen::MatrixXf homogeneous_transform(4,4);
  homogeneous_transform << world_to_target,
    reference_pt,
    Eigen::RowVector4f(0,0,0,1);
  
  std::cout<<"Homogeneous transformation:" <<homogeneous_transform<<std::endl;

  if(visualize)
    visualize_data(world_to_target, reference_pt);

  return homogeneous_transform;
}


/** Saves the transformation frames at the end points of the cloud.*/
void circle3d::get_end_frames(Eigen::MatrixXf &min_frame,
		    Eigen::MatrixXf &max_frame) {
  Eigen::MatrixXf pts = pcl_to_eigen(_cloud).transpose();
  Eigen::MatrixXf pts_h(4, pts.cols());
  pts_h << pts, Eigen::MatrixXf::Ones(1, pts.cols());

  Eigen::Matrix4f trans;
  trans << _rotation.transpose(),
          -1*_rotation.transpose()*_center,
           Eigen::RowVector4f(0,0,0,1);

  Eigen::MatrixXf center_pts = (trans*pts_h).block(0,0,3,pts.cols());

  double min_angle = +100;
  double max_angle = -100;  
  
  float maxX, maxY, minX, minY;
  Eigen::Vector3f min_pt, max_pt;
  for(int i = 0; i < center_pts.cols(); i+=1) {
    float pt_x = center_pts(0,i);
    float pt_y = center_pts(1,i);
    
    float theta = atan2(pt_y, pt_x);
    if(theta < 0)
      theta += 2*PI;

    if(theta > max_angle)  {
      max_angle = theta;
      max_pt    = center_pts.col(i); 
    }

    if(theta < min_angle) {
      min_angle = theta;
      min_pt     = center_pts.col(i);
    }
  }
  
  min_pt    = _rotation*min_pt + _center;
  min_frame = this->get_frame(min_pt, false);
  

  max_pt = _rotation*max_pt + _center;
  max_frame = this->get_frame(max_pt,false);
  max_frame.col(0) *= -1;
  max_frame.col(1) *= -1;
} 
