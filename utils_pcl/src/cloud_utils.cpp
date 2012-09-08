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


/** Finds a 3D circle which fits ALL the points in in_cloud. */
circle3d::circle3d(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud) : _cloud(in_cloud),
								      _normal(),
								      _a(0),_b(0),_c(0),_d(0),
								      _rotation(3,3),
								      _translation(),
								      _center(),
								      _radius(0) { }

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
  float coeffs_arr[] = {_a,_b,_c,_d};
  vector<float> coeffs(coeffs_arr, coeffs_arr + sizeof(coeffs_arr)/sizeof(float));
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  cloud->width  = 1;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);
  cloud->points[0].x = pt(0);
  cloud->points[0].y = pt(1);
  cloud->points[0].z = pt(2);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr project = project_points_plane(cloud, coeffs);
  Eigen::Vector3f project_pt(project->points[0].x,
			     project->points[0].y,
			     project->points[0].z);
  return _center + _radius*((project_pt - _center).normalized());
}


/** Displays:
    1. The input points.
    2. The best fitting 3D circle.
    3. The given coordinate frame and the coordinate frame at 0,0,0
    4. The best fitting plane. */
void circle3d::visualize_data(Eigen::MatrixXf &frame) {
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

  Eigen::Affine3f t;
  t.linear() = frame.transpose().block(0,0,3,3);
  t.translation() = frame.block(0,3,3,1);
  viewer->addCoordinateSystem (1.0,t);

  while (!viewer->wasStopped ()) {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}


/** Walks distance DIST on the circumference of the 3D circle,
    starting at the REFERENCE_PT, in the DIR direction.
    Returns the TRANSFORM of the destination point in the world frame.*/
Eigen::MatrixXf circle3d::extend_circumference(Eigen::Vector3f pt,
					       double dist, orientation dir,
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

  Eigen::Vector3f center_local    =  _rotation*(_center - _translation);
  Eigen::Vector3f reference_local =  _rotation*(reference_pt - _translation);
  Eigen::Vector3f center_to_ref   = reference_local - center_local;

  float angle = (dir==CCW)? dist/_radius : -dist/_radius;
  Eigen::Rotation2Df rot(angle);
  Eigen::Vector2f target_circle_local = rot*Eigen::Vector2f(center_to_ref(0),
							    center_to_ref(1));
  Eigen::Vector3f target_local = ( Eigen::Vector3f(target_circle_local(0),
						   target_circle_local(1),0) 
				   + center_local);
  Eigen::Vector3f target_world = (_rotation.transpose()*target_local)+_translation;

  //finding the tangent at Target : numerical difference.
  float theta = (dist+0.001)/_radius;
  float diff_angle = (dir==CCW)? theta : -theta;
  Eigen::Rotation2Df diff_rot(diff_angle);
  Eigen::Vector2f tangent_circle = ( diff_rot*Eigen::Vector2f(center_to_ref(0),
							      center_to_ref(1))
				     - target_circle_local );
  tangent_circle.normalize();

  Eigen::Vector3f tangent_local = ( Eigen::Vector3f(tangent_circle(0),
						    tangent_circle(1),
						    0)
				    + center_local);
  Eigen::Vector3f world_tangent_x = (_rotation.transpose()*tangent_local);
  Eigen::Vector3f world_tangent_z = _normal;
  Eigen::Vector3f world_tangent_y = world_tangent_z.cross(world_tangent_x);

  Eigen::Matrix3f world_to_target(3,3);
  world_to_target << world_tangent_x.transpose(),
                     world_tangent_y.transpose(),
                     world_tangent_z.transpose();

  world_to_target.col(0).normalize();
  world_to_target.col(1).normalize();
  world_to_target.col(2).normalize();

  Eigen::MatrixXf homogenous_transform(4,4);
  homogenous_transform << world_to_target,
                          world_to_target * target_world,
                          Eigen::RowVector4f(0,0,0,1);

  if(visualize)
    visualize_data(homogenous_transform);

  return homogenous_transform;
}
