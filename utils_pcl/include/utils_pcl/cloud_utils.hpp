#ifndef _UTILS_PCL_CLOUD_UTILS_H_
#define _UTILS_PCL_CLOUD_UTILS_H_

#include <boost/foreach.hpp>
#include <boost/thread/thread.hpp>

#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

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


/** Fits a 3d circle to the given input point CLOUD.
void compute_circle3d(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
bool verbose);*/
class circle3d {
private:

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud;
  Eigen::Vector3f _normal;
  Eigen::Matrix3f _rotation;
  Eigen::Vector3f _translation;
  Eigen::Vector3f _center;

  float _radius;

  /**  Coefficients which define the best fitting plane
       to the given point cloud. 
       _Ax + _By + _Cz + _D = 0  */
  float _a,_b,_c,_d;

public:

  /** Clockwise/ counter-clockwise. */
  enum orientation{CW, CCW};

  /** Finds a 3D circle which fits ALL the points in in_cloud. */
  circle3d(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud);

  /** Returns a point on this circle closest to the given PT. */
  Eigen::Vector3f snap_to_circle(Eigen::Vector3f pt);

  /** Displays:
      1. The input points.
      2. The best fitting 3D circle.
      3. The given coordinate frame and the coordinate frame at 0,0,0
      4. The best fitting plane. */
  void visualize_data(Eigen::MatrixXf &frame);

  /** Saves an orthogonal matrix corresponding to a basis defined
      on the "best fitting" plane to the given point-cloud in the matrix BASIS [input].
      The z-axis (col(3)) points in the direction of the normal.
      col(1) is the x-axis, and col(2) is the y-axis. 

      Returns the projections of the points on the plane found. */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_plane_basis(Eigen::Matrix3f &basis,
							 Eigen::Vector4f &coeffs,
							 bool verbose=false);

  /** Fits a circle to the points in the given 3D  pointcloud
      and saves them in the CENTER and RADIUS.
      It assumes that the points lie on the SAME XY plane,
      i.e. their Z-component is the same.  */
  void get_circle(pcl::PointCloud<pcl::PointXYZRGB>::Ptr points,
		  Eigen::Vector3f &center,
		  float &radius,  bool verbose=false,
		  float ransac_thresh=0.005);

  /** The main function : should be called before any other.
      Calculates a 3D circle which fits the threee given points.*/
  void compute_circle3d(bool verbose=false);

  /** Returns whether PT2 is clockwise or counter-clockwise
      with respect to pt1 in the plane defined by the three points. */
  orientation get_orientation(Eigen::Vector3f &pt1,
			      Eigen::Vector3f &pt2);

  /** Walks distance DIST on the circumference of the 3D circle,
      starting at the REFERENCE_PT, in the DIR direction.
      Returns the TRANSFORM of the destination point in the world frame.*/
  Eigen::MatrixXf extend_circumference(Eigen::Vector3f reference_pt,
				       double dist, orientation dir,
				       bool visualize=false);
};

#endif
