/** Author: Ankush Gupta
    Date  : 25th August 2012. */

#include <iostream>
#include <Eigen/Dense>
#include <math.h>

/** Fits a 3d circle to three 3d points.*/
class circle3d {
private:
  Eigen::Vector3d _normal;
  Eigen::Matrix3d _rotation;
  Eigen::Vector3d _translation;
  Eigen::Vector3d _center;
  Eigen::Vector3d _x_axis;
  Eigen::Vector3d _y_axis;
  Eigen::Vector3d _z_axis;

  double _radius;
  Eigen::Vector3d _pt1, _pt2, _pt3;

public:

  /** Clockwise/ counter-clockwise. */
  enum orientation{CW, CCW};

  /** Finds a 3D circle which fits PT1, PT2, PT3. */
  circle3d(Eigen::Vector3d &pt1,
	   Eigen::Vector3d &pt2,
	   Eigen::Vector3d &pt3);

  /** Returns a unit normal to the plane defined
      by the three input points.*/
  Eigen::Vector3d get_plane_normal(bool verbose=false);

  /** Fits a circle to the given 2D _PTs and saves them
      in the CENTER and RADIUS. */
  void get_circle(Eigen::Vector2d &pt1, Eigen::Vector2d &pt2,
		  Eigen::Vector2d &pt3, Eigen::Vector2d &center,
		  double &radius, bool verbose=false);

  /** The main function : should be called before any other.
      Calculates a 3D circle which fits the threee given points.*/
  void compute_circle3d(bool verbose=false);

  /** Returns whether PT2 is clockwise or counter-clockwise
      with respect to pt1 in the plane defined by the three points. */
  orientation get_orientation(Eigen::Vector3d &pt1,
			      Eigen::Vector3d &pt2);


  /** Flips the normal's direction. */
  void flip_normal();

  /** Walks distance DIST on the circumference of the 3D circle,
      starting at the REFERENCE_PT, in the DIR direction.
      Returns the TRANSFORM of the destination point in the world frame.*/
  Eigen::MatrixXd extend_circumference(Eigen::Vector3d reference_pt,
				       double dist, orientation dir);
};
