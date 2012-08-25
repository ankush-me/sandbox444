/** Author: Ankush Gupta
    Date  : 24th August 2012. */

#include <iostream>
#include <Eigen/Dense>
#include <math.h>

using namespace Eigen;

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
	   Eigen::Vector3d &pt3) : _normal(),
				   _rotation(3,3),
				   _translation(),
				   _center(),
				   _x_axis(), _y_axis(),
				   _z_axis(), _radius(0) {
    _pt1 = pt1; _pt2 = pt2; _pt3 = pt3;
 }

  /** Returns a unit normal to the plane defined
      by the three input points.*/
  Eigen::Vector3d get_plane_normal(bool verbose=false) {
    Eigen::Vector3d vec1 = _pt2-_pt1;
    Eigen::Vector3d vec2 = _pt3-_pt1;

    if (vec1.norm() <= 1e-5 || vec2.norm() <= 1e-5) {
      std::cout<<"Error: Plane normal : The given points are not unique."<<std::endl;
      throw;
    }
    
    Eigen::Vector3d normal = vec1.cross(vec2);
    normal /= normal.norm();

    if (verbose) {
      std::cout<< "The plane normal found is: ("<<normal(0)<<","
	       <<normal(1)<<","<<normal(2)<<")"<<std::endl;
    }
    return normal;
  }


  /** Fits a circle to the given 2D _PTs and saves them
      in the CENTER and RADIUS. */
  void get_circle(Vector2d &pt1, Vector2d &pt2, Vector2d &pt3,
		  Vector2d &center, double &radius, bool verbose=false) {
    Eigen::Matrix3d A(3,3);
    A << 2*pt1(0), 2*pt1(1), 1,
         2*pt2(0), 2*pt2(1), 1,
         2*pt3(0), 2*pt3(1), 1;

    Eigen::Vector3d b(pt1.squaredNorm(),
	              pt2.squaredNorm(),
		      pt3.squaredNorm() );

    Eigen::Vector3d x = A.fullPivLu().solve(b);
    center = Vector2d(x(0), x(1));
    radius = (pt1-center).norm();
    double relative_error = (A*x - b).norm() / b.norm();

    if (verbose) {
      std::cout <<"The relative error is :\n" << relative_error << std::endl;
      std::cout<< "The center is at : ("<<center(0)<<","<<center(1)<<")"<<std::endl;
      std::cout<< "The radius is   : "<<radius<<std::endl;
    }
  }

  void compute_circle3d(bool verbose=false) {
    _normal = get_plane_normal(verbose);

    _x_axis = (_pt2-_pt1);
    _z_axis = _normal;
    _y_axis = _z_axis.cross(_x_axis);
  
    _x_axis /= _x_axis.norm();
    _y_axis /= _y_axis.norm();
    _z_axis /= _z_axis.norm();

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

  /** Returns whether PT2 is clockwise or counter-clockwise
      with respect to pt1 in the plane defined by the three points. */
  orientation get_orientation(Eigen::Vector3d &pt1, Eigen::Vector3d &pt2) {
    return _normal.dot((pt1 - _center).cross(pt2 - _center)) >= 0? CCW : CW;
  }


  /** Walks distance DIST on the circumference of the 3D circle,
      starting at the REFERENCE_PT, in the DIR direction.
      Returns the TRANSFORM of the destination point in the world frame. */
  Vector3d extend_circumfrenece(Eigen::Vector3d reference_pt, double dist,
				orientation dir) {
    if (((_center - reference_pt).norm() - _radius) > 0.005
	|| (_pt1 - reference_pt).dot(_normal) > 1e-5) {
      std::cout<<"Error: extend_circumference: Reference point, not on the circle."
	       <<std::endl;
      throw;
    }

    Eigen::Vector3d center_local    =  _rotation*(_center + _translation);
    Eigen::Vector3d reference_local =  _rotation*(reference_pt + _translation);
    Eigen::Vector3d center_to_ref = reference_local - center_local;

    double angle = (dir==CCW)? dist/_radius : -dist/_radius;
    Eigen::Rotation2Dd rot(angle);
    Eigen::Vector2d target_local = rot*Eigen::Vector2d(center_to_ref(0), center_to_ref(1));
    
    //finding the tangent at Target : numerical difference.
    double diff_angle = (dir==CCW)? (dist+0.001)/_radius : -(dist + 0.001)/_radius;
    Eigen::Rotation2Dd diff_rot(diff_angle);
    Eigen::Vector2d tangent = (diff_rot*Eigen::Vector2d(center_to_ref(0), center_to_ref(1))) - target_local;
    tangent/= tangent.norm();

    Eigen::Vector3d tangent_x(tangent(0), tangent(1), 0);
    Eigen::Vector3d world_tangent_x = (_rotation.transpose()*tangent_x) - _translation;
    Eigen::Vector3d world_tangent_z = _normal;
    Eigen::Vector3d world_tangent_y = world_tangent_z.cross(world_tangent_x);

    Eigen::Vector3d target_world = (_rotation.transpose()*Eigen::Vector3d(target_local(0), target_local(1),0)) - _translation;
  }
};


int main() {
  Vector3d p(1,0,2);
  Vector3d q(0,1,2);
  Vector3d r(0,0,3);
  circle3d c3d(p,q,r);
  c3d.compute_circle3d(true);

  return 0;
}
