#include <iostream>
#include <Eigen/Dense>
#include <math.h>

using namespace Eigen;

void get_circle(Vector2d &pt1, Vector2d &pt2, Vector2d &pt3,
		bool verbose=false) {
  Matrix3d A(3,3);
  A << 2*pt1(0), 2*pt1(1), 1,
       2*pt2(0), 2*pt2(1), 1,
       2*pt3(0), 2*pt3(1), 1;

  Vector3d b(pt1.squaredNorm(),
	     pt2.squaredNorm(),
	     pt3.squaredNorm() );

  Vector3d x = A.fullPivLu().solve(b);
  Vector2d center(x(0), x(1));
  double radius = (pt1-center).norm();
  double relative_error = (A*x - b).norm() / b.norm();

  if (verbose) {
      std::cout << "The relative error is:\n" << relative_error << std::endl;
      std::cout<< "The center is at: ("<<center(0)<<","<<center(1)<<")"<<std::endl;
      std::cout<< "The radius is   : "<<radius<<std::endl;
    }
}


int main() {
  Vector2d pt1( 2,3);
  Vector2d pt2( 1,2);
  Vector2d pt3( 2.8660254,2.5);

  get_circle(pt1, pt2, pt3,true);
  return 0;
}
