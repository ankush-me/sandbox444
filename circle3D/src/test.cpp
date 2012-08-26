#include "circle3d.h"

int main() {
  Eigen::Vector3d p(1,0,2);
  Eigen::Vector3d q(-1,0,2);
  Eigen::Vector3d r(0,1,2);
  circle3d c3d(p,q,r);
  c3d.compute_circle3d(true);
  Eigen::MatrixXd pt = c3d.extend_circumference(p,1.57079632679,circle3d::CCW);
  std::cout<<"circum pt: \n"<<pt<<std::endl;
  return 0;
}

