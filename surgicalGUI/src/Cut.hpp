/** Author: Ankush Gupta
    Date  : 15th August, 2012. */

#ifndef _CUT_H_
#define _CUT_H_

#include "Hole.hpp"
#include <opencv2/core/core.hpp>
#include <pcl/point_types.h>


class Cut {
 private:
  std::vector<Hole::Ptr> _pts;

 public:
  Cut() : _pts() {}

  typedef boost::shared_ptr<Cut> Ptr;

  std::vector<Hole::Ptr> inline get_nodes() {return _pts;}

  void inline add_point(pcl::PointXYZRGB *pt, int row, int col) {
    Hole::Ptr support_pt(new Hole(pt, row, col));
    _pts.push_back(support_pt);
  }

  void paint(cv::Mat &mat, cv::Scalar color=cv::Scalar(50,0,50), int radius=3) {
    for(int i = 0; i < _pts.size(); i += 1) {
      _pts[i]->paint(mat, color, radius);
      if (i > 0) {
	cv::Point end1(_pts[i-1]->get_row(), _pts[i-1]->get_col());
	cv::Point end2(_pts[i]->get_row(), _pts[i]->get_col());
	cv::line(mat, end1, end2, color);
      }
    }
  }
};

#endif
