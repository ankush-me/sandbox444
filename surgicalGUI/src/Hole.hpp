/** Author: Ankush Gupta
    Date  : 15th August, 2012. */

#ifndef _HOLE_H_
#define _HOLE_H_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/point_types.h>

class Hole {
 private:
  pcl::PointXYZRGB _pt;
  int _row_idx, _col_idx;

 public:
  typedef boost::shared_ptr<Hole> Ptr;
  Hole(pcl::PointXYZRGB* pt, int row, int col) : _pt(*pt),
						_row_idx(row),
						_col_idx(col) {}

  pcl::PointXYZ inline get_position() {return pcl::PointXYZ(_pt.x, _pt.y, _pt.z);}
  int inline get_row() {return _row_idx;}
  int inline get_col() {return _col_idx;}

  void paint(cv::Mat &mat, cv::Scalar color=cv::Scalar(50,0,0),
	     int radius=6) {
    cv::Point center(_row_idx, _col_idx);
    cv::circle(mat, center, radius, color, CV_FILLED);
  }
};

#endif
