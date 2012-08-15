/** Author: Ankush Gupta
    Date  : 15th August, 2012. */

#ifndef _IMAGE_COMMUNICATOR_
#define _IMAGE_COMMUNICATOR_

#include <cv.h>
#include <list>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/point_types.h>
#include "Hole.hpp"
#include "Cut.hpp"
#include "mouseHandler.h"

class SurgicalGUI;
//void mouseHandler(int event, int x, int y, int flags, void* data);

class ImageCommunicator {
private:
  cv::Mat _main_frame;
  cv::Mat _working_frame;
  std::string _window_name;
  SurgicalGUI * _gui;


public:
  ImageCommunicator(SurgicalGUI* gui,
		    std::string window_name="SurgiC@l") : _gui(gui),
                                                          _main_frame(cv::Mat::zeros(640, 480, CV_32FC3)),
							  _working_frame(),
							  _window_name(window_name) {
    cv::namedWindow(_window_name);
    cv::imshow(_window_name, _main_frame);
    cv::setMouseCallback(_window_name, mouseHandler, (void *) _gui);
  }

  void fix_frame() {}
  
  void repaint(std::list<Hole::Ptr> &holes, std::list<Cut::Ptr> &cuts) {
    _main_frame.copyTo(_working_frame);

    std::list<Hole::Ptr>::iterator holes_iter;
    for (holes_iter = holes.begin(); holes_iter != holes.end(); holes_iter++)
      (*holes_iter)->paint(_working_frame);

    std::list<Cut::Ptr>::iterator cuts_iter;
    for (cuts_iter = cuts.begin(); cuts_iter != cuts.end(); cuts_iter++)
      (*cuts_iter)->paint(_working_frame);

    /*for(int i = 0; i < cuts.size(); i += 1)
      cuts[i]->paint(_working_frame);*/

    cv::imshow(_window_name, _working_frame);
  }
  
};

#endif
