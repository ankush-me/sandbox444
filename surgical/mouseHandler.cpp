#include "mouseHandler.h"

#include <cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "SurgicalGUI.h"

void mouseHandler(int event, int x, int y, int flags, void* data) {
  if (event == CV_EVENT_LBUTTONDOWN) {
    SurgicalGUI* gui = (SurgicalGUI *) data;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
      cloud_ptr = gui->get_image_communicator()->get_cloud_ptr();

    pcl::PointXYZRGB pt = cloud_ptr->at(x,y);
    gui->interact(&pt, x, y);
  }
}
