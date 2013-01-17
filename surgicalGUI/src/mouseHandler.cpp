#include "mouseHandler.h"

#include <cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "SurgicalGUI.h"
#include "utils/utils_pcl.h"
#include <vector>

void mouseHandler(int event, int x, int y, int flags, void* data) {
  if (event == CV_EVENT_LBUTTONDOWN) {
    SurgicalGUI* gui = (SurgicalGUI *) data;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
      cloud_ptr = gui->get_image_communicator()->get_cloud_ptr();

    int cloud_height = cloud_ptr->height;

    int off[] = {0,1,-1,2,-2,-3,3,-4,4,-5,5};
    std::vector<int> offsets(off, off+sizeof(off)/sizeof(int));

    for (int x_idx = 0; x_idx < offsets.size(); x_idx += 1) {
      for (int y_idx = 0; y_idx < offsets.size(); y_idx += 1) {
	int x_new = round(x/ZOOM_FACTOR) + offsets[x_idx];
	int y_new = round(y/ZOOM_FACTOR) + offsets[y_idx];
	if ( ( (0 <= x_new) && (x_new < cloud_ptr->width))
	     && ( (0 <= y_new) && (y_new < cloud_ptr->height)) ) {
	  pcl::PointXYZRGB pt = cloud_ptr->at(x_new,y_new);
	  if (pointIsFinite(pt)) {
	    gui->interact(&pt, x_new, y_new);
	    return;
	  }
	}
      }
    }
  }
}
