#include "mouseHandler.h"

#include <cv.h>                                                                                                                                       
#include <opencv2/core/core.hpp>                                                                                                                      
#include <opencv2/highgui/highgui.hpp>
#include "SurgicalGUI.h"

void mouseHandler(int event, int x, int y, int flags, void* data) {                                                                                   
  if (event == CV_EVENT_LBUTTONDOWN) {                                                                                                                
    SurgicalGUI* gui = (SurgicalGUI *) data;                                                                                                          
    pcl::PointXYZRGB pt;                                                                                                                              
    pt.x =x; pt.y = y; pt.z = 0;                                                                                                                      
    pt.r = 4; pt.g = 4; pt.b = 4;                                                                                                                     
    gui->interact(&pt, x, y);                                                                                                                         
  }                                                                                                                                                   
}  
