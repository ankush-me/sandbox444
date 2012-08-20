/** Author: Ankush Gupta
    Date  : 14th August, 2012 */

#include <ros/ros.h>

#include "SurgicalGUI.h"
#include "ImageCommunicator.hpp"

#include <QDesktopWidget>
#include <QApplication>



int main(int argc, char *argv[]) {
  ros::init(argc, argv, "surgical_gui");
  ros::NodeHandle nh;

  ImageCommunicator img_comm(&nh);

  QApplication app(argc, argv);  
  SurgicalGUI gui(&img_comm);

  gui.show();

  ros::Rate timer(1000);
  while (ros::ok()) {
    ros::spinOnce();
    app.processEvents();
    timer.sleep();
  }
  app.quit();
  return 0;
}
