#include <boost/thread/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <cstdlib>

#include <ros/ros.h>
#include <Eigen/Dense>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/model_types.h>

#include <filter_cloud_color/Cut.h>
#include <filter_cloud_color/utils/pcl_typedefs.h>

#include "line_finding.h"

using namespace std;

extern vector<float> getLineCoeffsRansac(ColorCloudPtr cloud);

int main (int argc, char * argv[]) {

  ros::init(argc, argv, "cut_line_finder_test");

  int index;
  if (argc != 2)
    index = 1;
  else 
    index = atoll (argv[1]);

  cout<<"Index: "<<index<<endl;

  ros::NodeHandle nh;
  ros::ServiceClient cutClient = 
    nh.serviceClient<filter_cloud_color::Cut>("getCut");

  //pcl::visualization::CloudViewer viewer ("Viewer");

  filter_cloud_color::Cut cutSrv;
  cutSrv.request.index = index;
  ColorCloudPtr cut_pcl (new ColorCloud ());

  if (cutClient.call(cutSrv)){

    cout<<"Here is the point cloud."<<endl;
    fromROSMsg (cutSrv.response.cut, *cut_pcl);
    //    viewer.showCloud(cut_pcl);

    vector<float> coeff = getLineCoeffsRansac (cut_pcl);

    cout<<"Size of cut: "<<cut_pcl->size()<<endl;
    for (int i = 0; i<coeff.size(); i++)
      cout<<"Coefficient 1: "<<coeff[i]<<endl;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> visualizer
      (new pcl::visualization::PCLVisualizer ("Visualizer"));

    
    visualizer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cut_pcl);
    visualizer->addPointCloud<pcl::PointXYZRGB> (cut_pcl, rgb, "cut cloud");
    visualizer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cut cloud");
    //visualizer->addCoordinateSystem (0.5);
    visualizer->initCameraParameters ();

    pcl::ModelCoefficients sphereCoeffs;
    sphereCoeffs.values.push_back (coeff[0]);
    sphereCoeffs.values.push_back (coeff[1]);
    sphereCoeffs.values.push_back (coeff[2]);
    sphereCoeffs.values.push_back (0.02);
    visualizer->addSphere (sphereCoeffs, "sphere");


    float SCALE = 0.3;
    pcl::PointXYZRGB p1, p2;
    p1.x = coeff[0] + SCALE*coeff[3];
    p1.y = coeff[1] + SCALE*coeff[4];
    p1.z = coeff[2] + SCALE*coeff[5];
    p2.x = coeff[0] - SCALE*coeff[3];
    p2.y = coeff[1] - SCALE*coeff[4];
    p2.z = coeff[2] - SCALE*coeff[5];    
    visualizer->addLine (p1, p2, "line");

    /*    pcl::ModelCoefficients lineCoeffs;
    lineCoeffs.values.resize (6);
    for (int i = 0; i < coeff.size(); ++i)
      lineCoeffs.values[i] = coeff[i];
    visualizer->addLine (lineCoeffs);

    pcl::ModelCoefficients lineCoeffs2;
    lineCoeffs.values.resize (6);
    for (int i = 0; i < 3; ++i)
      lineCoeffs.values[i] = coeff[i];
    for (int i = 3; i < 6; ++i)
      lineCoeffs.values[i] = -coeff[i];
    */
    
    ros::Rate rate(120);
    while (ros::ok()) {
      visualizer->spinOnce (50);
      ros::spinOnce();
      rate.sleep();
      //boost::this_thread::sleep (boost::posix_time::microseconds (100000));    
    }
    return 0;
  }
  else {
    ROS_ERROR("Failed to call service getCut");
    return 1;
  }
  
  cout<<"Ended main...?"<<endl;

}
