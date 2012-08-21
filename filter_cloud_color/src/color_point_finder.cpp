#include <ros/topic.h>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/visualization/cloud_viewer.h>

#include "filter_wrapper.h"
#include "utils/config.h"
#include "cloud_ops.h"

/* 
   Configuration for command line options
*/

struct LocalConfig : Config {
  static std::string pcTopic;
  static float dsRatio;
  static int maxH;
  static int minH;
  static int maxS;
  static int minS;
  static int maxV;
  static int minV;

  LocalConfig() : Config() {
    params.push_back(new Parameter<std::string>("pcTopic", &pcTopic, "Point Cloud Topic"));
    params.push_back(new Parameter<float>("dsRatio", &dsRatio, "Downsampling ratio"));
    params.push_back(new Parameter<int>("maxH", &maxH, "Maximum hue"));
    params.push_back(new Parameter<int>("minH", &minH, "Minimum hue"));
    params.push_back(new Parameter<int>("maxS", &maxS, "Maximum saturation"));
    params.push_back(new Parameter<int>("minS", &minS, "Minimum saturation"));
    params.push_back(new Parameter<int>("maxV", &maxV, "Maximum value"));
    params.push_back(new Parameter<int>("minV", &minV, "Minimum value"));
  }
};

std::string LocalConfig::pcTopic = "/kinect/depth_registered/points";
float LocalConfig::dsRatio = 0.02;
int LocalConfig::maxH = 180;
int LocalConfig::minH = 0;
int LocalConfig::maxS = 255;
int LocalConfig::minS = 0;
int LocalConfig::maxV = 255;
int LocalConfig::minV = 0;

  
static ColorCloudPtr cloud_pcl (new ColorCloud);
bool pending = false;

/*
  Callback to store last message.
*/
void callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  fromROSMsg(*msg, *cloud_pcl);
  pending = true;
}

/*
  Append filters to your cascader
*/
void createFilter (filter_cascader &cascader) {
  downsample_wrapper downsampler(LocalConfig::dsRatio);
  hueFilter_wrapper hue_filter;
  removeOutliers_wrapper outlier_remover;

  // Maybe isolate only some parts of the point cloud?


  hue_filter.setMinHue(LocalConfig::minH);
  hue_filter.setMaxHue(LocalConfig::maxH);
  hue_filter.setMaxSat(LocalConfig::maxS);
  hue_filter.setMinSat(LocalConfig::minS);
  hue_filter.setMaxVal(LocalConfig::maxV);
  hue_filter.setMinVal(LocalConfig::minV);
  
  //cascader.appendFilter(&downsampler);
  //cascader.appendFilter(&hue_filter);
  //cascader.appendFilter(&outlier_remover);
}


int main (int argc, char* argv[]) {
  Parser parser;
  parser.addGroup(LocalConfig());
  parser.read(argc, argv);

  ros::init(argc, argv, "find_color_points");

  ros::NodeHandle nh;
  ros::Subscriber pc_sub = 
    nh.subscribe(LocalConfig::pcTopic, 1, &callback);

  //  ros::Publisher pc_pub = 
  //  nh.advertise<sensor_msgs::PointCloud2>
  //  (LocalConfig::camNS + "depth_registered/filtered_points", 5);

  filter_cascader cascader;
  createFilter(cascader);

  while (!pending) {
    ros::spinOnce();
    sleep(.001);
    if (!ros::ok()) 
      throw std::runtime_error("caught signal while waiting for first message");
  }
  pcl::visualization::CloudViewer viewer ("Visualizer");

  while (ros::ok()) {        
    ColorCloudPtr cloud_pcl_filtered (new ColorCloud);
    std::cout<<"Debug 1"<<std::endl;    
    cascader.filter(cloud_pcl, cloud_pcl_filtered);

    viewer.showCloud(cloud_pcl_filtered);
    std::cout<<"Debug 2"<<std::endl;    

    //sensor_msgs::PointCloud2 cloud_ros_filtered;
    //pcl::toROSMsg(*cloud_pcl_filtered, cloud_ros_filtered);
    
    //pc_pub.publish(cloud_ros_filtered);

    pending = false;
    while (ros::ok() && !pending) {
      sleep(.01);
      ros::spinOnce();
    }
  }
}
