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
  static float downsample;
  static int maxH;
  static int minH;
  static int maxS;
  static int minS;
  static int maxV;
  static int minV;
  static int debugging;

  LocalConfig() : Config() {
    params.push_back(new Parameter<std::string>("pcTopic", &pcTopic, "Point Cloud Topic"));
    params.push_back(new Parameter<float>("downsample", &downsample, "Downsampling"));
    params.push_back(new Parameter<int>("maxH", &maxH, "Maximum hue"));
    params.push_back(new Parameter<int>("minH", &minH, "Minimum hue"));
    params.push_back(new Parameter<int>("maxS", &maxS, "Maximum saturation"));
    params.push_back(new Parameter<int>("minS", &minS, "Minimum saturation"));
    params.push_back(new Parameter<int>("maxV", &maxV, "Maximum value"));
    params.push_back(new Parameter<int>("minV", &minV, "Minimum value"));
    params.push_back(new Parameter<int>("debugging", &debugging, "Debug flag: 1/0 - Yes/No"));
  }
};

std::string LocalConfig::pcTopic = "/kinect/depth_registered/points";
float LocalConfig::downsample = 0.008;
int LocalConfig::maxH = 180;
int LocalConfig::minH = 0;
int LocalConfig::maxS = 255;
int LocalConfig::minS = 0;
int LocalConfig::maxV = 255;
int LocalConfig::minV = 0;
int LocalConfig::debugging = 0;
  
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
  Fix: Issue with pointers
*/
void createFilter (filter_cascader &cascader) {
  boost::shared_ptr<downsample_wrapper>  downsampler (new downsample_wrapper());
  boost::shared_ptr<hueFilter_wrapper> hue_filter (new hueFilter_wrapper());
  boost::shared_ptr<removeOutliers_wrapper> outlier_remover (new removeOutliers_wrapper());

  // Maybe isolate only some parts of the point 

  if (LocalConfig::downsample) {
    downsampler->setSize(LocalConfig::downsample);
    cascader.appendFilter(downsampler);
  }
   
  hue_filter->setMinHue(LocalConfig::minH);
  hue_filter->setMaxHue(LocalConfig::maxH);
  hue_filter->setMaxSat(LocalConfig::maxS);
  hue_filter->setMinSat(LocalConfig::minS);
  hue_filter->setMaxVal(LocalConfig::maxV);
  hue_filter->setMinVal(LocalConfig::minV);
  
  cascader.appendFilter(hue_filter);
  
  cascader.appendFilter(outlier_remover);
}

template <typename vectype>
void makeIntoOne (std::vector< std::vector <vectype> > *in, std::vector <vectype> *out) {
  for (int i = 0; i < in->size(); i++) {
    for (int j = 0; j < in->at(i).size(); j++) 
      out->push_back(in->at(i)[j]);
  }
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

  if (LocalConfig::debugging)
    std::cout<<"Setting up the filter."<<std::endl;

  filter_cascader cascader;
  createFilter(cascader);

  if (LocalConfig::debugging)
    std::cout<<"Finished setting up the filter."<<std::endl;
  
  ////////// Setting up the filter ///////////////
  /* 
  if (LocalConfig::debugging)
    std::cout<<LocalConfig::downsample<<std::endl;

  downsample_wrapper downsampler(LocalConfig::downsample);
  hueFilter_wrapper hue_filter;
  removeOutliers_wrapper outlier_remover;

  // Maybe isolate only some parts of the point cloud?
  hue_filter.setMinHue(LocalConfig::minH);
  hue_filter.setMaxHue(LocalConfig::maxH);
  hue_filter.setMaxSat(LocalConfig::maxS);
  hue_filter.setMinSat(LocalConfig::minS);
  hue_filter.setMaxVal(LocalConfig::maxV);
  hue_filter.setMinVal(LocalConfig::minV);
  
  cascader.appendFilter(&downsampler);
  cascader.appendFilter(&hue_filter);
  cascader.appendFilter(&outlier_remover);
  */
  ////////////////////////////////////////////////

  if (LocalConfig::debugging)
    std::cout<<"Waiting for the first message."<<std::endl;

  while (!pending) {
    ros::spinOnce();
    sleep(.001);
    if (!ros::ok()) 
      throw std::runtime_error("caught signal while waiting for first message");
  }

  if (LocalConfig::debugging)
    std::cout<<"First message found. Setting up viewer and starting filters."<<std::endl;

  pcl::visualization::CloudViewer viewer ("Visualizer");

  while (ros::ok()) {        
    ColorCloudPtr cloud_pcl_filtered (new ColorCloud);

    if (LocalConfig::debugging)
      std::cout<<"Debug 1"<<std::endl;    
    
    cascader.filter(cloud_pcl, cloud_pcl_filtered);

    std::vector < std::vector <int> > colorClusters = findClusters (cloud_pcl_filtered);
    std::vector < int > colorCluster;
    makeIntoOne (&colorClusters, &colorCluster);
    ColorCloudPtr pc2 (new ColorCloud (*cloud_pcl_filtered, colorCluster));
    //std::vector < std::vector <ColorPoint> > 

    //viewer.showCloud(cloud_pcl_filtered);
    viewer.showCloud(pc2);

    if (LocalConfig::debugging)
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
