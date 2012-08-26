/* TODO:
   Figure out how to move information between nodes.
   Finish make_unit.hpp to return a proper unit.
   Maybe take into account where holes/cuts were last iteration.
*/

#include <ros/topic.h>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/visualization/cloud_viewer.h>

#include "filter_wrapper.h"
#include "utils/config.h"
#include "get_table.h"
#include "utils/conversions.h"

#include "surgical_units.hpp"
#include "make_unit.hpp"
#include "extract_inds.hpp"

using namespace Eigen;

/* 
   Configuration for command line options
*/
struct LocalConfig : Config {
  static std::string pcTopic;
  static float downsample;
  static int tableMaxH;
  static int tableMinH;
  static int tableMaxS;
  static int tableMinS;
  static int tableMaxV;
  static int tableMinV;
  static bool tableNeg;
  static float zClipHigh;
  static float zClipLow;
  static int maxH;
  static int minH;
  static int maxS;
  static int minS;
  static int maxV;
  static int minV;
  static bool useHF;
  static int debugging;

  LocalConfig() : Config() {
    params.push_back(new Parameter<std::string>
		     ("pcTopic", &pcTopic, "Point Cloud Topic"));
    params.push_back(new Parameter<float>
		     ("downsample", &downsample, "Downsampling"));
    params.push_back(new Parameter<int>
		     ("tableMaxH", &tableMaxH, "Maximum table hue"));
    params.push_back(new Parameter<int>
		     ("tableMinH", &tableMinH, "Minimum table hue"));
    params.push_back(new Parameter<int>
		     ("tableMaxS", &tableMaxS, "Maximum table saturation"));
    params.push_back(new Parameter<int>
		     ("tableMinS", &tableMinS, "Minimum table saturation"));
    params.push_back(new Parameter<int>
		     ("tableMaxV", &tableMaxV, "Maximum table value"));
    params.push_back(new Parameter<int>
		     ("tableMinV", &tableMinV, "Minimum table value"));
    params.push_back(new Parameter<bool>
		     ("tableNeg", &tableNeg, "Filter out/in table")); 
    params.push_back(new Parameter<float>
		     ("zClipHigh", &zClipHigh, "Clip above this"));
    params.push_back(new Parameter<float>
		     ("zClipLow", &zClipLow, "Clip below this"));
    params.push_back(new Parameter<int>("maxH", &maxH, "Maximum hue"));
    params.push_back(new Parameter<int>("minH", &minH, "Minimum hue"));
    params.push_back(new Parameter<int>("maxS", &maxS, "Maximum saturation"));
    params.push_back(new Parameter<int>("minS", &minS, "Minimum saturation"));
    params.push_back(new Parameter<int>("maxV", &maxV, "Maximum value"));
    params.push_back(new Parameter<int>("minV", &minV, "Minimum value"));
    params.push_back(new Parameter<bool>("useHF", &useHF, "Use hue filter"));
    params.push_back(new Parameter<int>
		     ("debugging", &debugging, "Debug flag: 1/0 - Yes/No"));
  }
};

/* 
   Structure for storing the values to create the 
   oriented box filter.
*/
struct {
  bool m_init;
  Vector3f m_mins, m_maxes;
  btTransform m_trans;
} boxProp;

/*
  Default values for command line options.
*/
std::string LocalConfig::pcTopic = "/kinect/depth_registered/points";
float LocalConfig::downsample = 0.008;
int LocalConfig::tableMaxH = 10;
int LocalConfig::tableMinH = 170;
int LocalConfig::tableMaxS = 255;
int LocalConfig::tableMinS = 150;
int LocalConfig::tableMaxV = 255;
int LocalConfig::tableMinV = 0;
bool LocalConfig::tableNeg = false;
float LocalConfig::zClipLow = -0.05;
float LocalConfig::zClipHigh = 0.5;
int LocalConfig::maxH = 180;
int LocalConfig::minH = 0;
int LocalConfig::maxS = 255;
int LocalConfig::minS = 0;
int LocalConfig::maxV = 255;
int LocalConfig::minV = 0;
bool LocalConfig::useHF = false;
int LocalConfig::debugging = 0;
  
/* 
   Variables for the callback.
*/
static ColorCloudPtr cloud_pcl (new ColorCloud);
bool pending = false;

/*
  Variables for storing the holes/cuts/suture.
*/
vector<Hole::Ptr> holes;
vector<Cut::Ptr> cuts;
Suture::Ptr suture;

//Scale for stddev of hue to find points of similar color
int HUE_STD_SCALE = 2;
/*
  Returns a point cloud displaying only the hole specified by the index.
  Runs a hueFilter and assumes holes are of unique colors.
*/
ColorCloudPtr showHole (ColorCloudPtr in, int index) {
  Hole::Ptr hole = holes[index-1];
  
  uint8_t hole_minH = hole->_H - HUE_STD_SCALE*hole->_Hstd;
  uint8_t hole_maxH = hole->_H + HUE_STD_SCALE*hole->_Hstd;
  uint8_t hole_minS = hole->_S - hole->_Sstd;
  uint8_t hole_maxS = hole->_S + hole->_Sstd;
  uint8_t hole_minV = hole->_V - hole->_Vstd;
  uint8_t hole_maxV = hole->_V + hole->_Vstd;
  
  hueFilter_wrapper 
    hole_HF (hole_minH, hole_maxH, hole_minS, hole_maxS,
	     hole_minV, hole_maxV, false);

  ColorCloudPtr out (new ColorCloud());
  
  hole_HF.filter(in, out);
  
  return out;
}

/*
  Returns a point cloud displaying only the cut specified by the index.
  Runs a hueFilter and assumes cuts are of unique colors.
*/
ColorCloudPtr showCut (ColorCloudPtr in, int index) {
  Cut::Ptr cut = cuts[index-1];
  
  uint8_t cut_minH = cut->_H - HUE_STD_SCALE*cut->_Hstd;
  uint8_t cut_maxH = cut->_H + HUE_STD_SCALE*cut->_Hstd;
  uint8_t cut_minS = cut->_S - cut->_Sstd;
  uint8_t cut_maxS = cut->_S + cut->_Sstd;
  uint8_t cut_minV = cut->_V - cut->_Vstd;
  uint8_t cut_maxV = cut->_V + cut->_Vstd;
  
  hueFilter_wrapper 
    cut_HF (cut_minH, cut_maxH, cut_minS, cut_maxS,
	     cut_minV, cut_maxV, false);

  ColorCloudPtr out (new ColorCloud());
  
  cut_HF.filter(in, out);
  
  return out;
}

/*
  Returns a point cloud displaying only the suture.
  Runs a hueFilter and assumes the suture is of a unique color.
*/
ColorCloudPtr showSuture (ColorCloudPtr in) {

  uint8_t suture_minH = suture->_H - HUE_STD_SCALE*suture->_Hstd;
  uint8_t suture_maxH = suture->_H + HUE_STD_SCALE*suture->_Hstd;
  uint8_t suture_minS = suture->_S - suture->_Sstd;
  uint8_t suture_maxS = suture->_S + suture->_Sstd;
  uint8_t suture_minV = suture->_V - suture->_Vstd;
  uint8_t suture_maxV = suture->_V + suture->_Vstd;
  
  hueFilter_wrapper 
    suture_HF (suture_minH, suture_maxH, suture_minS, suture_maxS,
	     suture_minV, suture_maxV, false);

  ColorCloudPtr out (new ColorCloud());
  
  suture_HF.filter(in, out);
  
  return out;
}

/*
  Initializing values for the box filter.
  Taken from preprocessor node's initTable.
*/
void initBoxFilter (ColorCloudPtr cloud) {  
  MatrixXf corners = getTableCornersRansac(cloud);

  Vector3f xax = corners.row(1) - corners.row(0);
  xax.normalize();
  Vector3f yax = corners.row(3) - corners.row(0);
  yax.normalize();
  Vector3f zax = xax.cross(yax);

  Matrix3f m_axes;

  m_axes.col(0) = xax;
  m_axes.col(1) = yax;
  m_axes.col(2) = zax;
  
  MatrixXf rotCorners = corners * m_axes;
  
  boxProp.m_mins = rotCorners.colwise().minCoeff();
  boxProp.m_maxes = rotCorners.colwise().maxCoeff();
  boxProp.m_mins(0) += 0.03;
  boxProp.m_mins(1) += 0.03;
  boxProp.m_maxes(0) -= 0.03;
  boxProp.m_maxes(1) -= 0.03;
  boxProp.m_mins(2) = rotCorners(0,2) + LocalConfig::zClipLow;
  boxProp.m_maxes(2) = rotCorners(0,2) + LocalConfig::zClipHigh;
  
  boxProp.m_trans.setBasis(toBulletMatrix(m_axes));
  boxProp.m_trans.setOrigin
    (btVector3(corners(0,0), corners(0,1), corners(0,2)));

  boxProp.m_init = true;
}

/*
  Callback to store last message.
*/
void callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  fromROSMsg(*msg, *cloud_pcl);

  if (!boxProp.m_init)  {
    hueFilter_wrapper hue_filter(LocalConfig::tableMinH, 
				 LocalConfig::tableMaxH, 
				 LocalConfig::tableMinS, 
				 LocalConfig::tableMaxS,
				 LocalConfig::tableMinV,
				 LocalConfig::tableMaxV, 
				 LocalConfig::tableNeg);

    ColorCloudPtr cloud_color (new ColorCloud());
    hue_filter.filter(cloud_pcl, cloud_color);

    cloud_color = clusterFilter(cloud_color, 0.01, 100);      

    if (cloud_color->size() < 50)
      ROS_ERROR("The table cannot be seen.");
    else
      initBoxFilter (cloud_color);
  } 
  pending = true;
}

/*
  Function to append filters to a cascader.
*/
void createFilter (filter_cascader &cascader) {

  //Downsampler
  if (LocalConfig::downsample) {
    boost::shared_ptr<downsample_wrapper> 
      downsampler (new downsample_wrapper());
    downsampler->setSize(LocalConfig::downsample);
    cascader.appendFilter(downsampler);
  }

  //Oriented Box Filter
  boost::shared_ptr<orientedBoxFilter_wrapper> 
    oBoxFilter (new orientedBoxFilter_wrapper());
  
  oBoxFilter->setOrigin(toEigenMatrix(boxProp.m_trans.getBasis()));
  oBoxFilter->setMins(boxProp.m_mins);
  oBoxFilter->setMaxes(boxProp.m_maxes);
    
  cascader.appendFilter(oBoxFilter);

  //Hue Filter
  if (LocalConfig::useHF) {
    boost::shared_ptr<hueFilter_wrapper> 
      hue_filter (new hueFilter_wrapper());

    hue_filter->setMinHue(LocalConfig::minH);
    hue_filter->setMaxHue(LocalConfig::maxH);
    hue_filter->setMaxSat(LocalConfig::maxS);
    hue_filter->setMinSat(LocalConfig::minS);
    hue_filter->setMaxVal(LocalConfig::maxV);
    hue_filter->setMinVal(LocalConfig::minV);
    
    cascader.appendFilter(hue_filter);
  }

  //Outlier remover
  boost::shared_ptr<removeOutliers_wrapper> 
    outlier_remover (new removeOutliers_wrapper());
  
  cascader.appendFilter(outlier_remover);
  
}

/*
  Given a vector of vectors, combines to form one vector.
*/
template <typename vectype>
void makeIntoOne (std::vector< std::vector <vectype> > *in, 
		  std::vector <vectype> *out) {
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

  while (!pending) {
    ros::spinOnce();
    sleep(.001);
    if (!ros::ok()) 
      throw std::runtime_error
	("caught signal while waiting for first message");
  }

  filter_cascader cascader;
  createFilter(cascader);

  pcl::visualization::CloudViewer viewer ("Visualizer");
  
  while (ros::ok()) {
    ColorCloudPtr cloud_pcl_filtered (new ColorCloud);

    cascader.filter(cloud_pcl, cloud_pcl_filtered);
      
    std::vector < std::vector <int> > 
      colorClusters = findClusters (cloud_pcl_filtered);
    //std::vector < int > colorCluster;
    //makeIntoOne (&colorClusters, &colorCluster);
    //ColorCloudPtr pc2 (new ColorCloud (*cloud_pcl_filtered, colorCluster));
    //std::vector < std::vector <ColorPoint> > 

    viewer.showCloud(cloud_pcl_filtered);
    //viewer.showCloud(pc2);

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
