#include <ros/topic.h>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/visualization/cloud_viewer.h>

#include "filter_wrapper.h"
#include "utils/config.h"
#include "get_table2.h"
#include "utils/conversions.h"

#include "surgical_units.cpp"

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
  static int oldDebugging;

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
    params.push_back(new Parameter<int>
		     ("oldDebugging", &oldDebugging, "Old debug flag."));
  }
};

struct {
  bool m_init;
  Vector3f m_mins, m_maxes;
  btTransform m_trans;
} boxProp;

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
int LocalConfig::oldDebugging = 0;
  
/*
  Initializing values for the box filter.
  Taken from preprocessor node's initTable.
*/
void initBoxFilter (ColorCloudPtr cloud) {
  
  MatrixXf corners = getTableCornersRansac(cloud);

  if (LocalConfig::debugging)
    std::cout<<"Corners: "<<corners<<std::endl;

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

  if (LocalConfig::debugging) {
    std::cout<<"boxProp.m_trans.getBasis(): ";
    std::cout<<boxProp.m_trans.getBasis()<<std::endl;
    std::cout<<"boxProp.m_mins: "<<boxProp.m_mins<<std::endl;
    std::cout<<"boxProp.m_maxes: "<<boxProp.m_maxes<<std::endl;
  }

  boxProp.m_init = true;
}

/*
  Variables and callback to store last message.
*/
static ColorCloudPtr cloud_pcl (new ColorCloud);
bool pending = false;

void callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  
  if (LocalConfig::oldDebugging)
    std::cout<<"Start of callback."<<std::endl;

  fromROSMsg(*msg, *cloud_pcl);

  if (LocalConfig::oldDebugging)
    std::cout<<"Before entering if to call initBoxFilter"<<std::endl;

  if (!boxProp.m_init)  {
    
    if (LocalConfig::debugging) {
      std::cout<<"tminh: "<<LocalConfig::tableMinH<<std::endl;
      std::cout<<"tmaxh: "<<LocalConfig::tableMaxH<<std::endl;
      std::cout<<"tmins: "<<LocalConfig::tableMinS<<std::endl;
      std::cout<<"tmaxs: "<<LocalConfig::tableMaxS<<std::endl;
      std::cout<<"tminv: "<<LocalConfig::tableMinV<<std::endl;
      std::cout<<"tmaxv: "<<LocalConfig::tableMaxV<<std::endl;
    }

    hueFilter_wrapper hue_filter(LocalConfig::tableMinH, 
				 LocalConfig::tableMaxH, 
				 LocalConfig::tableMinS, 
				 LocalConfig::tableMaxS,
				 LocalConfig::tableMinV,
				 LocalConfig::tableMaxV, 
				 LocalConfig::tableNeg);

    if (LocalConfig::debugging)
      std::cout<<"Set up huefilter."<<std::endl;

    ColorCloudPtr cloud_color (new ColorCloud());
    hue_filter.filter(cloud_pcl, cloud_color);

    if (LocalConfig::debugging)
      std::cout<<"Finished hue filter"<<std::endl;

    cloud_color = clusterFilter(cloud_color, 0.01, 100);      

    if (LocalConfig::debugging)
      std::cout<<"Clustering complete"<<std::endl;

    if (cloud_color->size() < 50)
      ROS_ERROR("The table cannot be seen.");
    else
      initBoxFilter (cloud_color);
  } 
  pending = true;

  if (LocalConfig::oldDebugging)
    std::cout<<"End of callback."<<std::endl;

}

/*
  Append filters to a cascader.
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
  Given a vector of vectors, combines to form one vector
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

  if (LocalConfig::debugging)
    std::cout<<"Setting up the filter."<<std::endl;

  if (LocalConfig::debugging) {
    std::cout<<"Finished setting up the filter."<<std::endl;
    std::cout<<"Waiting for the first message."<<std::endl;
  }

  while (!pending) {
    ros::spinOnce();
    sleep(.001);
    if (!ros::ok()) 
      throw std::runtime_error
	("caught signal while waiting for first message");
  }

  filter_cascader cascader;
  createFilter(cascader);

  if (LocalConfig::debugging) {
    std::cout<<"First message found.";
    std::cout<<"Setting up viewer and starting filters."<<std::endl;
  }

  pcl::visualization::CloudViewer viewer ("Visualizer");
  
  while (ros::ok()) {
    ColorCloudPtr cloud_pcl_filtered (new ColorCloud);

    if (LocalConfig::oldDebugging)
      std::cout<<"Before filtering."<<std::endl;    

    cascader.filter(cloud_pcl, cloud_pcl_filtered);
      
    std::vector < std::vector <int> > 
      colorClusters = findClusters (cloud_pcl_filtered);
    //std::vector < int > colorCluster;
    //makeIntoOne (&colorClusters, &colorCluster);
    //ColorCloudPtr pc2 (new ColorCloud (*cloud_pcl_filtered, colorCluster));
    //std::vector < std::vector <ColorPoint> > 

    viewer.showCloud(cloud_pcl_filtered);
    //viewer.showCloud(pc2);

    if (LocalConfig::oldDebugging)
      std::cout<<"After filtering."<<std::endl;    

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
