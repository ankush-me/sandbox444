/* TODO:
   cout -> ROSLOG_DEBUG
   Maybe extract the sponge by cascading orientedBoxFilters.
   Modularize code.
   Remove most of bullet.
*/

#include <ros/topic.h>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/search/pcl_search.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>

#include "filter_wrapper.h"
#include "filter_config.h"
#include "get_table.h"
#include "make_unit.hpp"
#include "extract_inds.hpp"
#include "line_finding.h"

#include "utils/conversions.h"

#include "filter_cloud_color/Corners.h"
#include "filter_cloud_color/Cut.h"
#include "filter_cloud_color/PointDir.h"

#include <surgical_msgs/HoleCutInfo.h>

using namespace Eigen;

/** Create a shared pointer type for pcl::Normal.*/
typedef boost::shared_ptr<pcl::Normal> NormalPtr;
typedef pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> NormalEstimation;
typedef boost::shared_ptr<NormalEstimation> NormalEstimationPtr;

/*
  Variables and functions declared in surgical_extraction.cpp.
*/
extern std::vector<Hole::Ptr> holes;
extern std::vector<Cut::Ptr> cuts;
extern Needle::Ptr needle;
extern ColorCloudPtr extractSurgicalUnits (ColorCloudPtr in,
					   vector<int> *holeInds,
					   vector<int> *cutInds,
					   int needleFlag);
extern ColorCloudPtr showHole (ColorCloudPtr in, int index);
extern ColorCloudPtr showCut (ColorCloudPtr in, int index);

/* 
   Structure for storing the values to create the 
   oriented box filter.
*/
struct {
  bool m_init, m_foundPC;
  Vector3f m_mins, m_maxes;
  btTransform m_trans;
  MatrixXf m_corners;
  std::vector<Vector3f> m_pcPoints;
  std::vector<Vector2f> m_imageInds;
} boxProp;

/* 
   Variables to store the current pointCloud, the filtered
   pointCloud and whether message is new.
   Modified in callback/main.
*/
static ColorCloudPtr cloud_pcl (new ColorCloud);
static ColorCloudPtr cloud_pcl_filtered (new ColorCloud);
bool pending = false;

/*
  Function to find the closest points to each point in inPoints
  from the stored point cloud, cloud_pcl. It also stores the row
  and column indices for the points found.
  This is basically to find the corners of the table in the
  organized point_cloud.
  Also for finding the image indices for the holes.
  Returns true if all points are found within distance minTol to 
  maxTol and stores it them nearestPoint.
  Returns false if any point is not found.
*/
bool findNearestPoints (ColorCloudPtr cloud,
						const vector<Vector3f> *inPoints,
		       	   	    vector<Vector3f> *nearestPoints,
		       	   	    vector<Vector2f> *imageInds,
		       	   	    int minTol=0.001, int maxTol=1) {

  
  pcl::search::KdTree<ColorPoint>::Ptr 
    tree (new pcl::search::KdTree<ColorPoint>);
  
  ColorPoint searchPoint;

  tree->setInputCloud(cloud);
  std::vector<int> indices;
  std::vector<float> dists;

  if (LocalConfig::debugging) {
    std::cout<<"inPoints size: "<<inPoints->size()<<std::endl;
  }

  for (int i = 0; i < inPoints->size(); ++i) {

    float tol = minTol;
    float tolStepSize = 0.001;
    bool foundFlag  = false;
    
    searchPoint.x = inPoints->at(i)[0];
    searchPoint.y = inPoints->at(i)[1];
    searchPoint.z = inPoints->at(i)[2];

    if (LocalConfig::debugging) {
      std::cout<<"Search Point: "<<searchPoint<<std::endl;
    }

    while (tol <= maxTol) {
      int ret = tree->radiusSearch(searchPoint, tol, indices, dists, 1);
      
      if( ret == -1) {
	PCL_ERROR("findNearestPoint got error code -1 from radiusSearch\n");
	exit(0);
      }
      if (!ret) {
	tol += tolStepSize;
	continue;
      }

      ColorPoint point = cloud_pcl->at(indices[0]);

      Vector3f foundPoint;
      foundPoint[0] = point.x;
      foundPoint[1] = point.y;
      foundPoint[2] = point.z;
      nearestPoints->push_back(foundPoint);

      Vector2f imageInd;
      imageInd[1] = indices[0]%cloud_pcl->width;
      imageInd[0] = (indices[0]-imageInd[1])/cloud_pcl->width;
      imageInds->push_back(imageInd);

      foundFlag = true;
      if (LocalConfig::debugging) {
	std::cout<<"Found something." <<point<<std::endl;
      }
      break;
    }
    if (!foundFlag)
      return false;
  }
  return true;
}

/*
  Function to find the indices of the k closest points to point
  given from the stored point cloud, cloud_pcl.
*/
void findkClosestPoints ( Vector3f inPoint, int k,
						  vector< int > &indices,
		       	   	   	  int minTol=0.001, int maxTol=1) {


	pcl::search::KdTree<ColorPoint>::Ptr
    	tree (new pcl::search::KdTree<ColorPoint>);

	ColorPoint searchPoint;

	tree->setInputCloud(cloud_pcl);
	std::vector<float> dists;

	indices.clear();

	float tol = minTol;
	float tolStepSize = 0.001;

	searchPoint.x = inPoint[0];
	searchPoint.y = inPoint[1];
	searchPoint.z = inPoint[2];

	while (tol <= maxTol) {
		int ret = tree->radiusSearch(searchPoint, tol, indices, dists, k);

		if( ret == -1) {
			PCL_ERROR("findNearestPoint got error code -1 from radiusSearch\n");
			exit(0);
		}
		if (!ret || (ret < k && tol <= maxTol - tolStepSize)) {
			tol += tolStepSize;
			continue;
		}

		if (LocalConfig::debugging) {
			std::cout<<"findkClosestPoints: Found something."<<std::endl;
		}
		break;
    }
}

/* Finds the average point of the points in a pointCloud. */
Vector3f findAveragePoint (ColorCloudPtr cloud) {
	Vector3f avgPoint(0,0,0);
	int cloudSize = cloud->size();

	for (int i = 0; i < cloudSize; ++i)
		avgPoint = Vector3f (cloud->at(i).x, cloud->at(i).y, cloud->at(i).z);
	avgPoint = avgPoint/cloudSize;

	return avgPoint;
}

/** Finds the surface normal around a point.
    X_IDX, Y_IDX : The x and y indices of the point at which
                   the normal is to be found in the organized point-cloud.
    NORMAL_ESTIMATOR : A normal_estimator structure appropriately
                       initialized for finding normals.
    [see:
    http://www.pointclouds.org/documentation/tutorials/normal_estimation_using_integral_images.php ]*/
NormalPtr findPointNormal(  const int x_idx, const int y_idx,
							NormalEstimationPtr normal_estimator) {

  std::cout<<"  Number 1."<<std::endl;
  int pt_idx = x_idx*normal_estimator->getInputCloud()->width + y_idx;
  pcl::Normal normal;
  std::cout<<"  Number 2. x:"<<x_idx<<" y:"<<y_idx<<" p:"<<pt_idx<<std::endl;
  std::cout<<"  Organized? "<<normal_estimator->getInputCloud()->isOrganized()<<std::endl;
  normal_estimator->computePointNormal(x_idx, y_idx, pt_idx, normal);
  std::cout<<"  Number 3."<<std::endl;
  NormalPtr ret_normal(new pcl::Normal);
  *ret_normal  = normal;
  std::cout<<"  Number 4."<<std::endl;
  return ret_normal;
}

NormalPtr findPointNormal2(const int x_idx, const int y_idx,
							NormalEstimationPtr normal_estimator) {

  std::cout<<"  Number 1."<<std::endl;
  int pt_idx = x_idx*normal_estimator->getInputCloud()->width + y_idx;
  pcl::Normal normal;
  std::cout<<"  Number 2. x:"<<x_idx<<" y:"<<y_idx<<" p:"<<pt_idx<<std::endl;
  std::cout<<"  Organized? "<<normal_estimator->getInputCloud()->isOrganized()<<std::endl;
  normal_estimator->computePointNormal(x_idx, y_idx, pt_idx, normal);
  std::cout<<"  Number 3."<<std::endl;
  NormalPtr ret_normal(new pcl::Normal);
  *ret_normal  = normal;
  std::cout<<"  Number 4."<<std::endl;
  return ret_normal;
}


/*
  Initializing values for the box filter.
  Taken from preprocessor node's initTable.
*/
void initBoxFilter (ColorCloudPtr cloud) {  
  boxProp.m_corners = getTableCornersRansac(cloud);

  Vector3f xax = boxProp.m_corners.row(1) - boxProp.m_corners.row(0);
  xax.normalize();
  Vector3f yax = boxProp.m_corners.row(3) - boxProp.m_corners.row(0);
  yax.normalize();
  Vector3f zax = xax.cross(yax);

  Matrix3f m_axes;

  
  float zsgn = (zax(2) < 0) ? 1 : -1;
  xax *= zsgn;
  zax *= zsgn; // so z axis points up

  m_axes.col(0) = xax;
  m_axes.col(1) = yax;
  m_axes.col(2) = zax;
  
  MatrixXf rotCorners = boxProp.m_corners * m_axes;
  
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
    (btVector3(boxProp.m_corners(0,0), 
	       boxProp.m_corners(0,1), 
	       boxProp.m_corners(0,2)));

  std::vector<Vector3f> corners;
  for (int i=0; i<4; ++i)
    corners.push_back(boxProp.m_corners.row(i).transpose());

  ColorCloudPtr newCloud (new ColorCloud);
  *newCloud = *cloud_pcl;

  boxProp.m_foundPC = findNearestPoints(  newCloud, &corners,
		  	  	  	  	  	  	  	  	  &boxProp.m_pcPoints,
										  &boxProp.m_imageInds );

  if (LocalConfig::debugging) {
    for (int i=0; i<4; ++i)
      std::cout<<"Corner "<<i<<": "<<corners[i]<<std::endl;
    std::cout<<"Truth: "<< boxProp.m_foundPC<< std::endl;
  }

  boxProp.m_init = true;
}

/*
  Filter cascader to filter incoming point clouds.
*/
filter_cascader cascader;

/*
  Function to append filters to a cascader.
*/
void createFilter (filter_cascader &cascader) {

  cascader.removeAll();

  //Downsampler
  if (LocalConfig::downsample) {
    boost::shared_ptr<downsample_wrapper> 
      downsampler (new downsample_wrapper());
    downsampler->setSize(LocalConfig::downsample);
    cascader.appendFilter(downsampler);
  }

  //Oriented Box Filter
  //Maybe add another orientedBoxFilter for the foam block.
  if (boxProp.m_init) {
    boost::shared_ptr<orientedBoxFilter_wrapper> 
      oBoxFilter (new orientedBoxFilter_wrapper());
    
    oBoxFilter->setOrigin(toEigenMatrix(boxProp.m_trans.getBasis()));
    oBoxFilter->setMins(boxProp.m_mins);
    oBoxFilter->setMaxes(boxProp.m_maxes);
    
    cascader.appendFilter(oBoxFilter);
  }

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
  Update function in case there is a change in orientation of PR2.
  Uninitializes properties of the boxFilter. Changes the cascader
  back to not having a boxFilter.
*/
void update() {
  boxProp.m_init = false;
  boxProp.m_foundPC = false;
  createFilter(cascader);
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
    else {
      initBoxFilter (cloud_color);
    }
    createFilter(cascader);
 
  } 
  pending = true;
}

/*
  Service call back to return corners.
*/
bool cornerCallback(filter_cloud_color::Corners::Request &req,
		    filter_cloud_color::Corners::Response &resp) {
  if (!boxProp.m_init || !boxProp.m_foundPC)
    return false;
  else {
    for (int i=0; i<4; ++i) {
      Vector3f boxCorner = boxProp.m_corners.row(i).transpose();
      resp.boxCorners[i].x = boxCorner[0];
      resp.boxCorners[i].y = boxCorner[1];
      resp.boxCorners[i].z = boxCorner[2];
      
      Vector3f pcCorner = boxProp.m_pcPoints[i];
      resp.pcCorners[i].x = pcCorner[0];
      resp.pcCorners[i].y = pcCorner[1];
      resp.pcCorners[i].z = pcCorner[2];
      
      Vector2f imageInd = boxProp.m_imageInds[i];
      resp.imageCorners[i].row = imageInd[0];
      resp.imageCorners[i].col = imageInd[1];
      
    }
  }
  return true;
}

/*
  Service call back to return cut specified by index.
  Useful but not being used.
*/
bool cutCallback(filter_cloud_color::Cut::Request &req,
		    filter_cloud_color::Cut::Response &resp) {

  if (req.index > cuts.size() || req.index < 1)
    return false;

  ColorCloudPtr cut_pcl = showCut (cloud_pcl_filtered, req.index);
  sensor_msgs::PointCloud2 cut_msg;

  toROSMsg (*cut_pcl, cut_msg);
  resp.cut = cut_msg;

  return true;
}

/*
  Service callback to return approximate line through cut by index.
*/
bool cutLineCallback(filter_cloud_color::PointDir::Request &req,
		    		 filter_cloud_color::PointDir::Response &resp) {

  if (req.index > cuts.size() || req.index < 1 || !cloud_pcl_filtered)
    return false;

  ColorCloudPtr cut_pcl = showCut (cloud_pcl_filtered, req.index);

  std::vector<float> lineCoeffs = getLineCoeffsRansac(cut_pcl);

  resp.point.x = lineCoeffs[0];
  resp.point.y = lineCoeffs[1];
  resp.point.z = lineCoeffs[2];
  resp.dir.x   = lineCoeffs[3];
  resp.dir.y   = lineCoeffs[4];
  resp.dir.z   = lineCoeffs[5];

  return true;
}

/*
  Service callback to return the midpoint of and surface normal at
  the hole given by the index.
*/
bool holeNormalCallback(filter_cloud_color::PointDir::Request &req,
		    		 	filter_cloud_color::PointDir::Response &resp) {

  if (req.index > holes.size() || req.index < 1 || !cloud_pcl_filtered)
    return false;

  ColorCloudPtr saved_pcl (new ColorCloud());
  *saved_pcl = *cloud_pcl;

  ColorCloudPtr hole_pcl = showHole (cloud_pcl_filtered, req.index);

  std::cout<<"Number 1."<<std::endl;
  Vector3f avgPoint = findAveragePoint (hole_pcl);

  vector<Vector3f> inPoints;
  inPoints.push_back(avgPoint);
  vector<Vector3f> nearestPoints;
  vector<Vector2f> imageInds;

  std::cout<<"Number 2."<<std::endl;
  *saved_pcl = *cloud_pcl;
  bool found = findNearestPoints (saved_pcl, &inPoints, &nearestPoints, &imageInds);
  std::cout<<"Number 3."<<std::endl;
  if (!found) return found;

  // Initialize normal-estimation structure
  NormalEstimationPtr normalEstimator(new NormalEstimation);
  normalEstimator->setNormalEstimationMethod (normalEstimator->AVERAGE_3D_GRADIENT);
  normalEstimator->setMaxDepthChangeFactor(0.02f);
  normalEstimator->setNormalSmoothingSize(0.5f);
  normalEstimator->setInputCloud(saved_pcl);

  std::cout<<"Number 4."<<std::endl;
  NormalPtr holeNormal = findPointNormal((int) imageInds[0][0], (int) imageInds[0][1], normalEstimator);
  std::cout<<"Number 5."<<std::endl;

  resp.point.x = avgPoint[0];
  resp.point.y = avgPoint[1];
  resp.point.z = avgPoint[2];
  resp.dir.x   = holeNormal->normal_x;
  resp.dir.y   = holeNormal->normal_y;
  resp.dir.z   = holeNormal->normal_z;
  std::cout<<"Number 6."<<std::endl;

  return true;
}

/*
  Hardcoded test for cuts, holes and needle.
*/
void testHolesCuts () {
  Cut::Ptr cut1 (new Cut());
  cut1->_H = 173; cut1->_S = 213; cut1->_V = 187;
  cut1->_Hstd = 6; cut1->_Sstd = 44; cut1->_Vstd = 50;

  Cut::Ptr cut2 (new Cut());
  cut2->_H = 114; cut2->_S = 150; cut2->_V = 202;
  cut2->_Hstd = 10; cut2->_Sstd = 36; cut2->_Vstd = 50;

  cuts.push_back(cut1);
  cuts.push_back(cut2);

  Hole::Ptr hole1 (new Hole());
  hole1->_H = 60; hole1->_S=150; hole1->_V = 150;
  hole1->_Hstd = 15; hole1->_Sstd=50; hole1->_Vstd = 100;

  holes.push_back(hole1);

  needle->_H = 70; needle->_S = 63; needle->_V = 198;
  needle->_Hstd = 10; needle->_Sstd = 41; needle->_Vstd = 44;
}

/*
 * Function to get the data for the holes/cuts/needle from GUI.
 * Block code till data is available. Assume data will be available at some point.
 * TODO: Specify ordering of cuts/holes so that their indices mean something.
 */
void getGUIData (ros::ServiceClient * infoClient) {

	while (true) {
		surgical_msgs::HoleCutInfo hcInfo;
		if (infoClient->call(hcInfo)) {
			for (int i = 0; i < hcInfo.response.holeH.size(); ++i) {
				Hole::Ptr hole =
				  make_hole ( hcInfo.response.holeH[i],
							  hcInfo.response.holeS[i],
							  hcInfo.response.holeV[i],
							  hcInfo.response.holeH_std[i],
							  hcInfo.response.holeS_std[i],
							  hcInfo.response.holeV_std[i]);
				
				holes.push_back(hole);
			}
			
			for (int i = 0; i < hcInfo.response.cutH.size(); ++i) {
				Cut::Ptr cut =
					make_cut ( hcInfo.response.cutH[i],
							   hcInfo.response.cutS[i],
							   hcInfo.response.cutV[i],
							   hcInfo.response.cutH_std[i],
							   hcInfo.response.cutS_std[i],
							   hcInfo.response.cutV_std[i]);
				
				cuts.push_back(cut);
			}
			break;
		}
		else {
		    ros::spinOnce();
		    sleep(0.3);
		    if (!ros::ok()) 
		      throw std::runtime_error
		      	  ("caught signal while waiting for hole/cut data");
		}
	}
	if(LocalConfig::debugging) 
		std::cout<<"Holes size: "<<holes.size()<<" and Cuts size: "<<cuts.size()<<std::endl;
}


int main (int argc, char* argv[]) {
  Parser parser;
  parser.addGroup(LocalConfig());
  parser.read(argc, argv);

  ros::init(argc, argv, "find_color_points");

  ros::NodeHandle nh;
  ros::Subscriber pc_sub = 
    nh.subscribe(LocalConfig::pcTopic, 1, &callback);

  ros::ServiceServer cornersService = 
    nh.advertiseService("getCorners", cornerCallback);

  //Useful but not used.
  //ros::ServiceServer cutService =
  // nh.advertiseService("getCut", cutCallback);
  ros::ServiceServer cutLineService =
     nh.advertiseService("getCutLine", cutLineCallback);
  ros::ServiceServer holeNormalService =
     nh.advertiseService("getHoleNormal", holeNormalCallback);

  ros::ServiceClient infoClient =
    nh.serviceClient<surgical_msgs::HoleCutInfo>("getGUIData");

  if(LocalConfig::debugging) 
    std::cout<<"After defining subscriber, services and clients."<<std::endl;

  //Test:
  testHolesCuts();
  //getGUIData(&infoClient);

  if (LocalConfig::debugging) {
    std::cout<<"Size of holes: "<<holes.size()<<std::endl;
    std::cout<<"Size of cuts: "<<cuts.size()<<std::endl;
  }

  vector<int> holeInds = extractInds (LocalConfig::holes);
  vector<int> cutInds = extractInds (LocalConfig::cuts);

  if (LocalConfig::debugging) {   
    for (int i = 0; i < holeInds.size(); i++)
      std::cout<<"holeInds "<<i<<": "<<holeInds[i]<<std::endl;
    for (int i = 0; i < cutInds.size(); i++)
      std::cout<<"cutInds "<<i<<": "<<cutInds[i]<<std::endl;
  }

  while (!pending) {
    ros::spinOnce();
    sleep(.001);
    if (!ros::ok()) 
      throw std::runtime_error
	("caught signal while waiting for first message");
  }

  if(LocalConfig::debugging) 
    std::cout<<"After first message."<<std::endl;

  boost::shared_ptr<pcl::visualization::CloudViewer> viewer;

  if (LocalConfig::display)
	  viewer.reset(new pcl::visualization::CloudViewer ("Visualizer"));

  
  while (ros::ok()) {
    cascader.filter(cloud_pcl, cloud_pcl_filtered);
    
    ColorCloudPtr surgicalUnits_cloud (new ColorCloud);

    surgicalUnits_cloud = extractSurgicalUnits (cloud_pcl_filtered,
						&holeInds,
						&cutInds,
						LocalConfig::needle);

    if (LocalConfig::display) {
    	if (LocalConfig::debugging) {
    		viewer->showCloud (cloud_pcl_filtered);
    	} else {
    		viewer->showCloud (surgicalUnits_cloud);
    	}
    }

    pending = false;
    while (ros::ok() && !pending) {
      sleep(.01);
      ros::spinOnce();
    }
  }
}
