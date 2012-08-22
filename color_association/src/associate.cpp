/** Author: Ankush Gupta
    Date  : 21st August, 2012. */

#include <ros/ros.h>

#include <surgical_msgs/InitInfo.h>
#include <surgical_msgs/Hole.h>
#include <surgical_msgs/Cut.h>
#include <geometry_msgs/Point.h>

#include <vector.h>

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <opencv2/core/core.hpp>


/** Structure to store HSV information. */
struct HSVInfo {
  /** The HSV values. */
  int h,s,v;
  
  /** The standard deviations of the H,S,V, values. */
  float h_std, s_std, v_std;

  typedef boost::shared_ptr<HSV> Ptr;

  /* Constructor. */
  HSV() {
    h = s = v = 0;
    h_std = s_std = v_std = 0.0;
  }

  /* Copy constructor. */ 
  HSV(const HSV& o) {
    h = o.h; s = o.s; v = o.v;
    h_std = o.h_std; s_std = o.s_std; v_std = o.v_std;
  }
};


/** Neighbor search : Let the INIT_PT fall down (increase Z coordinate)
    until it finds a point in a disk of radius SEARCH_RADIUS (normal along z) 
    and +- Z_DELTA. Uses the input k-d tree.
   
    Modifies the INIT_PT. The distance the point falls is STEP_DIST per iteration.
    Returns the index of the point found. 
    
    Returns -1 if after falling for Z_MAX it still cannot find a point.*/
int fall_and_lookaround(pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdTree,
			pcl::PointXYZ* init_pt, float search_radius,
			float z_delta, float z_max, float step_dist=0.01) {
  float z_init = init_pt->z;
  
  std::vector<int> neighbors;
  std::vector<float> sqr_dists;

  pcl::PointXYZRGB pt;
  pt.x = init_pt->x; pt.y = init_pt->y; pt.z = init_pt->z;
    
  while(pt.z <= z_init + z_max) {
    int pts_found = kdTree->radiusSearch(pt, search_radius, neighbors, sqr_dists, 1);
    if (pts_found) {
      pcl::PointXYZRGB neighbor = kdTree->getInputCloud()->at(neighbors.at(0));
      if (fabs(pt.z - neighbor.z) <= z_delta)
	return neighbors.at(0);
    }
    pt.z += step_dist;
  }
  return -1;
}



/** Returns the representative HSV for each hole in HOLES.
    For each hole, It looks in a sphere of radius RADIUS [default = 1cm]
    around it and gets the HSV value of the points-found.

    Then it clusters the neighbors on the basis of hue.
    If the hue lies b/w +-HUE_THRESH, they are clustered into 1 hue group.

    Then, it takes the mean of the HSV values of the dominant hue group
    and spits it out in a vector.
    
    The mapping is maintained by the indices. 

    ORG_SEARCH : Used for nearest-neighbor searches in organized point-cloud.
    -----------  It is ASSUMED, that the HOLES match-up with the PointCloud
		 which is used to instantiate ORG_SEARCH. */
std::vector<HSVInfo::Ptr> get_HSV(pcl::search::OrganizedNeighbor<pcl::PointXYZRGB>::Ptr org_search,
				  std::vector<surgical_msgs::Hole> &holes,
				  float radius=0.01, int hue_thresh=15) {

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = org_search->getInputCloud();
  std::vector<HSVInfo::Ptr> hsv_info;

  for (int i = 0; i < holes.size(); i += 1) {
    surgical_msgs::Hole hole = holes[i];
    geometry_msgs::Point hole_position = hole.pt;

    int pts_found = kdTree->radiusSearch(pt, search_radius, neighbors, sqr_dists, 1);


    HSVInfo::Ptr hole_hsv(new HSVInfo);
    
  }
  return hsv_info;
}


Mat image, imageHSV;
image_in->copyTo(image);
cvtColor(image, imageHSV,CV_BGR2HSV);

img = imageHSV;
uchar* ptr_hsv = cvPtr2D(&img, y, x, NULL);
int hue = (int) ptr_hsv[0];
int sat = (int) ptr_hsv[1];
int value   = (int) ptr_hsv[2];




void initInfoCB(const surgical_msgs::InitInfo::ConstPtr& info) {
  ROS_INFO("Initinfo recieved.");
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pcl (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(info->cloud, *cloud_pcl);

  std::vector<surgical_msgs::Hole> holes
}

void main(int argc, char** argv) {
  ros::init(argc, argv, "associate_color");
  ros::NodeHandle nh;

  std::string init_info_topic = "/surgical_init";

  ros::Subscriber  sub;
  sub = nh.subscribe(init_info_topic, 1, initInfoCB);
  ros::spin();
}
