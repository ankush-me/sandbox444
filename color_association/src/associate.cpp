/** Author: Ankush Gupta
    Date  : 21st August, 2012. */

#include <ros/ros.h>

#include <surgical_msgs/InitInfo.h>
#include <surgical_msgs/Hole.h>
#include <surgical_msgs/Cut.h>
#include <surgical_msgs/HoleCutInfo.h>
#include <geometry_msgs/Point.h>

#include <vector>
#include <math.h>
#include <sstream>

//#include <pcl/point_types.h>
#include <pcl/search/organized.h>
#include <pcl/features/integral_image_normal.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


/** Create a shared pointer type for pcl::Normal.*/
typedef boost::shared_ptr<pcl::Normal> NormalPtr;
typedef pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> NormalEstimation;
typedef boost::shared_ptr<NormalEstimation> NormalEstimationPtr;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pcl;



/** Structure to store HSV values. */
struct HSV {
  uint8_t h,s,v;
  typedef boost::shared_ptr<HSV> Ptr;

  HSV (uint8_t _h, uint8_t _s, uint8_t _v) {
    h=_h; s=_s; v=_v;
  }

  HSV (const HSV &o) {
    h=o.h; s=o.s; v=o.v;
  }
};


/** Structure to store RGB values. */
struct RGB {
  uint8_t r,g,b;
  typedef boost::shared_ptr<RGB> Ptr;

  RGB (uint8_t _r, uint8_t _g, uint8_t _b) {
    r=_r; g=_g; b=_b;
  }

  RGB (const RGB &o) {
    r=o.r; g=o.g; b=o.b;
  }
};



/** Structure to store HSV information. */
struct HSVInfo {
  /** The HSV values. */
  uint8_t h,s,v;

  /** The standard deviations of the H,S,V, values. */
  float h_std, s_std, v_std;

  typedef boost::shared_ptr<HSVInfo> Ptr;

  /* Constructor. */
  HSVInfo() {
    h = s = v = 0;
    h_std = s_std = v_std = 0.0;
  }

  HSVInfo(uint8_t _h, uint8_t _s, uint8_t _v,
	  float _h_std, float _s_std, float _v_std) {
    h = _h; s = _s; v = _v;
    h_std = _h_std; s_std = _s_std; v_std = _v_std;
  }

  /* Copy constructor. */
  HSVInfo(const HSVInfo& o) {
    h = o.h; s = o.s; v = o.v;
    h_std = o.h_std; s_std = o.s_std; v_std = o.v_std;
  }
};

/** Converts opencv RGB values to HSV. */
HSV RGB_to_HSV(uint8_t r, uint8_t g, uint8_t b) {
  uint8_t pixel_data[] = {r,g,b};
  cv::Mat pixel(1,1,CV_8UC3, (void*) pixel_data);
  cv::Mat pixelHSV;
  cv::cvtColor(pixel,pixelHSV,CV_RGB2HSV);
  return HSV(pixelHSV.at<uint8_t>(0),
	     pixelHSV.at<uint8_t>(1),
	     pixelHSV.at<uint8_t>(2));
}

/** Converts opencv HSV values to RGB. */
RGB HSV_to_RGB(uint8_t h, uint8_t s, uint8_t v) {
  uint8_t pixel_data[] = {h,s,v};
  cv::Mat pixel(1,1,CV_8UC3, (void*) pixel_data);
  cv::Mat pixelRGB;
  cv::cvtColor(pixel,pixelRGB,CV_HSV2RGB);
  return RGB(pixelRGB.at<uint8_t>(0),
	     pixelRGB.at<uint8_t>(1),
	     pixelRGB.at<uint8_t>(2));
}


/** Returns the 'distance' b/w openCV hue values.*/
uint8_t hue_dist(uint8_t hue1, uint8_t hue2) {
  int delta = 2 * abs(((int)hue1) - ((int)hue2));
  return (uint8_t) ((( abs(360 - delta) < delta ) ? abs(360-delta) : delta) / 2);
}


/** Returns a vector of vector of point-cloud indices, grouped according
    to hue similarity.*/
std::vector<boost::shared_ptr<std::vector<int> > >
get_hue_clusters(pcl::PointXYZRGB seed, int seed_idx,
		 std::vector<int> indices,
		 pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
		 uint8_t hue_thresh) {

  std::vector<boost::shared_ptr<std::vector<int> > > clusters;
  HSV baseHSV = RGB_to_HSV(seed.r, seed.g, seed.b);
  boost::shared_ptr<std::vector<int> > base_cluster(new std::vector<int>);
  base_cluster->push_back(seed_idx);
  clusters.push_back(base_cluster);

  for (unsigned int i=0; i < indices.size(); i += 1) {
    pcl::PointXYZRGB neighbor_pt = cloud->at(indices[i]);
    HSV neighborHSV = RGB_to_HSV(neighbor_pt.r, neighbor_pt.g, neighbor_pt.b);

    /** Find the closest cluster to this neighbor pt. */
    int closest_cluster_idx = -1;
    int closest_cluster_dist = 500;
    for(unsigned int j=0; j < clusters.size(); j+=1) {
      pcl::PointXYZRGB rep_pt = cloud->at(clusters.at(j)->at(0));
      HSV repHSV = RGB_to_HSV(rep_pt.r, rep_pt.g, rep_pt.b);     
      uint8_t hueDist = hue_dist(repHSV.h, neighborHSV.h);
      if (hueDist <= hue_thresh && hueDist < closest_cluster_dist) {
	closest_cluster_dist = hueDist;
	closest_cluster_idx  = j;
      }
    }

    if (closest_cluster_idx != -1)
      clusters[closest_cluster_idx]->push_back(indices[i]);
    else {
      boost::shared_ptr<std::vector<int> > cluster(new std::vector<int>);
      cluster->push_back(indices[i]);
      clusters.push_back(cluster);
    }
  }
  return clusters;
}


/** Displays the color corresponding to the H,S,V
    values in a small window. */
void  display_HSV(uint8_t h, uint8_t s, uint8_t v) {
  RGB rgb = HSV_to_RGB(h,s,v);

  cv::Mat image = cv::Mat::zeros(100,100,CV_8UC3);
  cv::rectangle(image, cv::Point(0,0), cv::Point(100,100),
		CV_RGB(rgb.r,rgb.g,rgb.b),CV_FILLED);
  std::stringstream ss;
  ss<<"Color "<<ros::Time::now();
  cv::namedWindow(ss.str());
  cv::waitKey(100);
  cv::imshow(ss.str(), image);
  cv::waitKey(100);   
}


/** Averages the HSVINFO in the input vector.
    The output HSV values are mean of the input.
    The output standard deviations are the max of the input.
    Considers wrap-around by assuming that the points chosen do
    not vary wildly in H values (points are chosen properly).
    Basicallly a hack for red.*/
HSVInfo::Ptr average_HSV_info(std::vector<HSVInfo::Ptr> hsv_input) {
  int h_total, s_total, v_total;
  h_total = s_total = v_total = 0;

  float h_std, s_std, v_std;
  h_std = s_std = v_std = 0.0;

  bool below30 = false;
  bool above150 = false;

  for(unsigned int i = 0; i < hsv_input.size(); i+=1) {
	if (!i) {
		h_total += (int) hsv_input[i]->h;
		below30 = h_total < 30 ? true : false;
		above150 = h_total > 150 ? true : false;
	}
	else {
		if (below30 && (int) hsv_input[i]->h > 150)
			h_total += (int) hsv_input[i]->h - 180;
		else if (above150 && (int) hsv_input[i]->h < 30)
			h_total += (int) hsv_input[i]->h + 180;
		else
			h_total += (int) hsv_input[i]->h;
	}
    s_total += (int) hsv_input[i]->s;
    v_total += (int) hsv_input[i]->v;

    h_std = h_std > hsv_input[i]->h_std? h_std : hsv_input[i]->h_std;
    s_std = s_std > hsv_input[i]->s_std? s_std : hsv_input[i]->s_std;
    v_std = v_std > hsv_input[i]->v_std? v_std : hsv_input[i]->v_std;
  }
  h_total /= hsv_input.size();
  while (h_total > 180) h_total -= 180;
  while (h_total < 0) h_total += 180;
  s_total /= hsv_input.size();
  v_total /= hsv_input.size();

  return HSVInfo::Ptr(new HSVInfo((uint8_t) h_total,
				  (uint8_t) s_total,
				  (uint8_t) v_total,
				  h_std, s_std, v_std));
}


/** Returns a cummulative [average] HSV, given a list of
    INDICES of the points in the CLOUD, over which the average
    needs to be taken. */
HSVInfo::Ptr get_HSV_info(boost::shared_ptr<std::vector<int> > indices,
			  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) {
  if (indices->size()) {
    int h_total, s_total, v_total;
    h_total = s_total = v_total = 0;

    for(unsigned int i=0; i < indices->size(); i+=1) {
      pcl::PointXYZRGB pt = cloud->at(indices->at(i));
      HSV hsv = RGB_to_HSV(pt.r, pt.g, pt.b);

      h_total += 2 * ((int) hsv.h);
      s_total += hsv.s;
      v_total += hsv.v;
    }
    float h_mean1 = ((float)h_total)/indices->size();
    float v_mean = ((float)v_total)/indices->size();
    float s_mean = ((float)s_total)/indices->size();

    float h_std1, s_std, v_std;
    h_std1 = s_std = v_std = 0.0;
    for(unsigned int i=0; i<indices->size(); i+=1) {
      pcl::PointXYZRGB pt = cloud->at(indices->at(i));
      HSV hsv = RGB_to_HSV(pt.r, pt.g, pt.b);
      int h = 2 * ((int) hsv.h);
      h_std1 += (  h_mean1 - h  )*(   h_mean1 -h  );
      s_std += (s_mean - hsv.s)*(s_mean - hsv.s);
      v_std += (v_mean - hsv.v)*(v_mean - hsv.v);
    }
    h_std1 = sqrt(h_std1/indices->size());
    s_std  = sqrt(s_std/indices->size());
    v_std  = sqrt(v_std/indices->size());


    h_total = 0;
    for(unsigned int i=0; i < indices->size(); i+=1) {
      pcl::PointXYZRGB pt = cloud->at(indices->at(i));
      HSV hsv = RGB_to_HSV(pt.r, pt.g, pt.b);
      h_total += (2 * ((int) hsv.h) + 180) % 360;
    }

    float h_mean2 = ((float)h_total)/indices->size();
    float h_std2 = 0.0;
    for(unsigned int i=0; i<indices->size(); i+=1) {
      pcl::PointXYZRGB pt = cloud->at(indices->at(i));
      HSV hsv = RGB_to_HSV(pt.r, pt.g, pt.b);
      int h = (2 * ((int) hsv.h) + 180) % 360;
      h_std2 += (  h_mean2 - h  )*(   h_mean2 -h  );
    }
    h_std2 = sqrt(h_std2/indices->size());

    float h_mean, h_std;
    if (h_std1 < h_std2) {
      h_std  = h_std1;
      h_mean = h_mean1;
    } else {
      h_std  = h_std2;
      if (h_mean2 < 180)
	h_mean = h_mean2 + 180;
      else
	h_mean = h_mean2 - 180;
    }

    uint8_t mean_hue = (uint8_t) (h_mean/2.0);
    HSVInfo::Ptr hsv_info(new HSVInfo(mean_hue, (uint8_t) s_mean, (uint8_t) v_mean,
				      h_std, s_std, v_std));
    return hsv_info;
  } else {
    HSVInfo::Ptr hsv_info(new HSVInfo);
    return hsv_info;
  }
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
std::vector<HSVInfo::Ptr> get_hole_HSV(pcl::search::OrganizedNeighbor<pcl::PointXYZRGB>::Ptr org_search,
				       std::vector<surgical_msgs::Hole> holes,
				       float radius=0.01, uint8_t hue_thresh=5) {
  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud = org_search->getInputCloud();
  std::vector<HSVInfo::Ptr> hsv_info;

  for (unsigned int i = 0; i < holes.size(); i += 1) {
    surgical_msgs::Hole hole = holes[i];

    std::vector<int> neighbors;
    std::vector<float> sqr_dists;

    int pts_found = org_search->radiusSearch(cloud->at(hole.x_idx, hole.y_idx),
					     radius, neighbors, sqr_dists, 20);
    if (pts_found) { 
      std::vector<boost::shared_ptr<std::vector<int> > > hue_clusters =
	get_hue_clusters(cloud->at(hole.x_idx,hole.y_idx), (hole.x_idx*cloud->width + hole.y_idx), 
			 neighbors, cloud, hue_thresh);
      int max_cluster_size = -1;
      int max_cluster_idx  = -1;
      for(unsigned int i = 0; i < hue_clusters.size(); i += 1) {
	if (((int) hue_clusters[i]->size()) > max_cluster_size) {
	  max_cluster_size = hue_clusters[i]->size();
	  max_cluster_idx = i;
	}
      }
      HSVInfo::Ptr hole_hsv(get_HSV_info(hue_clusters[max_cluster_idx], cloud));
      hsv_info.push_back(hole_hsv);
    }
  }
  return hsv_info;
}


/** Returns a vector of HSVInfo for all the cuts in CUTS.
    ORG_SEARCH : is the organizedNeighbor data-structure to be
                 used for nearest neighbor searches in an organized
		 point-cloud.
    RADIUS     : The radius [meters] to search for neighbors in.
    HUE_THRESH : The tolerance for putting hue values in the same bin. */   
std::vector<HSVInfo::Ptr> get_cut_HSV(pcl::search::OrganizedNeighbor<pcl::PointXYZRGB>::Ptr org_search,
				      std::vector<surgical_msgs::Cut> cuts,
				      float radius=0.01, uint8_t hue_thresh=5) {
  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud = org_search->getInputCloud();
  std::vector<HSVInfo::Ptr> cuts_hsv_info;

  for (unsigned int i = 0; i < cuts.size(); i += 1) {
    surgical_msgs::Cut cut = cuts[i];
    std::vector<surgical_msgs::Hole> nodes = cut.nodes;
    std::vector<HSVInfo::Ptr> nodes_hsv_info = get_hole_HSV(org_search, nodes,
							    radius, hue_thresh);
    HSVInfo::Ptr cut_hsv_info(average_HSV_info(nodes_hsv_info));
    display_HSV(cut_hsv_info->h,cut_hsv_info->s,cut_hsv_info->v);
    cuts_hsv_info.push_back(cut_hsv_info);
    }
    return cuts_hsv_info;
}


/** Finds the surface normal around a point. 
    X_IDX, Y_IDX : The x and y indices of the point at which
                   the normal is to be found in the organized point-cloud.
    NORMAL_ESTIMATOR : A normal_estimator structure appropriately
                       initialized for finding normals.
    [see:
    http://www.pointclouds.org/documentation/tutorials/normal_estimation_using_integral_images.php ]*/
NormalPtr get_point_normal(const int x_idx, const int y_idx,
			   NormalEstimationPtr normal_estimator) {
  int pt_idx = x_idx*normal_estimator->getInputCloud()->width + y_idx;
  pcl::Normal normal;
  normal_estimator->computePointNormal(x_idx, y_idx, pt_idx, normal);

  NormalPtr ret_normal(new pcl::Normal);
  *ret_normal  = normal;
  return ret_normal;
}


/** Returns the surface normal a the HOLE. */
NormalPtr get_hole_normal(const surgical_msgs::Hole &hole,
			  NormalEstimationPtr normal_estimator) {
  return get_point_normal(hole.x_idx, hole.y_idx, normal_estimator);
}

/** Returns a vector of surface normals for each node in every cut inside CUTS.*/
std::vector<boost::shared_ptr<std::vector<NormalPtr> > >
get_cuts_normal(const std::vector<surgical_msgs::Cut> cuts,
		NormalEstimationPtr normal_estimator) {
  std::vector<boost::shared_ptr<std::vector<NormalPtr> > > normals;
  for (unsigned int i = 0; i < cuts.size(); i+= 1) {
    surgical_msgs::Cut cut = cuts[i];
    boost::shared_ptr<std::vector<NormalPtr> > cut_normals(new std::vector<NormalPtr>); 
    for(unsigned int j =0; j < cut.nodes.size(); j += 1) {
      cut_normals->push_back(get_hole_normal(cut.nodes[j], normal_estimator));
    }
    normals.push_back(cut_normals);
  }
  return normals;
}

//Global variables to store hole/cut info.
//Reset every time callback is called.
std::vector<HSVInfo::Ptr> hole_hsv_info;
std::vector<HSVInfo::Ptr> cut_hsv_info;

/** Service callback to provide info of holes and cuts. */
bool infoCallback (surgical_msgs::HoleCutInfo::Request &req,
					surgical_msgs::HoleCutInfo::Response &resp) {

	if (!hole_hsv_info.size() && !cut_hsv_info.size())
		return false;

	for(int i=0; i < hole_hsv_info.size(); i+=1) {
		resp.holeH.push_back (hole_hsv_info[i]->h);
		resp.holeS.push_back (hole_hsv_info[i]->s);
		resp.holeV.push_back (hole_hsv_info[i]->v);
		resp.holeH_std.push_back (hole_hsv_info[i]->h_std);
		resp.holeS_std.push_back (hole_hsv_info[i]->s_std);
		resp.holeV_std.push_back (hole_hsv_info[i]->v_std);
	}

	for(int i=0; i < cut_hsv_info.size(); i+=1) {
		resp.cutH.push_back (cut_hsv_info[i]->h);
		resp.cutS.push_back (cut_hsv_info[i]->s);
		resp.cutV.push_back (cut_hsv_info[i]->v);
		resp.cutH_std.push_back (cut_hsv_info[i]->h_std);
		resp.cutS_std.push_back (cut_hsv_info[i]->s_std);
		resp.cutV_std.push_back (cut_hsv_info[i]->v_std);
	}

	return true;
}

/** The callback to handle new surgical::initInfo messages.

    1. Detects hole and cut colors.
    2. Finds the local surface normal directions at the holes and the cuts. */
void initInfoCB(const surgical_msgs::InitInfo::ConstPtr& info) {
  ROS_INFO("Initinfo recieved.");

  cloud_pcl = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(info->cloud, *cloud_pcl);

  // Initialize organized neighbor structure for nearest neighbor search
  pcl::search::OrganizedNeighbor<pcl::PointXYZRGB>::Ptr
    org_search(new pcl::search::OrganizedNeighbor<pcl::PointXYZRGB>);
  org_search->setInputCloud(cloud_pcl);

  // Initialize normal-estimation structure
  NormalEstimationPtr normal_estimator(new NormalEstimation);
  normal_estimator->setNormalEstimationMethod (normal_estimator->AVERAGE_3D_GRADIENT);
  normal_estimator->setMaxDepthChangeFactor(0.02f);
  normal_estimator->setNormalSmoothingSize(10.0f);
  normal_estimator->setInputCloud(cloud_pcl);

  // Data pointed to automatically garbage-collected.
  cut_hsv_info.clear();
  hole_hsv_info.clear();

  std::vector<surgical_msgs::Hole> holes = info->holes;
  hole_hsv_info = get_hole_HSV(org_search, holes);

  std::vector<surgical_msgs::Cut> cuts = info->cuts;
  cut_hsv_info = get_cut_HSV(org_search, cuts);

  ROS_INFO("Hole info:");

  for(int i=0; i < hole_hsv_info.size(); i+=1) {
      std::stringstream ss;
      ss<<"HSV : ("<<(int)hole_hsv_info[i]->h<<","
        <<(int)hole_hsv_info[i]->s<<","
        <<(int)hole_hsv_info[i]->v<<") | std: ("
        <<(int)hole_hsv_info[i]->h_std<<","
        <<(int)hole_hsv_info[i]->s_std<<","
        <<(int)hole_hsv_info[i]->v_std<<")\n";
      ROS_INFO(ss.str().c_str());
    }

  ROS_INFO("Cut info:");

  for(int i=0; i < cut_hsv_info.size(); i+=1) {
    std::stringstream ss;
    ss<<"HSV : ("<<(int)cut_hsv_info[i]->h<<","
      <<(int)cut_hsv_info[i]->s<<","
      <<(int)cut_hsv_info[i]->v<<") | std: ("
      <<(int)cut_hsv_info[i]->h_std<<","
      <<(int)cut_hsv_info[i]->s_std<<","
      <<(int)cut_hsv_info[i]->v_std<<")";
    ROS_INFO(ss.str().c_str());
  }
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "associate_color");
  ros::NodeHandle nh;

  std::string init_info_topic = "/surgical_init";

  ros::Subscriber  sub;
  sub = nh.subscribe(init_info_topic, 1, initInfoCB);

  ros::ServiceServer infoService;
  infoService = nh.advertiseService("getGUIData", infoCallback);

  ros::spin();
}
