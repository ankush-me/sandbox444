/** Author: Ankush Gupta
    Date  : 21st August, 2012. */

#include <ros/ros.h>

#include <surgical_msgs/InitInfo.h>
#include <surgical_msgs/Hole.h>
#include <surgical_msgs/Cut.h>
#include <geometry_msgs/Point.h>

#include <vector>
#include <math.h>

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>


struct HSV {
  uint8_t h,s,v;

  HSV (uint8_t _h, uint8_t _s, uint8_t _v) {
    h=_h; s=_s; v=_v;
  }

  HSV (const HSV &o) {
    h=o.h; s=o.s; v=o.v;
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


HSV RGB_to_HSV(uint8_t r, uint8_t g, uint8_t b) {
  uint8_t pixel_data[] = {r,g,b};
  cv::Mat pixel(1,1,CV_8UC3, (void*) pixel_data);
  cv::Mat pixelHSV;
  cv::cvtColor(pixel,pixelHSV,CV_RGB2HSV);
  return HSV(pixelHSV.at<uint8_t>(0),
	     pixelHSV.at<uint8_t>(1),
	     pixelHSV.at<uint8_t>(2));
}


uint8_t hue_dist(uint8_t hue1, uint8_t hue2) {
  float rad1 = 2.0 * hue1/180.0 * CV_PI;
  float rad2 = 2.0 * hue2/180.0 * CV_PI;

  float ang = fabs(atan2(sin(rad1-rad2), cos(rad1-rad2)));
  return (uint8_t) (ang/CV_PI*180.0/2.0);
}


/** Returns a vector of vector of point-cloud indices, grouped according
    to hue similarity.*/
std::vector<boost::shared_ptr<std::vector<int> > > get_hue_clusters(pcl::PointXYZRGB seed, int seed_idx,
								     std::vector<int> indices,
								     pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
								     uint8_t hue_thresh) {
  std::vector<boost::shared_ptr<std::vector<int> > > clusters;
  HSV baseHSV = RGB_to_HSV(seed.r, seed.g, seed.b);
  boost::shared_ptr<std::vector<int> > base_cluster(new std::vector<int>);
  base_cluster->push_back(seed_idx);
  clusters.push_back(base_cluster);

  for (int i=0; i < indices.size(); i += 1) {
    pcl::PointXYZRGB neighbor_pt = cloud->at(indices[i]);
    HSV neighborHSV = RGB_to_HSV(neighbor_pt.r, neighbor_pt.g, neighbor_pt.b);

    /** Find the closest cluster to this neighbor pt. */
    int closest_cluster_idx = -1;
    int closest_cluster_dist = 500;
    for(int j=0; j < clusters.size(); j+=1) {
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
  std::cout<< "[get_hue_clusters] Returning "<<clusters.size()<<" clusters."<<std::endl;
  for (int c=0; c < clusters.size(); c++) {
    std::cout<<"\tcluster "<<c<<" has "<<clusters[c]->size()<<" points."<<std::endl;
    for (int d=0; d<clusters.at(c)->size(); d++) {
      std::cout<<"\t\t"<<clusters.at(c)->at(d)<<" ";
    }
    std::cout<<std::endl;
  }
  std::cout<<"\t------------"<<std::endl;
  return clusters;
}


/** Returns a cummulative [average] HSV, given a list of
    INDICES of the points in the CLOUD, over which the average
    needs to be taken. */
HSVInfo::Ptr get_HSV_info(boost::shared_ptr<std::vector<int> > indices,
			  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) {
  if (indices->size()) {
    float h_sin_total, h_cos_total;
    h_sin_total = h_cos_total =  0.0;
    int s_total, v_total;
    s_total = v_total = 0;

    for(int i=0; i < indices->size(); i+=1) {
      pcl::PointXYZRGB pt = cloud->at(indices->at(i));
      HSV hsv = RGB_to_HSV(pt.r, pt.g, pt.b);
      h_sin_total += sin(CV_PI*2*hsv.h/180.0);
      h_cos_total += cos(CV_PI*2*hsv.h/180.0);
      s_total += hsv.s;
      v_total += hsv.v;
    }
    float h_mean = atan2(h_sin_total, h_cos_total);
    float v_mean = ((float)v_total)/indices->size();
    float s_mean = ((float)s_total)/indices->size();

    float h_std, s_std, v_std;
    h_std = s_std = v_std = 0.0;
    for(int i=0; i<indices->size(); i+=1) {
      pcl::PointXYZRGB pt = cloud->at(indices->at(i));
      HSV hsv = RGB_to_HSV(pt.r, pt.g, pt.b);
      float h = CV_PI*2*hsv.h/180.0;
      h_std += (h_mean - h)*(h_mean -h);
      s_std += (s_mean - hsv.s)*(s_mean - hsv.s);
      v_std += (v_mean - hsv.v)*(v_mean - hsv.v);
    }
    h_std = sqrt(h_std/indices->size());
    s_std = sqrt(s_std/indices->size());
    v_std = sqrt(v_std/indices->size());

    uint8_t mean_hue = (uint8_t) (h_mean/CV_PI*180.0/2.0);
    float std_hue = h_std/CV_PI*180.0/2.0;
    HSVInfo::Ptr hsv_info(new HSVInfo(mean_hue, s_mean, v_mean,
				      std_hue, s_std, v_std));
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
std::vector<HSVInfo::Ptr> get_HSV(pcl::search::OrganizedNeighbor<pcl::PointXYZRGB>::Ptr org_search,
				  std::vector<surgical_msgs::Hole> holes,
				  float radius=0.01, uint8_t hue_thresh=8) {
  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud = org_search->getInputCloud();
  std::vector<HSVInfo::Ptr> hsv_info;

  for (int i = 0; i < holes.size(); i += 1) {
    surgical_msgs::Hole hole = holes[i];

    std::vector<int> neighbors;
    std::vector<float> sqr_dists;

    int pts_found = org_search->radiusSearch(cloud->at(hole.x_idx, hole.y_idx),
					     radius, neighbors, sqr_dists, 20);
    if (pts_found) { 
      std::vector<boost::shared_ptr<std::vector<int> > > hue_clusters =
	get_hue_clusters(cloud->at(hole.x_idx,hole.y_idx), (hole.x_idx*cloud->width + hole.y_idx), 
			 neighbors, cloud, hue_thresh);

      int max_cluster_size = 0;
      int max_cluster_idx = 0;
      for(int i = 0; i < hue_clusters.size(); i += 1) {
	if (hue_clusters[i]->size() > max_cluster_size) {
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


void initInfoCB(const surgical_msgs::InitInfo::ConstPtr& info) {
  ROS_INFO("Initinfo recieved.");
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pcl (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(info->cloud, *cloud_pcl);
  
  pcl::search::OrganizedNeighbor<pcl::PointXYZRGB>::Ptr
    org_search(new pcl::search::OrganizedNeighbor<pcl::PointXYZRGB>);
  org_search->setInputCloud(cloud_pcl);
  std::vector<surgical_msgs::Hole> holes = info->holes;
  get_HSV(org_search, holes);
  ROS_INFO("\treturned from getHSV");
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "associate_color");
  ros::NodeHandle nh;

  std::string init_info_topic = "/surgical_init";

  ros::Subscriber  sub;
  sub = nh.subscribe(init_info_topic, 1, initInfoCB);
  ros::spin();
}
