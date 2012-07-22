#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <rosbag/view.h>
#include <rosbag/bag.h>
#include <rosbag/query.h>

#include <boost/foreach.hpp>
#include <boost/random.hpp>
#include <boost/generator_iterator.hpp>
#include<boost/unordered_set.hpp>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include<unistd.h>
#include<stdio.h>
#include<stdlib.h>
#include<time.h>


std::vector<pcl::PointXYZ> *SEEDS;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr CLOUD;


/** Returns N UNIQUE points sampled unioformly at random from CLOUD
    and stores in SAMPLES.

    Uniform integers reference:
    http://stackoverflow.com/questions/2254909/boost-random-number-generator */
template<typename vector_type>
void get_random_samples(std::vector<vector_type>* cloud,
			std::vector<vector_type>* samples, int N) {

  typedef boost::mt19937 RNGType;
  RNGType rng;
  boost::uniform_int<> distribution(0, cloud->size());
  boost::variate_generator< RNGType, boost::uniform_int<> >
    sample(rng, distribution);
  
  boost::unordered_set<int> hash_set;
  for (int i = 0; i < N; i++) {
    int rand_num = sample();
    if (hash_set.find(rand_num) == hash_set.end()) {
      hash_set.insert(rand_num);
      samples->push_back(cloud->at(rand_num));
    } else {
      i--;
    }
  }
}


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


/** Returns one neighbor (point index) each of SEEDS in the CLOUD.
    Stores in NEIGHBORS.*/
void get_neighbors_in_cloud(std::vector<pcl::PointXYZ> *seeds,
			    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
			    std::vector<int> *neighbors){

  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(cloud);

  for(int i = 0; i < seeds->size(); i++) {
    int neighbor_index = fall_and_lookaround(tree, &seeds->at(i), 0.02, 0.005, 0.2);
    if (neighbor_index != -1) {
      neighbors->push_back(neighbor_index);
      std::cout<<neighbor_index<<", ";
    }
  }
  std::cout<<std::endl;
}


/** Adds spheres to the visualizer.*/
void viewerOneOff (pcl::visualization::PCLVisualizer& viz) {

  pcl::PointXYZ right_hand(0.512, 0.142, 1.331);
  pcl::PointXYZ left_hand(0.500, 0.521, 1.268);
  
  std::vector<int> final_seeds;
  get_neighbors_in_cloud(SEEDS, CLOUD, &final_seeds);

  for(int i = 0; i < final_seeds.size(); i++) {
    if (final_seeds[i] >= 0) {
      char sphere_id[5];
      sprintf(sphere_id,"%d",i);
      viz.addSphere(CLOUD->at(final_seeds[i]), 0.02, std::string(sphere_id), 0);
    }
  }

}


/** Generates N random points in a plane: with a given Z and
    (x,y) \in [(x_min, y_min) : (x_max, y_max)] and stores them in OUT_POINTS.*/
void get_random_points(int N, std::vector<pcl::PointXYZ> *out_points,
		       float x_min, float x_max, float y_min, float y_max,
		       float z_all) {
  float x,y;
  pcl::PointXYZ pt;
  
  for (int i = 0; i < N; i += 1) {
    x = x_min + (float)rand()/((float)RAND_MAX/(x_max-x_min));
    y = y_min + (float)rand()/((float)RAND_MAX/(y_max-y_min));
    pt = pcl::PointXYZ(x,y,z_all);
    out_points->push_back(pt);
  }
}



class visualizer {
  /** A simple class to help visualize the PCL point clouds.*/
public: 
    /** A PCL viewer instance.*/
  pcl::visualization::CloudViewer viewer;

  /** Initialize the viewer with the name as WINDOWNAME.*/
  visualizer(std::string *windowName =  new std::string("visualizer")) : viewer(*windowName) {
    //viewer.runOnVisualizationThreadOnce(viewerOneOff);
  }

  /** Display the point_cloud CLOUD till the user exits.
      Holds the thread busy.*/
  void display(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) {
    viewer.showCloud(cloud);
  }

  /** Returns TRUE iff the gui of the viewer was stopped.*/
  bool wasStopped() {return viewer.wasStopped();}
};


void display_bagfile(std::string filename, std::string cloud_topic,
		     std::vector<pcl::PointXYZ> *seeds) {
  rosbag::Bag bag;
  bag.open(filename, rosbag::bagmode::Read);
  rosbag::View view(bag, rosbag::TopicQuery(cloud_topic));

  visualizer viz;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pcl (new pcl::PointCloud<pcl::PointXYZRGB>);

  BOOST_FOREACH(rosbag::MessageInstance const msg, view) {
    sensor_msgs::PointCloud2ConstPtr cloud_ros = msg.instantiate<sensor_msgs::PointCloud2>();
    if (cloud_ros != NULL) {
      pcl::fromROSMsg(*cloud_ros, *cloud_pcl);
      viz.display(cloud_pcl);

      CLOUD = cloud_pcl;
      viz.viewer.runOnVisualizationThreadOnce(viewerOneOff);

      std::cout<< "Next? : "<<std::endl;
      getchar();
      //sleep(0.5);
      viz.wasStopped();
    }
  }
  bag.close();
  return;
}


int main(int argc, char** argv) {

  srand((unsigned)time(0));
  float x_min = 0.500;
  float x_max = -0.750;
  float y_min = 0.142;
  float y_max = 0.521;
  float z     = 1.300;
  std::vector<pcl::PointXYZ> rand_pts;
  get_random_points(40, &rand_pts, x_min, x_max, y_min, y_max, z);
  SEEDS = &rand_pts;

  std::string bag_file = "/home/ankush/sandbox/bulletsim/bagfiles/folding_data_new.bag";
  std::string cloud_topic = "/drop/points";

  display_bagfile(bag_file, cloud_topic, &rand_pts);

  return 0;
}
