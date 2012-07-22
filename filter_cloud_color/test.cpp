#include <pcl/io/io.h>
#include <pcl/point_types.h>

#include "filter_color.h"

#include<stdio.h>                                                                                                                                 
#include<stdlib.h>

void identity(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,
	      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out) {
  *cloud_out = *cloud_in;
}


int main(int argc, char** argv) {
  std::string bag_file = "/home/ankush/sandbox/bulletsim/bagfiles/folding_data_new.bag";
  std::string bag_out = "/home/ankush/sandbox/bulletsim/bagfiles/folding_data_filtered.bag";
  std::string cloud_topic = "/drop/points";

  filter_point_cloud_bagfile(*identity, bag_file,
			     bag_out, cloud_topic);

  return 0;
}
