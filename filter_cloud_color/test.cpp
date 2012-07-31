#include <pcl/io/io.h>
#include <pcl/point_types.h>

#include "filter_wrapper.h"
#include "filter_color.h"

#include<stdio.h>
#include<stdlib.h>

int main(int argc, char** argv) {
  std::string bag_file =
    "/home/ankush/sandbox/bulletsim/bagfiles/manual_pc.bag";
  std::string bag_out =
    "/home/ankush/sandbox/bulletsim/bagfiles/manual_pc_filtered2.bag";
  std::string cloud_topic = "/drop/points";

  hueFilter_wrapper hue_filter;
  hue_filter.setMinHue(170);
  hue_filter.setMaxHue(10);
  hue_filter.setMaxSat(255);
  hue_filter.setMinSat(200);

  removeOutliers_wrapper out_remover;
  filterZ_wrapper z_filter(0.50, 2.50);  

  filter_cascader cascader;
  cascader.appendFilter(&z_filter);
  cascader.appendFilter(&hue_filter);
  cascader.appendFilter(&out_remover);
  
  filter_point_cloud_bagfile(&cascader, bag_file,
			     bag_out, cloud_topic);
  return 0;
}
