#include <pcl/io/io.h>
#include <pcl/point_types.h>

#include "filter_wrapper.h"
#include "filter_color.h"

#include<stdio.h>
#include<stdlib.h>

int main(int argc, char** argv) {
  std::string bag_file =
    "/home/ankush/sandbox/bulletsim/bagfiles/manual_trial4_filtered.bag";
  std::string bag_out =
    "/home/ankush/sandbox/bulletsim/bagfiles/manual_trial4_filtered1.bag";
  std::string cloud_topic = "/kinect1/depth_registered/points";

  /* hueFilter_wrapper hue_filter;
  hue_filter.setMinHue(170);
  hue_filter.setMaxHue(10);
  hue_filter.setMaxSat(255);
  hue_filter.setMinSat(200);
  hue_filter.setMinVal(89);

  removeOutliers_wrapper out_remover;
  filterZ_wrapper z_filter(0.50, 2.50);  

  filter_cascader cascader;
  cascader.appendFilter(&z_filter);
  cascader.appendFilter(&hue_filter);
  cascader.appendFilter(&out_remover);*/

  downsample_wrapper downsampler(0.02);
  filter_point_cloud_bagfile(&downsampler, bag_file,
			     bag_out, cloud_topic);
  return 0;
}
