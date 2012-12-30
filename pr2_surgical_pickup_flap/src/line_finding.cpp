#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include "line_finding.h"

typedef ColorPoint PointT;

std::vector<float> getLineCoeffsRansac(ColorCloudPtr cloud) {

  pcl::ModelCoefficients::Ptr coeffs (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  pcl::SACSegmentation<ColorPoint> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_LINE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(800);
  seg.setDistanceThreshold(.0025);
  seg.setInputCloud(cloud);
  seg.segment(*inliers,*coeffs); 

  return coeffs->values;
}
