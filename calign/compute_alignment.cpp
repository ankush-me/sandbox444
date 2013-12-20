#include "compute_alignment.h"

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
  MyPointRepresentation ()
  {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray (const PointNormalT &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};

Eigen::Matrix4f compute_alignment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr target, pcl::PointCloud<pcl::PointXYZRGB>::Ptr input) {
  // /*
  // Currently computes the alignment transformation betewen two point clouds
  // using PCL's implementation of ICP. Will implement custom thing later.
  // */
  PointCloudWithNormals::Ptr points_with_normals_input (new PointCloudWithNormals);
  PointCloudWithNormals::Ptr points_with_normals_target (new PointCloudWithNormals);

  pcl::NormalEstimation<PointT, PointNormalT> norm_est;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  norm_est.setSearchMethod (tree);
  norm_est.setKSearch (30);
  
  norm_est.setInputCloud (input);
  norm_est.compute (*points_with_normals_input);
  pcl::copyPointCloud (*input, *points_with_normals_input);

  norm_est.setInputCloud (target);
  norm_est.compute (*points_with_normals_target);
  pcl::copyPointCloud (*target, *points_with_normals_target);

  MyPointRepresentation point_representation;
  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues (alpha);

  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> icp;
  icp.setTransformationEpsilon (1e-6);
  // Set the maximum distance between two correspondences (input<->target) to 10cm
  // Note: adjust this based on the size of your datasets
  icp.setMaxCorrespondenceDistance (0.1);  
  // Set the point representation
  icp.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

  icp.setInputCloud (points_with_normals_input);
  icp.setInputTarget (points_with_normals_target);

  icp.align(*points_with_normals_input);


  // pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
  // // icp.setMaximumIterations(20);
  // icp.setMaxCorrespondenceDistance(0.05);

  // icp.setInputTarget(target);
  // icp.setInputCloud(input);
  // icp.align(*input);

  return icp.getFinalTransformation();
}
