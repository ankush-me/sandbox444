#include "segment_object.h"

void segment_object(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

  // Find plane
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01);
  seg.setInputCloud(cloud);
  pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices);
  seg.segment(*plane_inliers, *coefficients);

  // Store plane
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(plane_inliers);
  extract.setNegative(false);
  extract.filter(*plane);

  // Remove plane
  extract.setInputCloud(cloud);
  extract.setIndices(plane_inliers);
  extract.setNegative(true);
  extract.filter(*cloud);

  // Compute convex hull of plane
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::ConvexHull<pcl::PointXYZRGB> c_hull;
  c_hull.setInputCloud(plane);
  c_hull.setDimension(2);
  c_hull.reconstruct(*cloud_hull);

  // Find points that are in the polygonal prism(objects)
  pcl::ExtractPolygonalPrismData<pcl::PointXYZRGB> ex;
  ex.setInputCloud(cloud);
  ex.setInputPlanarHull(cloud_hull);
  pcl::PointIndices::Ptr object_inliers(new pcl::PointIndices);
  ex.segment(*object_inliers);

  // Remove non-objects
  extract.setInputCloud(cloud);
  extract.setIndices(object_inliers);
  extract.setNegative(false);
  extract.filter(*cloud);

  return;

  // Find clusters
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance(0.02); // 2cm
  ec.setMinClusterSize(100);
  ec.setMaxClusterSize(25000);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  // Find largest cluster
  int largest_cluster_size = 0;
  int largest_idx = 0; 
  for (int i=0; i < cluster_indices.size(); i++) {
    int cluster_size = cluster_indices[i].indices.size();
    if (cluster_size > largest_cluster_size) {
      largest_cluster_size = cluster_size;
      largest_idx = i;
    }
  }

  // Keep largest cluster
  pcl::PointIndices::Ptr largest_cluster(new pcl::PointIndices);
  largest_cluster->indices = cluster_indices[largest_idx].indices;
  extract.setInputCloud(cloud);
  extract.setIndices(largest_cluster);
  extract.setNegative(false);
  extract.filter(*cloud);

}
