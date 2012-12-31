/** Author: Ankush Gupta
    Date  : 31st December, 2012. */

/** Description : A simple node which listens to a topic for an
		              organized point cloud, estimates the normals and 
                  displays them in the viewer.*/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <vector>
#include <math.h>
#include <sstream>
#include <pcl/search/organized.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <boost/thread.hpp>


/** Create a shared pointer type for pcl::Normal.*/
typedef boost::shared_ptr<pcl::Normal> NormalPtr;
typedef pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> NormalEstimation;
typedef boost::shared_ptr<NormalEstimation> NormalEstimationPtr;


/** Globals. */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pcl;
pcl::visualization::PCLVisualizer viewer("PCL Viewer");
boost::mutex updateModelMutex;


void cloudCB(const sensor_msgs::PointCloud2::ConstPtr& cloud_data) {
  ROS_INFO("DisplayCloud: Data recieved.");
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pcl (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*cloud_data, *cloud_pcl);
	
	// estimate normals
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

  NormalEstimationPtr ne(new NormalEstimation);
	ne->setNormalEstimationMethod (ne->AVERAGE_3D_GRADIENT);
	ne->setMaxDepthChangeFactor(0.02f);
	ne->setNormalSmoothingSize(10.0f);
	ne->setInputCloud(cloud_pcl);
	ne->compute(*normals);

	// visualize normals
	boost::mutex::scoped_lock updateLock(updateModelMutex);
	viewer.removeAllPointClouds();
	viewer.addPointCloud(cloud_pcl, "scene");
	viewer.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(cloud_pcl, normals);
	updateLock.unlock();
}


/** Just spins the viewer.
		For thread safety, it uses a mutex to access the viewer */
void spin_viewer() {
	while (!viewer.wasStopped () && ros::ok()) {
		boost::mutex::scoped_lock updateLock(updateModelMutex);
		viewer.spinOnce();
		updateLock.unlock();
	}
}



int main(int argc, char** argv) {
  ros::init(argc, argv, "cloud_normal");
  ros::NodeHandle nh;

	std::string cloud_topic = "/camera/depth_registered/points";
  ros::Subscriber  sub;
  sub = nh.subscribe(cloud_topic, 1, cloudCB);

	boost::thread workerThread(spin_viewer);

	ros::spin();
	workerThread.join();
	return 0;
}
