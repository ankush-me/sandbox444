/** Simple class to visualize point-clouds in a new thread.
    Author: Ankush Gupta */

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include <boost/thread.hpp>

class CloudViewer {
private: 

  boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer; 
  bool _is_init, _updated;
  boost::thread _thread;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud;
  

  void spin();

  /** Initializes the point-cloud viewer.
      [Should not be explicitly called by the user].*/
  void initialize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

public:

  /** Call this to update the point-cloud displayed. */
  void view(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
 
  CloudViewer();
  ~CloudViewer();
};
