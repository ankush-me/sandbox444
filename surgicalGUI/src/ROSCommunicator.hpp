#ifndef _SURGICAL_ROS_COMM_
#define _SURGICAL_ROS_COMM_

#include "Hole.hpp"
#include "Cut.hpp"
#include <geometry_msgs/Point.h>
#include <surgical_msgs/InitInfo.h>
#include <pcl/point_types.h>

class ROSCommunicator {
private:
  /** Node handle of the ros node, this should attach to. */
  ros::NodeHandle* _nh_ptr;

  /** Name of the topic on which the info is published. */
  std::string _out_topic;

  /** The publisher. */
  ros::Publisher _info_pub;
  
  /** Converts pcl::PointXYZ to geomtry_msgs::Point. */
  geometry_msgs::Point toROSPoint(pcl::PointXYZ pt) {
    geometry_msgs::Point geom_pt;
    geom_pt.x = pt.x; geom_pt.y = pt.y; geom_pt.z = pt.z;
    return geom_pt;
  }

public:
  ROSCommunicator(ros::NodeHandle * nh_ptr,
		  std::string topic="surgical_init"): _nh_ptr(nh_ptr),
						      _out_topic(topic) {
    _info_pub = _nh_ptr->advertise<surgical_msgs::InitInfo>(_out_topic, 10);
  }

  /** Publishes the info. */
  void publish(std::list<Hole::Ptr> &holes, std::list<Cut::Ptr> &cuts) {
    surgical_msgs::InitInfo info;

    std::list<Hole::Ptr>::iterator holes_iter;
    for (holes_iter = holes.begin(); holes_iter != holes.end(); holes_iter++) {
      info.holes.push_back( toROSPoint((*holes_iter)->get_position()) );
    }

    std::list<Cut::Ptr>::iterator cuts_iter;
    for (cuts_iter = cuts.begin(); cuts_iter != cuts.end(); cuts_iter++) {
      surgical_msgs::Cut cut;
      std::vector<Hole::Ptr> holes = (*cuts_iter)->get_nodes();
      for (int i = 0; i < holes.size(); i += 1) {
	cut.nodes.push_back(toROSPoint( holes[i]->get_position()) );
      }
      info.cuts.push_back(cut);
    }
    _info_pub.publish(info);
    ROS_INFO("Surgic@lGUI.ROSCommunicator : Initial holes and cuts published.");
  }
};

#endif
