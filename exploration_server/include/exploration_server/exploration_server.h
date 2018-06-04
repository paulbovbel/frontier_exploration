#include <actionlib/server/simple_action_server.h>
#include <exploration_msgs/ExploreAction.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <ros/ros.h>

namespace exploration_server{

class ExplorationServer
{
public:
  ExplorationServer(ros::NodeHandle nh, ros::NodeHandle private_nh);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  tf::TransformListener tf_listener_;

  actionlib::ActionServer<exploration_msgs::ExploreAction> explore_action_server_;
  // TODO move_base_client

  boost::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros_;

};

}
