#include "exploration_server/exploration_server.h"

#include <ros/ros.h>

namespace exploration_server{

ExplorationServer::ExplorationServer(ros::NodeHandle nh, ros::NodeHandle private_nh):
  nh_(nh),
  private_nh_(private_nh),
  tf_listener_(),
  explore_action_server_(nh, "explore", false)
{
  costmap_ros_ = boost::make_shared<costmap_2d::Costmap2DROS>("explore_costmap", tf_listener_);

  // explore_action_server_.registerGoalCallback();
  // explore_action_server_.registerCancelCallback();
  explore_action_server_.start();

}

// exploreGoalCb(goal_handle)
//   1. preempt active goal handle, if any
//   2. set as active goal handle
//   3. initialize exploration planner plugin
//   4. update boundary polygon on costmap, if necessary
//   5. request next goal from planner pluigin
//   6. send goal to move_base
//
// moveBaseResultCb(result)
//   1. if successful,
//     a) request next goal from planner pluginlib
//   2. if failed
//     a) 'handle' (retry, add to exploration blacklist PR#25, etc?)
//     b) move on to next exploration target
//     c) fail exploration?
//
// cancelGoalCb(goal_handle)
//   1. if active goal handle, cancel


}
