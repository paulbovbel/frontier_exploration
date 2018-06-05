#include "exploration_server/exploration_server.h"
#include "exploration_server/planner_base.h"

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <pluginlib/class_loader.h>

#include <move_base_msgs/MoveBaseAction.h>

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

void ExplorationServer::goalCB(GoalHandle gh){

  // set as active goal GoalHandle
  gh.setAccepted();

  // initialize exploration planner plugin
  pluginlib::ClassLoader<planner_base::RegularPlanner> planner_loader("exploration_server", "planner_base::RegularPlanner");
  try{
    boost::shared_ptr<planner_base::RegularPlanner> planner = planner_loader.createInstance(gh.getGoal()->strategy_plugin);
    planner->initialize();
  }
  catch(pluginlib::PluginlibException& ex){
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
  }
  //   1. preempt active goal handle, if any
  //   2. set as active goal handle
  //   3. initialize exploration planner plugin
  //   4. update boundary polygon on costmap, if necessary
  //   5. request next goal from planner pluigin
  //   6. send goal to move_base
}

void ExplorationServer::moveBaseResultCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result){
  //   1. if successful,
  //     a) request next goal from planner pluginlib
  //   2. if failed
  //     a) 'handle' (retry, add to exploration blacklist PR#25, etc?)
  //     b) move on to next exploration target
  //     c) fail exploration?
}

void ExplorationServer::cancelGoalCb(GoalHandle gh){
  //   1. if active goal handle, cancel
}



}
