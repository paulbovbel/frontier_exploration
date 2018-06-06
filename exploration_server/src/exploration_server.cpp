#include "exploration_server/exploration_server.h"
#include "exploration_server/planner_base.h"
#include "exploration_msgs/SetPolygon.h"
#include "exploration_msgs/GetNextGoal.h"
#include "exploration_msgs/BlacklistPoint.h"

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <pluginlib/class_loader.h>

#include <tf/transform_listener.h>

#include <move_base_msgs/MoveBaseAction.h>

namespace exploration_server{

ExplorationServer::ExplorationServer(ros::NodeHandle nh, ros::NodeHandle private_nh):
  nh_(nh),
  private_nh_(private_nh),
  retry_(5),
  tf_listener_(),
  explore_action_server_(nh, "explore", false)
{
  costmap_ros_ = boost::make_shared<costmap_2d::Costmap2DROS>("explore_costmap", tf_listener_);
  // TODO (vmcdermott) need to set up move base client up here (move_client_)
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

  // update boundary polygon on costmap, if necessary
  ros::ServiceClient polygon_client = private_nh_.serviceClient<exploration_msgs::SetPolygon>("set_polygon");
  exploration_msgs::SetPolygon polygon_service;
  polygon_service.request.polygon = gh.getGoal()->boundary;
  if(polygon_client.call(polygon_service)){
    ROS_INFO("Updating polygon");
  }
  else{
    ROS_ERROR("Failed to call update polygon service.");
  }

  // request next goal from planner plugin
  ros::ServiceClient goal_client = private_nh_.serviceClient<exploration_msgs::GetNextGoal>("get_goal");
  exploration_msgs::GetNextGoal goal_service;
  tf::Stamped<tf::Pose> robot_pose;
  costmap_ros_->getRobotPose(robot_pose);
  goal_service.request.start_pose = robot_pose;
  feedback_.robot_pose = robot_pose;
  if(goal_client.call(goal_service)){
    ROS_INFO("requesting a new goal from planner plugin");
    // send the goal to the move_base client
    feedback_.current_goal = goal_service.response.next_goal;
    boost::unique_lock<boost::mutex> lock(move_client_lock_);
    move_client_.sendGoal(goal_service.response.next_goal, boost::bind(&ExplorationServer::moveBaseResultCb, this, _1, _2),0,0);
    lock.unlock();
    retry_ = 5;
  }
  else{
    ROS_DEBUG("Failed to get a goal from the planner plugin.");
    if(retry_==0){
      ROS_ERROR("Failed exploration");
    }
    retry_--;
  }

}

void ExplorationServer::moveBaseResultCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result){
    if (state == actionlib::SimpleClientGoalState::ABORTED){
        ROS_ERROR("Failed to move. Blacklisting point.");

        // Find the blacklist service
        ros::ServiceClient blacklistPointService = private_nh_.serviceClient<BlacklistPoint>("blacklist_point");
        // Create the service request
        BlacklistPoint srv;
        srv.request.point = feedback_.current_goal;

        // Call the service
        if (!blacklistPointService.call(srv)) {
            ROS_ERROR("Failed to blacklist point.");
        }
    }else if(state == actionlib::SimpleClientGoalState::SUCCEEDED){

        // Find the clear blacklist service
        ros::ServiceClient clearBlacklistService = private_nh_.serviceClient<std_srvs::Empty>("clear_blacklist");

        // No argument
        std_srvs::Empty srv;

        // Call the clear blacklist service
        if (!clearBlacklistService.call(srv)) {
            ROS_ERROR("Failed to clear blacklist.");
        }
        // request next goal from planner plugin
        ros::ServiceClient goal_client = private_nh_.serviceClient<exploration_msgs::GetNextGoal>("get_goal");
        exploration_msgs::GetNextGoal goal_service;
        tf::Stamped<tf::Pose> robot_pose;
        costmap_ros_->getRobotPose(robot_pose);
        goal_service.request.start_pose = robot_pose;
        feedback_.robot_pose = robot_pose;
        if(goal_client.call(goal_service)){
          ROS_INFO("requesting a new goal from planner plugin");
          //TODO: (vmcdermott) Am I doing too much here/feels repetitive and redundant with stuff in goalCB?
        }
        else{
          ROS_DEBUG("Failed to get a goal from the planner plugin.");
        }

      }
}

void ExplorationServer::cancelGoalCb(GoalHandle gh){
    boost::unique_lock<boost::mutex> lock(move_client_lock_);
    move_client_.cancelGoalsAtAndBeforeTime(ros::Time::now());
    lock.unlock();
    ROS_WARN("Current exploration task cancelled");
    gh.setCancelled();
}



}
