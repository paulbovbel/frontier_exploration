#include <exploration_server/exploration_server.h>
#include <exploration_msgs/SetPolygon.h>
#include <exploration_server/geometry_tools.h>

#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <std_srvs/Empty.h>

#include <tf/transform_listener.h>
#include <limits>
#include <utility>
#include <vector>

namespace exploration_server
{

ExplorationServer::ExplorationServer(ros::NodeHandle nh, ros::NodeHandle private_nh):
  nh_(nh),
  private_nh_("~"),
  retry_(5),
  tf_listener_(),
  tf2_listener_(tf2_buffer_),
  success_(false),
  moving_(false),
  move_client_("p3_001/move_base", true),
  previous_state_(actionlib::SimpleClientGoalState::PENDING),
  explore_action_server_(nh,
                        "exploration_server_node",
                        boost::bind(&ExplorationServer::goalCb, this, _1),
                        boost::bind(&ExplorationServer::cancelGoalCb, this, _1),
                        false)
{
  costmap_ros_ = boost::make_shared<costmap_2d::Costmap2DROS>("explore_costmap", tf2_buffer_);
  explore_action_server_.start();
}

void ExplorationServer::goalCb(GoalHandle gh)
{
  explore_center_ = gh.getGoal()->start_point;

  // set as active goal GoalHandle
  gh.setAccepted();
  active_gh_ = gh;

  // initialize exploration planner plugin
  pluginlib::ClassLoader<exploration_server::BasePlugin> planner_loader("exploration_server",
  "exploration_server::BasePlugin");
  try
  {
    planner_ = planner_loader.createInstance(gh.getGoal()->strategy_plugin);
    planner_->initialize(costmap_ros_);
  }
  catch(const pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
    gh.setAborted();
    return;
  }

  // update boundary polygon on costmap, if necessary
  ros::ServiceClient polygon_client = private_nh_.serviceClient<exploration_msgs::SetPolygon>
      ("explore_costmap/polygon_layer/set_polygon");
  exploration_msgs::SetPolygon polygon_service;
  // make use of the polygon service to set the boundary on the polygon using what is stored in the goalhandle
  polygon_service.request.polygon = gh.getGoal()->boundary;
  polygon_ = gh.getGoal()->boundary;

  if (polygon_client.call(polygon_service))
  {
    ROS_INFO("Updating polygon");
  }
  else
  {
    ROS_ERROR("Failed to call update polygon service.");
    gh.setAborted();
    return;
  }

  // request a goal from the plugin and send to move_base
  ROS_INFO("Requesting a goal");
  if (ExplorationServer::inBoundary())
  {
    ExplorationServer::requestAndSendGoal();
  }
}

void ExplorationServer::moveBaseResultCb(const actionlib::SimpleClientGoalState& state,
  const move_base_msgs::MoveBaseResultConstPtr& result)
  {
    previous_state_ = state;
    planner_->addToVisited(move_client_goal_.target_pose.pose.position, previous_state_);
    if (state == actionlib::SimpleClientGoalState::ABORTED)
    {
        if (move_client_goal_.target_pose.pose.position.x == explore_center_.point.x &&
        move_client_goal_.target_pose.pose.position.y == explore_center_.point.y)
        {
          ROS_ERROR("Failed to move to exploration center point. Exploration Failed.");
          return;
        }
        else
        {
          ROS_ERROR("Failed to move.");
        }
    }
    else if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        // reset number of retries back to 5
        retry_ = 5;
        success_ = true;
        ROS_INFO("Move base succeeded");
    }
    moving_ = false;

    // regardless of whether the robot was successful in reaching the point or not,
    // request the next goal from the plugin and send it to move_base
    if (ExplorationServer::inBoundary())
    {
      ExplorationServer::requestAndSendGoal();
    }
}

void ExplorationServer::cancelGoalCb(GoalHandle gh)
{
    {  // scope the lock
    // grab the move_client mutex, lock it, then cancel all move_base goals
    boost::unique_lock<boost::mutex> lock(move_client_lock_);
    move_client_.cancelGoalsAtAndBeforeTime(ros::Time::now());
    lock.unlock();
    }
    ROS_WARN("Current exploration task cancelled");
    // set the GoalHandle as cancelled
    if (gh == active_gh_)
    {
      gh.setCanceled();
    }
}

bool ExplorationServer::inBoundary()
{
  if (moving_)
    return false;
  // get current robot pose in frame of exploration boundary
  geometry_msgs::PoseStamped pose1;
  costmap_ros_->getRobotPose(pose1);
  // evaluate if robot is within exploration boundary using robot_pose in boundary frame
  geometry_msgs::PoseStamped eval_pose = pose1;
  if (eval_pose.header.frame_id != polygon_.header.frame_id)
  {
      try
      {
        tf_listener_.waitForTransform(polygon_.header.frame_id, pose1.header.frame_id,
            pose1.header.stamp, ros::Duration(.1));
        tf_listener_.transformPose(polygon_.header.frame_id, pose1, eval_pose);
      }
      catch(tf::TransformException &ex)
      {
        ROS_ERROR("%s", ex.what());
      }
  }
  // check if robot is not within exploration boundary and needs to return to center of search area
  if (polygon_.polygon.points.size() > 0 && !pointInPolygon(eval_pose.pose.position, polygon_.polygon))
  {
      // check if robot has explored at least one point, and promote debug message to warning
      if (success_)
      {
          ROS_WARN("Robot left exploration boundary, returning to center");
      }
      else
      {
          ROS_DEBUG("Robot not initially in exploration boundary, traveling to center");
      }
      // get current robot position in frame of exploration center
      geometry_msgs::PointStamped eval_point;
      eval_point.header = eval_pose.header;
      eval_point.point = eval_pose.pose.position;
      if (eval_point.header.frame_id != explore_center_.header.frame_id)
      {
          geometry_msgs::PointStamped temp = eval_point;
          tf_listener_.waitForTransform(explore_center_.header.frame_id, temp.header.frame_id,
              temp.header.stamp, ros::Duration(.1));
          tf_listener_.transformPoint(explore_center_.header.frame_id, temp, eval_point);
      }

      geometry_msgs::PoseStamped goal_pose;

      // set goal pose to exploration center
      goal_pose.header = explore_center_.header;
      goal_pose.pose.position = explore_center_.point;
      goal_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yawOfVector(eval_point.point, explore_center_.point));
      // send the goal to the move_base client
      feedback_.current_goal = goal_pose;
      boost::unique_lock<boost::mutex> lock(move_client_lock_);
      move_client_goal_.target_pose = goal_pose;
      move_client_.sendGoal(move_client_goal_, boost::bind(&ExplorationServer::moveBaseResultCb, this, _1, _2), 0, 0);
      moving_ = true;
      lock.unlock();
      return false;
  }
  return true;
}

void ExplorationServer::requestAndSendGoal()
{
  if (moving_)
    return;
  std::vector<geometry_msgs::Point> point_list;
  // get the robot's current position for the request to the plugin planner
  geometry_msgs::PoseStamped current_pose;
  costmap_ros_->getRobotPose(current_pose);
  feedback_.robot_pose = current_pose;
  // request next goal from planner plugin
  point_list = planner_->whereToExplore(current_pose, move_client_goal_.target_pose.pose.position, previous_state_);

  if (point_list.size() != 0)
  {
    // select which point the robot should move to
    geometry_msgs::Point selected_point;
    geometry_msgs::PoseStamped next_goal;
    float min_distance = std::numeric_limits<double>::infinity();
    float dist_to_robot;
    ROS_INFO("Robot is now at %f, %f", current_pose.pose.position.x, current_pose.pose.position.y);
    for (const auto & point : point_list)
    {
      // calculate the distance between a given point and the robot location
      dist_to_robot = sqrt(pow(current_pose.pose.position.x-point.x, 2)+pow(current_pose.pose.position.y-point.y, 2));
      // if it is the closest point to the robot, select it
      if (dist_to_robot < min_distance)
      {
        min_distance = dist_to_robot;
        selected_point = point;
      }
    }

    ROS_INFO("Robot moving to: %f, %f", selected_point.x, selected_point.y);
    // set up the StampedPose message to send to move_base
    next_goal.header.frame_id = costmap_ros_ -> getGlobalFrameID();
    next_goal.header.stamp = ros::Time::now();
    next_goal.pose.position = selected_point;
    next_goal.pose.orientation = tf::createQuaternionMsgFromYaw(yawOfVector(current_pose.pose.position,
        next_goal.pose.position));
    {  // scope the mutex lock
      // send the goal to the move_base client
      feedback_.current_goal = next_goal;
      boost::unique_lock<boost::mutex> lock(move_client_lock_);
      move_client_goal_.target_pose = next_goal;
      move_client_.sendGoal(move_client_goal_, boost::bind(&ExplorationServer::moveBaseResultCb,
          this, _1, _2), 0, 0);
      moving_ = true;
      lock.unlock();
    }
  }
  else
  {
    ROS_INFO("Failed to get a goal from the planner plugin.");
    // try to get a goal from the planner 5 times but throw an error on the 5th time
    if (retry_ == 0)
    {
      ROS_INFO("No valid points found, exploration complete");
      active_gh_.setSucceeded();
    }
    else
    {
      retry_--;
      ExplorationServer::requestAndSendGoal();
    }
  }
}

void ExplorationServer::start()
{
  explore_action_server_.start();
}

}  // namespace exploration_server
