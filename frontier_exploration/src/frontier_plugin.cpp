#include <pluginlib/class_list_macros.h>
#include <frontier_exploration/frontier_plugin.h>

#include <exploration_server/base_plugin.h>
#include <visualization_msgs/Marker.h>
#include <list>
#include <vector>

PLUGINLIB_EXPORT_CLASS(frontier_exploration::FrontierPlugin, exploration_server::BasePlugin)

namespace frontier_exploration
{
void FrontierPlugin::initialize(boost::shared_ptr<costmap_2d::Costmap2DROS>& costmap)
{
  // set up global variables
  FrontierPlugin::explore_costmap_ros_ = costmap;
  blacklist_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("blacklist", 5);
  nh_.param<double>("blacklist_box_size", blacklist_box_size_, 0.5);
}

void FrontierPlugin::blacklistPointVisually(geometry_msgs::Point point)
{
  ROS_WARN("Blacklist point added %f, %f", point.x, point.y);

  // Show point in blacklist topic
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.ns = "blacklist";
  marker.id = visited_points_.size();
  marker.action = visualization_msgs::Marker::ADD;

  marker.header.frame_id = explore_costmap_ros_->getGlobalFrameID();
  marker.header.stamp = ros::Time::now();

  marker.pose.position = point;
  marker.pose.orientation.w = 1.0;

  // Scale is the diameter of the shape
  marker.scale.x = 2 * blacklist_box_size_;
  marker.scale.y = 2 * blacklist_box_size_;
  // Circle
  marker.scale.z = 0.05;

  marker.color.r = 1.0;
  marker.color.a = 0.6;

  blacklist_marker_pub_.publish(marker);
}

void FrontierPlugin::clearBlacklistVisually()
{
  ROS_WARN("Blacklist cleared");

  // Delete all markers from visualization
  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.ns = "blacklist";
  // The constant does not exist in ROS Indigo, although functionality is implemented. We use our own.
  marker.action = DELETEALL;
}

void FrontierPlugin::addToVisited(geometry_msgs::Point previous_goal, const actionlib::SimpleClientGoalState& state)
{
  visited_points_.insert(previous_goal, state);
  if (state == actionlib::SimpleClientGoalState::ABORTED)
  {
    blacklistPointVisually(previous_goal);
  }
  else if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    clearBlacklistVisually();
    visited_points_.clear();
  }
}

std::vector<geometry_msgs::Point> FrontierPlugin::whereToExplore(const geometry_msgs::PoseStamped& start_pose,
  const geometry_msgs::Point& previous_goal, const actionlib::SimpleClientGoalState& state)
{
  std::vector<geometry_msgs::Point> result;
  geometry_msgs::PoseStamped tf_start_pose = start_pose;

  if (start_pose.header.frame_id != explore_costmap_ros_->getGlobalFrameID())
  {
      // error out if no transform available
      if (!tf_listener_.waitForTransform(explore_costmap_ros_->getGlobalFrameID(),
          start_pose.header.frame_id, ros::Time::now(), ros::Duration(.1)))
      {
          ROS_ERROR_STREAM("Couldn't transform from " << explore_costmap_ros_->getGlobalFrameID() <<
              " to " << start_pose.header.frame_id);
          return result;
      }
      geometry_msgs::PoseStamped temp_pose = start_pose;
      tf_listener_.transformPose(explore_costmap_ros_->getGlobalFrameID(), temp_pose, tf_start_pose);
  }

  // initialize frontier search implementation
  FrontierSearch frontierSearch(*(explore_costmap_ros_->getCostmap()), min_frontier_size_, frontier_travel_point_);
  // get list of frontiers from search implementation
  std::list<Frontier> frontier_list = frontierSearch.searchFrom(tf_start_pose.pose.position);

  if (frontier_list.size() == 0)
  {
      ROS_DEBUG("No frontiers found, exploration complete");
      return result;
  }

  for (Frontier frontier : frontier_list)
  {
    if (!visited_points_.contains(frontier.travel_point) ||
    (visited_points_.getWorstValue(frontier.travel_point, blacklist_box_size_) !=
    actionlib::SimpleClientGoalState::ABORTED))
    {
      result.push_back(frontier.travel_point);
    }
  }

  return result;
}

}  // namespace frontier_exploration
