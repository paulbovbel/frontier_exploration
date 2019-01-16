#ifndef FRONTIER_EXPLORATION_FRONTIER_PLUGIN_H
#define FRONTIER_EXPLORATION_FRONTIER_PLUGIN_H
#include <exploration_server/base_plugin.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <frontier_exploration/Frontier.h>
#include <frontier_exploration/frontier_search.h>

#include <boost/foreach.hpp>
#include <string>
#include <vector>
#include <tf/transform_listener.h>

#include <ros/ros.h>

namespace frontier_exploration
{
/**
 * @brief Frontier exploration plugin that works with exploration server
 */
class FrontierPlugin : public exploration_server::BasePlugin
{
  private:
    boost::shared_ptr<costmap_2d::Costmap2DROS> explore_costmap_ros_;
    tf::TransformListener tf_listener_;
    ros::NodeHandle nh_;
    int min_frontier_size_ = 1;
    std::string frontier_travel_point_ = "closest";
    ros::Publisher blacklist_marker_pub_;
    double blacklist_box_size_;

  public:
    /**
     * @brief Constructor for FrontierPlanner plugin.
     */
    FrontierPlugin() = default;

    /**
     * @brief Implementation of method to initialize the FrontierPlanner plugin
     * @param costmap pointer to the costmap that can be used by the planner plugin
     */
    virtual void initialize(boost::shared_ptr<costmap_2d::Costmap2DROS>& costmap);  // NOLINT (runtime/references)

    /**
     * @brief Future define for visualization_msgs::Marker::DELETEALL. Constant is not defined in ROS Indigo, but functionality is implemented
     */
    static const int DELETEALL = 3;

    /**
    * @brief Method to add a visualization marker to visually 'blacklist' the given point
    * @param point the point to be blacklisted
    */
    void blacklistPointVisually(geometry_msgs::Point point);

    /**
    * @brief Method to visually clear the blacklist
    */
    void clearBlacklistVisually();

    /**
    * @brief Method to add a point to the collection of visited points
    * @param previous_goal the point that was previously the goal sent to move_base
    * @param state the resulting state from attempting to move to that point
    */
    virtual void addToVisited(geometry_msgs::Point previous_goal, const actionlib::SimpleClientGoalState& state);

    /**
     * @brief Implementation of method to determine where the robot should explore next
     * @param start_pose PoseStamped which represents the current location of the robot
     * @param previous_goal the point that was previously sent to move base by the plugin or the server (if robot
     * outside of the exploration boundary)
     * @return list of possible points to explore next
     */
     virtual std::vector<geometry_msgs::Point> whereToExplore(const geometry_msgs::PoseStamped& start_pose,
       const geometry_msgs::Point& previous_goal, const actionlib::SimpleClientGoalState& state);
};
}  // namespace frontier_exploration
#endif  // FRONTIER_EXPLORATION_FRONTIER_PLUGIN_H
