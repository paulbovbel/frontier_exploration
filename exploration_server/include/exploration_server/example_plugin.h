#ifndef EXPLORATION_SERVER_EXAMPLE_PLUGIN_H
#define EXPLORATION_SERVER_EXAMPLE_PLUGIN_H

#include <exploration_server/base_plugin.h>

#include <boost/algorithm/string.hpp>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

#include <math.h>
#include <vector>
#include <fstream>
#include <string>

#include <ros/ros.h>

namespace exploration_server
{
/**
 * @brief Example of planner plugin that works with exploration server
 */
class ExamplePlugin : public exploration_server::BasePlugin
{
  private:
    boost::shared_ptr<costmap_2d::Costmap2DROS> explore_costmap_ros_;
    tf::TransformListener tf_listener_;
    ros::NodeHandle nh_;

  public:
    /**
     * @brief Constructor for ExamplePlugin plugin.
     */
    ExamplePlugin() = default;

    /**
     * @brief Implementation of method to initialize the ExamplePlugin plugin
     * @param costmap pointer to the costmap that can be used by the planner plugin
     */
    virtual void initialize(boost::shared_ptr<costmap_2d::Costmap2DROS>& costmap);  // NOLINT (runtime/references)

    /**
     * @brief Funtion to read in a list of points from a txt file
     * @return list of points stored in the txt file represented as Point objects
     */
    std::vector<geometry_msgs::Point> readPoints();

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
}  // namespace exploration_server
#endif  // EXPLORATION_SERVER_EXAMPLE_PLUGIN_H
