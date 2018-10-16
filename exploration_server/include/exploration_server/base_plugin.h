#ifndef EXPLORATION_SERVER_BASE_PLUGIN_H
#define EXPLORATION_SERVER_BASE_PLUGIN_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <actionlib/client/simple_action_client.h>
#include <exploration_server/visited_points.h>

#include <unordered_map>
#include <utility>

#include <vector>
#include <std_srvs/Empty.h>

namespace exploration_server
{
/**
* @brief Interface for defining an exploration planner plugin to work with exploration_server
*/
class BasePlugin
{
  public:
    VisitedPoints visited_points_;

    /**
    * @brief Method to initialize the planner plugin
    * @param costmap pointer to the costmap that can be used by the planner plugin
    */
    virtual void initialize(boost::shared_ptr<costmap_2d::Costmap2DROS>& costmap) = 0;  // NOLINT (runtime/references)

    /**
    * @brief Method to add a point to the collection of visited points
    * @param previous_goal the point that was previously the goal sent to move_base
    * @param state the resulting state from attempting to move to that point
    */
    virtual void addToVisited(geometry_msgs::Point previous_goal, const actionlib::SimpleClientGoalState& state)
    {
      visited_points_.insert(previous_goal, state);
    }

    /**
    * @brief Method to determine where the robot should explore next
    * @param start_pose PoseStamped which represents the current location of the robot
    * @param previous_goal the point that was previously sent to move base by the plugin or the server (if robot
    * outside of the exploration boundary)
    * @return list of possible points to explore next
    */
    virtual std::vector<geometry_msgs::Point> whereToExplore(const geometry_msgs::PoseStamped& start_pose,
      const geometry_msgs::Point& previous_goal, const actionlib::SimpleClientGoalState& state) = 0;

    /**
    * @brief Destructor for base planner class (BasePlugin)
    */
    virtual ~BasePlugin() = default;

  protected:
    /**
    * @brief Constructor for the base planner class (BasePlugin)
    */
    BasePlugin() = default;
};
}  // namespace exploration_server
#endif  // EXPLORATION_SERVER_BASE_PLUGIN_H
